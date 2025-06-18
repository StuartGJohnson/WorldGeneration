from typing import List, Tuple, Union
import random
import xml.etree.ElementTree as ET
from pxr import Usd, UsdGeom, UsdLux, UsdShade, Gf, Sdf, UsdPhysics
import noise
import numpy as np
import cv2
import yaml
import matplotlib.pyplot as plt
from skimage.graph import MCP, MCP_Connect


# Abstract scene element classes
class SceneObject:
    def __init__(self, name: str, pose: Tuple[float, float, float], size: Tuple[float, float, float]):
        self.name = name
        self.pose = pose
        self.size = size
        self.color = (random.uniform(0.2, 0.8),) * 3  # RGB tuple of gray
        self.physics_material = {
            "density": 1000.0,
            "staticFriction": 100,
            "dynamicFriction": 50,
            "restitution": 0.01
        }

class Box(SceneObject):
    pass

class Cylinder(SceneObject):
    def __init__(self, name: str, pose: Tuple[float, float, float], radius: float, height: float):
        super().__init__(name, pose, (radius*2, radius*2, height))
        self.radius = radius
        self.height = height

# Scene class to hold objects
class Scene:
    def __init__(self):
        self.objects: List[Union[Box, Cylinder]] = []

    def add_box(self, name, pose, size):
        self.objects.append(Box(name, pose, size))

    def add_cylinder(self, name, pose, radius, height):
        self.objects.append(Cylinder(name, pose, radius, height))

    def add_ground_plane(self, size):
        pose = (0.0, 0.0, -size[2] / 2)
        self.add_box("ground_plane", pose, size)

    def generate_perlin_navigable_zone(self,
                                       area_size: Tuple[float, float],
                                       resolution: float,
                                       threshold: float,
                                       scale: float,
                                       seed: int= 42 ) -> (float, np.ndarray):
        max_x, max_y = area_size
        self.add_ground_plane(size=(max_x, max_y, 0.1))
        np_area_size = np.array(area_size)
        max_x //= 2
        max_y //= 2
        #seed = random.randint(0, 1000)
        random.seed(seed)
        count = 0
        occupancy_map = np.zeros((np_area_size//resolution + 1).astype(int), dtype=np.uint8)
        i_max = int(max_x//resolution)
        j_max = int(max_y//resolution)
        length_x = max_x * 2
        length_y = max_y * 2
        z = 1.0
        self.add_box(f"pn_wall_{count}", (-max_x, 0.0, z), (0.2, length_y, 2.0))
        count += 1
        self.add_box(f"pn_wall_{count}", (max_x, 0.0, z), (0.2, length_y, 2.0))
        count += 1
        self.add_box(f"pn_wall_{count}", (0.0, -max_y, z), (length_x, 0.2, 2.0))
        count += 1
        self.add_box(f"pn_wall_{count}", (0.0, max_y, z), (length_x, 0.2, 2.0))
        count += 1
        occupancy_map[:, 0] = 1
        occupancy_map[:, -1] = 1
        occupancy_map[0, :] = 1
        occupancy_map[-1, :] = 1
        for i in range(-i_max+1, i_max-1):
            for j in range(-j_max+1, j_max-1):
                x = i * resolution
                y = j * resolution
                value = noise.pnoise2(i * scale, j * scale, octaves=4, repeatx=1024, repeaty=1024, base=seed)
                if value < threshold:
                    z = 1.0
                    rnum = random.random()
                    if rnum < 0.3:
                        radius = random.uniform(0.2, 0.4)
                        self.add_cylinder(f"pn_pillar_{count}", (x, y, z), radius, 2.0)
                    elif rnum < 0.6:
                        length = random.uniform(1.0, 3.0)
                        self.add_box(f"pn_wall_{count}", (x, y, z), (length, 0.2, 2.0))
                    else:
                        length = random.uniform(1.0, 3.0)
                        self.add_box(f"pn_wall_{count}", (x, y, z), (0.2, length, 2.0))
                    count += 1
                    occupancy_map[i+i_max,j+j_max] = 1
        print(f"Added {count} objects.")
        occupancy_map *= 255
        return resolution, occupancy_map

def get_reachable_locations(resolution: float, occupancy_map: np.ndarray) -> (np.ndarray, np.ndarray):
    # a helper function to compute a good location for starting the robot.
    # will need to be updated to compute a good z
    # switch bg->fg
    free_map = np.max(occupancy_map)-occupancy_map

    # do conn comp analysis
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(free_map, connectivity=4)

    # get the biggest component which is not the background (occupied)
    max_free_label = np.argmax(stats[1:,4]) + 1

    # compute distance to the nearest occupied location
    dist_xform = cv2.distanceTransform(free_map, cv2.DIST_L2, 0)

    # grab the biggest connected component
    biggest_region = labels==max_free_label

    # compute location desirability
    max_image = biggest_region * dist_xform

    # find x,y of good robot start location
    ij_max = np.unravel_index(np.argmax(max_image), max_image.shape)
    #compute xy location - center is 0,0, so...
    map_size = np.array(max_image.shape)
    start_point_xy = resolution * (ij_max - np.array((map_size-1)//2))

    # compute distances to all free points in the biggest free area
    mask = (biggest_region > 0)  # binary mask
    cost_map = np.where(mask, 1.0, np.inf)  # non-zero pixels cost 1, others are blocked
    start = tuple(ij_max)
    # Compute distances to all reachable points
    mcp = MCP_Connect(cost_map)
    distances, _ = mcp.find_costs([start])
    # post process out infinities
    distances[distances==np.inf] = -1.0
    # find x,y of good robot stop location
    ij_max_stop = np.unravel_index(np.argmax(distances), distances.shape)
    #compute xy location - center is 0,0, so...
    map_size = np.array(distances.shape)
    stop_point_xy = resolution * (ij_max_stop - np.array((map_size-1)//2))
    return start_point_xy, stop_point_xy

# Exporters

def export_metadata(seed: int,
                    threshold: float,
                    resolution: float,
                    area: Tuple[float,float],
                    scale: float,
                    robot_start_xy: np.ndarray,
                    robot_stop_xy: np.ndarray,
                    occ_map: np.ndarray,
                    filename: str) -> None:
    # assemble serialization dictionary
    data = {'seed': seed,
            'threshold': threshold,
            'resolution': resolution,
            'area': list(area),
            'scale': scale,
            'robot_start_xy': robot_start_xy.tolist(),
            'robot_stop_xy': robot_stop_xy.tolist(),
            }
    with open(filename + ".yml", 'w') as file:
        yaml.dump(data, file)

    cv2.imwrite(filename + ".png", occ_map)


def export_sdf(scene: Scene, filename: str):
    sdf = ET.Element("sdf", version="1.7")
    world = ET.SubElement(sdf, "world", name="default")

    # add gazebo plugins
    ET.SubElement(world, "plugin", filename="gz-sim-physics-system", name="gz::sim::systems::Physics")
    ET.SubElement(world, "plugin", filename="gz-sim-user-commands-system", name="gz::sim::systems::UserCommands")
    ET.SubElement(world, "plugin", filename="gz-sim-scene-broadcaster-system", name="gz::sim::systems::SceneBroadcaster")
    gz_sensors = ET.SubElement(world, "plugin", filename="gz-sim-sensors-system", name="gz::sim::systems::Sensors")
    ET.SubElement(gz_sensors, "render_engine").text = "ogre2"
    ET.SubElement(world, "plugin", filename="gz-sim-imu-system", name="gz::sim::systems::Imu")
    ET.SubElement(world, "plugin", filename="gz-sim-contact-system", name="gz::sim::systems::Contact")
    # gz_ground_truth = ET.SubElement(world, "plugin", filename="gz-sim-ros-gz-bridge-system", name="gz::sim::systems::ParameterBridge")
    # ET.SubElement(gz_ground_truth, "topic_name").text = "/world/default/pose/info"
    # ET.SubElement(gz_ground_truth, "ros_type").text = "ros_gz_interfaces/msg/EntityPose_V"
    # ET.SubElement(gz_ground_truth, "gz_type").text = "gz.msgs.Pose_V"

    # Add sunlight to SDF
    light = ET.SubElement(world, "light", name="sun")
    light.set("type", "directional")
    ET.SubElement(light, "cast_shadows").text = "true"
    ET.SubElement(light, "direction").text = "-0.5 -0.5 -1"
    ET.SubElement(light, "diffuse").text = "1 1 1 1"
    ET.SubElement(light, "specular").text = "0.1 0.1 0.1 1"
    ET.SubElement(light, "intensity").text = "1.0"

    # general physics solver
    phy = ET.SubElement(world, "physics", type="ode")
    ET.SubElement(phy, "real_time_update_rate").text = "1000.0"
    ET.SubElement(phy, "max_step_size").text = "0.001"
    ET.SubElement(phy, "real_time_factor").text = "1"
    phy_ode = ET.SubElement(phy, "ode")
    phy_ode_solver = ET.SubElement(phy_ode, "solver")
    ET.SubElement(phy_ode_solver, "type").text = "quick"
    ET.SubElement(phy_ode_solver, "iters").text = "150"
    ET.SubElement(phy_ode_solver, "precon_iters").text = "0"
    ET.SubElement(phy_ode_solver, "sor").text = "1.4"
    ET.SubElement(phy_ode_solver, "use_dynamic_moi_rescaling").text = "1"
    phy_ode_constraints = ET.SubElement(phy_ode, "constraints")
    ET.SubElement(phy_ode_constraints, "cfm").text = "0.00001"
    ET.SubElement(phy_ode_constraints, "erp").text = "0.2"
    ET.SubElement(phy_ode_constraints, "contact_max_correcting_vel").text = "2000.0"
    ET.SubElement(phy_ode_constraints, "contact_surface_layer").text = "0.01"

    for obj in scene.objects:
        model = ET.SubElement(world, "model", name=obj.name)
        static = ET.SubElement(model, "static")
        static.text = "true"
        link = ET.SubElement(model, "link", name="link")
        pose = ET.SubElement(link, "pose")
        pose.text = f"{obj.pose[0]} {obj.pose[1]} {obj.pose[2]} 0 0 0"
        visual = ET.SubElement(link, "visual", name="visual")
        mat = ET.SubElement(visual, "material")
        ambient = ET.SubElement(mat, "ambient")
        ambient.text = f"{obj.color[0]} {obj.color[1]} {obj.color[2]} 1"
        diffuse = ET.SubElement(mat, "diffuse")
        diffuse.text = f"{obj.color[0]} {obj.color[1]} {obj.color[2]} 1"
        geom = ET.SubElement(visual, "geometry")
        if isinstance(obj, Box):
            box = ET.SubElement(geom, "box")
            size = ET.SubElement(box, "size")
            size.text = f"{obj.size[0]} {obj.size[1]} {obj.size[2]}"
        elif isinstance(obj, Cylinder):
            cyl = ET.SubElement(geom, "cylinder")
            radius = ET.SubElement(cyl, "radius")
            radius.text = str(obj.radius)
            length = ET.SubElement(cyl, "length")
            length.text = str(obj.height)
        collision = ET.SubElement(link, "collision", name="collision")
        geom2 = ET.SubElement(collision, "geometry")
        if isinstance(obj, Box):
            box = ET.SubElement(geom2, "box")
            size = ET.SubElement(box, "size")
            size.text = f"{obj.size[0]} {obj.size[1]} {obj.size[2]}"
        elif isinstance(obj, Cylinder):
            cyl = ET.SubElement(geom2, "cylinder")
            radius = ET.SubElement(cyl, "radius")
            radius.text = str(obj.radius)
            length = ET.SubElement(cyl, "length")
            length.text = str(obj.height)
        surf = ET.SubElement(collision, "surface")
        friction = ET.SubElement(surf, "friction")
        ode = ET.SubElement(friction, "ode")
        ET.SubElement(ode, "mu").text = str(obj.physics_material["staticFriction"])
        ET.SubElement(ode, "mu2").text = str(obj.physics_material["staticFriction"]/2.0)
        ET.SubElement(ode, "slip1").text = "0.0"
        ET.SubElement(ode, "slip2").text = "0.0"
        ET.SubElement(surf, "bounce")
        contact = ET.SubElement(surf, "contact")
        ode2 = ET.SubElement(contact, "ode")
        ET.SubElement(ode2, "kp").text = "10000.0"
        ET.SubElement(ode2, "kd").text = "100.0"

    tree = ET.ElementTree(sdf)
    ET.indent(sdf)
    tree.write(filename + ".sdf")

def export_usda(scene: Scene, filename: str):
    stage = Usd.Stage.CreateNew(filename + ".usda")
    stage.SetMetadata("metersPerUnit", 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    world_xform = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world_xform.GetPrim())

    # Define the physics scene (usually at "/World")
    phys_scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")

    # Gravity
    phys_scene.CreateGravityDirectionAttr().Set((0.0, 0.0, -1.0))  # "-Z"
    phys_scene.CreateGravityMagnitudeAttr().Set(9.81)

    # Contact and rest offsets
    phys_scene.GetPrim().CreateAttribute("physics:contactOffset", Sdf.ValueTypeNames.Float, custom=False).Set(0.02)
    phys_scene.GetPrim().CreateAttribute("physics:restOffset", Sdf.ValueTypeNames.Float, custom=False).Set(0.001)

    # Solver iteration counts
    phys_scene.GetPrim().CreateAttribute("physics:positionIterations", Sdf.ValueTypeNames.Int, custom=False).Set(16)
    phys_scene.GetPrim().CreateAttribute("physics:velocityIterations", Sdf.ValueTypeNames.Int, custom=False).Set(8)

    # Add directional light (sunlight)
    light = UsdLux.DistantLight.Define(stage, "/SunLight")
    light.CreateIntensityAttr(1000.0)
    light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 0.95))
    light.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 45.0))

    for obj in scene.objects:
        xform = UsdGeom.Xform.Define(stage, f"/World/{obj.name}")
        geom = []
        if isinstance(obj, Box):
            geom = UsdGeom.Cube.Define(stage, f"/World/{obj.name}/geom")
        elif isinstance(obj, Cylinder):
            geom = UsdGeom.Cylinder.Define(stage, f"/World/{obj.name}/geom")
        size= tuple(np.array(obj.size) * 0.5)
        xform.AddTranslateOp().Set(Gf.Vec3d(*obj.pose))
        xform.AddScaleOp().Set(Gf.Vec3d(*size))

        # Collision and Physics
        rigid_api = UsdPhysics.RigidBodyAPI.Apply(geom.GetPrim())
        rigid_api.CreateRigidBodyEnabledAttr().Set(True)
        geom.GetPrim().CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool, custom=False).Set(True)
        phys = UsdPhysics.MaterialAPI.Apply(geom.GetPrim())
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        UsdPhysics.MaterialAPI.Apply(geom.GetPrim())
        phys.CreateStaticFrictionAttr().Set(obj.physics_material["staticFriction"])
        phys.CreateDynamicFrictionAttr().Set(obj.physics_material["dynamicFriction"])
        phys.CreateRestitutionAttr().Set(obj.physics_material["restitution"])
        phys.CreateDensityAttr().Set(obj.physics_material["density"])

        # add on physx manually
        existing_tokens = geom.GetPrim().GetMetadata("apiSchemas")
        tmp = list(existing_tokens.explicitItems)
        tmp.append("PhysxCollisionAPI")
        new_tokens = Sdf.TokenListOp.CreateExplicit(tmp)
        geom.GetPrim().SetMetadata("apiSchemas", new_tokens)
        geom.GetPrim().CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float, custom=False).Set(0.02)
        geom.GetPrim().CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float, custom=False).Set(0.001)

        material = UsdShade.Material.Define(stage, f"/World/{obj.name}_material")
        shader = UsdShade.Shader.Define(stage, f"/World/{obj.name}_material/diffuseShader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*obj.color))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(geom).Bind(material)

    stage.GetRootLayer().Save()

# Example usage
# scene = Scene()
# scene.add_ground_plane()
# scene.generate_perlin_navigable_zone(area_size=(20.0, 20.0), resolution=1.0, threshold=0.0)
#
# export_sdf(scene, "scene.sdf")
# export_usda(scene, "scene.usda")

