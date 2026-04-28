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
import os



# Abstract scene element classes
class SceneObject:
    def __init__(self, name: str, pose: Tuple[float, float, float, float, float, float], size: Tuple[float, float, float]):
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
    def __init__(self, name: str, pose: Tuple[float, float, float, float, float, float], radius: float, height: float):
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
        pose = (0.0, 0.0, -size[2] / 2, 0.0, 0.0, 0.0)
        self.add_box("ground_plane", pose, size)

    def make_calibration_scene(self):
        # my calibration box is denominated in inches!
        bx=14.0*2.54/100.0
        by=19.0*2.54/100.0
        bz=14.0*2.54/100.0
        # the corner will be placed at (0,0,0), with the long side to the left
        # compute box corner location relative to box center
        r = np.pi-np.pi/4.0
        xc = np.cos(r)*bx/2.0 - np.sin(r)*by/2.0
        yc = np.sin(r) * bx / 2.0 + np.cos(r) * by / 2.0
        pose = (-xc, -yc, bz/2.0, 0.0, 0.0, r)
        self.add_box("calibration_target", pose, (bx,by,bz))
        self.add_ground_plane(size=(3, 3, 0.001))

    def export_ros2_map(self,
                        filename: str,
                        min_object_height: float,
                        map_resolution_meters: float,
                        map_buffer_xy_meters: float) -> Tuple[np.ndarray, dict]:
        """
        Export a Nav2-compatible occupancy map from the actual scene geometry.

        The returned/generated ros2_occupancy_map uses ROS OccupancyGrid values:
        -1 unknown, 0 free, 100 occupied. The PNG encodes those as gray, white,
        and black respectively for Nav2's trinary map loader.
        """
        if map_resolution_meters <= 0.0:
            raise ValueError("map_resolution_meters must be positive")
        if map_buffer_xy_meters < 0.0:
            raise ValueError("map_buffer_xy_meters must be non-negative")

        ground = next((obj for obj in self.objects if obj.name == "ground_plane" and isinstance(obj, Box)), None)
        if ground is not None:
            scene_min_x = ground.pose[0] - ground.size[0] / 2.0
            scene_max_x = ground.pose[0] + ground.size[0] / 2.0
            scene_min_y = ground.pose[1] - ground.size[1] / 2.0
            scene_max_y = ground.pose[1] + ground.size[1] / 2.0
        else:
            bounds = [self._object_xy_bounds(obj) for obj in self.objects if obj.name != "ground_plane"]
            if not bounds:
                raise ValueError("Cannot export a ROS2 map for an empty scene")
            scene_min_x = min(bound[0] for bound in bounds)
            scene_max_x = max(bound[1] for bound in bounds)
            scene_min_y = min(bound[2] for bound in bounds)
            scene_max_y = max(bound[3] for bound in bounds)

        origin_x = scene_min_x - map_buffer_xy_meters
        origin_y = scene_min_y - map_buffer_xy_meters
        max_x = scene_max_x + map_buffer_xy_meters
        max_y = scene_max_y + map_buffer_xy_meters
        width = int(np.ceil((max_x - origin_x) / map_resolution_meters))
        height = int(np.ceil((max_y - origin_y) / map_resolution_meters))

        solid_map = np.zeros((height, width), dtype=np.uint8)
        scene_mask = self._scene_extent_mask(height, width, origin_x, origin_y,
                                             map_resolution_meters,
                                             scene_min_x, scene_max_x,
                                             scene_min_y, scene_max_y)
        for obj in self.objects:
            if obj.name == "ground_plane":
                continue
            top_z = obj.pose[2] + obj.size[2] / 2.0
            if top_z < min_object_height:
                continue
            self._rasterize_object_xy(solid_map, obj, origin_x, origin_y,
                                      map_resolution_meters)

        reachable_free = self._reachable_free_space(solid_map, scene_mask)
        visible_obstacles = self._visible_obstacle_surfaces(solid_map, reachable_free)

        ros2_occupancy_map = np.full((height, width), -1, dtype=np.int8)
        ros2_occupancy_map[reachable_free] = 0
        ros2_occupancy_map[visible_obstacles] = 100

        png_map = np.full((height, width), 205, dtype=np.uint8)
        png_map[ros2_occupancy_map == 0] = 254
        png_map[ros2_occupancy_map == 100] = 0

        image_filename = filename + ".png"
        yaml_filename = filename + ".yaml"
        # Image rows are top-to-bottom; ROS map origin is the lower-left cell.
        cv2.imwrite(image_filename, np.flipud(png_map))

        metadata = {
            "image": os.path.basename(image_filename),
            "mode": "trinary",
            "resolution": float(map_resolution_meters),
            "origin": [float(origin_x), float(origin_y), 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.25,
        }
        with open(yaml_filename, "w") as file:
            yaml.dump(metadata, file, sort_keys=False)

        return ros2_occupancy_map, metadata

    def _object_xy_bounds(self, obj: SceneObject) -> Tuple[float, float, float, float]:
        if isinstance(obj, Cylinder):
            return (obj.pose[0] - obj.radius, obj.pose[0] + obj.radius,
                    obj.pose[1] - obj.radius, obj.pose[1] + obj.radius)

        sx, sy = obj.size[0], obj.size[1]
        yaw = obj.pose[5]
        corners = np.array([
            [-sx / 2.0, -sy / 2.0],
            [ sx / 2.0, -sy / 2.0],
            [ sx / 2.0,  sy / 2.0],
            [-sx / 2.0,  sy / 2.0],
        ])
        rot = np.array([[np.cos(yaw), -np.sin(yaw)],
                        [np.sin(yaw),  np.cos(yaw)]])
        world = corners @ rot.T + np.array([obj.pose[0], obj.pose[1]])
        return (float(np.min(world[:, 0])), float(np.max(world[:, 0])),
                float(np.min(world[:, 1])), float(np.max(world[:, 1])))

    def _scene_extent_mask(self,
                           height: int,
                           width: int,
                           origin_x: float,
                           origin_y: float,
                           resolution: float,
                           scene_min_x: float,
                           scene_max_x: float,
                           scene_min_y: float,
                           scene_max_y: float) -> np.ndarray:
        cols = np.arange(width)
        rows = np.arange(height)
        xs = origin_x + (cols + 0.5) * resolution
        ys = origin_y + (rows + 0.5) * resolution
        return ((ys[:, None] >= scene_min_y) & (ys[:, None] <= scene_max_y) &
                (xs[None, :] >= scene_min_x) & (xs[None, :] <= scene_max_x))

    def _rasterize_object_xy(self,
                             solid_map: np.ndarray,
                             obj: SceneObject,
                             origin_x: float,
                             origin_y: float,
                             resolution: float) -> None:
        min_x, max_x, min_y, max_y = self._object_xy_bounds(obj)
        height, width = solid_map.shape
        col_min = max(0, int(np.floor((min_x - origin_x) / resolution)))
        col_max = min(width - 1, int(np.ceil((max_x - origin_x) / resolution)))
        row_min = max(0, int(np.floor((min_y - origin_y) / resolution)))
        row_max = min(height - 1, int(np.ceil((max_y - origin_y) / resolution)))
        if col_min > col_max or row_min > row_max:
            return

        cols = np.arange(col_min, col_max + 1)
        rows = np.arange(row_min, row_max + 1)
        xs = origin_x + (cols + 0.5) * resolution
        ys = origin_y + (rows + 0.5) * resolution
        grid_x, grid_y = np.meshgrid(xs, ys)

        if isinstance(obj, Cylinder):
            mask = (grid_x - obj.pose[0]) ** 2 + (grid_y - obj.pose[1]) ** 2 <= obj.radius ** 2
        else:
            yaw = obj.pose[5]
            dx = grid_x - obj.pose[0]
            dy = grid_y - obj.pose[1]
            local_x = np.cos(yaw) * dx + np.sin(yaw) * dy
            local_y = -np.sin(yaw) * dx + np.cos(yaw) * dy
            mask = ((np.abs(local_x) <= obj.size[0] / 2.0) &
                    (np.abs(local_y) <= obj.size[1] / 2.0))

        solid_map[row_min:row_max + 1, col_min:col_max + 1][mask] = 1

    def _reachable_free_space(self, solid_map: np.ndarray, scene_mask: np.ndarray) -> np.ndarray:
        free_map = ((solid_map == 0) & scene_mask).astype(np.uint8)
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(free_map, connectivity=4)
        if num_labels <= 1:
            return np.zeros_like(solid_map, dtype=bool)
        max_free_label = np.argmax(stats[1:, cv2.CC_STAT_AREA]) + 1
        return labels == max_free_label

    def _visible_obstacle_surfaces(self,
                                   solid_map: np.ndarray,
                                   reachable_free: np.ndarray) -> np.ndarray:
        kernel = np.array([[0, 1, 0],
                           [1, 1, 1],
                           [0, 1, 0]], dtype=np.uint8)
        adjacent_to_reachable_free = cv2.dilate(reachable_free.astype(np.uint8), kernel) > 0
        return (solid_map > 0) & adjacent_to_reachable_free

    def fill_map_high(self, np_area_size, i_max, j_max, resolution, threshold, perlin_map):
        occupancy_map = np.zeros((np_area_size//resolution + 1).astype(int), dtype=np.uint8)
        count = 0
        for i in range(-i_max+1, i_max-1):
            for j in range(-j_max+1, j_max-1):
                x = i * resolution
                y = j * resolution
                value = perlin_map[i+i_max,j+j_max]
                if value < threshold:
                    z = 1.0
                    rnum = random.random()
                    if rnum < 0.3:
                        radius = random.uniform(0.2, 0.4) * resolution
                        self.add_cylinder(f"pn_pillar_{count}", (x, y, z, 0.0, 0.0, 0.0), radius, 2.0)
                    elif rnum < 0.6:
                        length = random.uniform(0.8, 1.0) * resolution
                        length2 = resolution/4.0
                        self.add_box(f"pn_wall_{count}", (x, y, z, 0.0, 0.0, 0.0), (length, length2, 2.0))
                    else:
                        length = random.uniform(0.8, 1.0) * resolution
                        length2 = resolution / 4.0
                        self.add_box(f"pn_wall_{count}", (x, y, z, 0.0, 0.0, 0.0), (length2, length, 2.0))
                    count += 1
                    occupancy_map[i+i_max,j+j_max] = 1
        return count, occupancy_map

    def fill_map_clutter(self, i_max, j_max, resolution, threshold, perlin_map):
        occupancy_map = (perlin_map < threshold).astype(np.uint8)
        # invert the occupancy map, and find distance to the nearest non-zero pixel
        # free_map  = 1 - occupancy_map
        dist_map = cv2.distanceTransform(occupancy_map, cv2.DIST_L2, 0)
        # plt.figure()
        # plt.imshow(dist_map)
        # plt.colorbar()
        # plt.show()
        # plt.figure()
        # plt.imshow(occupancy_map*255)
        # plt.colorbar()
        # plt.show()
        # peripheral pixels have z = 0.05
        # interior pixels have z = 2.0
        count = 0
        for i in range(-i_max + 1, i_max - 1):
            for j in range(-j_max + 1, j_max - 1):
                x = i * resolution
                y = j * resolution
                value = dist_map[i + i_max, j + j_max]
                if value > 1:
                    height = 1.0
                    z = height / 2
                    xy = resolution
                    self.add_box(f"pn_wall_{count}", (x, y, z, 0.0, 0.0, 0.0), (xy, xy, height))
                    count += 1
                elif value > 0.5:
                    height = random.uniform(0.05, 1.0) * resolution
                    z = height / 2
                    radius = random.uniform(0.2, 0.5) * resolution
                    self.add_cylinder(f"pn_pillar_{count}", (x, y, z, 0.0, 0.0, 0.0), radius, height)
                    rnum = random.random()
                    count += 1
        return count, occupancy_map


    def generate_perlin_navigable_zone(self,
                                       area_size: Tuple[float, float],
                                       resolution: float,
                                       threshold: float,
                                       scale: float,
                                       scene_type_tall: bool,
                                       seed: int= 42 ) -> (float, np.ndarray):
        max_x, max_y = area_size
        self.add_ground_plane(size=(max_x, max_y, 0.1))
        np_area_size = np.array(area_size)
        max_x //= 2
        max_y //= 2
        #seed = random.randint(0, 1000)
        random.seed(seed)
        #occupancy_map = np.zeros((np_area_size//resolution + 1).astype(int), dtype=np.uint8)
        perlin_map = np.zeros((np_area_size//resolution + 1).astype(int), dtype=float)
        height_map = np.zeros((np_area_size//resolution + 1).astype(int), dtype=float)
        i_max = int(max_x//resolution)
        j_max = int(max_y//resolution)
        length_x = max_x * 2
        length_y = max_y * 2
        # first, sample perlin noise over the map
        for i in range(-i_max, i_max):
            for j in range(-j_max, j_max):
                x = i * resolution
                y = j * resolution
                perlin_map[i + i_max, j + j_max] = noise.pnoise2(i * scale, j * scale, octaves=4, repeatx=1024, repeaty=1024, base=seed)
        if scene_type_tall:
            count, occupancy_map = self.fill_map_high(np_area_size, i_max, j_max, resolution, threshold, perlin_map)
        else:
            count, occupancy_map = self.fill_map_clutter(i_max, j_max, resolution, threshold, perlin_map)
        # add walls
        z = 0.5
        self.add_box(f"pn_wall_{count}", (-max_x, 0.0, z, 0.0, 0.0, 0.0), (0.2, length_y, 1.0))
        count += 1
        self.add_box(f"pn_wall_{count}", (max_x, 0.0, z, 0.0, 0.0, 0.0), (0.2, length_y, 1.0))
        count += 1
        self.add_box(f"pn_wall_{count}", (0.0, -max_y, z, 0.0, 0.0, 0.0), (length_x, 0.2, 1.0))
        count += 1
        self.add_box(f"pn_wall_{count}", (0.0, max_y, z, 0.0, 0.0, 0.0), (length_x, 0.2, 1.0))
        count += 1
        occupancy_map[:, 0] = 1
        occupancy_map[:, -1] = 1
        occupancy_map[0, :] = 1
        occupancy_map[-1, :] = 1

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
                    occ_map: np.ndarray | None,
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

    if occ_map is not None:
        cv2.imwrite(filename + ".png", occ_map)


def export_ros2_map(scene: Scene,
                    filename: str,
                    min_object_height: float,
                    map_resolution_meters: float,
                    map_buffer_xy_meters: float) -> Tuple[np.ndarray, dict]:
    return scene.export_ros2_map(filename,
                                 min_object_height,
                                 map_resolution_meters,
                                 map_buffer_xy_meters)


def export_sdf(scene: Scene, filename: str, do_texture: bool = False):
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
    #gz_pose = ET.SubElement(world, "plugin", filename="gz-sim-pose-publisher-system", name="gz::sim::systems::PosePublisher")
    #ET.SubElement(gz_pose, "publish_nested_model_pose").text = "true"
    #ET.SubElement(gz_pose, "use_pose_vector_msg").text = "true"
    #ET.SubElement(gz_pose, "update_frequency").text = "-1"
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
    ET.SubElement(phy, "max_step_size").text = "0.001"
    ET.SubElement(phy, "real_time_factor").text = "1.0"
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
        pose.text = f"{obj.pose[0]} {obj.pose[1]} {obj.pose[2]} {obj.pose[3]} {obj.pose[4]} {obj.pose[5]}"
        visual = ET.SubElement(link, "visual", name="visual")
        mat = ET.SubElement(visual, "material")
        if obj.name == "ground_plane" and do_texture:
            pbr = ET.SubElement(mat, "pbr")
            ambient = ET.SubElement(mat, "ambient")
            ambient.text = "1 1 1 1"
            diffuse = ET.SubElement(mat, "diffuse")
            diffuse.text = "1 1 1 1"
            metal = ET.SubElement(pbr, "metal")
            albedo_map = ET.SubElement(metal, "albedo_map")
            # some might say the right way to do this is via a
            # gazebo resource env variable
            cwd = os.getcwd()
            abspath = os.path.join("file://" + cwd, "floor_texture.png")
            albedo_map.text  = abspath
            metalness = ET.SubElement(metal, "metalness")
            metalness.text = "0.0"
            roughness = ET.SubElement(metal, "roughness")
            roughness.text = "1.0"
        else:
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
        xform.AddTranslateOp().Set(Gf.Vec3d(*obj.pose[0:3]))
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
