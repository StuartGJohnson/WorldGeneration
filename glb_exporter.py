# glb_exporter.py
from __future__ import annotations
import os
from typing import Optional, Union, List, Tuple
import numpy as np, io
import trimesh as tm
from trimesh.visual import TextureVisuals, ColorVisuals
from trimesh.visual.material import PBRMaterial, SimpleMaterial
from trimesh.transformations import euler_matrix
from gpt5 import Scene, SceneObject, Cylinder, Box
from PIL import Image
from trimesh.exchange import gltf as gltf_io

# --- coordinate transforms ---

def rx_neg_90() -> np.ndarray:
    """Rotate -90° about X (Z-up -> Y-up part)."""
    R = np.eye(4)
    # [ 1, 0, 0;
    #   0, 0, 1;
    #   0,-1, 0 ]
    R[1,1]=0; R[1,2]=1
    R[2,1]=-1; R[2,2]=0
    return R

def ry_pos_90() -> np.ndarray:
    """Rotate +90° about Y (X-forward -> Z-forward)."""
    R = np.eye(4)
    # [ 0, 0, 1;
    #   0, 1, 0;
    #  -1, 0, 0 ]
    R[0,0]=0;  R[0,2]=1
    R[2,0]=-1; R[2,2]=0
    return R

def translate(x: float, y: float, z: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T

def xform(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    Trot = euler_matrix(yaw, pitch, roll, axes="rzyx")
    return T @ Trot
# --- mesh builders ---

def _ensure_material(mesh: tm.Trimesh, color_rgb: Tuple[float, float, float]) -> None:
    """
    Attach a matte PBR material with baseColorFactor (no texture).
    """
    if mesh.visual is None or not hasattr(mesh.visual, "material"):
        mesh.visual = ColorVisuals(mesh)  # create a visuals holder
    r, g, b = [float(max(0.0, min(1.0, c))) for c in color_rgb]
    mesh.visual.material = PBRMaterial(
        name="mat_matte",
        baseColorFactor=[r, g, b, 1.0],
        metallicFactor=0.0,
        roughnessFactor=1.0,
    )

def _box_mesh(size: Tuple[float, float, float],
              pose_xyz: Tuple[float, float, float, float, float, float],
              color_rgb: Tuple[float, float, float]) -> tm.Trimesh:
    mesh = tm.creation.box(extents=size)
    mesh.apply_transform(xform(*pose_xyz))
    _ensure_material(mesh, color_rgb)
    return mesh

def _cylinder_mesh(radius: float, height: float,
                   pose_xyz: Tuple[float, float, float, float, float, float],
                   color_rgb: Tuple[float, float, float],
                   sections: int = 64) -> tm.Trimesh:
    # trimesh cylinder is along +Z, centered at origin
    mesh = tm.creation.cylinder(radius=radius, height=height, sections=sections)
    mesh.apply_transform(xform(*pose_xyz))
    _ensure_material(mesh, color_rgb)
    return mesh


def _ground_mesh(size, pose_xyz, texture_path):
    """
    Render-friendly ground using a single textured plane (no z-fighting).
    The plane sits at the *top* of your thin box: z = pose_z + size_z/2.
    Texture shows exactly once (UV 0..1).
    """
    sx, sy, sz = size
    px, py, pz, *_ = pose_xyz
    z_top = pz + sz * 0.5 + .001

    # 4 verts (counter-clockwise), 2 triangles
    V = np.array([
        [-sx/2, -sy/2, z_top],   # 0 bottom-left
        [ sx/2, -sy/2, z_top],   # 1 bottom-right
        [ sx/2,  sy/2, z_top],   # 2 top-right
        [-sx/2,  sy/2, z_top],   # 3 top-left
    ], dtype=float)
    # translate by (px,py) in XY
    V[:, 0] += px
    V[:, 1] += py

    F  = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)
    UV = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32)
    # If it appears upside-down in a viewer, flip V:
    # UV[:, 1] = 1.0 - UV[:, 1]

    mesh = tm.Trimesh(vertices=V, faces=F, process=False)

    if not texture_path:
        # fallback: flat gray (matches your other objects)
        r = g = b = 0.7
        mesh.visual = tm.visual.ColorVisuals(
            mesh, vertex_colors=[int(255*r), int(255*g), int(255*b), 255]
        )
        return mesh

    # IMPORTANT: use a PIL image on a PBR material’s baseColorTexture
    img = Image.open(texture_path).convert("RGBA")
    mat = PBRMaterial(
        name="mat_floor",
        baseColorTexture=img,
        metallicFactor=0.0,
        roughnessFactor=1.0,
    )

    # Attach UVs + material (no `image=` arg here)
    mesh.visual = TextureVisuals(uv=UV, material=mat)
    return mesh

# --- main entrypoint ---

def export_glb(scene, filename: str, do_texture: bool = False,
               texture_path: str = "floor_texture.png") -> str:
    g = tm.Scene()
    for obj in scene.objects:
        name = getattr(obj, "name", "obj")
        size = obj.size
        pose = obj.pose
        color = getattr(obj, "color", (0.6, 0.6, 0.6))

        if name == "ground_plane":
            mesh = _ground_mesh(size=size, pose_xyz=pose,
                                texture_path=(texture_path if do_texture else None))
            g.add_geometry(mesh, node_name=name)
            mesh = _box_mesh(size=size, pose_xyz=pose, color_rgb=color)
            g.add_geometry(mesh, node_name="ground_plane_box")
        elif isinstance(obj, Cylinder):
            r = obj.radius; h = obj.height
            mesh = _cylinder_mesh(radius=r, height=h, pose_xyz=pose, color_rgb=color)
            g.add_geometry(mesh, node_name=name)
        else:
            mesh = _box_mesh(size=size, pose_xyz=pose, color_rgb=color)
            g.add_geometry(mesh, node_name=name)

    # Your convention: Y-up, Z-forward → Rx(-90°) then Ry(+90°)
    g.apply_transform(rx_neg_90())
    g.apply_transform(ry_pos_90())

    # Export as GLB bytes (embeds the texture), then write file
    blob = gltf_io.export_glb(g, extension_webp=False)  # returns bytes
    with open(filename, "wb") as f:
        f.write(blob)
    return filename