# mwe_glb_texture.py
import os, numpy as np
import trimesh as tm
from PIL import Image
from trimesh.visual.texture import TextureVisuals
from trimesh.visual.material import PBRMaterial
from trimesh.exchange import gltf as gltf_io  # to call export_glb directly

def textured_plane(w=10.0, h=10.0, z=0.0, png_path="floor_texture.png"):
    # 4 verts, 2 triangles, Y-up plane (Z-forward after your global fix)
    V = np.array([
        [-w/2, -h/2, z],  # 0 bottom-left
        [ w/2, -h/2, z],  # 1 bottom-right
        [ w/2,  h/2, z],  # 2 top-right
        [-w/2,  h/2, z],  # 3 top-left
    ], dtype=float)
    F = np.array([[0,1,2],[0,2,3]], dtype=np.int32)
    UV = np.array([[0,0],[1,0],[1,1],[0,1]], dtype=np.float32)

    mesh = tm.Trimesh(vertices=V, faces=F, process=False)
    # IMPORTANT: use PIL.Image here — PBRMaterial expects a PIL image
    img = Image.open(png_path).convert("RGBA")
    mat = PBRMaterial(
        name="mat_floor",
        baseColorTexture=img,
        metallicFactor=0.0,
        roughnessFactor=1.0
    )
    mesh.visual = TextureVisuals(uv=UV, material=mat)
    return mesh

def export_glb_textured_floor(png_path, out_glb="only_floor.glb"):
    m = textured_plane(png_path=png_path)
    scene = tm.Scene([m])

    # Apply your global convention: Rx(-90°) then Ry(+90°) → Y-up, Z-forward
    Rx = np.array([[1,0,0,0],[0,0,1,0],[0,-1,0,0],[0,0,0,1]], float)
    Ry = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]], float)
    scene.apply_transform(Rx)
    scene.apply_transform(Ry)

    # Use the exporter directly to get bytes, then write them
    blob = gltf_io.export_glb(scene, extension_webp=False)  # returns bytes
    with open(out_glb, "wb") as f:
        f.write(blob)
    print("Wrote", out_glb, "size:", os.path.getsize(out_glb), "bytes")

if __name__ == "__main__":
    export_glb_textured_floor("floor_texture.png", "only_floor.glb")
