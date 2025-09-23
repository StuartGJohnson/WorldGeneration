# make_floor_texture.py
import numpy as np
from PIL import Image, ImageDraw, ImageFilter
import random, os

import numpy as np

def hsv_to_rgb(h, s, v):
    # h,s,v in [0,1], vectorized
    i = np.floor(h*6).astype(int)
    f = h*6 - i
    p = v*(1 - s)
    q = v*(1 - f*s)
    t = v*(1 - (1 - f)*s)
    i_mod = i % 6
    r = np.choose(i_mod, [v,q,p,p,t,v])
    g = np.choose(i_mod, [t,v,v,q,p,p])
    b = np.choose(i_mod, [p,p,t,v,v,q])
    return r, g, b

def add_subtle_color(arr_rgb01, seed=0, sat=0.12, hue_grid=64):
    """
    arr_rgb01: float32 [H,W,3] in 0..1 from your grayscale/dots base
    Adds a low-sat hue field + slight value modulation to preserve grayscale features.
    """
    H, W, _ = arr_rgb01.shape
    # Build a smooth hue field
    hue = value_noise(size=H, grid=hue_grid, seed=seed ^ 0x1234)  # 0..1
    # Saturation field, small and smooth
    s_field = np.clip(sat * (0.8 + 0.4*value_noise(size=H, grid=hue_grid//2, seed=seed ^ 0xBEEF)), 0.02, 0.2)
    # Keep value tied to current luminance so features remain
    lum = (0.2126*arr_rgb01[...,0] + 0.7152*arr_rgb01[...,1] + 0.0722*arr_rgb01[...,2])
    # Gentle value wobble so color also moves brightness a bit
    v = np.clip(lum * (0.95 + 0.1*value_noise(size=H, grid=hue_grid//2, seed=seed ^ 0x55AA)), 0.0, 1.0)
    r,g,b = hsv_to_rgb(hue, s_field, v)
    colored = np.stack([r,g,b], axis=-1)
    # Mix: mostly original luminance texture, with a hint of color
    out = np.clip(0.7*arr_rgb01 + 0.3*colored, 0.0, 1.0)
    return out

def value_noise(size=4096, grid=256, seed=0):
    rng = np.random.default_rng(seed)
    base = rng.random((grid, grid), dtype=np.float32)
    # upscale with bilinear to 'size'
    img = Image.fromarray((base * 255).astype(np.uint8), mode='L')
    img = img.resize((size, size), resample=Image.BILINEAR)
    arr = np.asarray(img).astype(np.float32) / 255.0
    return arr

def make_floor(path="floor_texture.png", size=4096, seed=None):
    if seed is None: seed = random.randrange(1<<31)
    rng = random.Random(seed)

    # start light gray to avoid blown highlights under lighting
    img = Image.new("RGB", (size, size), (232, 232, 232))
    draw = ImageDraw.Draw(img, "RGBA")

    # Multi-scale dots (good corners)
    for radius, count, alpha in [(2, 20000, 140), (4, 8000, 110), (8, 3000, 90), (12, 1500, 80)]:
        for _ in range(count):
            x = rng.randrange(size); y = rng.randrange(size)
            c = rng.randrange(80, 200)  # mid-tone dots
            draw.ellipse((x-radius, y-radius, x+radius, y+radius), fill=(c, c, c, alpha))

    # Faint random short lines (add oriented gradients)
    for _ in range(4000):
        x = rng.randrange(size); y = rng.randrange(size)
        length = rng.uniform(8, 60)
        ang = rng.uniform(0, 2*np.pi)
        x2 = int(x + length*np.cos(ang)); y2 = int(y + length*np.sin(ang))
        c = rng.randrange(90, 180)
        draw.line((x, y, x2, y2), fill=(c, c, c, 70), width=1)

    # Low-frequency value noise (large-scale variation)
    n = value_noise(size=size, grid=256, seed=seed ^ 0xA5A5)
    n = (n - 0.5) * 0.25  # ±25% contrast
    arr = (np.asarray(img).astype(np.float32) / 255.0)
    arr = np.clip(arr * (1.0 + n[..., None]), 0, 1)
    arr = add_subtle_color(arr, seed=seed, sat=0.15)  # <-- optional colorization step
    img = Image.fromarray((arr * 255).astype(np.uint8), mode="RGB")

    # Gentle blur to remove aliasing spikes
    img = img.filter(ImageFilter.GaussianBlur(radius=0.6))

    img.save(path, format="PNG", optimize=True)
    print(f"Saved {path} (seed={seed})")

if __name__ == "__main__":
    make_floor("floor_texture.png", size=2048, seed=42)
