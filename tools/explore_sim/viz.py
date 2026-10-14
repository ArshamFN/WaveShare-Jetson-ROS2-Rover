"""Render a finished run: ground truth, what the robot mapped, its path and the frontiers left."""
import numpy as np
from PIL import Image, ImageDraw

from sim import FREE, OCC, UNKNOWN, find_frontiers


def render(sim, out, scale=4, excluded=None, title=None):
    gt = sim.gt
    H, W = gt.shape
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[~gt] = (235, 235, 235)        # true free space not yet seen
    img[gt] = (205, 205, 205)
    b = sim.belief
    img[b == FREE] = (255, 255, 255)
    img[b == OCC] = (30, 30, 30)
    seen_free_missing = (~gt) & (b == UNKNOWN)
    img[seen_free_missing] = (255, 214, 170)   # free in reality, never observed
    for f in find_frontiers(b, 10):
        col = (220, 40, 40)
        if excluded is not None and excluded[f.cells[:, 0], f.cells[:, 1]].mean() > 0.5:
            col = (150, 60, 200)
        img[f.cells[:, 0], f.cells[:, 1]] = col
    im = Image.fromarray(img).resize((W * scale, H * scale), Image.NEAREST)
    d = ImageDraw.Draw(im)
    p = [(c * scale + scale / 2, r * scale + scale / 2) for r, c in sim.st.path]
    if len(p) > 1:
        d.line(p, fill=(30, 110, 230), width=2)
    r0, c0 = sim.start
    d.ellipse([c0 * scale - 6, r0 * scale - 6, c0 * scale + 6, r0 * scale + 6], outline=(0, 150, 0), width=3)
    if title:
        d.text((6, 6), title, fill=(0, 0, 0))
    im.save(out)
