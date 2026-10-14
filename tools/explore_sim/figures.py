"""Draw the simulator figures used in the Session 016 log.

    python3 figures.py [OUT_DIR]

Each panel is one simulated run: blue is the path driven, red the frontiers
left unexplored at the end, orange free space the robot never saw, and the
green circle the start. OUT_DIR defaults to images/testing/session-016.
"""
import os
import sys

from PIL import Image, ImageDraw, ImageFont

from compare import MAPS, MAPS_DIR, POLICIES, starts
from sim import FREE, Sim, load_gt
from viz import render

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, '..', '..', 'images', 'testing', 'session-016')
LABELS = {'A': 'A explore_lite', 'B': 'B sector rule', 'C': 'C path planner',
          'D': 'D path planner + route', 'rover': 'rover_explorer'}
# The panels use a few flat colours (see viz.py); greys cover the anti-aliased text.
# A fixed palette keeps those colours exact and the files small.
_COLOURS = [(255, 255, 255), (235, 235, 235), (205, 205, 205), (30, 30, 30), (255, 214, 170),
            (220, 40, 40), (150, 60, 200), (30, 110, 230), (0, 150, 0)]
_COLOURS += [(v, v, v) for v in range(0, 256, 17)]
PALETTE = Image.new('P', (1, 1))
PALETTE.putpalette([c for rgb in _COLOURS for c in rgb] + [0] * (768 - 3 * len(_COLOURS)))
MAP_NAMES = {'s015': 's015', 'oct7': 'Oct 7'}
SCALE, HEADER, GAP = 2, 38, 8

FIGURES = {
    'session-016-sim-explore-lite-sector-path.png':
        [[('s015', 0, 'A'), ('s015', 0, 'B'), ('s015', 0, 'C')],
         [('s015', 7, 'A'), ('s015', 7, 'B'), ('s015', 7, 'C')]],
    'session-016-sim-oct7-real-start.png':
        [[('oct7', 0, 'A'), ('oct7', 0, 'B'), ('oct7', 0, 'D')]],
    'session-016-sim-explore-lite-vs-rover-explorer.png':
        [[('s015', 0, 'A'), ('s015', 0, 'rover')],
         [('s015', 7, 'A'), ('s015', 7, 'rover')],
         [('oct7', 0, 'A'), ('oct7', 0, 'rover')]],
}


def font(size):
    for name in ('DejaVuSans.ttf', '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


def panel(map_name, si, policy, cache={}):
    key = (map_name, si, policy)
    if key in cache:
        return cache[key]
    pgm, origin = MAPS[map_name]
    gt = load_gt(os.path.join(MAPS_DIR, pgm))
    st, main = starts(gt, origin, 9)
    (r0, c0), heading = st[si]
    sim = Sim(gt, (r0, c0), heading, POLICIES[policy]())
    res = sim.run()
    seen = 100.0 * float(((sim.belief == FREE) & main).sum()) / float(main.sum())
    tmp = os.path.join(OUT, '_panel.png')
    render(sim, tmp, scale=SCALE)
    img = Image.open(tmp).convert('RGB')
    os.remove(tmp)
    where = 'real start pose' if si == 0 else f'start {si}'
    plural = '' if res.failures == 1 else 's'
    lines = (f'{LABELS[policy]}  ({MAP_NAMES[map_name]} map, {where})',
             f'{res.distance:.1f} m driven, {res.time:.0f} s, {res.failures} failed goal{plural}, {seen:.0f}% seen')
    print(lines[0], '|', lines[1], flush=True)
    cache[key] = (img, lines)
    return cache[key]


def main():
    os.makedirs(OUT, exist_ok=True)
    f1, f2 = font(13), font(11)
    for name, grid in FIGURES.items():
        cells = [[panel(*spec) for spec in row] for row in grid]
        w = max(img.width for row in cells for img, _ in row)
        h = max(img.height for row in cells for img, _ in row)
        cols, rows = len(grid[0]), len(grid)
        out = Image.new('RGB', (cols * w + (cols - 1) * GAP, rows * (h + HEADER) + (rows - 1) * GAP), 'white')
        d = ImageDraw.Draw(out)
        for ri, row in enumerate(cells):
            for ci, (img, lines) in enumerate(row):
                x, y = ci * (w + GAP), ri * (h + HEADER + GAP)
                d.text((x + 5, y + 3), lines[0], fill=(0, 0, 0), font=f1)
                d.text((x + 5, y + 21), lines[1], fill=(70, 70, 70), font=f2)
                out.paste(img, (x, y + HEADER))
        path = os.path.join(OUT, name)
        out.quantize(palette=PALETTE, dither=Image.Dither.NONE).save(path, optimize=True)
        print('wrote', os.path.relpath(path, HERE), flush=True)


if __name__ == '__main__':
    main()
