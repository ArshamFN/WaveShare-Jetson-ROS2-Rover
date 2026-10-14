"""Run exploration policies from the same start poses on a saved map.

    python3 compare.py MAP POLICY [POLICY ...] [--starts N] [--jobs J] [--images DIR]

MAP is one of the names in MAPS. POLICY is one of the names in POLICIES. Every
policy runs from the real start pose plus N - 1 random poses drawn with a fixed
seed, so all policies share the same starts. Each policy's rows are written to
results/<MAP>_<POLICY>_n<N>.json and a one-line summary is printed.
"""
import argparse
import json
import math
import os
from multiprocessing import Pool

import numpy as np
from scipy import ndimage

import module_policy as MP
import policies as P
from sim import FREE, RES, Sim, load_gt

HERE = os.path.dirname(os.path.abspath(__file__))
MAPS_DIR = os.path.join(HERE, '..', '..', 'maps')

# map name -> (map file, cell (row, col) of the map frame origin, where the real run started)
MAPS = {
    's015': ('s015_full_house_map.pgm', (108, 32)),
    'oct7': ('explore_20261007-122528_final.pgm', (101, 27)),
}

POLICIES = {
    'A': lambda: P.ExploreLite(),
    'A2': lambda: P.ExploreLite(gain=0.0, freq=0.05, name='A2 explore_lite tuned'),
    'B': lambda: P.Sector(),
    'C': lambda: P.PathFrontier(),
    'D': lambda: P.PathFrontierTour(keep_ratio=1.0),
    'rover': lambda: MP.Module(),
}

_job = {}


def starts(gt, origin_cell, n, seed=7):
    """The real start pose, then n random poses with 0.25 m of clearance in the
    largest free region. The fixed seed gives every policy the same starts."""
    clear = ndimage.distance_transform_edt(~gt) * RES
    lab, _ = ndimage.label(~gt)
    main = lab == np.bincount(lab[lab > 0]).argmax()
    ok = main & (clear > 0.25)
    rr, cc = np.nonzero(ok)
    rng = np.random.default_rng(seed)
    out = [(origin_cell, 0.0)]
    for i in rng.choice(len(rr), n, replace=False):
        out.append(((int(rr[i]), int(cc[i])), float(rng.uniform(-math.pi, math.pi))))
    return out, main


def _init(map_name, n_starts, policy, images):
    pgm, origin = MAPS[map_name]
    gt = load_gt(os.path.join(MAPS_DIR, pgm))
    st, main = starts(gt, origin, n_starts - 1)
    _job.update(gt=gt, st=st, main=main, policy=policy, map=map_name, images=images)


def _one(si):
    gt, st, main = _job['gt'], _job['st'], _job['main']
    (r0, c0), heading = st[si]
    pol = POLICIES[_job['policy']]()
    sim = Sim(gt, (r0, c0), heading, pol)
    res = sim.run()
    cov = 100.0 * float(((sim.belief == FREE) & main).sum()) / float(main.sum())
    row = dict(start=si, cell=[r0, c0], heading=round(heading, 4),
               distance=round(res.distance, 3), time=round(res.time, 2),
               coverage=round(cov, 2), goals=res.goals, switches=res.switches,
               failures=res.failures, end=res.end_reason)
    stats = getattr(getattr(pol, 'pl', None), 'stats', None)
    if stats:
        row['planner'] = dict(stats)
    if _job['images']:
        from viz import render
        render(sim, os.path.join(_job['images'], f"{_job['map']}_{_job['policy']}_{si}.png"),
               excluded=getattr(pol, 'excluded', None),
               title=f"{_job['policy']} start {si}: {res.distance:.1f} m, {cov:.1f}%")
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map', choices=sorted(MAPS))
    ap.add_argument('policies', nargs='+', choices=sorted(POLICIES))
    ap.add_argument('--starts', type=int, default=10)
    ap.add_argument('--jobs', type=int, default=2)
    ap.add_argument('--images', default='', help='folder for one image per run (optional)')
    a = ap.parse_args()
    out_dir = os.path.join(HERE, 'results')
    os.makedirs(out_dir, exist_ok=True)
    if a.images:
        os.makedirs(a.images, exist_ok=True)
    for name in a.policies:
        with Pool(a.jobs, initializer=_init, initargs=(a.map, a.starts, name, a.images)) as pool:
            rows = pool.map(_one, range(a.starts))
        out = os.path.join(out_dir, f'{a.map}_{name}_n{a.starts}.json')
        with open(out, 'w') as f:
            json.dump(rows, f, indent=1)
        n = len(rows)
        print(f"{a.map} {name:6s} distance {sum(r['distance'] for r in rows) / n:6.2f} m"
              f"  time {sum(r['time'] for r in rows) / n:6.1f} s"
              f"  coverage min {min(r['coverage'] for r in rows):5.1f}%"
              f"  failed goals {sum(r['failures'] for r in rows) / n:.2f}"
              f"  -> {os.path.relpath(out, HERE)}", flush=True)


if __name__ == '__main__':
    main()
