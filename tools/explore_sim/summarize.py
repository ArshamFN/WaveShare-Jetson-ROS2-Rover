"""Print the comparison tables from the files compare.py wrote to results/.

    python3 summarize.py [--starts N]

Starts where explore_lite (A) ran into the one-hour limit are left out of every
policy's mean on that map, so all policies are compared on the same starts.
"""
import argparse
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ORDER = ['A', 'A2', 'B', 'C', 'D', 'rover']
NAMES = {'A': 'A explore_lite', 'A2': 'A2 parameters only', 'B': 'B sector rule',
         'C': 'C path planner', 'D': 'D path planner + route', 'rover': 'rover_explorer'}


def load(map_name, policy, n):
    p = os.path.join(HERE, 'results', f'{map_name}_{policy}_n{n}.json')
    if not os.path.exists(p):
        return None
    with open(p) as f:
        return json.load(f)


def mean(rows, key, keep):
    return sum(rows[i][key] for i in keep) / len(keep)


def pct(a, b):
    return f'{100.0 * (a / b - 1.0):+.0f}%'


def table(map_name, n):
    runs = {k: load(map_name, k, n) for k in ORDER}
    runs = {k: v for k, v in runs.items() if v}
    if not runs:
        return
    keep = list(range(n))
    if 'A' in runs:
        keep = [i for i in keep if runs['A'][i]['end'] != 'timeout']
    left_out = n - len(keep)
    print(f'\n{map_name}, {len(keep)} starts' + (f' ({left_out} left out: explore_lite hit the time limit)' if left_out else ''))
    print('| Policy | Distance | Time | Failed goals | Worst-start coverage |')
    print('|---|---|---|---|---|')
    base = runs.get('A')
    for k, rows in runs.items():
        d, t = mean(rows, 'distance', keep), mean(rows, 'time', keep)
        fail = mean(rows, 'failures', keep)
        cov = min(rows[i]['coverage'] for i in keep)
        if base is None or k == 'A':
            ds, ts = f'{d:.1f} m', f'{t:.0f} s'
        else:
            bd, bt = mean(base, 'distance', keep), mean(base, 'time', keep)
            worst = max(rows[i]['distance'] / base[i]['distance'] - 1.0 for i in keep)
            ds = f'{pct(d, bd)} ({d:.1f} m, worst start {100.0 * worst:+.0f}%)'
            ts = pct(t, bt)
        print(f'| {NAMES[k]} | {ds} | {ts} | {fail:.1f} | {cov:.1f}% |')


def versus(map_name, n):
    d_rows, r_rows = load(map_name, 'D', n), load(map_name, 'rover', n)
    if d_rows and r_rows:
        d = sum(r['distance'] for r in d_rows) / n
        r = sum(r['distance'] for r in r_rows) / n
        print(f'{map_name}, {n} starts: rover_explorer {r:.1f} m against D {d:.1f} m ({pct(r, d)})')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--starts', type=int, default=10)
    a = ap.parse_args()
    for m in ('s015', 'oct7'):
        table(m, a.starts)
    print()
    for n in (a.starts, 30):
        for m in ('s015', 'oct7'):
            versus(m, n)


if __name__ == '__main__':
    main()
