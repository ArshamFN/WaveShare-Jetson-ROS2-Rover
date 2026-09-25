#!/usr/bin/env python3
"""One-shot /scan diagnostic: where is the wall, and what is occluding us?"""
import math, sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Dump(Node):
    def __init__(self):
        super().__init__('scan_dump')
        self.create_subscription(LaserScan, '/scan', self._cb, 10)
        self.done = False

    def _cb(self, msg):
        if self.done:
            return
        self.done = True

        n = len(msg.ranges)
        r = np.asarray(msg.ranges, dtype=float)
        ang = np.degrees(msg.angle_min + np.arange(n) * msg.angle_increment)
        ang = (ang + 180.0) % 360.0 - 180.0

        finite = np.isfinite(r) & (r > 0.0)
        print(f'\nbeams={n}  finite={finite.sum()}  '
              f'range_min={msg.range_min}  range_max={msg.range_max}')

        # Every 10 degrees: min range in that bin
        print('\n--- range by bearing (10 deg bins, nearest return) ---')
        for lo in range(-180, 180, 10):
            m = (ang >= lo) & (ang < lo + 10) & finite
            if m.sum() == 0:
                print(f'  {lo:+5d}..{lo+10:+5d} :  no returns')
            else:
                print(f'  {lo:+5d}..{lo+10:+5d} :  min={r[m].min():6.3f}  '
                      f'max={r[m].max():6.3f}  n={m.sum()}')

        # Where are the ~3 m returns?
        target = (r > 2.7) & (r < 3.3) & finite
        print(f'\n--- bearings reading 2.7-3.3 m: {target.sum()} beams ---')
        if target.sum():
            ta = ang[target]
            print(f'  bearing span: {ta.min():+.1f} to {ta.max():+.1f} deg')
            # contiguous-ish clusters
            order = np.argsort(ta)
            ta_s = ta[order]
            breaks = np.where(np.diff(ta_s) > 5.0)[0]
            starts = np.concatenate(([0], breaks + 1))
            ends = np.concatenate((breaks, [len(ta_s) - 1]))
            for s, e in zip(starts, ends):
                print(f'    cluster {ta_s[s]:+7.1f} .. {ta_s[e]:+7.1f} deg  '
                      f'({e - s + 1} beams)')
        else:
            print('  NONE — the wall is not visible at 3 m in this scan.')

        # Near-field occlusion map
        near = finite & (r < 0.25)
        print(f'\n--- returns under 0.25 m (self-occlusion): {near.sum()} beams ---')
        if near.sum():
            na = ang[near]
            order = np.argsort(na)
            na_s = na[order]
            breaks = np.where(np.diff(na_s) > 5.0)[0]
            starts = np.concatenate(([0], breaks + 1))
            ends = np.concatenate((breaks, [len(na_s) - 1]))
            for s, e in zip(starts, ends):
                print(f'    blocked {na_s[s]:+7.1f} .. {na_s[e]:+7.1f} deg')

        print(f'\n  Longest return: {r[finite].max():.3f} m at '
              f'{ang[finite][np.argmax(r[finite])]:+.1f} deg\n')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = Dump()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    sys.exit(0)


if __name__ == '__main__':
    main()
