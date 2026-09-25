#!/usr/bin/env python3
"""
measure_max_speed.py — LiDAR wall-ranging speed calibration (3 m runway)
========================================================================
Measures true forward velocity by ranging against a flat perpendicular
wall, using the LiDAR as a factory-calibrated ruler.

Rev 2: narrowed the beam fan (a +/-30 deg fan spans 3.46 m of wall at 3 m
range, wide enough to catch side walls and furniture and fail the flatness
check) and added diagnostics so rejected scans report WHY rather than
vanishing silently.
"""

import math
import signal
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# ── Configuration ───────────────────────────────────────────────────────────

# Half-width of the beam fan used for the wall fit, in radians.
# At range R the fan spans 2*R*tan(FAN_HALF_ANGLE) metres of wall.
# 10 deg -> 1.06 m at 3 m. Keep this comfortably inside the flat surface.
FAN_HALF_ANGLE = math.radians(10.0)

# Direction of travel expressed in the laser frame, radians.
# 0.0 assumes the LiDAR's 0-angle beam points forward along +x.
FORWARD_ANGLE_OFFSET = math.pi   # laser 0-bearing faces the rover REAR (S013)

# Maximum RMS residual (m) of the wall line fit. Above this the geometry
# is not a flat wall and the sample is rejected.
RESIDUAL_MAX = 0.05

MIN_VALID_RANGE = 0.15
MAX_VALID_RANGE = 12.0
MIN_POINTS = 6

VELOCITY_WINDOW = 3
STOP_DISTANCE = 0.45
EXPECTED_RAMP_RATE = 0.8
DRIFT_WARN_DEG = 10.0

CSV_PATH = '/tmp/speed_profile.csv'

# ────────────────────────────────────────────────────────────────────────────


class SpeedMeasureNode(Node):

    def __init__(self):
        super().__init__('measure_max_speed')

        self._samples = []
        self._t0 = None
        self._guard_fired = False
        self._diag_done = False
        self._reject_count = 0

        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self._stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('=' * 62)
        self.get_logger().info('  LiDAR wall-ranging speed calibration (rev 2)')
        self.get_logger().info('=' * 62)
        self.get_logger().info(
            f'  Fan +/-{math.degrees(FAN_HALF_ANGLE):.0f} deg | '
            f'window {VELOCITY_WINDOW} | guard {STOP_DISTANCE:.2f} m')
        self.get_logger().info('  Waiting for /scan...')
        self.get_logger().info('')

    # ── Startup diagnostic ──────────────────────────────────────────────────

    def _diagnose(self, msg: LaserScan):
        """One-shot dump of scan geometry, to locate the wall in the frame."""
        n = len(msg.ranges)
        r = np.asarray(msg.ranges, dtype=float)
        self.get_logger().info('--- SCAN GEOMETRY ---')
        self.get_logger().info(
            f'  beams={n}  angle_min={math.degrees(msg.angle_min):.1f} deg  '
            f'angle_max={math.degrees(msg.angle_max):.1f} deg  '
            f'inc={math.degrees(msg.angle_increment):.3f} deg')
        self.get_logger().info(
            f'  range_min={msg.range_min:.2f}  range_max={msg.range_max:.2f}')
        self.get_logger().info('  Range at selected bearings:')
        for deg in (-180, -135, -90, -45, -20, 0, 20, 45, 90, 135):
            ang = math.radians(deg)
            i = int(round((ang - msg.angle_min) / msg.angle_increment))
            if 0 <= i < n and np.isfinite(r[i]):
                self.get_logger().info(f'    {deg:+5d} deg : {r[i]:6.3f} m')
            else:
                self.get_logger().info(f'    {deg:+5d} deg :   ---')
        self.get_logger().info('---------------------')
        self.get_logger().info(
            '  The bearing reading ~your tape distance is the wall. '
            'If it is not 0 deg, set FORWARD_ANGLE_OFFSET to it.')
        self.get_logger().info('')

    # ── Scan handling ───────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        if not self._diag_done:
            self._diag_done = True
            self._diagnose(msg)

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = t
        t -= self._t0

        pts = self._extract_wall_points(msg)
        if pts is None or len(pts) < MIN_POINTS:
            self._reject('too few valid beams in fan '
                         f'({0 if pts is None else len(pts)} < {MIN_POINTS})')
            return

        perp, drift, resid = self._fit_wall(pts)
        if perp is None:
            self._reject(f'fit residual {resid:.3f} m > {RESIDUAL_MAX:.2f} m '
                         '(fan is not seeing a flat wall)')
            return

        if perp < STOP_DISTANCE:
            self._stop_pub.publish(Twist())
            if not self._guard_fired:
                self._guard_fired = True
                self.get_logger().warn(
                    f'WALL GUARD at {perp:.2f} m — publishing zero cmd_vel.')

        self._samples.append((t, perp, drift, len(pts)))
        self._report(t, perp, drift, len(pts), resid)

    def _reject(self, reason):
        self._reject_count += 1
        if self._reject_count % 10 == 1:
            self.get_logger().warn(
                f'REJECTED ({self._reject_count} so far): {reason}')

    def _extract_wall_points(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return None

        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        ranges = np.asarray(msg.ranges, dtype=float)

        rel = np.arctan2(np.sin(angles - FORWARD_ANGLE_OFFSET),
                         np.cos(angles - FORWARD_ANGLE_OFFSET))

        keep = (
            (np.abs(rel) <= FAN_HALF_ANGLE)
            & np.isfinite(ranges)
            & (ranges >= MIN_VALID_RANGE)
            & (ranges <= MAX_VALID_RANGE)
        )
        if not np.any(keep):
            return None

        r = ranges[keep]
        a = rel[keep]
        return np.column_stack((r * np.cos(a), r * np.sin(a)))

    @staticmethod
    def _fit_wall(pts):
        """Fit x = m*y + c; return perpendicular distance, tilt, residual."""
        x = pts[:, 0]
        y = pts[:, 1]
        try:
            m, c = np.polyfit(y, x, 1)
        except Exception:
            return None, None, float('nan')

        tilt = math.atan(m)
        perp = c * math.cos(tilt)
        resid = float(np.std(x - (m * y + c)))

        if resid > RESIDUAL_MAX:
            return None, None, resid
        return perp, tilt, resid

    # ── Reporting ───────────────────────────────────────────────────────────

    def _report(self, t, perp, drift, npts, resid):
        vel = self._velocity_at(len(self._samples) - 1)
        drift_deg = math.degrees(drift)
        flag = '  <-- DRIFT' if abs(drift_deg) > DRIFT_WARN_DEG else ''
        vel_s = f'{vel:6.3f} m/s' if vel is not None else '   ---   '
        self.get_logger().info(
            f't={t:6.2f}s  dist={perp:6.3f} m  v={vel_s}  '
            f'drift={drift_deg:+6.1f} deg  n={npts}  res={resid:.3f}{flag}')

    def _velocity_at(self, i):
        if i < VELOCITY_WINDOW:
            return None
        w = self._samples[i - VELOCITY_WINDOW + 1:i + 1]
        ts = np.array([s[0] for s in w])
        ds = np.array([s[1] for s in w])
        if ts[-1] - ts[0] <= 0:
            return None
        return -float(np.polyfit(ts, ds, 1)[0])

    # ── Summary ─────────────────────────────────────────────────────────────

    def summarise(self):
        print(f'\nRejected scans: {self._reject_count}')
        if len(self._samples) < VELOCITY_WINDOW + 3:
            print('Not enough accepted samples to summarise.')
            return

        ts = np.array([s[0] for s in self._samples])
        ds = np.array([s[1] for s in self._samples])
        dr = np.array([s[2] for s in self._samples])

        vels = [(self._samples[i][0], self._velocity_at(i))
                for i in range(len(self._samples))
                if self._velocity_at(i) is not None]
        if not vels:
            print('No velocity samples.')
            return

        vt = np.array([t for t, _ in vels])
        vv = np.array([v for _, v in vels])

        with open(CSV_PATH, 'w') as f:
            f.write('t_s,distance_m,drift_deg,velocity_mps\n')
            for i, (t, d, g, _) in enumerate(self._samples):
                v = self._velocity_at(i)
                f.write(f'{t:.3f},{d:.4f},{math.degrees(g):.2f},'
                        f'{"" if v is None else f"{v:.4f}"}\n')

        peak = float(np.max(vv))
        plateau = vv[vv >= 0.95 * peak]

        ramp_mask = (vv > 0.2 * peak) & (vv < 0.8 * peak)
        accel = None
        if np.count_nonzero(ramp_mask) >= 3:
            rt, rv = vt[ramp_mask], vv[ramp_mask]
            if rt[-1] > rt[0]:
                accel = float(np.polyfit(rt, rv, 1)[0])

        print('\n' + '=' * 62)
        print('  SUMMARY')
        print('=' * 62)
        print(f'  Accepted samples : {len(self._samples)}')
        print(f'  Distance covered : {ds[0] - ds[-1]:.3f} m '
              f'({ds[0]:.3f} -> {ds[-1]:.3f})')
        print(f'  Duration         : {ts[-1] - ts[0]:.2f} s')
        print(f'  Drift angle      : mean {math.degrees(dr.mean()):+.1f} deg, '
              f'max |{math.degrees(np.abs(dr).max()):.1f}| deg')
        print('')
        print(f'  Peak velocity    : {peak:.3f} m/s')
        print(f'  Plateau mean     : {plateau.mean():.3f} m/s '
              f'(n={len(plateau)}, sd={plateau.std():.3f})')
        if accel is not None:
            print(f'  Measured accel   : {accel:.3f} m/s^2 '
                  f'(expected {EXPECTED_RAMP_RATE:.2f})')
        print('')
        print(f'  ==> MAX_WHEEL_SPEED = {plateau.mean():.3f}  # m/s at full duty')
        print('')
        print(f'  CSV written to   : {CSV_PATH}')

        if len(plateau) < 3:
            print('\n  WARNING: fewer than 3 plateau samples — the rover may')
            print('  not have reached steady state. Treat as a lower bound.')
        if accel is not None and abs(accel - EXPECTED_RAMP_RATE) > 0.3:
            print('\n  WARNING: measured acceleration disagrees with the')
            print('  command ramp rate. Investigate before trusting the number.')
        if math.degrees(np.abs(dr).max()) > DRIFT_WARN_DEG:
            print('\n  WARNING: drift exceeded threshold — the fan may have')
            print('  wandered off the flat wall. Check the CSV.')
        print('=' * 62)


def main():
    rclpy.init()
    node = SpeedMeasureNode()

    def _sigint(_sig, _frm):
        node.summarise()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
