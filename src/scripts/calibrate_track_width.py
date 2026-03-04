#!/usr/bin/env python3
"""
TRACK_WIDTH Calibration Script
================================
Uses the onboard gyroscope (gz) to perform a precise 90° clockwise turn,
then computes the exact TRACK_WIDTH from odometry deltas.

Phases:
  0 — Bias calibration   : robot sits still, we measure gz noise floor
  2 — Scale calibration  : you rotate robot 360° CW by hand, press Enter when done
  3 — 90° motor turn     : gyro-controlled CW turn, record odom before/after
  4 — Report             : print TRACK_WIDTH and all findings

Run with:
  python3 ~/calibrate_track_width.py

Requires rover_driver_node running (for /imu/gz and /odom topics).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time
import threading


# ── Tunable parameters ────────────────────────────────────────────────────────
BIAS_DURATION       = 3.0   # seconds to collect bias samples at rest
TURN_SPEED          = 0.4   # rad/s command for the 90 degree motor turn
TARGET_ANGLE_RAD    = math.pi / 2   # 90 degrees

# Update this to match TRACK_WIDTH currently in rover_driver_node.py
TRACK_WIDTH_CURRENT = 0.08
# ─────────────────────────────────────────────────────────────────────────────


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('track_width_calibrator')

        self.gz_sub = self.create_subscription(
            Float32, '/imu/gz', self._gz_callback, 50)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.gz_raw       = None
        self.gz_bias      = None
        self.gz_scale     = None
        self.bias_samples = []

        self.scale_integral  = 0.0
        self.scale_last_time = None

        self.turn_integral   = 0.0
        self.turn_last_time  = None

        self.odom_theta = None
        self.odom_x     = None
        self.odom_y     = None

        self.snap_before   = None
        self.snap_after    = None
        self.enter_pressed = False

        self.state      = 'BIAS'
        self.bias_start = time.time()

        self.get_logger().info('=' * 55)
        self.get_logger().info('  TRACK_WIDTH Calibration Script')
        self.get_logger().info('=' * 55)
        self.get_logger().info('PHASE 0: Bias calibration — keep robot STILL...')

        self.timer = self.create_timer(0.05, self._loop)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _gz_callback(self, msg):
        self.gz_raw = msg.data

    def _odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_theta = math.atan2(siny, cosy)
        self.odom_x     = msg.pose.pose.position.x
        self.odom_y     = msg.pose.pose.position.y

    def _wait_for_enter(self):
        input()
        self.enter_pressed = True

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _turn_cw(self):
        twist = Twist()
        twist.angular.z = -TURN_SPEED
        self.cmd_pub.publish(twist)

    def _gz_corrected(self):
        if self.gz_raw is None or self.gz_bias is None:
            return None
        return self.gz_raw - self.gz_bias

    def _start_enter_thread(self):
        self.enter_pressed = False
        t = threading.Thread(target=self._wait_for_enter, daemon=True)
        t.start()

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _loop(self):
        now = time.time()

        # PHASE 0 — Bias collection
        if self.state == 'BIAS':
            if self.gz_raw is not None:
                self.bias_samples.append(self.gz_raw)

            if now - self.bias_start >= BIAS_DURATION:
                if len(self.bias_samples) < 10:
                    self.get_logger().error(
                        'Not enough gz samples. Is rover_driver_node running?')
                    self.state = 'DONE'
                    return

                self.gz_bias = sum(self.bias_samples) / len(self.bias_samples)
                noise = max(self.bias_samples) - min(self.bias_samples)

                self.get_logger().info(f'  Samples   : {len(self.bias_samples)}')
                self.get_logger().info(f'  Bias      : {self.gz_bias:.3f} counts')
                self.get_logger().info(f'  Noise p-p : {noise:.3f} counts')
                self.get_logger().info('')
                self.get_logger().info('PHASE 2: Scale calibration.')
                self.get_logger().info('  Rotate robot ONE full turn (360 deg) CW by hand.')
                self.get_logger().info('  >>> Press ENTER when DONE rotating. <<<')

                self.state           = 'SCALE_CAL'
                self.scale_last_time = now
                self.scale_integral  = 0.0
                self._start_enter_thread()

        # PHASE 2 — Scale calibration via hand rotation
        elif self.state == 'SCALE_CAL':
            gz_c = self._gz_corrected()
            if gz_c is None:
                return

            dt = now - self.scale_last_time
            self.scale_last_time = now
            self.scale_integral += gz_c * dt

            if self.enter_pressed:
                if abs(self.scale_integral) < 10.0:
                    self.get_logger().warn(
                        f'Integral too small ({self.scale_integral:.2f} count.s). '
                        'Rotate a full 360 deg then press Enter.')
                    self.scale_integral  = 0.0
                    self.scale_last_time = now
                    self._start_enter_thread()
                    return

                # One full CW turn = -2pi radians
                self.gz_scale = (-2.0 * math.pi) / self.scale_integral

                self.get_logger().info('')
                self.get_logger().info(f'  Raw integral : {self.scale_integral:.3f} count.s')
                self.get_logger().info(f'  Scale factor : {self.gz_scale:.6f} rad / count.s')
                self.get_logger().info('')
                self.get_logger().info('PHASE 3: 90 deg clockwise motor turn.')
                self.get_logger().info('  Place robot on floor, clear space to the RIGHT.')
                self.get_logger().info('  >>> Press ENTER to start the turn. <<<')

                self.state = 'WAIT_TURN'
                self._start_enter_thread()

        # Wait for Enter + odom before starting motor turn
        elif self.state == 'WAIT_TURN':
            if self.enter_pressed:
                if self.odom_theta is None:
                    # Odom not yet received — keep waiting, log once
                    if not hasattr(self, '_odom_wait_logged'):
                        self.get_logger().info('  Waiting for /odom...')
                        self._odom_wait_logged = True
                    return

                self.snap_before    = (self.odom_theta, self.odom_x, self.odom_y)
                self.turn_integral  = 0.0
                self.turn_last_time = now
                self.get_logger().info('  GO!')
                self.state = 'TURNING'

        # PHASE 3 — Gyro-controlled 90 deg CW turn
        elif self.state == 'TURNING':
            gz_c = self._gz_corrected()
            if gz_c is None:
                return

            dt = now - self.turn_last_time
            self.turn_last_time = now

            self.turn_integral += gz_c * self.gz_scale * dt

            if self.turn_integral <= -TARGET_ANGLE_RAD:
                self._stop()
                self.snap_after = (self.odom_theta, self.odom_x, self.odom_y)
                self.get_logger().info(
                    f'  Done! Gyro measured: '
                    f'{math.degrees(abs(self.turn_integral)):.2f} deg')
                self.state = 'REPORT'
            else:
                self._turn_cw()
                pct = abs(self.turn_integral) / TARGET_ANGLE_RAD * 100
                if int(now * 2) != int((now - dt) * 2):
                    self.get_logger().info(
                        f'  Turning... {pct:.0f}%  '
                        f'({math.degrees(abs(self.turn_integral)):.1f} deg)')

        # PHASE 4 — Report
        elif self.state == 'REPORT':
            self.state = 'DONE'

            theta_before = self.snap_before[0]
            theta_after  = self.snap_after[0]

            odom_delta = theta_after - theta_before
            while odom_delta >  math.pi: odom_delta -= 2 * math.pi
            while odom_delta < -math.pi: odom_delta += 2 * math.pi

            actual = -TARGET_ANGLE_RAD  # gyro truth: -90 deg CW

            if abs(odom_delta) < 0.001:
                self.get_logger().warn(
                    'Odometry shows near-zero rotation. '
                    'Check rover_driver_node is running.')
                return

            tw = TRACK_WIDTH_CURRENT * (actual / odom_delta)

            self.get_logger().info('')
            self.get_logger().info('=' * 55)
            self.get_logger().info('  CALIBRATION RESULTS')
            self.get_logger().info('=' * 55)
            self.get_logger().info(f'  Gyro bias           : {self.gz_bias:.3f} counts')
            self.get_logger().info(f'  Gyro scale          : {self.gz_scale:.6f} rad/count.s')
            self.get_logger().info(
                f'  Gyro turn measured  : '
                f'{math.degrees(abs(self.turn_integral)):.2f} deg')
            self.get_logger().info(
                f'  Odom turn reported  : {math.degrees(odom_delta):.2f} deg')
            self.get_logger().info(
                f'  Current TRACK_WIDTH : {TRACK_WIDTH_CURRENT:.4f} m')
            self.get_logger().info('')
            self.get_logger().info(f'  >>> TRACK_WIDTH = {tw:.4f} m <<<')
            self.get_logger().info('')
            self.get_logger().info(
                f'  Set TRACK_WIDTH = {tw:.4f} in rover_driver_node.py')
            self.get_logger().info('=' * 55)


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
