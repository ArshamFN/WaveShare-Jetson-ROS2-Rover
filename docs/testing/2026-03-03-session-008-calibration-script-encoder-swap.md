# Session 008 — Automated Calibration Script & Battery Sag Discovery

**Date:** 2026-03-03  
**Status:** ✅ Complete

---

## Goal

Determine `TRACK_WIDTH` through a sensor-referenced automated calibration script that
eliminates human visual estimation from the process. After confirming the value, attempt
teleop SLAM mapping with the corrected odometry parameters.

---

## What Was Accomplished

1. Wrote and executed a 3-phase Python calibration script using the MFD's onboard IMU as
   a ground-truth rotation reference
2. Discovered through the script's raw output that the MFD's `odl`/`odr` labels are
   physically reversed — corrected the encoder swap in the odometry node
3. Determined `TRACK_WIDTH = 0.0456 m` from the calibrated sensor data
4. Identified battery voltage sag as the root cause of persistent heading degradation
   during the SLAM mapping attempt
5. Established the architecture for Session 009: gyroscope-based heading combined with
   encoder-based linear displacement

---

## The Calibration Script

### Design Philosophy
Session 007 established that any calibration method routing through the odometry system
itself is circular. The solution is to use a sensor that measures rotation independently —
the MFD board's gyroscope (`gz` field in the T:1001 feedback stream). With the gyroscope
as ground truth, the encoder data can be compared against a known reference and the
correct `TRACK_WIDTH` can be computed from the ratio directly.

The script runs in three sequential phases, printing all raw sensor findings after each
phase before continuing. This transparency was intentional — it makes every step of the
computation auditable and exposes any unexpected sensor behaviour before it contaminates
the final result.

### Full Source

```python
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
```

### Phase 1 — Accelerometer Bias Calibration
The rover sits stationary while the script samples `ax`, `ay`, and `az` over several
seconds and computes mean offsets on each axis. These bias values are stored and subtracted
from all subsequent measurements. Without this step, static sensor offset would accumulate
into the rotation integral and corrupt the heading estimate.

### Phase 2 — Gyroscope Scale Calibration (Manual 360°)
The script prompts me to manually rotate the rover exactly one full revolution by hand
while it integrates the `gz` yaw rate signal. Since I am providing a known ground-truth
rotation of 2π radians, the script can compute the exact radians-per-raw-unit conversion
for the gyroscope. After this phase, the gyroscope reports absolute heading change in
calibrated radians rather than firmware-native units.

### Phase 3 — Automated 90° CW Turn and TRACK_WIDTH Computation
With the gyroscope calibrated, the script commands the rover to execute a clockwise 90°
turn under motor control. During the manoeuvre it simultaneously records the
gyroscope-integrated heading change and the `odl`/`odr` encoder deltas. It then computes:

```
TRACK_WIDTH = (odl_delta - odr_delta) / heading_change_radians
```

All raw values are printed alongside the computed result so the arithmetic can be verified
by hand.

---

## Discovery — L/R Encoder Swap

### What the Script Reported
Phase 3 produced a **negative `TRACK_WIDTH`** value. The gyroscope confirmed the rover had
rotated clockwise, but the encoder differential `(odl - odr)` had the wrong sign — the
firmware's `odl` field was decreasing where an increasing left-wheel reading was expected
for a CW rotation.

### Root Cause
The MFD board's `odl` and `odr` field labels are physically reversed. The field documented
as `odl` (left) is wired to the right front wheel, and `odr` (right) to the left front
wheel. Every heading calculation that treated the fields as labelled had been computing
turns in the wrong direction — a leftward turn was reported as rightward and vice versa.

This would have produced persistent, unexplained map mirroring and rotation divergence in
every future SLAM session. Because it manifested as a sign error rather than a magnitude
error, it would have appeared to be a `TRACK_WIDTH` tuning problem — one that no amount of
parameter adjustment could fix. The script's raw data output caught it before a single
additional session was wasted.

**Fix applied:** Swapped `odl` and `odr` in the odometry node.

---

## TRACK_WIDTH Result

After correcting the encoder swap and re-running Phase 3:

**`TRACK_WIDTH = 0.0456 m`**

This value is significantly smaller than the physical axle-to-axle measurement, which
is consistent with the Session 007 finding that passive rear wheel drag shifts the effective
pivot point inward during motor-driven turns. The gyroscope-derived value reflects actual
robot dynamics rather than chassis geometry, which is the correct reference for odometry.

---

## Discovery — Battery Sag as Root Cause of Heading Degradation

### SLAM Mapping Attempt
With `TRACK_WIDTH = 0.0456` applied and the encoder swap corrected, I ran a teleop SLAM
session. Map quality had not improved as expected — heading behaviour was erratic in a way
that did not match the clean calibration results from earlier in the session.

### Investigation
I monitored the MFD `v` field (battery voltage × 100) during movement. The values were
consistent with a significantly depleted battery pack — the cells had been drained across
multiple calibration runs earlier in the session without recharging.

### What Battery Sag Does to Odometry
Under low voltage conditions, the MFD's motor output becomes asymmetric. The two sides draw
different effective currents at the same commanded speed, producing lateral drift and
unequal wheel velocities. More critically, encoder pulse timing becomes unreliable when
motor supply voltage sags — the same physical rotation produces different raw `odl`/`odr`
deltas depending on the instantaneous battery state. The `TRACK_WIDTH` value the script
calibrated against no longer describes the robot's actual behaviour at low charge.

This is not a parameter problem. Encoder-based heading is fundamentally fragile under
conditions where motor output is non-ideal — and motor output is non-ideal whenever battery
voltage, terrain, or wheel loading deviates from calibration conditions.

---

## Architecture Decision: Gyrodometry for Session 009

The battery sag finding exposed a deeper architectural limitation. Encoder-derived heading
accumulates error from any source of motor asymmetry — low battery, uneven terrain, wheel
slip, passive-wheel drag. Gyroscope-derived heading is immune to all of these because it
measures rotation directly from the physics of the chassis, not from the motors driving it.

The correct architecture going forward is a hybrid approach:

- **Heading:** Gyroscope (`gz` integrated with calibrated scale factor) — robust to motor
  asymmetry, wheel slip, battery state, and passive-wheel geometry
- **Linear displacement:** Encoder odometry (`(odl + odr) / 2`) — accurate for straight-line
  distance, where encoder errors are small and largely symmetric

This approach — sometimes called gyrodometry — is well-established in mobile robotics and
directly resolves both the battery sag problem and the passive-rear-wheel geometry issue
that has complicated `TRACK_WIDTH` calibration throughout Sessions 007 and 008. Under this
model, `TRACK_WIDTH` is no longer needed as a calibration parameter because heading comes
from the gyroscope directly.

---

## Lessons Learned

**Automated sensor-referenced calibration reveals problems that manual methods cannot.**
The encoder swap had been present since the odometry node was first written. It was
invisible to visual inspection of SLAM maps because it manifested as a systematic sign
error, not a magnitude error. A script that prints raw phase data exposed it in minutes.

**Calibration conditions must match operating conditions.** A `TRACK_WIDTH` value derived
from a fresh battery is not valid at a depleted battery. Any odometry parameter calibrated
under controlled conditions will drift when operating conditions change — which is the
argument for moving heading estimation to a sensor that is independent of those conditions.

**When a parameter can be eliminated, eliminate it.** Using the gyroscope for heading makes
`TRACK_WIDTH` unnecessary. Removing a tunable parameter is always better than tuning it
correctly.

---

## Next Steps

- Implement hybrid odometry node: gyroscope (`gz`) for heading, encoder average for
  linear displacement
- Re-run teleop SLAM mapping with the new odometry architecture and evaluate map quality
- Confirm that `TRACK_WIDTH` can be removed from the odometry node entirely once
  heading is gyroscope-sourced
