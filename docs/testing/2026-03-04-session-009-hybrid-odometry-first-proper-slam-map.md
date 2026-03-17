# Session 009 — Hybrid Odometry & First Proper SLAM Map

**Date:** 2026-03-04  
**Status:** ✅ Complete

---

## Goal

Implement a hybrid odometry node that sources heading from the gyroscope (`gz`) and
linear displacement from the encoder average, removing `TRACK_WIDTH` from the
odometry architecture entirely. Validate with a calibration run, deploy the node,
and produce a reliable teleop SLAM map.

---

## What Was Accomplished

1. Ran Phase 2 of `calibrate_track_width.py` to obtain `GZ_SCALE = 0.001058 rad/(count·s)`
   — the gyroscope scale factor needed to convert raw `gz` counts to radians per second
2. Wrote and deployed a new `rover_driver_node.py` implementing hybrid odometry: `gz`
   gyroscope integration for heading, encoder average `(odl + odr) / 2` for linear
   displacement, `TRACK_WIDTH` removed entirely
3. Validated heading accuracy: 90° motor turn measured by the gyro as 90.04°
4. Produced the first geometrically correct SLAM map of a room — sharp corners, tight
   wall definition, no doubling
5. Identified and permanently resolved a stale install directory conflict that was causing
   the node to crash after every rebuild

---

## GZ_SCALE Calibration

Running Phase 2 of `calibrate_track_width.py` (360° hand rotation):

```
Raw integral : -5939.916 count.s
Scale factor : 0.001058 rad / count.s
```

The raw integral is negative because the rotation was clockwise — clockwise is negative
in the right-hand coordinate convention. Phase 3 (90° motor turn) validated the result:

```
Turning... 99%  (89.2 deg)
Done! Gyro measured: 90.04 deg
```

The gyro measured 90.04° against a commanded 90° motor turn — 0.04° of error. The
derivation is straightforward: one full CW rotation = 2π radians, so:

```
GZ_SCALE = 2π / |raw_integral| = 6.2832 / 5939.916 = 0.001058 rad / (count·s)
```

The `WARN` at the end of calibration ("Odometry shows near-zero rotation") was expected
and irrelevant. That check was written to compute `TRACK_WIDTH` from encoder-based `/odom`.
Since the new node no longer derives heading from encoders, the check has nothing meaningful
to compare against.

---

## Hybrid Odometry Architecture

### Design

The core problem with encoder-based heading is that it depends on the difference between
two wheel measurements — a difference that is sensitive to wheel slip, surface friction,
voltage sag, and the UGV02's passive rear wheels, which contribute no encoder data. Any of
these conditions shifts the effective turning radius, and no static `TRACK_WIDTH` value
can account for all of them simultaneously.

The gyroscope measures rotation directly from the physics of the chassis — not from the
motors driving it. This makes it immune to motor asymmetry, wheel slip, and battery state.
The correct architecture is to pair it with encoder linear displacement, where encoder
errors are small and largely symmetric on straight runs:

- **Heading:** Gyroscope (`gz` integrated with calibrated `GZ_SCALE`) — robust to all
  sources of motor asymmetry
- **Linear displacement:** Encoder average `(odl + odr) / 2 × LINEAR_SCALE` — accurate
  for straight-line distance

Under this model, `TRACK_WIDTH` is not a tunable parameter — it simply does not exist.

**Update equations (midpoint approximation):**

```
d_linear = (Δodl + Δodr) / 2  ×  LINEAR_SCALE
dθ       = (gz_raw − gz_bias)  ×  GZ_SCALE  ×  dt

x       += d_linear × cos(θ + dθ/2)
y       += d_linear × sin(θ + dθ/2)
θ       += dθ
```

The midpoint approximation uses the heading at the midpoint of the timestep (`θ + dθ/2`)
for position integration, which reduces accumulated error at the 20 Hz update rate the
node runs at.

### Result

The new node produced geometrically correct odometry immediately. Turns registered
accurately. The SLAM map closed correctly on a rectangular room — the best map the rover
has produced across all sessions.

**First proper SLAM map — Session 009:**

![Session 009 — First Proper SLAM Map](../../images/testing/session-009/session-009-First-Proper-SLAM-Map.jpg)

Clean rectangular wall definition, sharp corners, and a clear open interior. The gap at
the top-right reflects an area I did not drive close enough for the LiDAR to cover — that is
expected, not an odometry error.

### Full Source — rover_driver_node.py

```python
#!/usr/bin/env python3
"""
rover_driver_node.py — Hybrid Odometry Node
============================================
Heading  : gyroscope gz integration   → drift-free turns, no TRACK_WIDTH needed
Position : encoder average (odl+odr)/2 → robust linear displacement

Startup sequence
----------------
The node collects gz samples for BIAS_DURATION seconds before publishing
any odometry. Keep the robot completely still during this window; a log
message will confirm when calibration is done.

Tunable constants
-----------------
LINEAR_SCALE   Converts raw odl/odr firmware units to metres.
               Determined empirically: 0.01 m/unit.

GZ_SCALE       Converts (raw_gz × seconds) to radians.
               Calibrated 2026-03-06 via 360° hand rotation: 0.001058 rad/(count·s)

BIAS_DURATION  Seconds of gz samples collected at startup for bias removal.
"""

import math
import json
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import serial
import tf2_ros

# ── Tunable parameters ──────────────────────────────────────────────────────

LINEAR_SCALE    = 0.01      # m / raw_unit  (empirically determined)
GZ_SCALE        = 0.001058  # rad / (count · s)  (calibrated 2026-03-06)
BIAS_DURATION   = 3.0       # seconds

# ── Serial device ────────────────────────────────────────────────────────────
ROVER_PORT      = '/dev/rover'
BAUD_RATE       = 115200

# ── Odometry covariance (6×6 row-major: x, y, z, roll, pitch, yaw) ──────────
POSE_COV = [
    0.002, 0.0,   0.0,  0.0,  0.0,  0.0,
    0.0,   0.002, 0.0,  0.0,  0.0,  0.0,
    0.0,   0.0,   1e6,  0.0,  0.0,  0.0,
    0.0,   0.0,   0.0,  1e6,  0.0,  0.0,
    0.0,   0.0,   0.0,  0.0,  1e6,  0.0,
    0.0,   0.0,   0.0,  0.0,  0.0,  0.001,
]
TWIST_COV = [
    0.001, 0.0,  0.0,  0.0,  0.0,  0.0,
    0.0,   1e6,  0.0,  0.0,  0.0,  0.0,
    0.0,   0.0,  1e6,  0.0,  0.0,  0.0,
    0.0,   0.0,  0.0,  1e6,  0.0,  0.0,
    0.0,   0.0,  0.0,  0.0,  1e6,  0.0,
    0.0,   0.0,  0.0,  0.0,  0.0,  0.001,
]


class RoverDriverNode(Node):

    def __init__(self):
        super().__init__('rover_driver')

        self._odom_pub  = self.create_publisher(Odometry, '/odom', 10)
        self._gz_pub    = self.create_publisher(Float32,  '/imu/gz', 50)
        self._cmd_sub   = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._ser = serial.Serial(ROVER_PORT, BAUD_RATE, timeout=0.1)

        self._x         = 0.0
        self._y         = 0.0
        self._theta     = 0.0
        self._prev_odl  = None
        self._prev_odr  = None
        self._prev_time = None

        self._gz_bias      = None
        self._bias_samples = []
        self._bias_start   = time.time()
        self._calibrated   = False

        self._lock          = threading.Lock()
        self._latest_cmd    = None
        self._reader_thread = threading.Thread(
            target=self._serial_reader, daemon=True)
        self._reader_thread.start()

        self._timer = self.create_timer(0.05, self._loop)

        self.get_logger().info(
            'Rover driver node started (hybrid odometry: gyro heading + encoder linear).')
        self.get_logger().info(
            f'Calibrating gz bias for {BIAS_DURATION:.0f}s — keep robot STILL...')

    def _serial_reader(self):
        buf = b''
        while rclpy.ok():
            try:
                chunk = self._ser.read(256)
            except Exception:
                break
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                try:
                    msg = json.loads(line.decode('utf-8', errors='ignore').strip())
                    with self._lock:
                        self._latest_cmd = msg
                except json.JSONDecodeError:
                    pass

    def _cmd_vel_cb(self, msg: Twist):
        linear  = msg.linear.x
        angular = msg.angular.z
        left  = linear - angular * 0.5
        right = linear + angular * 0.5
        left  = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))
        cmd = json.dumps({'T': 1, 'L': round(left, 3), 'R': round(right, 3)}) + '\n'
        self._ser.write(cmd.encode())

    def _loop(self):
        with self._lock:
            data = self._latest_cmd
            self._latest_cmd = None

        if data is None or data.get('T') != 1001:
            return

        gz_raw = data.get('gz')
        odl    = data.get('odl')
        odr    = data.get('odr')

        if gz_raw is None or odl is None or odr is None:
            return

        now = time.time()

        gz_msg = Float32()
        gz_msg.data = float(gz_raw)
        self._gz_pub.publish(gz_msg)

        if not self._calibrated:
            self._bias_samples.append(float(gz_raw))
            if now - self._bias_start >= BIAS_DURATION:
                if len(self._bias_samples) < 10:
                    self.get_logger().error(
                        'Too few gz samples during bias calibration. '
                        'Is the MFD sending T:1001 feedback?')
                    return
                self._gz_bias = sum(self._bias_samples) / len(self._bias_samples)
                noise = max(self._bias_samples) - min(self._bias_samples)
                self._calibrated  = True
                self._prev_odl    = float(odl)
                self._prev_odr    = float(odr)
                self._prev_time   = now
                self.get_logger().info(
                    f'gz bias calibrated: {self._gz_bias:.3f} counts  '
                    f'(noise p-p: {noise:.3f})')
                self.get_logger().info('Odometry active — ready for SLAM.')
            return

        dt    = now - self._prev_time
        d_odl = float(odl) - self._prev_odl
        d_odr = float(odr) - self._prev_odr
        gz_c  = float(gz_raw) - self._gz_bias

        self._prev_odl  = float(odl)
        self._prev_odr  = float(odr)
        self._prev_time = now

        d_linear     = (d_odl + d_odr) * 0.5 * LINEAR_SCALE
        dtheta       = gz_c * GZ_SCALE * dt
        heading_mid  = self._theta + dtheta * 0.5
        self._x     += d_linear * math.cos(heading_mid)
        self._y     += d_linear * math.sin(heading_mid)
        self._theta += dtheta

        vx     = d_linear / dt if dt > 0 else 0.0
        vtheta = dtheta   / dt if dt > 0 else 0.0

        odom_msg = Odometry()
        stamp = self.get_clock().now().to_msg()
        odom_msg.header.stamp    = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id  = 'base_link'

        odom_msg.pose.pose.position.x = self._x
        odom_msg.pose.pose.position.y = self._y
        odom_msg.pose.pose.position.z = 0.0

        q = _yaw_to_quat(self._theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x  = vx
        odom_msg.twist.twist.angular.z = vtheta
        odom_msg.pose.covariance  = POSE_COV
        odom_msg.twist.covariance = TWIST_COV

        self._odom_pub.publish(odom_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp    = stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id  = 'base_link'
        tf_msg.transform.translation.x = self._x
        tf_msg.transform.translation.y = self._y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]
        self._tf_broadcaster.sendTransform(tf_msg)

    def destroy_node(self):
        try:
            self._ser.write(b'{"T":1,"L":0,"R":0}\n')
            self._ser.close()
        except Exception:
            pass
        super().destroy_node()


def _yaw_to_quat(yaw: float):
    """Convert a 2D yaw angle (radians) to a unit quaternion (x, y, z, w)."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def main(args=None):
    rclpy.init(args=args)
    node = RoverDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Discovery 1 — Stale Install Directory Conflict

### What the Error Showed
After a clean build, the node kept crashing at the covariance assignment line despite the
source file being correct. The traceback path revealed the problem:

```
File "/home/arshamfn/install/rover_driver/lib/python3.10/site-packages/rover_driver/rover_driver_node.py"
```

The node was running from `~/install/` — not `~/ros2_ws/install/`. A stale second install
tree was taking priority over the correct build, so every `ros2 run` launched the old file
regardless of what the workspace build produced.

### Root Cause
At some point in an earlier session, `colcon build` was run from `~/` instead of
`~/ros2_ws/`, which created a second install tree at `~/install/`. That directory was never
removed. Because a previous terminal had sourced `~/install/setup.bash`, the stale node
appeared earlier in the Python path than the correct one — `ros2 run` found it first.

Checking `.bashrc` confirmed it only sources `~/ros2_ws/install/setup.bash`, so the
problem was purely from lingering terminal state. The fix was to delete the stale
directory entirely:

```bash
rm -rf ~/install/
```

The Ubuntu file manager briefly crashed when the directory it was watching disappeared —
that was harmless. After opening a fresh terminal, the node ran correctly with no
ambiguity.

---

## Discovery 2 — Battery Sag Under Hard Acceleration

### What the Test Showed
During the teleop SLAM run, attempting to accelerate the rover from a standstill at full
speed caused the Jetson to shut down mid-session. Monitoring the MFD `v` field (battery
voltage × 100) showed a sharp voltage drop at the moment of acceleration — the motors
drawing peak current simultaneously caused the battery pack to sag below the Jetson's
minimum operating voltage, triggering a protective shutdown.

The problem does not occur at sustained moderate speed — only at the initial surge from
rest. Gradual acceleration from a standstill keeps the current draw low enough that the
battery can deliver it without sagging.

### Root Cause
The UPS module and drive motors share the same battery cells. When all four motors
accelerate simultaneously from zero, the instantaneous current demand exceeds what the
cells can deliver cleanly. The voltage sag is brief but deep enough to reset the Jetson.
This is not a fault in the UPS module or the batteries — it is a consequence of routing
high-current motor drive and compute power through the same source without any buffering.

A software acceleration limiter in the driver node — ramping `cmd_vel` output gradually
rather than passing through step changes directly — will cap the instantaneous current
spike and prevent the sag from reaching a depth that affects the Jetson. A dedicated
separate power supply for the Jetson is the correct long-term hardware fix, but the
software limiter is a viable solution for the current sessions.

---

## Lessons Learned

**When `TRACK_WIDTH` can be eliminated, eliminate it.** Once heading is sourced from the
gyroscope, `TRACK_WIDTH` is not a tunable parameter — it simply does not exist in the
equation. The only parameters the node needs are `LINEAR_SCALE`, `GZ_SCALE`, and
`gz_bias`, none of which depend on chassis geometry.

**ROS2 covariance arrays require Python `float` literals.** Bare integer zeros (`0`) pass
the length check at definition time but fail the type check when assigned to the message
field at runtime, producing a cryptic `AssertionError` with no obvious connection to the
type mismatch. All elements must be `0.0`.

**Always build from the workspace root.** Running `colcon build` from any directory other
than `~/ros2_ws/` creates a second install tree that silently shadows the correct build.
The traceback file path is the only sign something is wrong.

**Gyro bias drifts with temperature.** Auto-calibrating it at every startup (3-second still
period) is more reliable than hardcoding a value from a one-time measurement.

---

## Next Steps

- [ ] Implement a software acceleration limiter in the driver node — ramp `cmd_vel`
      velocity gradually to prevent sudden current spikes that cause battery sag and
      Jetson shutdown under hard starts
- [ ] Tune SLAM Toolbox `transform_timeout` and scan matching parameters to eliminate
      wall smearing at moderate teleop speeds
- [ ] Confirm maps are clean and repeatable across multiple runs at limited speed
- [ ] Plan hardware power separation (dedicated Jetson UPS module) for a future session
