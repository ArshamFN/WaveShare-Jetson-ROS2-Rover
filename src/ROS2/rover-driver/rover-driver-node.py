#!/usr/bin/env python3
"""
rover_driver_node.py — Hybrid Odometry Node
============================================
Heading  : gyroscope gz integration   → drift-free turns, no TRACK_WIDTH needed
Position : encoder average (odl+odr)/2 → robust linear displacement

Startup sequence
----------------
The node collects gz samples for BIAS_DURATION seconds before publishing
any odometry.  Keep the robot completely still during this window; a log
message will confirm when calibration is done.

Tunable constants
-----------------
LINEAR_SCALE   Converts raw odl/odr firmware units to metres.
               Determined empirically: 0.01 m/unit.

GZ_SCALE       Converts (raw_gz × seconds) to radians.
               Obtain with calibrate_track_width.py (Phase 2 — 360° hand
               rotation).  A fresh run of that script prints the value;
               paste it here before using this node for SLAM.

BIAS_DURATION  Seconds of gz samples collected at startup for bias removal.

HEADING_KP     Proportional gain for the heading hold controller.
               During straight-line travel (angular == 0), any drift from
               the locked heading is fed back as a corrective angular.z
               command.  Currently set to 1.4 — the universal tune from
               Session 011, validated on both hard floor and carpet:
               - Too low  → robot still wanders noticeably
               - Too high → robot oscillates (fishtails) while driving

HEADING_SETTLE_THRESHOLD
               Maximum gyro rate (rad/s) that must be seen before the
               heading hold will lock onto a new straight-line heading.
               Prevents the controller fighting the tail end of a turn.

LINEAR_RAMP_RATE
               Maximum linear acceleration in m/s².  Limits how quickly the
               commanded linear velocity is applied to the motors, preventing
               the current spike that causes Jetson voltage sag and shutdown.

ANGULAR_RAMP_RATE
               Maximum angular acceleration in rad/s².  Higher than linear
               so turns still feel responsive.

ZUPT_ODO_THRESHOLD
               Maximum encoder delta (raw units, per tick) below which the
               robot is considered stationary for Zero Velocity Update.

ZUPT_SETTLE_TICKS
               Consecutive stationary ticks required before the bias update
               is allowed to fire.  Prevents mid-turn pauses corrupting the
               estimate.  At 20 Hz, 20 ticks = 1 second.

ZUPT_ALPHA     Exponential moving average factor for the bias update.
               Each qualifying stationary tick nudges _gz_bias by this
               fraction toward the current gz_raw reading.  0.05 is
               conservative — effective over a multi-minute session.

BIAS_NOISE_THRESHOLD
               Maximum acceptable standard deviation (raw gyro counts) of the
               bias calibration window.  Uses std dev rather than peak-to-peak
               so a single spike does not falsely extend calibration.
               If noisier than this, calibration extends until settled or
               BIAS_MAX_DURATION is reached.

BIAS_MAX_DURATION
               Hard ceiling on bias calibration time (seconds).  If the gyro
               has not converged by this point, the best available estimate
               is accepted and a warning is logged.
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

# Linear scale: raw odl/odr units → metres (empirically determined)
LINEAR_SCALE    = 0.01      # m / raw_unit

# Gyro scale: raw_gz_count × seconds → radians
# Run calibrate_track_width.py Phase 2 to obtain this value.
GZ_SCALE        = 0.001058  # rad / (count · s)

# Bias calibration: seconds to collect gz samples at startup (robot must be still)
BIAS_DURATION   = 5.0       # seconds

# Bias convergence: calibration extends beyond BIAS_DURATION if gyro is noisy.
BIAS_NOISE_THRESHOLD = 3.0   # raw counts standard deviation
BIAS_MAX_DURATION    = 15.0  # seconds — hard ceiling

# Heading hold: proportional gain (rad/s correction per radian of heading error)
# Applied only during straight-line travel (cmd angular.z == 0, linear.x != 0).
# Sign is automatically flipped for reverse travel.
HEADING_KP      = 1.4

# Derivative gain: damps heading correction to prevent overshoot/oscillation.
HEADING_KD      = 0.5

# Gyro settle threshold: heading hold will not lock until the corrected gz rate
# drops below this value (rad/s).  Prevents fighting the tail of a turn.
HEADING_SETTLE_THRESHOLD = 0.05   # rad/s  (~3 deg/s)

# Gyro spike clamp: any gz_c implying a rotation rate above this value is noise.
# Not a physical limit — deliberately tightened from 3.0 to 2.0 in Session 011
# to filter vibration spikes on carpet without rejecting legitimate fast turns.
GZ_MAX_RATE     = 2.0             # rad/s

# Heading hold deadband: corrections smaller than this angle are ignored.
# Widened in Session 011 to filter carpet vibration without re-tuning per surface.
HEADING_DEADBAND = 0.045          # rad (~2.6 degrees)

# Maximum angular correction the heading hold controller can inject (rad/s).
# Caps the P-controller output to prevent large errors causing full-speed spins.
HEADING_MAX_CORRECTION = 0.3          # rad/s

# Velocity ramp: maximum rate of change per second applied to motor commands.
# Limits inrush current to prevent Jetson voltage sag under hard acceleration.
LINEAR_RAMP_RATE  = 0.8           # m/s²
ANGULAR_RAMP_RATE = 2.0           # rad/s²

# Zero Velocity Update (ZUPT): continuous gyro bias correction while stationary.
# Counteracts temperature-dependent gyro drift over long mapping sessions.
ZUPT_ODO_THRESHOLD = 0.5    # raw encoder units per tick (~0.5 mm)
ZUPT_SETTLE_TICKS  = 20     # ticks (~1 second at 20 Hz)
ZUPT_ALPHA         = 0.05   # EMA factor — slow, stable bias nudge

# ── Serial device ────────────────────────────────────────────────────────────
ROVER_PORT      = '/dev/rover'
BAUD_RATE       = 115200

# ── Odometry covariance ──────────────────────────────────────────────────────
# 6×6 row-major: [x, y, z, roll, pitch, yaw]
# Position variance is larger (encoder slip); heading is tighter (gyro).
POSE_COV = [
    0.002, 0.0,  0.0,  0.0,  0.0,  0.0,
    0.0,   0.002,0.0,  0.0,  0.0,  0.0,
    0.0,   0.0,  1e6,  0.0,  0.0,  0.0,
    0.0,   0.0,  0.0,  1e6,  0.0,  0.0,
    0.0,   0.0,  0.0,  0.0,  1e6,  0.0,
    0.0,   0.0,  0.0,  0.0,  0.0,  0.001,
]
TWIST_COV = [
    0.001, 0.0,  0.0,  0.0,  0.0,  0.0,
    0.0,   1e6,  0.0,  0.0,  0.0,  0.0,
    0.0,   0.0,  1e6,  0.0,  0.0,  0.0,
    0.0,   0.0,  0.0,  1e6,  0.0,  0.0,
    0.0,   0.0,  0.0,  0.0,  1e6,  0.0,
    0.0,   0.0,  0.0,  0.0,  0.0,  0.001,
]
# ─────────────────────────────────────────────────────────────────────────────


class RoverDriverNode(Node):

    def __init__(self):
        super().__init__('rover_driver')

        # ── Publishers ───────────────────────────────────────────────────────
        # NOTE (Session 012): renamed from '/odom' to '/odom_wheel' — RF2O now
        # publishes '/odom' from lidar scan matching. Kept alive here as a
        # standalone topic for comparing wheel-derived odometry against RF2O.
        self._odom_pub  = self.create_publisher(Odometry, '/odom_wheel', 10)
        self._gz_pub    = self.create_publisher(Float32,  '/imu/gz', 50)
        self._cmd_sub   = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # ── TF broadcaster ───────────────────────────────────────────────────
        # NOTE (Session 012): broadcaster is still constructed but sendTransform
        # is no longer called — see _loop(). RF2O now owns odom → base_link.
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Serial ───────────────────────────────────────────────────────────
        self._ser = serial.Serial(ROVER_PORT, BAUD_RATE, timeout=0.1)

        # ── Odometry state ───────────────────────────────────────────────────
        self._x          = 0.0
        self._y          = 0.0
        self._theta      = 0.0   # heading in radians, gyro-integrated
        self._prev_odl   = None
        self._prev_odr   = None
        self._prev_time  = None

        # ── Gyro calibration ─────────────────────────────────────────────────
        self._gz_bias        = None
        self._bias_samples   = []
        self._bias_start     = time.time()
        self._calibrated     = False

        # ── Incoming cmd_vel (stored here, applied in _loop after odom update)
        self._cmd_linear     = 0.0   # latest commanded linear  velocity (m/s)
        self._cmd_angular    = 0.0   # latest commanded angular velocity (rad/s)

        # ── Velocity ramp state ─────────────────────────────────────────────
        # Tracks the currently-output velocity, which chases _cmd_linear/angular
        # at a limited rate to prevent current spikes on hard acceleration.
        self._ramp_linear    = 0.0
        self._ramp_angular   = 0.0

        # ── ZUPT state ──────────────────────────────────────────────────────
        # Counts consecutive ticks where both encoders read near-zero.
        # Once threshold is reached, gz_bias is nudged toward current gz_raw.
        self._zupt_ticks     = 0

        # ── Heading hold state ───────────────────────────────────────────────
        # Set to the current heading the moment straight-line travel begins.
        # Cleared to None whenever the robot turns or stops, so the next
        # straight segment gets a fresh lock on whatever heading it starts from.
        self._theta_hold     = None

        # ── Serial reader thread ─────────────────────────────────────────────
        self._lock          = threading.Lock()
        self._latest_cmd    = None
        self._reader_thread = threading.Thread(
            target=self._serial_reader, daemon=True)
        self._reader_thread.start()

        # ── Main loop timer (20 Hz) ──────────────────────────────────────────
        self._timer = self.create_timer(0.05, self._loop)

        self.get_logger().info(
            'Rover driver node started (hybrid odometry: gyro heading + encoder linear).')
        self.get_logger().info(
            f'Calibrating gz bias for {BIAS_DURATION:.0f}s — keep robot STILL...')

    # ── Serial reader (runs in background thread) ────────────────────────────

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

    # ── cmd_vel callback ─────────────────────────────────────────────────────
    # Stores the incoming command only — motor output happens in _loop so that
    # heading-hold correction can be applied using the freshly updated theta.

    def _cmd_vel_cb(self, msg: Twist):
        self._cmd_linear  = msg.linear.x
        self._cmd_angular = msg.angular.z

    # ── Main loop ────────────────────────────────────────────────────────────

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

        # ── Publish raw gz for calibration tools ─────────────────────────────
        gz_msg = Float32()
        gz_msg.data = float(gz_raw)
        self._gz_pub.publish(gz_msg)

        # ── Phase 0: bias collection ─────────────────────────────────────────
        if not self._calibrated:
            self._bias_samples.append(float(gz_raw))
            elapsed = now - self._bias_start

            # Minimum duration must pass before we evaluate convergence
            if elapsed < BIAS_DURATION:
                return

            if len(self._bias_samples) < 10:
                self.get_logger().error(
                    'Too few gz samples during bias calibration. '
                    'Is the MFD sending T:1001 feedback?')
                return

            mean  = sum(self._bias_samples) / len(self._bias_samples)
            variance = sum((s - mean) ** 2 for s in self._bias_samples) / len(self._bias_samples)
            noise = variance ** 0.5  # standard deviation
            converged = noise <= BIAS_NOISE_THRESHOLD
            timed_out = elapsed >= BIAS_MAX_DURATION

            if not converged and not timed_out:
                # Still noisy and time remains — log periodically and keep collecting
                if int(elapsed) != int(elapsed - 0.05):
                    self.get_logger().info(
                        f'gz bias noisy ({noise:.2f} counts std dev) — '
                        f'extending calibration... ({elapsed:.0f}s)')
                return

            # Accept the best available estimate
            self._gz_bias = sum(self._bias_samples) / len(self._bias_samples)
            self._calibrated  = True
            self._prev_odl    = float(odl)
            self._prev_odr    = float(odr)
            self._prev_time   = now

            if converged:
                self.get_logger().info(
                    f'gz bias stable — calibration accepted: '
                    f'{self._gz_bias:.3f} counts  (std dev: {noise:.2f})')
            else:
                self.get_logger().warn(
                    f'gz bias did not converge after {BIAS_MAX_DURATION:.0f}s — '
                    f'using best estimate: {self._gz_bias:.3f} counts  '
                    f'(std dev: {noise:.2f})')

            self.get_logger().info('Odometry active — ready for SLAM.')
            return  # Do not publish odom until calibrated

        # ── Compute deltas ───────────────────────────────────────────────────
        dt    = now - self._prev_time
        d_odl = float(odl) - self._prev_odl
        d_odr = float(odr) - self._prev_odr
        gz_c  = float(gz_raw) - self._gz_bias

        # Clamp gz_c to physically plausible range — spikes from motor vibration
        # on hard floors can cause dtheta jumps and spurious heading corrections.
        gz_c_max = GZ_MAX_RATE / GZ_SCALE
        gz_c = max(-gz_c_max, min(gz_c_max, gz_c))

        self._prev_odl  = float(odl)
        self._prev_odr  = float(odr)
        self._prev_time = now

        # ── ZUPT: continuous gyro bias correction ───────────────────────────
        # If both encoders show near-zero movement, the robot is stationary and
        # any gz_raw reading is pure bias.  Nudge _gz_bias toward gz_raw slowly.
        if abs(d_odl) < ZUPT_ODO_THRESHOLD and abs(d_odr) < ZUPT_ODO_THRESHOLD:
            self._zupt_ticks += 1
            if self._zupt_ticks >= ZUPT_SETTLE_TICKS:
                self._gz_bias += ZUPT_ALPHA * (float(gz_raw) - self._gz_bias)
        else:
            self._zupt_ticks = 0

        # ── Hybrid odometry update ───────────────────────────────────────────
        # Linear: average of both encoder wheels, scaled to metres
        d_linear = (d_odl + d_odr) * 0.5 * LINEAR_SCALE

        # Heading: gyroscope integration — no TRACK_WIDTH required
        dtheta = gz_c * GZ_SCALE * dt

        # Integrate pose (midpoint approximation)
        heading_mid   = self._theta + dtheta * 0.5
        self._x      += d_linear * math.cos(heading_mid)
        self._y      += d_linear * math.sin(heading_mid)
        self._theta  += dtheta

        # ── Velocities for twist ─────────────────────────────────────────────
        vx      = d_linear / dt if dt > 0 else 0.0
        vtheta  = dtheta   / dt if dt > 0 else 0.0

        # ── Build Odometry message ───────────────────────────────────────────
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

        # ── odom → base_link TF DISABLED (Session 012: RF2O now owns this transform) ──
        # tf_msg = TransformStamped()
        # tf_msg.header.stamp    = stamp
        # tf_msg.header.frame_id = 'odom'
        # tf_msg.child_frame_id  = 'base_link'
        # tf_msg.transform.translation.x = self._x
        # tf_msg.transform.translation.y = self._y
        # tf_msg.transform.translation.z = 0.0
        # tf_msg.transform.rotation.x = q[0]
        # tf_msg.transform.rotation.y = q[1]
        # tf_msg.transform.rotation.z = q[2]
        # tf_msg.transform.rotation.w = q[3]
        # self._tf_broadcaster.sendTransform(tf_msg)

        # ── Heading hold + motor output ──────────────────────────────────────
        # Odometry is fully updated above before we compute the motor command,
        # so self._theta reflects the current heading at this exact moment.

        linear  = self._cmd_linear
        angular = self._cmd_angular

        # ── Velocity ramp ────────────────────────────────────────────────────
        # Step _ramp_linear/_ramp_angular toward commanded values, capped at
        # the configured ramp rate.  dt is already computed above.
        max_d_lin = LINEAR_RAMP_RATE  * dt
        max_d_ang = ANGULAR_RAMP_RATE * dt

        diff_lin = linear  - self._ramp_linear
        diff_ang = angular - self._ramp_angular

        self._ramp_linear  += max(-max_d_lin, min(max_d_lin, diff_lin))
        self._ramp_angular += max(-max_d_ang, min(max_d_ang, diff_ang))

        # Use ramped values for both heading hold and motor output
        linear  = self._ramp_linear
        angular = self._ramp_angular

        if angular == 0.0 and linear != 0.0:
            # Straight-line travel: engage heading hold
            if self._theta_hold is None:
                # Wait for gyro to settle before locking — prevents fighting the
                # tail end of a preceding turn.
                gz_rate = abs(gz_c) * GZ_SCALE / dt if dt > 0 else 999.0
                if gz_rate < HEADING_SETTLE_THRESHOLD:
                    self._theta_hold = self._theta
                    self.get_logger().debug(
                        f'Heading hold locked: {math.degrees(self._theta_hold):.1f}°')
                # else: still spinning — skip correction this tick

            if self._theta_hold is not None:
                # Proportional correction: positive error → turn CCW (increase theta)
                error = self._theta_hold - self._theta
                # Wrap to [-pi, pi] to handle the 0/360 boundary
                while error >  math.pi: error -= 2.0 * math.pi
                while error < -math.pi: error += 2.0 * math.pi

                # Flip correction sign for reverse travel — the same angular
                # command that steers right going forward steers left in reverse.
                # Deadband: ignore small errors to avoid chasing gyro noise.
                if abs(error) > HEADING_DEADBAND:
                    # PD controller: P term corrects error, D term damps overshoot
                    angular_rate = gz_c * GZ_SCALE / dt if dt > 0 else 0.0
                    correction = (HEADING_KP * error
                                  - HEADING_KD * angular_rate
                                  * math.copysign(1.0, linear))
                    # Hard cap — prevents spikes from commanding a full-speed spin
                    angular = max(-HEADING_MAX_CORRECTION,
                                  min(HEADING_MAX_CORRECTION, correction))
                else:
                    angular = 0.0

        else:
            # Turning or stopped: release hold
            if self._theta_hold is not None:
                self.get_logger().debug('Heading hold released.')
            self._theta_hold = None

        left  = max(-1.0, min(1.0, linear - angular * 0.5))
        right = max(-1.0, min(1.0, linear + angular * 0.5))
        cmd = json.dumps({'T': 1, 'L': round(left, 3), 'R': round(right, 3)}) + '\n'
        self._ser.write(cmd.encode())

    # ── Cleanup ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        try:
            self._ser.write(b'{"T":1,"L":0,"R":0}\n')
            self._ser.close()
        except Exception:
            pass
        super().destroy_node()


# ── Helpers ──────────────────────────────────────────────────────────────────

def _yaw_to_quat(yaw: float):
    """Convert a 2D yaw angle (radians) to a unit quaternion (x, y, z, w)."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


# ── Entry point ──────────────────────────────────────────────────────────────

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
