# rover_driver

A ROS2 Python node that bridges the standard `/cmd_vel` topic to the Waveshare UGV02
Multi-Functional Driver (MFD) board's JSON-over-serial protocol, and publishes
wheel-derived odometry, raw gyroscope data, and battery voltage.

## What this node does

Every ROS2 navigation tool, whether teleop, Nav2, or RViz, speaks the same language for
motion: a `geometry_msgs/Twist` message on `/cmd_vel`, expressing "drive forward at
X m/s while rotating at Z rad/s". The MFD board does not understand that. It expects
JSON commands over a serial port specifying a target speed for each side.

This node is the translator between those two worlds. It subscribes to `/cmd_vel`,
applies differential-drive kinematics to convert linear and angular velocity into left
and right wheel speeds in m/s, and writes them to `/dev/rover` as T:1 commands. The
board runs its own closed-loop speed controller on those targets using the wheel
encoders.

Beyond translation, it does five things:

**Low-speed floor.** The board's speed controller cannot regulate a wheel below about
0.08 m/s; the 20 PPR encoders do not resolve slower motion. Without compensation, small
commands such as Nav2's default in-place rotation simply stall. When the faster wheel's
target is non-zero but under 0.10 m/s, the node scales both wheels up by the same factor
until it reaches 0.10 m/s. Scaling the pair keeps the ratio between the wheels, so the
commanded turn curvature is preserved and only the speed rises.

**Velocity ramping.** Commanding all four motors from a standstill to full speed draws a
current spike large enough to sag the battery below the Jetson's minimum operating
voltage, triggering a protective shutdown. The node limits acceleration to 0.8 m/s²
linear and 2.0 rad/s² angular, which eliminates the shutdown and smooths motion as a
side effect.

**Command watchdog.** If no `/cmd_vel` message arrives for 0.5 s, the node treats the
command as zero, so a crashed controller cannot leave the rover driving on its last
instruction. The ramp still applies, so the rover decelerates rather than stopping dead.

**Battery monitoring.** The board reports pack voltage in every T:1001 feedback packet
(`v`, volts × 100). The node publishes it on `/battery_voltage`, logs a warning at
10.5 V, and logs an error at 9.6 V, each at most once every 30 seconds. One pack feeds
both the Jetson and the motors, so a falling voltage is the early sign of a brownout.

**Zero Velocity Update (ZUPT).** Gyroscope bias drifts with temperature over a long
session. Whenever both encoders report near-zero movement for a full second, the robot
is stationary and any gyro reading is pure bias, so the node nudges its bias estimate
toward the current reading. This runs continuously throughout a session rather than only
at startup.

A forward-only **heading hold** PD controller is also implemented: during straight-line
travel it locks onto the current heading and corrects drift from it. It is disabled by
default (`USE_HEADING_HOLD = False`), because Nav2's controller runs its own closed loop,
and two controllers driving one actuator fight each other.

## Wheel commands

The mix lives in one function, `mix_to_wheel_speeds(linear, angular)`:

1. Non-finite input (NaN or infinity) returns zero for both wheels.
2. Wheel speeds are `linear ∓ angular × TRACK_WIDTH / 2`.
3. If the faster wheel is below 0.005 m/s, both wheels are sent exactly zero.
4. If the faster wheel is below 0.10 m/s, both are scaled up together to the floor.
5. Each wheel is clamped to ±`MAX_WHEEL_SPEED` (0.956 m/s).

The T:1 values are sent in m/s, not as a normalized duty cycle. With the wheels off the
ground, the board holds the commanded speed to within a few percent from 0.1 to
1.0 m/s, and on the floor it holds the same speed as in the air, which is how the
closed-loop behaviour was confirmed.

In a tight arc, the inner wheel's target can end up below the floor while the outer
wheel's is above it. That wheel will not track precisely, so the arc comes out slightly
tighter than commanded; Nav2's feedback corrects it.

## What this node does not do

**It does not broadcast `odom → base_link`.** Since Session 012, that transform has been
owned by `rf2o_laser_odometry`, which derives the robot's pose from LiDAR scan matching
rather than from wheel encoders and the gyroscope.

Gyro-based odometry works, but its calibration is conditional: constants tuned on a hard
floor do not transfer to carpet because the vibration noise floor changes with the
surface, and gyroscope bias drifts as the electronics warm up. The LiDAR is unaffected
by either. Rather than continue tuning around conditions that keep changing, pose
estimation was moved to the sensor that does not care about them.

A TF edge can only have one publisher. Two nodes broadcasting `odom → base_link`
produce a transform that alternates between disagreeing sources, which corrupts
slam_toolbox's motion prior without raising any error at all. The TF broadcast block in
`rover_driver_node.py` is therefore commented out rather than deleted, so the change is
visible and reversible.

## Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Subscribes | `/cmd_vel` | `geometry_msgs/Twist` | Standard ROS2 motion command |
| Publishes | `/odom_wheel` | `nav_msgs/Odometry` | Wheel and gyro odometry, retained for comparison against RF2O |
| Publishes | `/imu/gz` | `std_msgs/Float32` | Raw gyroscope Z-axis, for calibration tools and future EKF fusion |
| Publishes | `/battery_voltage` | `std_msgs/Float32` | Pack voltage in volts, on every feedback packet (~12 Hz) |

The odometry topic is `/odom_wheel`, not `/odom`. `/odom` belongs to
`rf2o_laser_odometry`, and publishing both to the same topic name would put two nodes
in conflict over it.

## Prerequisites

The MFD board must be reachable at `/dev/rover`. This is a persistent udev symlink
rather than a raw `/dev/ttyACM*` path, because device numbering changes between boots.

Verify it exists:

```bash
ls -l /dev/rover
```

If it does not, create the udev rule:

```bash
sudo nano /etc/udev/rules.d/99-rover.rules
```

Add the following, substituting your board's serial number if it differs:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", ATTRS{serial}=="585A087742", SYMLINK+="rover"
```

Reload and replug the board:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

The node also needs `pyserial`:

```bash
sudo apt install python3-serial
```

## Build

The package is part of the repository's colcon workspace. From the workspace root:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select rover_driver
source install/setup.bash
```

If a rebuild does not seem to take effect, confirm which install directory is actually
being sourced:

```bash
echo $AMENT_PREFIX_PATH
```

Every entry should point inside `~/ros2_ws/install`. A stale `~/install/` directory
elsewhere in your home folder will silently shadow the workspace and run old code after
every rebuild.

## Run

The node normally starts as part of `robot_description`'s `bringup.launch.py`. To run it
on its own:

```bash
ros2 run rover_driver rover_driver_node
```

On startup the node collects gyroscope samples for five seconds to establish a bias
baseline. **Keep the robot completely still during this window.** It logs when
calibration is accepted:

```
[rover_driver]: Calibrating gz bias for 5s — keep robot STILL...
[rover_driver]: gz bias stable — calibration accepted: 10.925 counts  (std dev: 2.87)
[rover_driver]: Odometry active — ready for SLAM.
```

If the environment is electrically noisy, calibration extends beyond five seconds until
the standard deviation settles below the threshold, up to a fifteen-second ceiling.

Test motion with teleop in a second terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Press `x` repeatedly to reduce linear speed before driving; the default of 0.5 m/s is
considerably faster than is useful for mapping. Because of the low-speed floor, the
slowest the rover actually moves is 0.10 m/s.

## Tuning constants

All tunable values sit at the top of `rover_driver_node.py` with inline documentation.
The motor constants were measured in Sessions 013 and 014; the heading hold values are
the universal set arrived at in Session 011, validated across both hard floors and
carpet.

| Constant | Value | Purpose |
|---|---|---|
| `MAX_WHEEL_SPEED` | 0.956 m/s | Measured top wheel speed; clamp for wheel targets |
| `TRACK_WIDTH` | 0.174 m | Physical track width, used by the motor mix |
| `MIN_WHEEL_SPEED` | 0.10 m/s | Low-speed floor for wheel targets |
| `WHEEL_ZERO_EPS` | 0.005 m/s | Wheel targets below this are sent as zero |
| `CMD_TIMEOUT` | 0.5 s | Watchdog: a stale `/cmd_vel` is treated as zero |
| `USE_HEADING_HOLD` | False | Heading hold master switch |
| `LINEAR_RAMP_RATE` | 0.8 m/s² | Acceleration limit, prevents Jetson brownout |
| `ANGULAR_RAMP_RATE` | 2.0 rad/s² | Angular acceleration limit |
| `LINEAR_SCALE` | 0.01 | Raw encoder units to metres |
| `GZ_SCALE` | 0.001058 | Raw gyro counts × seconds to radians |
| `ZUPT_ALPHA` | 0.05 | Bias correction rate while stationary |
| `HEADING_KP` | 1.4 | Heading hold proportional gain |
| `HEADING_KD` | 0.5 | Heading hold derivative gain |
| `HEADING_DEADBAND` | 0.045 rad | Ignore corrections below about 2.6°, above the carpet vibration noise floor |
| `GZ_MAX_RATE` | 2.0 rad/s | Gyro spike clamp |
| `HEADING_MAX_CORRECTION` | 0.3 rad/s | Cap on injected heading correction |

`GZ_SCALE` can be re-derived with the Phase 2 hand-rotation procedure in
`scripts/calibrate_track_width.py` if the MFD board is ever replaced.

## Hardware notes

**Encoder wiring is swapped.** The `odl` and `odr` fields in the MFD's T:1001 feedback
packet are physically reversed on the board. The odometry code accounts for this; do
not "fix" it by swapping them again.

**Only the front wheels have encoders.** The rear wheels are passive and unencoded.
Linear displacement is computed from the front axle average, `(odl + odr) / 2`.

**`TRACK_WIDTH` is used only by the motor mix.** Odometry heading comes from the
gyroscope rather than from a differential of the two wheel encoders, so the odometry
does not depend on a wheel separation constant. The motor mix uses the physical
caliper-measured value, 0.174 m.

**Skid-steer scrub.** In-place rotation achieves about 43% of the rate predicted from
the wheel speeds, consistently across surfaces, because the six wheels scrub sideways.
That corresponds to an effective track width of about 0.405 m. The mix deliberately
uses the physical value; Nav2's closed loop absorbs the difference.
