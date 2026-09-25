# rover_driver

A ROS2 Python node that bridges the standard `/cmd_vel` topic to the Waveshare UGV02
Multi-Functional Driver (MFD) board's JSON-over-serial protocol, and publishes
wheel-derived odometry alongside raw gyroscope data.

## What this node does

Every ROS2 navigation tool — teleop, Nav2, RViz — speaks the same language for motion:
a `geometry_msgs/Twist` message on `/cmd_vel`, expressing "drive forward at X m/s while
rotating at Z rad/s". The MFD board does not understand that. It expects JSON commands
over a serial port specifying left and right wheel speeds directly.

This node is the translator between those two worlds. It subscribes to `/cmd_vel`,
applies skid-steer kinematics to convert linear and angular velocity into differential
wheel speeds, and writes the resulting JSON to `/dev/rover`.

It also does three things beyond simple translation:

**Velocity ramping.** Commanding all four motors from a standstill to full speed draws a
current spike large enough to sag the battery below the Jetson's minimum operating
voltage, triggering a protective shutdown. The node limits acceleration to 0.8 m/s²
linear and 2.0 rad/s² angular, which eliminates the shutdown and smooths motion as a
side effect.

**Heading hold.** During straight-line travel, the node locks onto the current heading
and applies a PD correction to any drift from it. This runs only when the commanded
angular velocity is zero and the robot is moving, and releases the moment a turn is
commanded.

**Zero Velocity Update (ZUPT).** Gyroscope bias drifts with temperature over a long
session. Whenever both encoders report near-zero movement for a full second, the robot
is stationary and any gyro reading is pure bias — so the node nudges its bias estimate
toward the current reading. This runs continuously throughout a session rather than only
at startup.

## What this node does not do

**It does not broadcast `odom → base_link`.** As of Session 012, that transform is owned
by `rf2o_laser_odometry`, which derives the robot's pose from LiDAR scan matching rather
than from wheel encoders and the gyroscope.

The reason for this is worth understanding. Gyro-based odometry works, but its
calibration is conditional: constants tuned on a hard floor do not transfer to carpet
because the vibration noise floor changes with the surface, and gyroscope bias drifts as
the electronics warm up. The LiDAR is unaffected by either. Rather than continue tuning
around conditions that keep changing, pose estimation was moved to the sensor that
doesn't care about them.

A TF edge can only have one publisher. Two nodes broadcasting `odom → base_link`
produces a transform that alternates between disagreeing sources — which corrupts
slam_toolbox's motion prior without raising any error at all. The TF broadcast block in
`rover_driver_node.py` is therefore commented out rather than deleted, so the change is
visible and reversible.

## Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Subscribes | `/cmd_vel` | `geometry_msgs/Twist` | Standard ROS2 motion command |
| Publishes | `/odom_wheel` | `nav_msgs/Odometry` | Wheel + gyro odometry, retained for comparison against RF2O |
| Publishes | `/imu/gz` | `std_msgs/Float32` | Raw gyroscope Z-axis, for calibration tools and future EKF fusion |

Note that the odometry topic is `/odom_wheel`, not `/odom`. `/odom` belongs to
`rf2o_laser_odometry`. Publishing both to the same topic name would put two nodes in
conflict over it.

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

You also need `pyserial`:

```bash
pip3 install pyserial
```

## Build

Copy the package into your ROS2 workspace and build it:

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

```bash
ros2 run rover_driver rover_driver_node
```

On startup the node collects gyroscope samples for five seconds to establish a bias
baseline. **Keep the robot completely still during this window.** It will log when
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

Press `x` repeatedly to reduce linear speed before driving — the default of 0.5 m/s is
considerably faster than is useful for mapping.

## Tuning constants

All tunable values sit at the top of `rover_driver_node.py` with inline documentation.
The current values are the universal set arrived at in Session 011, validated across
both hard floors and carpet:

| Constant | Value | Purpose |
|---|---|---|
| `LINEAR_SCALE` | 0.01 | Raw encoder units → metres |
| `GZ_SCALE` | 0.001058 | Raw gyro counts × seconds → radians |
| `HEADING_KP` | 1.4 | Heading hold proportional gain |
| `HEADING_KD` | 0.5 | Heading hold derivative gain |
| `HEADING_DEADBAND` | 0.045 rad | Ignore corrections below ~2.6°, above the carpet vibration noise floor |
| `GZ_MAX_RATE` | 2.0 rad/s | Gyro spike clamp |
| `HEADING_MAX_CORRECTION` | 0.3 rad/s | Cap on injected correction |
| `LINEAR_RAMP_RATE` | 0.8 m/s² | Acceleration limit, prevents Jetson brownout |
| `ANGULAR_RAMP_RATE` | 2.0 rad/s² | Angular acceleration limit |
| `ZUPT_ALPHA` | 0.05 | Bias correction rate while stationary |

`GZ_SCALE` can be re-derived with the Phase 2 hand-rotation procedure in
`scripts/calibrate_track_width.py` if the MFD board is ever replaced.

## Hardware notes

**Encoder wiring is swapped.** The `odl` and `odr` fields in the MFD's T:1001 feedback
packet are physically reversed on the board. The odometry code accounts for this — do
not "fix" it by swapping them again.

**Only the front wheels have encoders.** The rear wheels are passive and unencoded.
Linear displacement is computed from the front axle average, `(odl + odr) / 2`.

**There is no `TRACK_WIDTH`.** Heading comes from the gyroscope rather than from a
differential of the two wheel encoders, which removes the need for a wheel separation
constant entirely. Session 007 spent a full session failing to calibrate that constant
before this approach replaced it.
