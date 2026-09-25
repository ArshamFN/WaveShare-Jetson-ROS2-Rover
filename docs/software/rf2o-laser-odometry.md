# RF2O_laser_odometry

Range Flow-based 2D Odometry — estimates the robot's planar motion directly from
consecutive LiDAR scans, with no wheel encoders or IMU involved.

## Credit

This is third-party software, included here in full so the repository is self-contained
and reproducible.

Original algorithm and implementation by the MAPIR lab at the University of Málaga.
The ROS2 port used here is the [Adlink-ROS
fork](https://github.com/Adlink-ROS/rf2o_laser_odometry), `humble-devel` branch.

The algorithm is described in *Planar Odometry from a Radial Laser Scanner. A Range
Flow-based Approach* (ICRA 2016), by Jaimez, Monroy, and Gonzalez-Jimenez.

## Why this package is here

This rover originally derived its pose from gyrodometry — gyroscope integration for
heading, encoder average for linear displacement. That approach works, but its
calibration is conditional on the operating environment in two ways that cannot be tuned
away:

**Surface dependence.** Constants tuned on a hard floor do not transfer to carpet,
because the vibration noise floor the controller has to filter changes with the surface.
Session 011 documents this directly.

**Temperature dependence.** Gyroscope bias drifts as the electronics warm up, so a
calibration valid at boot is not necessarily valid thirty minutes into a mapping session.

The LiDAR is unaffected by either. It does not care what surface the robot is driving on
and it does not drift as the board heats. Its main weakness is very large spaces where
the surrounding geometry falls outside its range — not a realistic operating condition
for an indoor rover of this size.

RF2O is how that sensor gets turned into odometry. It aligns consecutive scans on their
range gradients to compute how the robot must have moved between them, and publishes the
result as standard `nav_msgs/Odometry`.

## Topics and transforms

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/scan` | `sensor_msgs/LaserScan` |
| Publishes | `/odom` | `nav_msgs/Odometry` |
| Broadcasts | `odom → base_link` | tf2 transform |

This node owns `odom → base_link`. The `rover_driver` node's TF broadcast is disabled
for exactly this reason — a TF edge can only have one publisher, and two nodes
broadcasting the same edge produces a transform that alternates between disagreeing
sources without raising any error.

## Build

```bash
cd ~/ros2_ws/src
git clone -b humble-devel https://github.com/Adlink-ROS/rf2o_laser_odometry.git
cd ~/ros2_ws
colcon build --symlink-install --packages-select rf2o_laser_odometry
source install/setup.bash
```

The package depends on `tf2`, `Boost`, and `Eigen3`, all of which come with a standard
ROS2 Humble desktop installation.

## Run

```bash
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
  -p laser_scan_topic:=/scan \
  -p odom_topic:=/odom \
  -p base_frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p 'init_pose_from_topic:=""' \
  -p publish_tf:=true \
  -p freq:=10.0
```

Start the LiDAR first — RF2O needs scans arriving before it can initialise.

Healthy output looks like this, repeating at roughly 10 Hz:

```
[CLaserOdometry2D]: [rf2o] execution time (ms): 34.7
[CLaserOdometry2D]: [rf2o] LASERodom = [0.483681 -0.019605 0.010547]
[CLaserOdometry2D]: BASEodom = [0.483681 -0.019605 0.010547]
```

## The `init_pose_from_topic` trap

**This parameter must be set empty. Leaving it at its default will hang the node
silently.**

`init_pose_from_topic` defaults to `/base_pose_ground_truth` — a topic that exists in
simulation, where a ground-truth pose is available for benchmarking the algorithm
against. Nothing on real hardware publishes it. Left at the default, the node waits
indefinitely for a pose that will never arrive and prints nothing but:

```
[WARN] [CLaserOdometry2DNode]: Waiting for laser_scans....
```

That message is deeply misleading, because the scans are arriving perfectly. There is no
error, no exception, no crash — even at `--log-level debug`. It is easy to lose an hour
to this chasing QoS mismatches and TF timing that were never the problem.

Note the nested quoting in the run command above. A bare `''` is stripped by bash before
rcl ever sees it, producing:

```
[ERROR] [rcl]: Failed to parse global arguments
Couldn't parse parameter override rule: '-p init_pose_from_topic:='
```

The quotes have to survive the shell, so `-p 'init_pose_from_topic:=""'` is the form
that works.

## Verifying it works

Before trusting RF2O in a mapping run, verify it in isolation with only the LiDAR and
RF2O running — no driver, no SLAM.

Confirm it is publishing:

```bash
ros2 topic hz /odom
```

Confirm the transform resolves:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

Then push the robot by hand while watching the translation values. They should track the
motion and reverse sign correctly when you push it back. That position estimate comes
entirely from LiDAR scans — no encoders, no gyroscope.

## Known limitations

**Feature-poor geometry.** RF2O aligns consecutive scans against each other. In a long
blank corridor where successive scans look nearly identical, there is little to align
against and the estimate degrades. Rooms with corners, furniture, and varied wall
geometry are favourable; empty hallways are not.

**Compute headroom.** On the Jetson Orin Nano Super, execution time runs roughly 28–39 ms
per scan against a 100 ms budget at 10 Hz. That is comfortable, but noticeably slower
than the sub-millisecond figures quoted for desktop hardware in the original paper. Under
the full stack — SLAM, driver, teleop, and RViz all competing for CPU — the margin is
thinner than it appears. If execution time approaches 100 ms, the node will begin
skipping scans.

**Scan-to-scan, not absolute.** RF2O is odometry, not localisation. It accumulates drift
like any incremental method. What it avoids is drift driven by wheel slip, surface
friction, and thermal bias — which is what made it a better foundation than gyrodometry
on this robot.
