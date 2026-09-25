# Session 014 — 2026-09-24: Stack Audit, RF2O Startup Race, and the Wheel-Speed Floor

**Date:** 2026-09-24  
**Status:** ✅ Complete

---

## Goal

Make in-place rotation work under Nav2. Session 013 ended with four clean autonomous
runs, but whenever the planner asked the rover to turn on the spot, it struggled or
stalled. Frontier exploration, the next major milestone, is built out of exactly that
motion: drive to a frontier, rotate to face the next one, repeat. Until the rover could
rotate reliably on command, nothing built on top of Nav2 would work.

---

## Context

The session opened on the Session 013 stack:

| Component | Publishes | Owns TF |
|---|---|---|
| `rplidar_ros` | `/scan` @ 10 Hz | none |
| `rf2o_laser_odometry` | `/odom` @ 10 Hz | **`odom → base_link`** |
| `rover_driver` | `/odom_wheel`, `/imu/gz` | none (TF disabled) |
| `slam_toolbox` | `/map` | `map → odom` |
| `robot_state_publisher` | none | `base_link → laser` |

Nav2 ran Regulated Pure Pursuit on the live map, with the constants measured in
Session 013:

| Constant | Value |
|---|---|
| `MAX_WHEEL_SPEED` | 0.956 m/s |
| `TRACK_WIDTH` | 0.174 m |
| `rotate_to_heading_angular_vel` | 0.6 rad/s |

My first attempt was the obvious one. From driving under teleop, rotation felt like it
needed about 1.5 times the effort of straight-line motion to break loose, so I raised
`rotate_to_heading_angular_vel` from 0.6 to 3.8 rad/s, and then to 6.32. The first
helped somewhat. The second changed nothing about the turning, and the live map
corrupted during the run. I reverted to 0.6. Raising a velocity limit was a workaround
that never touched the cause, and I set the project aside for about three weeks.

When I came back, before touching the rotation problem again, I wanted to know that the
system I was about to change was the system I thought I had.

---

## Discovery 1 — The Deployed Stack Didn't Match My Session 013 Record

I expected a formality. I audited the Jetson against my Session 013 log: an inventory
of every file the stack depends on, a build from a fresh clone of the GitHub
repository, and a live `ros2 param dump` of every node compared against its YAML.

Three things did not hold up.

**The repository could not rebuild the robot.** GitHub `main` held the pre-Session 013
code: the old motor mix, `rpy="0 0 0"` on the laser joint, and none of the bringup
launch file, the RF2O and Nav2 parameter files, or the RF2O and rplidar sources. A
fresh clone failed to build at all, because the driver package directory was named
`rover-driver`, its `resource/` marker was missing, and the node sat at the package
root instead of inside a Python module. The only working copy of the stack was the
Jetson's SD card. I backed it up before changing anything else.

**Two Session 013 changes had never been deployed.** The `/battery_voltage` publisher
did not exist in the running driver, and the slam_toolbox file still contained the dead
keys `distance_penalty` and `angle_penalty`. My Session 013 log records both as done.
The parameter dump confirmed slam_toolbox was running its defaults:
`distance_variance_penalty` 0.5 and `angle_variance_penalty` 1.0.

For the penalty keys I deleted the dead lines rather than renaming them to the values I
had originally intended. Every good map this project has produced, including the
Session 013 loop-closure A/B test, was built on those defaults. Renaming the keys now
would have introduced an untested variable in the middle of a rotation investigation.

**The battery publisher needed rebuilding.** One 3S pack feeds both the Jetson and the
motors, so a voltage sag under load can brown out the computer mid-run, which is exactly
what the velocity ramp exists to prevent. Reading raw feedback from the board confirmed
that the `v` field of the T:1001 message is volts times 100 (`"v":1224` is 12.24 V),
and that the board streams T:1001 at about 20 Hz without being asked. The driver now
publishes `/battery_voltage` on every feedback message, warns at 10.5 V, and logs a
critical error at 9.6 V, both throttled to once every 30 seconds. It read about 12.2 V
at roughly 12 Hz on the first run.

---

## Discovery 2 — RF2O Could Initialize With the Laser Facing Backwards

While verifying the battery publisher, RF2O logged an error at startup that I had not
seen before:

```
[rf2o_laser_odometry]: "base_link" passed to lookupTransform argument target_frame does not exist.
```

It was followed by an initial laser pose of `LASERodom = [0.000000 0.000000 0.000000]`.
The last value is the laser's yaw relative to `base_link`. Since Session 013 fixed the
URDF, it should read `3.141593`. I rolled the driver back to its unmodified version and
launched again: the same error and the same zero yaw appeared, so the battery change
was not the cause. Across the first four launches I checked, it happened twice.

Reading the RF2O source explained it. `setLaserPoseFromTf()` looks up the
`base_link → laser` transform exactly once, on the first scan the node receives. The
lookup has no timeout and no retry. If the transform is not in the buffer yet, it
throws, the exception is logged, and execution continues with a default-constructed
transform, whose quaternion is identity. The caller ignores the function's return
value, and `first_laser_scan` is cleared unconditionally, so the lookup is never
attempted again:

```cpp
else
{
  setLaserPoseFromTf();
  rf2o_ref.init(last_scan, initial_robot_pose.pose.pose);
  rf2o_ref.first_laser_scan = false;
}
```

The TF listener is created in the node's constructor, moments before the scan
subscription. The lidar is already streaming at 10 Hz by the time RF2O starts, so the
first scan routinely arrives before the listener has finished discovery and received
`/tf_static` from `robot_state_publisher`. In Session 013 I measured DDS discovery
taking 1 to 2.7 seconds. The launch file's 3-second delay before RF2O does not help,
because the race is between RF2O's own listener and RF2O's own first scan, and both
start at the same moment.

The consequence is worse than a logged error. RF2O then believes the lidar faces
forward when it actually faces backward. Rotation comes out correct, because the lidar
sits over the chassis centre and a yaw rate is the same in both frames. Translation
comes out negated: drive forward and `/odom` reports the rover reversing. slam_toolbox
takes that as its motion prior for every scan match.

This bug was invisible until Session 013. Before that, the URDF declared the laser
joint with `rpy="0 0 0"`, so a failed lookup and a successful one produced the same
identity transform. Fixing the URDF turned a harmless race into a sign flip. It is also
a plausible cause of the map that corrupted at the start of this session, although I
can't prove that retroactively.

---

## Discovery 3 — The Motor Board Already Runs Closed-Loop Speed Control, With a Floor

With the stack verified, I returned to rotation.

My first plan was a duty sweep: step the rotation command up until the rover breaks
loose, record that threshold, and add it as a deadband offset in the motor mix. I
dropped it before running it. A threshold measured on one floor would be wrong on the
next, and this rover has to work on every surface in the house.

My second idea was to close the loop myself: keep increasing the output until the
measured wheel speed reaches the target. Thinking it through, that design has three
problems. An integrator starting from zero is sluggish, so the rover sits still while
Nav2 waits for motion. By the time static friction breaks, the integrator has wound
past what kinetic friction needs, so the rover lurches. And an integrator on a stalled
wheel keeps raising output exactly when motor current is highest, which on a shared
battery is a direct path to a brownout.

It also raised a question I should have asked earlier: what does the board do with the
numbers I send it? Hands-on driving had suggested the board regulates speed itself,
since it held pace under load, but my driver treated the `L` and `R` values in the T:1
command as normalized duty and divided every wheel speed by `MAX_WHEEL_SPEED` to get
there. Waveshare's documentation does not settle it. The UGV02 wiki describes T:1 as
closed-loop speed control in m/s, while the product manual describes a value of 0.5 as
100% PWM, which is open loop. My own Session 013 data did not fit the manual: a command
of 0.523 would have been full power under that scaling, and I had measured 0.516 m/s.

So I tested it directly. Closed and open loop differ when the load changes: a speed
controller holds the commanded speed with the wheels in the air or on the floor, while
an open-loop motor spins much faster unloaded. I wrote `t1_loop_test.py`, which owns the
serial port, sends T:1 commands at 20 Hz, and averages the wheel speeds the board
reports in its T:1001 feedback. With the chassis on a box and all six wheels in the
air:

| Command (m/s) | Left (m/s) | Right (m/s) |
|---|---|---|
| 0.10 | 0.099 | 0.096 |
| 0.20 | 0.212 | 0.197 |
| 0.30 | 0.359 | 0.306 |
| 0.50 | 0.490 | 0.505 |
| 0.70 | 0.727 | 0.729 |
| 1.00 | 0.998 | 0.998 |

On the floor, a command of 0.20 read 0.214 and 0.218, the same as in the air. **T:1 is
closed-loop wheel speed in m/s.** The board was already doing what I had been about to
build.

That exposed a unit error in my driver. Dividing by 0.956 inflated every wheel target
by 4.6%, which is why the Session 013 speed validation came out 1.4% and 3.2% fast.

The same test showed something else: the standard deviation of the reported speeds was
0.06 to 0.09 m/s at every level. With 20 PPR encoders, that is the resolution the
board's controller has to work with.

To see what that means for rotation, I wrote `t1_rotate_test.py`. It commands the
wheels in opposite directions, `L = -v` and `R = +v`, and reads both the wheel speeds
and the gyro's `gz` rate, so it measures what the chassis actually does as well as what
the wheels do. I ran it on three surfaces: the easiest in the house, the harshest, and
one in between. On the harshest:

| Wheel target (m/s) | Left (m/s) | Right (m/s) | Chassis rate (rad/s) | Ideal rate (rad/s) | Actual / ideal |
|---|---|---|---|---|---|
| 0.03 | -0.038 | 0.030 | 0.12 | 0.34 | 0.36 |
| 0.05 | -0.031 | 0.022 | 0.15 | 0.57 | 0.26 |
| 0.08 | -0.065 | 0.101 | 0.41 | 0.92 | 0.45 |
| 0.10 | -0.119 | 0.111 | 0.46 | 1.15 | 0.40 |
| 0.15 | -0.154 | 0.150 | 0.71 | 1.72 | 0.41 |
| 0.20 | -0.168 | 0.196 | 0.96 | 2.30 | 0.42 |
| 0.25 | -0.243 | 0.218 | 1.27 | 2.87 | 0.44 |

The pattern held on all three surfaces. At 0.03 and 0.05 m/s the wheels stalled or ran
erratically; on one surface a 0.05 target produced 70% of the command on one side and
151% on the other. From 0.08 m/s up, both wheels always turned in the right direction
and the chassis rate climbed steadily. Below about 0.08 m/s, the controller cannot
regulate a wheel.

That was the whole rotation problem. At `rotate_to_heading_angular_vel: 0.6`, the motor
mix asks each wheel for 0.6 × 0.174 / 2 = 0.052 m/s, below the floor. The rover never
lacked power. It was asked for a speed its controller cannot hold. It also explains why
raising the rotation rate to 3.8 rad/s helped: that puts the wheels well above the
floor.

The last column was a surprise. The chassis turned at 40% to 47% of the rate the wheel
speeds predict, and the ratio was nearly identical on all three surfaces. I checked it
by eye: a 0.15 m/s spin held for 8.7 seconds should produce one full revolution if the
ratio is 0.43, and two if the gyro scale were off by a factor of two. The rover turned
slightly more than 360°. The gyro is right; this is six wheels scrubbing sideways,
which gives this chassis an effective track width of about 0.405 m against its physical
0.174 m. My Session 013 figure of 91% rotation efficiency does not survive this
measurement and should be treated as superseded.

---

## Solution / What Was Done

### slam_toolbox dead keys

Deleted `distance_penalty` and `angle_penalty` from `slam_toolbox_params.yaml`.
slam_toolbox continues on the defaults every good map was built with.

### Battery voltage topic

`rover_driver` publishes `/battery_voltage` (`std_msgs/Float32`, volts) from the
T:1001 `v` field. The parse is guarded, so a malformed or missing field skips one
reading instead of stopping the serial path:

```python
v_raw = data.get('v')
if isinstance(v_raw, (int, float)) and not isinstance(v_raw, bool):
    volts = v_raw / 100.0
```

### RF2O startup race

One line in `CLaserOdometry2DNode.cpp`:

```diff
-      setLaserPoseFromTf();
+      if (!setLaserPoseFromTf()) return;
```

A failed lookup now skips initialization and leaves `first_laser_scan` set, so the
lookup is retried on every following scan until the transform exists. It also changes
the failure mode: if `robot_state_publisher` never comes up, RF2O refuses to publish
`/odom` rather than publishing wrong odometry.

I validated it by forcing the race instead of waiting for it. With the lidar running, I
started RF2O on its own, with no `robot_state_publisher`: it logged 67 failed lookups
and published nothing on `/odom`. Starting `robot_state_publisher` ended the errors, RF2O
initialized with yaw `3.141593`, and `/odom` came up at about 9.3 Hz. Four normal
bringups afterwards each logged exactly one failed lookup on the first scan and then
initialized correctly. Unpatched, each of those first failures would have locked in the
wrong orientation, which puts the race at 6 of the 8 launches I made that day. Session
013's successful Nav2 runs mean that particular launch happened to initialize
correctly.

Finally I checked the symptom itself: four fresh launches, each showing the correct
initial yaw, and on each one a hand push of about half a metre forward made `/odom` x
increase, reaching 0.546 m on the first.

The fix lives on my fork of the Adlink RF2O repository, `ArshamFN/rf2o_laser_odometry`,
branch `humble-startup-race-fix`, commit `82a99d4`.

### Motor mix

The mix is now one function. Wheel targets are sent in m/s, clamped to
`MAX_WHEEL_SPEED`. If the faster wheel's target is non-zero but under the floor, both
wheels are scaled up by the same factor until it reaches 0.10 m/s. Scaling the pair
rather than each wheel keeps their ratio, so the turn curvature Pure Pursuit asked for
is preserved and only the speed rises. Targets under 5 mm/s become an exact zero, so
stop still means stop. While rewriting it I noticed that a NaN in `/cmd_vel` would have
passed the old `min`/`max` clamp as full speed, so non-finite input now returns zero.

```python
def mix_to_wheel_speeds(linear, angular):
    if not (math.isfinite(linear) and math.isfinite(angular)):
        return 0.0, 0.0
    v_left  = linear - angular * TRACK_WIDTH * 0.5
    v_right = linear + angular * TRACK_WIDTH * 0.5
    peak = max(abs(v_left), abs(v_right))
    if peak < WHEEL_ZERO_EPS:
        return 0.0, 0.0
    if peak < MIN_WHEEL_SPEED:
        scale = MIN_WHEEL_SPEED / peak
        v_left  *= scale
        v_right *= scale
    left  = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, v_left))
    right = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, v_right))
    return left, right
```

I chose 0.10 m/s over the measured 0.08 for margin: 0.08 was the first level that
tracked, not a comfortable one. Before deploying it I tested the function against 19
cases covering the floor, the zero threshold, clamping, sign preservation, curvature
preservation, and non-finite input.

One known limitation: in a tight arc the inner wheel can sit below the floor while the
outer one is above it. The inner wheel then won't track, so the arc comes out slightly
tighter than commanded. Nav2's feedback corrects it.

I kept `TRACK_WIDTH` at the physical 0.174 m. The 0.43 scrub ratio makes an effective
track of about 0.405 m defensible, but I measured it only for in-place rotation, and
changing the floor and the kinematics in the same session would have left me unable to
attribute the result to either. It is a separate experiment.

| Constant | Before | After | Rationale |
|---|---|---|---|
| Wheel command units | normalized duty (÷ 0.956) | **m/s** | T:1 is closed-loop wheel speed |
| `MAX_WHEEL_SPEED` | divisor | **clamp only (0.956 m/s)** | No longer a unit conversion |
| `MIN_WHEEL_SPEED` | none | **0.10 m/s** | Regulation floor of about 0.08 m/s on all three surfaces, plus margin |
| `WHEEL_ZERO_EPS` | none | **0.005 m/s** | Stop means stop |
| Non-finite `/cmd_vel` | full speed | **zero** | Safety |
| `rotate_to_heading_angular_vel` | 0.6 | 0.6 | Unchanged; the floor makes it work |

### The repository and the workspace are now the same thing

Discovery 1 happened because the code on the robot and the code in the repository were
separate copies that drifted apart. I fixed that structurally rather than by copying
files: `~/ros2_ws` on the Jetson is now itself a git clone of the repository.

The repository is laid out as a colcon workspace:

```
.gitignore  LICENSE  README.md  rover.repos
cad/  docs/  images/
maps/                      saved slam_toolbox maps
scripts/                   calibration and test scripts
tools/launchers/           desktop launchers for bringup, teleop, and Nav2
src/rover_driver/
src/robot_description/
src/rf2o_laser_odometry/   not tracked; pulled via rover.repos
src/rplidar_ros/           not tracked; pulled via rover.repos
```

Third-party packages are pinned to exact commits in `rover.repos`: my RF2O fork at
`82a99d4`, and Slamtec's `rplidar_ros` at `24cc9b6`. A fresh machine needs
`git clone`, `vcs import src < rover.repos`, and `colcon build --symlink-install`. The
old ROS1 `.gitignore` was replaced with root-anchored rules; a plain `build/` pattern
would also have matched `images/build/` and silently ignored any photo added there
later. The package metadata was corrected: maintainer, license, the misspelled
`nav-msgs` dependency, and the missing `python3-serial`.

I proved the result the way someone else would use it: a fresh clone into a scratch
directory, `vcs import`, and a build in a clean shell environment so nothing from the
existing workspace could leak in. Every package and the running driver process resolved
to the clone, the stack came up with the correct RF2O initialization and normal topic
rates, and the code was byte-identical to what had been running on the robot apart from
the metadata changes. Only then did I rename the old workspace aside as a rollback and
clone the repository into `~/ros2_ws`. The scripts, maps, and launcher scripts that had
lived in my home directory moved into the repository, and `~/bin` now links into
`tools/launchers/`, so the desktop icons work unchanged. I added a third icon for Nav2.

I also aligned the test-log index with each log's own title and date, which meant
renaming the Session 012 log file, whose name disagreed with the date in its title.

---

## Result

The rover now turns in place on command. Under teleop it creeps at 0.10 m/s instead of
stalling at low speed, rotates reliably at the default turn rate, and stops dead when
the command goes to zero. Under Nav2, a goal behind the rover produces a smooth in-place
rotation, then the drive and a complete final approach, with
`rotate_to_heading_angular_vel` still at 0.6.

RF2O initializes with the correct laser orientation on every launch, `/battery_voltage`
reports the pack voltage continuously, and a fresh clone of the repository builds in
about a minute and a half and runs the full stack.

| Component | Publishes | Owns TF |
|---|---|---|
| `rplidar_ros` | `/scan` @ 10 Hz | none |
| `rf2o_laser_odometry` | `/odom` @ 10 Hz | **`odom → base_link`** |
| `rover_driver` | `/odom_wheel`, `/imu/gz`, `/battery_voltage` @ ~12 Hz | none (TF disabled) |
| `slam_toolbox` | `/map` | `map → odom` |
| `robot_state_publisher` | none | `base_link → laser` |

Three minor issues are still open:

- `rover_driver` prints an `ExternalShutdownException` traceback on Ctrl+C, because it
  catches only `KeyboardInterrupt`. The zero-speed command is still sent on shutdown.
- RF2O occasionally aborts on shutdown (exit code -6). It leaves no process behind, and
  it happened before the patch as well.
- RF2O reported 1.5° of yaw drift over 79 seconds with the rover stationary. Not yet
  investigated.

---

## Lessons Learned

**A logged error that execution continues past is the most expensive kind of bug.**
RF2O logged its failure, ignored the return value, and ran on a wrong default for the
rest of every affected launch. It is the same class as Session 013's YAML keys: nothing
a person would notice, and output that is wrong in a way that looks plausible.

**Fixing one bug can arm another.** The race in RF2O existed from the day I integrated
it, and it was harmless because the URDF's laser orientation was wrong in the same way
as the fallback. Correcting the URDF in Session 013 is what made the race matter.

**Measure what the hardware does before designing compensation for it.** I nearly
built a speed controller on top of a board that already runs one, and my first plan
would have calibrated a duty threshold that was never the variable. One test with the
wheels in the air changed the whole design, and the result works on every surface
because it targets the controller's floor rather than a surface's friction.

**Force a failure instead of waiting for it.** A race that hits some launches cannot be
validated by relaunching until it seems fixed. Withholding `robot_state_publisher` made
the failure deterministic, and it proved both halves of the fix: RF2O waits while the
transform is missing, and initializes correctly once it arrives.

---

## Next Session Goal

**A handheld field pendant.** Driving the rover and watching the live map from a
handheld device over the network instead of a laptop, so the rover can be operated
anywhere in the house.

**Effective track width A/B.** With in-place rotation fixed, the next accuracy gain is
making commanded turn rates match actual ones. Switching the forward mix to the
effective 0.405 m track has to be measured on real Nav2 paths against the current
geometric value, since the scrub ratio was measured only for in-place rotation.

**Saved-map navigation with AMCL.** Navigating on a saved map with `map_server` and
AMCL, so the rover can start anywhere in a mapped space. This needs a `slam:=false`
launch argument first, so that slam_toolbox and AMCL do not both publish `map → odom`.
