# Session 013 — 2026-08-21: Nav2 Integration and First Autonomous Navigation

**Date:** 2026-08-21  
**Status:** ✅ Complete

---

## Goal

Bring Nav2 up on top of the RF2O + slam_toolbox stack from Session 012 and reach a
working `NavigateToPose` demo. This is the milestone the project has been aimed at
since Session 001, and Session 012 cleared the last known blocker by producing a
geometrically correct map.

---

## Context

Session 012 ended with the architecture below, plus a punch list of housekeeping items
— a proper RF2O launch file, a `min_laser_range` warning appearing on every launch, and
RF2O's source still uncommitted.

| Component | Publishes | Owns TF |
|---|---|---|
| `rplidar_ros` | `/scan` @ 10 Hz | — |
| `rf2o_laser_odometry` | `/odom` @ 10 Hz | **`odom → base_link`** |
| `rover_driver` | `/odom_wheel`, `/imu/gz` | none (TF disabled) |
| `slam_toolbox` | `/map` | `map → odom` |
| static publisher | — | `base_link → laser` |

What none of that punch list anticipated was that most of this session would be spent
auditing numbers the stack had been running on for months without anyone verifying
them.

---

## Discovery 1 — Three Dead Keys and a Missing Parameter in slam_toolbox

Mapping felt sluggish — the map in RViz lagged noticeably behind the rover. I assumed
the cause was in the computation pipeline: scan matching falling behind, or compute
headroom finally running out with RF2O and SLAM both running.

`ros2 topic hz /map` returned 0.100 Hz, dead steady, standard deviation under 2 ms.
That is a timer firing on a fixed interval, not an algorithm struggling. A struggling
matcher produces jitter.

`map_update_interval` was not in `slam_toolbox_params.yaml` at all. slam_toolbox had
been running on its built-in 10-second default for the entire project. Setting it to
1.0 fixed the lag immediately.

That prompted a full audit — dumping the live parameters and comparing them line by
line against the file:

```bash
ros2 param dump /slam_toolbox
```

Two more keys were doing nothing. The file had `distance_penalty: 0.3` and
`angle_penalty: 1.0`; the real parameter names are `distance_variance_penalty` and
`angle_variance_penalty`. Both had been silently ignored since whichever session
introduced them, with slam_toolbox running its defaults of 0.5 and 1.0 instead. The
`distance_penalty: 0.5 → 0.3` change recorded in the Session 010 log never took effect.

The root cause is that ROS2 accepts unknown parameters on a YAML load without warning
or error. A typo'd or renamed key does not fail — it quietly does nothing.

While auditing, `loop_search_maximum_distance` at 15.0 came under suspicion as an
oversized value against upstream's default of 3.0. Reverting it to 3.0 produced a
visibly worse map on the next run of the same route; restoring 15.0 recovered the
original quality on both a slow and a deliberately fast drive. It is load-bearing and
evidence-backed, not a stray value to be tidied away.

---

## Discovery 2 — The C1's Zero-Bearing Beam Does Not Face the Arrow

Nav2 needs the motor mix to speak SI units, so `MAX_WHEEL_SPEED` had to be measured. I
wrote a LiDAR wall-ranging script: fit a least-squares line to a narrow forward fan of
beams, take the perpendicular distance to that line, and differentiate it over a
sliding window. The line fit rather than a single forward beam makes the measurement
immune to heading drift, and reports the drift angle for free.

The script rejected every scan. A geometry dump explained why — with a wall
tape-measured at exactly 3.00 m ahead of the rover, the only coherent 3 m surface in
the entire scan was a 38° arc spanning **±180°**, not 0°.

The forward arc showed something flat at 0.113 m. My first assumption was
self-occlusion from the Jetson stack. Checking the slant-range profile against a
flat-plane model ruled that out:

| bearing | predicted `0.113 / cos θ` | observed |
|---|---|---|
| 5° | 0.113 | 0.114 |
| 15° | 0.117 | 0.113 |
| 35° | 0.138 | 0.135 |
| 45° | 0.160 | 0.156 |
| 55° | 0.197 | 0.192 |

Agreement within 5 mm out to 55°. That is a flat wall 11.3 cm behind the rover, not the
Jetson stack — a textbook `1/cos(θ)` curve, which mounting hardware would not produce.

The unit is mounted the way its own documentation specifies — the directional arrow on
the C1's housing points forward along the rover's travel axis. What the scan data shows
is that the sensor's 0-bearing beam is not aligned with that arrow; it emits at the
opposite end of the sweep. Nothing in the mounting or the datasheet indicated this, and
nothing downstream had ever surfaced it.

The consequence is that `rover.urdf`'s `base_to_laser` joint, declared `rpy="0 0 0"`,
has been telling the TF tree the laser frame is aligned with `base_link` when it is
rotated 180° from it. That has been the case since the project's first URDF.

SLAM and RF2O were never affected, because every consumer of that transform was
consistently wrong together — the map came out self-consistent, just built on a laser
frame rotated 180° from what the URDF claimed. Nav2 would have exposed it on the first
goal by driving the rover backward along its planned path.

---

## Discovery 3 — TRACK_WIDTH Was Never Physical Geometry

Reading the motor mix while adding the unit conversion:

```python
left  = max(-1.0, min(1.0, linear - angular * 0.5))
right = max(-1.0, min(1.0, linear + angular * 0.5))
```

Two problems. `linear` arrives in m/s and is written straight into the motor command as
though it were a normalised duty value — there is no scale factor anywhere. And the
hardcoded `0.5` on angular is the differential term `ω·W/2`, which implies a track
width of 1.0 m.

Under teleop none of this mattered, because the operator presses `q`/`z` until the
speed feels right. Nav2 computes 0.22 m/s and means 0.22 m/s.

The file already had `TRACK_WIDTH = 0.08` from `calibrate_track_width.py`, and reusing
it would have been the path of least resistance. I didn't trust it — that number was
always a value that made the odometry math come out right, not a measurement of the
robot. A caliper across the front wheel contact patches gave 0.174 m, more than double.

The 0.08 was not wrong; it was the right value for a different job. That script *fits*
an effective track width from encoder-derived heading against a gyro reference, which
is the correct thing to do for the **inverse** path — heading computed from the encoder
differential — because wheel slip makes the effective value genuinely surface-dependent.
That is exactly why Sessions 007–008 kept having to recalibrate it.

The motor mix is the **forward** path: wheel speeds computed from a commanded angular
velocity. That needs physical geometry. The slip residual is absorbed downstream now by
RF2O and Nav2's closed loop, not by the constant. Using 0.08 here would have scaled
every Nav2 angular command by roughly 2.2×.

---

## Solution / What Was Done

**URDF corrected.** `base_to_laser` yaw set to `3.14159265`. The frame convention was
also finally written down: `base_link` sits at the chassis underside, 11.5 mm above the
floor, which is what makes the joint's existing `z=0.1685` consistent with the measured
180 mm floor-to-lidar height. That number had no recorded datum before this session.

**Constants measured, not assumed.**

| Constant | Before | After | How |
|---|---|---|---|
| `MAX_WHEEL_SPEED` | — (no conversion existed) | **0.956 m/s** | LiDAR wall ranging at full duty |
| `TRACK_WIDTH` | 0.08 m | **0.174 m** | Caliper, contact patch to contact patch |
| `acc_lim_x` (Nav2) | — | **0.648 m/s²** | Measured; commanded ramp is 0.8 and motors cannot follow it |
| `decel_lim_x` (Nav2) | — | **−0.699 m/s²** | Measured |
| footprint | — | **0.250 × 0.227 m** | Measured chassis |

Both conversions were validated independently. Commanding 0.5 m/s produced 0.5158 m/s
by plateau fit and 0.5068 m/s by raw distance-over-time — two different methods
agreeing, both within 3.2%. Commanding 0.5 rad/s produced 0.455 rad/s, a 9% shortfall
consistent with slip and nowhere near a factor-of-two error.

**Motor mix rewritten** to convert properly:

```python
v_left  = linear - angular * TRACK_WIDTH * 0.5
v_right = linear + angular * TRACK_WIDTH * 0.5
left  = max(-1.0, min(1.0, v_left  / MAX_WHEEL_SPEED))
right = max(-1.0, min(1.0, v_right / MAX_WHEEL_SPEED))
```

**Driver changes for Nav2.** Heading hold disabled behind a `USE_HEADING_HOLD` switch —
Nav2's controller server runs its own closed loop, and two controllers driving the same
actuator fight each other. Added a `cmd_vel` watchdog (`CMD_TIMEOUT = 0.5 s`) so a dead
controller server can't leave the rover running on its last command, and a
`/battery_voltage` publisher reading the MFD's `v` field, warning at 10.5 V and critical
at 9.6 V.

**slam_toolbox config fixed.** `map_update_interval: 1.0` added, penalty keys renamed to
their real names, `scan_queue_size` raised from 1 to 20 and `transform_timeout` to 1.0 —
the queue size of 1 was causing the tf2 message filter to evict every scan before the
transform resolved, which is what blocked slam_toolbox from ever publishing `map → odom`
once Nav2 bringup started.

**Nav2 configured.** Architecture A — slam_toolbox stays live and owns `map → odom`, no
AMCL, no map_server. Regulated Pure Pursuit as the controller rather than DWB: fewer
parameters and more forgiving of the measured 9% angular slip.

**Two calibration scripts written.** `measure_max_speed.py` (LiDAR wall ranging) and
`drive_pulse.py` / `turn_pulse.py` — dedicated command publishers that wait for a
confirmed subscriber before starting the clock, because `ros2 topic pub` spends 1–2.7 s
on DDS discovery and silently consumes a short `timeout` window.

---

## Result

Nav2 planned and executed autonomous navigation to a fixed goal pose from four
different starting positions inside the mapped room. Each run took a sensible path and
arrived. This is the milestone the project has been working toward since Session 001.

The path planning was better than expected. The rover routed through genuinely narrow
gaps without clipping either side, which is direct evidence that the measured footprint
and the costmap inflation settings are correct — a wrong footprint shows up immediately
as either phantom collisions or scraped furniture, and neither happened.

A fifth run aborted. That one was launched from the room's doorway, deliberately
showing the rover a large area it had never seen, with only part of the known room in
view. It moved a little, then gave up after 15 recovery attempts and 64 seconds,
reporting `distance_remaining: 0.0` with the pose frozen to four decimal places.

The cause is the architecture, not a tuning problem. slam_toolbox was launched once and
built a map of what the rover could see from that starting position. Moving the rover
into an area outside that map leaves it with nothing to localize against — RF2O is a
frame-to-frame scan matcher with no relocalization capability, and slam_toolbox had no
prior geometry there to match into. The rover did not know where it was, so
`distance_remaining: 0.0` reflects a pose estimate that had come loose from reality
rather than an arrival.

This is the known limitation of Architecture A, and it is exactly what AMCL against a
saved map exists to solve.

---

## Lessons Learned

**ROS2 silently accepts unknown YAML parameters.** A typo'd or renamed key does not
error — the node runs on its built-in default. Five separate config values in this
project turned out to be doing nothing. The only reliable check is `ros2 param dump`
compared against the file, not reading the YAML and assuming it took effect.

**Slip-fitted effective constants are not physical constants.** `TRACK_WIDTH = 0.08`
was correct for the inverse heading-from-encoders path and silently wrong for the
forward motor-mix path. The same symbol name does not guarantee the same physical
quantity in different parts of a codebase.

**URDF errors hide indefinitely when every consumer is wrong the same way.** A
180°-reversed lidar mount produced good-looking, self-consistent maps for twelve
sessions because nothing ever compared the map's frame against ground truth.

**Fixing one bug exposes the next.** The missing unit conversion and the wrong track
width were errors in opposite directions, each partially masking the other. Correcting
the units is what made the track-width problem visible. Bugs in this codebase have not
been failing independently, which means fixing one in isolation and declaring victory
is not safe.

**LiDAR wall-ranging is a strong calibration technique for this platform.** Fit a line
to a narrow forward fan — ±10°, not wider, since a wide fan catches unrelated geometry
and fails the flatness check — and differentiate the perpendicular distance. It doesn't
depend on RF2O or the SLAM map, both of which are exactly what degrade at the speeds
being characterised.

---

## Next Session Goal

**AMCL against a saved map (Architecture B).** This is the direct fix for the fifth-run
abort. The current setup only navigates within one continuous live-SLAM session,
starting from wherever the rover booted. Loading a saved map and localizing into it
with a particle filter is what allows the rover to be placed at an arbitrary point in a
previously-mapped space and work out where it is.

**Characterise the rotation deadband.** Separate from the abort, but real. Driving the
rover under teleop makes it clear that rotation needs meaningfully more duty to break
loose than straight-line motion does — at least 1.3× — which the current Nav2 config
does not account for. `rotate_to_heading_angular_vel: 0.6` works out to roughly 5.5%
duty through the corrected motor mix, well under what actually moves the rover. This
will surface as sluggish or missed final heading corrections once localization is
sorted. A duty sweep on both axes to find the real minimum-moving values, and deadband
compensation in the mix, is the fix.

**Commit the repo debt.** RF2O source to `src/ROS2/rf2o_laser_odometry/` — only a README
is committed, so the repo does not clone-and-build. Map artifacts and the Nav2
params/launch files alongside it.

**Measure RF2O compute headroom under full load.** 24–46 ms against a 100 ms budget was
measured in isolation, not with Nav2's controller and costmaps also running.
