# Session 007 — First Teleoperated SLAM Run & Odometry Calibration

**Date:** 2026-03-02  
**Status:** ⚠️ Partial — Linear scale calibrated, TRACK_WIDTH unresolved, SLAM map inaccurate on turns

---

## Goal

Run the full SLAM stack for the first time with a teleoperated robot and produce a
geometrically accurate map of the room. Before mapping could begin, wheel odometry
needed to be calibrated — specifically the linear scale factor and `TRACK_WIDTH`,
the parameter controlling how much rotation is computed from differential encoder deltas.

---

## What Was Accomplished

### 1. Linear Scale Calibration — Completed

The initial scale factor was `0.03125` (1/32), an estimate. A straight-line drive test
revealed odometry was overcounting by roughly 3×: a 1-metre drive reported ~3.19m.

I computed the correction from the Euclidean distance between before/after odom positions
(not just delta-X, since the robot wasn't aligned with the X axis at the time):

```
sqrt((5.076 - 5.573)² + (-0.478 - (-3.632))²) = 3.19m reported for 1.0m actual
new scale = 0.03125 / 3.19 ≈ 0.01
```

That's a clean 1/100 factor. Verification test (98cm actual):

```
sqrt(0.9879² + 0.1316²) = 0.9966m reported — 0.34% error
```

Linear scale factor `0.01` locked in.

### 2. First Coherent SLAM Map

With the corrected linear scale, SLAM Toolbox produced a single coherent structure for
the first time — a recognisable room outline rather than scattered fragments. This
confirmed the full pipeline was working: driver node → `/odom` → TF tree →
SLAM Toolbox → RViz.

### 3. TRACK_WIDTH Calibration — Attempted, Unresolved

The map looked correct on straight segments but broke on every turn: each turn caused
a new misaligned copy of the walls to appear, rotated relative to the original. The
cause is `TRACK_WIDTH`, which appears in the differential drive rotation formula:

```
delta_theta = (delta_r - delta_l) / TRACK_WIDTH
```

**Physical measurement attempt:** I first measured the wheel spacing with a ruler and
set `TRACK_WIDTH = 0.172m`. This produced a map where corners appeared ~120–130°
instead of 90° — the robot was undercounting rotation. At this point I realised the
physical wheel spacing is not the right measurement. `TRACK_WIDTH` in this formula
refers to the kinematic turning diameter: the diameter of the circle the wheels would
trace if the rover spun in place. That value is not the same as the physical distance
between wheels and cannot be measured directly with a ruler.

**L-shape visual test:** I switched to an empirical approach — drive forward ~1m along
a wall, turn 90° left using teleop, drive forward ~1m along the next wall, and judge
the corner angle in the SLAM map. An obtuse corner means `TRACK_WIDTH` is too large;
acute means too small. Values tested:

| TRACK_WIDTH | Outcome |
|---|---|
| `0.172` | Physical measurement. Corners ~120–130° — undercounting rotation. |
| `0.0682` | Calculated from a spin-test ratio. Over-compensated — turns too sharp. |
| `0.11` | Midpoint. Map still fragmented. |
| `0.08` | Best visual result — map looked room-shaped, corner close to 90°. |
| `0.075` | Small decrease from 0.08. Map quality degraded. |
| `0.24` | Based on RViz vs physical angle comparison. Still inaccurate. |

None produced a consistently accurate map. The core problem: every method derived the
correction from the same wheel encoders being calibrated, with no independent ground
truth for rotation angle. Hitting an exact 90° turn by eye with teleop also introduced
too much human error for any calibration ratio to be reliable.

### 4. Crab-Walking Identified

During floor testing I noticed the robot drifting sideways while maintaining its
heading. The UGV02 has 6 wheels — 4 driven (front and rear pairs) and 2 passive idler
wheels in the middle. Those middle wheels are wobbly and act as a lateral constraint,
pushing the chassis sideways during forward motion even when both driven sides report
equal encoder distances. This complicated all L-shape tests and contributed to
inconsistent map results.

### 5. SLAM Toolbox Parameters Tuned

In parallel with TRACK_WIDTH work I tightened scan matching in
`slam_toolbox_params.yaml`:

- `minimum_travel_heading`: `0.2` → `0.087` rad (triggers on 5° turns instead of 11.5°)
- `link_match_minimum_response_fine`: `0.1` → `0.45` (reject low-confidence matches)
- `minimum_time_interval: 0.5` added

---

## Files Modified

- `~/ros2_ws/src/rover_driver/rover_driver/rover_driver_node.py` — Linear scale factor
  `0.03125` → `0.01`. TRACK_WIDTH tested across the session: `0.172` → `0.0682` →
  `0.11` → `0.08` → `0.075` → `0.24`.
- `~/ros2_ws/src/robot_description/config/slam_toolbox_params.yaml` — Scan matching
  parameters tightened as above.

---

## Next Steps

Manual empirical calibration of `TRACK_WIDTH` is not viable on this platform — the
formula is too sensitive and there is no independent rotation reference.
In session 008, I will address this by writing a gyroscope-based automatic calibration script using the
`gz` field already present in every T:1001 feedback packet from the MFD board's
onboard IMU (QMI8658C).
The gyroscope provides a rotation measurement completely
independent of wheel encoders, making precise `TRACK_WIDTH` derivation possible
without any manual angle estimation.
