# Session 010 — Heading Hold, Velocity Ramp, ZUPT, PD Controller, and SLAM Tuning

**Date:** 2026-03-09  
**Status:** ✅ Complete

---

## Goal

Implement a software acceleration limiter to eliminate the Jetson protective shutdown
caused by hard-acceleration current spikes. Tune SLAM Toolbox parameters to reduce wall
smearing. Investigate and solve the heading drift problem on soft surfaces that was
causing map quality to degrade over long sessions.

---

## What Was Accomplished

1. Audited the odometry projection math and confirmed the midpoint approximation was
   already geometrically correct — heading drift was a physical problem, not a
   calculation bug
2. Implemented a forward-only heading hold PD controller that injects a corrective
   `angular.z` command during straight-line travel using the live gyroscope signal as
   both the error source and the derivative damping term
3. Implemented a software velocity ramp on both linear and angular axes, eliminating
   hard-acceleration current spikes and significantly improving motion smoothness
4. Tuned SLAM Toolbox parameters: full 10 Hz scan ingestion, looser match acceptance,
   reduced distance penalty, and an extended loop closure search radius appropriate for
   the mapped room size
5. Identified gyro integration drift as the primary map quality bottleneck over long
   sessions and implemented Zero Velocity Update (ZUPT) for continuous bias correction
   during stationary moments
6. Upgraded startup bias calibration from fixed-duration averaging to convergence-based
   standard deviation gating, with a 15-second hard ceiling and graceful fallback
7. Added a hard cap on heading correction output to prevent controller instability from
   causing runaway spins
8. Upgraded the heading hold from a P controller to a PD controller, eliminating
   oscillation without sacrificing correction strength
9. Produced the best map quality to date — three clean, tight walls over two full
   perimeter laps

---

## Discovery 1 — The drift problem was physical, not mathematical

The original hypothesis was that the linear displacement projection was using a stale
heading value — that when the rover drifted sideways on carpet, the position update was
projecting onto the wrong axis and smearing the map. Reading the source made it clear
that this was already handled:

```python
heading_mid  = self._theta + dtheta * 0.5
self._x     += d_linear * math.cos(heading_mid)
self._y     += d_linear * math.sin(heading_mid)
self._theta += dtheta
```

The midpoint approximation computes heading at the centre of the current timestep before
projecting displacement. There was no stale-heading bug. The odometry math was right.

The heading hold was still the correct intervention — but for a different reason. When
the rover physically wanders off-heading during a straight run, the LiDAR scans arrive
from slightly different angles on each tick. SLAM Toolbox must correlate each incoming
scan against the existing map — rotated scans correlate poorly, and walls smear. Keeping
the rover physically straight eliminates that scan rotation entirely. The fix was always
mechanical, not mathematical.

---

## Discovery 2 — Reverse travel is unstable under closed-loop heading control

The heading hold controller used `math.copysign(1.0, linear)` to flip the correction
sign for reverse travel, which seemed like an elegant way to handle both directions with
one controller. It was not.

Testing in reverse triggered runaway spinning. The gyro drifted slightly, the correction
fired, overcorrected, the error flipped direction, a larger correction fired back — and
within a few seconds the rover was spinning uncontrollably, drawing enough current to
kill the Jetson via voltage sag.

The passive rear wheels and forward-biased weight distribution make reverse travel
mechanically unstable under closed-loop heading control. The same KP that stabilises the
rover going forward becomes a diverging feedback loop in reverse. The fix was to disable
heading hold entirely for reverse by changing the activation condition from
`linear != 0.0` to `linear > 0.0`. Since all SLAM mapping is done driving forward, this
costs nothing in practice.

---

## Discovery 3 — Peak-to-peak is the wrong metric for sensor convergence

The startup bias calibration was upgraded to gate on noise level before accepting the
bias estimate — if the gyro samples were too spread out, calibration would extend up to
15 seconds rather than accepting a noisy result after the minimum 5 seconds. The
convergence threshold was set to 2.0 counts peak-to-peak.

On the first test run the node reported `noise p-p: 12.00` and timed out every time,
falling back to the best estimate with a warning. The map still started rotated. Yet the
bias estimate was numerically stable — the mean of 300 samples was consistent and
accurate.

Peak-to-peak is dominated by the single worst outlier in the entire sample window. In an
environment with motors, a PC, and a 3D printer nearby, there will always be occasional
spikes. A single spike in 300 samples blew up the range even though 299 were clean.

Switching the metric to standard deviation fixed the problem immediately. Standard
deviation measures the *typical* spread of samples rather than the worst case. With a
threshold of 3.0 counts, the calibration converged cleanly within the minimum 5-second
window on every subsequent startup:

```
[INFO] gz bias stable — calibration accepted: 11.352 counts  (std dev: 1.84)
```

---

## Discovery 4 — Pure P controllers overshoot; the fix is the D term, not a weaker P

After adding a 0.3 rad/s hard cap on the correction output, the heading hold was stable
but still exhibited left-right wobble during straight-line travel — the rover would
correct, slightly overshoot, correct back, overshoot again. Two obvious fixes existed:
lower KP to reduce correction strength, or widen the deadband to allow more drift
through. Both trade accuracy away to buy smoothness.

The actual problem was that a pure P controller has no awareness of whether the plant is
already moving toward the target heading. Adding a derivative term using the live
`gz_c * GZ_SCALE` angular rate signal gives the controller exactly that awareness. The
harder the rover is already rotating toward the target, the more the D term subtracts
from the correction — slowing the approach before it overshoots:

```python
angular_rate = gz_c * GZ_SCALE / dt
correction   = (HEADING_KP * error) - (HEADING_KD * angular_rate * copysign(1.0, linear))
```

With `HEADING_KD = 0.3`, oscillation stopped immediately. KP stayed at 1.6, the deadband
stayed at 1°, and no accuracy was surrendered. The controller became more aggressive and
snappy in feel — it corrects harder when there is genuine drift — but no longer
overshoots.

---

## ZUPT — Treating every pause as a calibration opportunity

The startup bias calibration already established the principle: when the rover is
stationary, any non-zero gyro reading is pure bias. ZUPT extends this to run
continuously throughout the session. Every time both encoder deltas are near zero for at
least one second, the bias estimate is nudged toward the current raw gyro reading via an
exponential moving average:

```python
if abs(d_odl) < ZUPT_ODO_THRESHOLD and abs(d_odr) < ZUPT_ODO_THRESHOLD:
    self._zupt_ticks += 1
    if self._zupt_ticks >= ZUPT_SETTLE_TICKS:
        self._gz_bias += ZUPT_ALPHA * (float(gz_raw) - self._gz_bias)
else:
    self._zupt_ticks = 0
```

The alpha of 0.05 is deliberately conservative — each qualifying tick moves the bias
estimate 5% toward the current reading, so the update is stable even if the rover is on
a gentle slope or the surface is slightly uneven. Over a multi-minute session with
multiple pauses, the cumulative effect is significant.

The improvement was visible in the maps. Before ZUPT, a 216-second full-loop session
produced a worse map than a 106-second partial session — more elapsed time meant more
accumulated drift with no correction mechanism. After ZUPT, longer sessions produced
cleaner maps because drift was being corrected at every pause.

---

## Velocity Ramp

The software acceleration limiter was straightforward: rather than passing `cmd_vel`
directly to the motors, `_ramp_linear` and `_ramp_angular` chase the commanded values at
a limited rate each tick:

```python
max_d_lin = LINEAR_RAMP_RATE * dt      # 0.8 m/s²
max_d_ang = ANGULAR_RAMP_RATE * dt     # 2.0 rad/s²

self._ramp_linear  += max(-max_d_lin, min(max_d_lin,  linear  - self._ramp_linear))
self._ramp_angular += max(-max_d_ang, min(max_d_ang,  angular - self._ramp_angular))
```

At 20 Hz, full speed from a standstill now takes approximately 0.5 seconds rather than
one tick. The effect was immediately noticeable — the rover's movement became smooth and
predictable, and the Jetson shutdown problem did not recur for the remainder of the
session. The ramp also improved SLAM scan correlation quality as a side effect, since
smoother motion reduces the angular jitter in consecutive scans.

---

## Map Quality Progression

**Attempt 1 — Fast driving, no full loop (106s)**

![Map attempt 1](../images/testing/session-010/session-010-map-attempt-1.png)

Significant smearing throughout. No loop closure. Too fast for SLAM to correlate scans
reliably.

---

**Attempt 2 — Slow driving, full loop, no ZUPT (216s)**

![Map attempt 2](../images/testing/session-010/session-010-map-attempt-2.png)

Worse than attempt 1 despite slower driving. More elapsed time meant more accumulated
gyro drift with no correction mechanism. By the time the rover returned to origin, the
scan pattern had rotated enough that SLAM could not recognise it as the same location.
Loop closure did not fire.

---

**Attempt 3 — With ZUPT, slow driving, two laps, pausing before turns (362s)**

![Map attempt 3](../images/testing/session-010/session-010-map-attempt-3.png)

Noticeably cleaner upper walls. ZUPT was correcting drift at every pause. The room shape
became recognisable for the first time. The bottom-right section remained messy — this is
a bed corner and shelf the rover could not fully navigate around.

---

**Attempt 4 — PD controller and convergence-based calibration, two laps (352s)**

![Map attempt 4](../images/testing/session-010/session-010-map-attempt-4.png)

Best result to date. Three walls are clean and tight. The map no longer drifts
rotationally during the session. The bottom-right furniture corner remains the only
unresolved region.

---

## Final Tuned Constants

**`rover_driver_node.py`:**

| Constant | Value | Notes |
|---|---|---|
| `BIAS_DURATION` | 5.0 s | Minimum calibration window |
| `BIAS_NOISE_THRESHOLD` | 3.0 counts std dev | Convergence gate |
| `BIAS_MAX_DURATION` | 15.0 s | Hard ceiling |
| `HEADING_KP` | 1.6 | Proportional gain |
| `HEADING_KD` | 0.3 | Derivative damping gain |
| `HEADING_SETTLE_THRESHOLD` | 0.05 rad/s | Post-turn gyro settle gate |
| `HEADING_MAX_CORRECTION` | 0.3 rad/s | Output clamp |
| `GZ_MAX_RATE` | 3.0 rad/s | Spike clamp |
| `HEADING_DEADBAND` | 0.017 rad (~1°) | Noise floor gate |
| `LINEAR_RAMP_RATE` | 0.8 m/s² | Acceleration limiter |
| `ANGULAR_RAMP_RATE` | 2.0 rad/s² | Turn acceleration limiter |
| `ZUPT_ODO_THRESHOLD` | 0.5 raw units | Stationary detection |
| `ZUPT_SETTLE_TICKS` | 20 ticks (~1s) | Settle window |
| `ZUPT_ALPHA` | 0.05 | EMA update rate |

**`slam_toolbox_params.yaml`:**

| Parameter | Before | After |
|---|---|---|
| `minimum_time_interval` | 0.5 | 0.0 |
| `link_match_minimum_response_fine` | 0.45 | 0.35 |
| `distance_penalty` | 0.5 | 0.3 |
| `loop_search_maximum_distance` | 3.0 | 15.0 |

---

## Lessons Learned

**Verify the math before assuming a calculation bug.** The heading drift on carpet looked
exactly like a projection bug. Reading the source carefully before writing any fix
revealed it wasn't — and changed the entire direction of the solution from a math patch
to a physical controller.

**Simple interventions that solve multiple problems are almost always the right choice.**
The velocity ramp was designed to fix battery sag. It also made motion smoother, which
directly improved scan correlation quality for SLAM. One change, two improvements.

**When a sensor convergence check fails in a noisy environment, question the metric
first.** Peak-to-peak is practically useless for any signal that has occasional spikes.
Standard deviation is almost always the right choice for sensor quality gating.

**A pure P controller that overshoots should get a D term, not a smaller P.** Lowering
KP or widening the deadband both reduce accuracy to buy smoothness. The D term buys
smoothness for free by using information the system already has — the live rotation rate.

**Driving technique matters as much as system parameters.** A slow perimeter loop with
deliberate pauses before turns produces substantially better maps than continuous fast
driving, regardless of what the code does.
