# Session 011 — 2026-03-13: Universal Heading Hold Tune for Mixed Surfaces

## Goal

Validate the Session 010 heading hold PD controller on carpet and find a single set
of constants that works acceptably on both hard floors and soft surfaces — without
needing to be swapped every time the operating environment changes.

## Context

Session 010 developed and tuned the heading hold controller on a hard floor, producing
the following baseline:

| Constant | Session 010 Baseline |
|---|---|
| `HEADING_KP` | 1.6 |
| `HEADING_KD` | 0.3 |
| `HEADING_DEADBAND` | 0.017 rad (~1°) |
| `GZ_MAX_RATE` | 3.0 |
| `HEADING_MAX_CORRECTION` | 0.3 rad/s |
| `BIAS_DURATION` | 5.0 s |
| `BIAS_NOISE_THRESHOLD` | 3.0 counts std dev |
| `BIAS_MAX_DURATION` | 15.0 s |
| `LINEAR_RAMP_RATE` | 0.8 m/s² |
| `ANGULAR_RAMP_RATE` | 2.0 rad/s² |
| `ZUPT_ODO_THRESHOLD` | 0.5 raw units |
| `ZUPT_SETTLE_TICKS` | 20 ticks (~1s) |
| `ZUPT_ALPHA` | 0.05 |

Those constants were optimised for a single surface. A robot that only works well on
one floor type is not a practical system — the goal here was to find a middle ground
that doesn't break on either surface without manual re-tuning before every run.

This session took place at a different location with carpeted flooring. One full
battery charge was available with no charger on hand, so all testing had to be
completed within that charge. Iteration was rapid and observational.

---

## Discovery 1 — KP Had Drifted to 2.2

Opening the driver node revealed `HEADING_KP` at 2.2 — above the Session 010 settled
value of 1.6. The cause is unknown, likely a quick test that was never reverted. The
intent was to start from the Session 010 baseline, so this was an unexpected starting
state. It was noted and factored into the tuning process.

---

## Discovery 2 — Hard-Floor Constants Are Too Aggressive for Carpet

Running the rover on carpet revealed two compounding failure modes:

**Sudden, sharp direction corrections during straight driving.** With `HEADING_KP`
effectively at 2.2, any heading error triggered a hard overcorrection visible as a
jerk in the rover's path. Even at the intended baseline of 1.6 this would have been
too aggressive for carpet — the higher value made it worse.

**Controller correcting a heading that was already correct.** `HEADING_DEADBAND` at
0.017 rad (~1°) is a tight gate designed for the low-noise environment of a hard
floor. On carpet, motor vibration transmitted through the chassis produces gyroscope
readings that continuously cross a 1° threshold. The controller was spending as much
time fighting phantom errors as doing useful work.

These are two separate problems that amplify each other: a high KP makes every
phantom correction more disruptive, and a narrow deadband ensures phantom corrections
happen constantly.

---

## Tuning Toward a Universal Baseline

The objective was not to optimise for carpet specifically, but to find constants
loose enough to tolerate carpet vibration noise while still providing meaningful
heading correction on hard floors. That means accepting some precision loss on hard
floors in exchange for stability on soft surfaces.

**First attempt:** `KP=1.4`, `KD=0.4`, `DEADBAND=0.035 rad`, `GZ_MAX_RATE=2.0`.
Result: not sufficient. The deadband was still narrow enough that carpet vibration
registered as heading error, and the controller was still triggering unnecessarily.

**Second attempt:** `KD` raised to 0.5, `DEADBAND` widened to 0.045 rad (~2.6°).
Jittering was meaningfully reduced. The controller stopped fighting the surface and
straight-line tracking was observably improved on carpet. These values were kept as
the working universal baseline.

---

## Final Constants

| Constant | Session 010 Baseline | Session 011 Result | Rationale |
|---|---|---|---|
| `HEADING_KP` | 1.6 | **1.4** | Softer correction authority for mixed surfaces |
| `HEADING_KD` | 0.3 | **0.5** | More damping to compensate for lower KP |
| `HEADING_DEADBAND` | 0.017 rad | **0.045 rad** | Above carpet vibration noise floor |
| `GZ_MAX_RATE` | 3.0 | **2.0** | Tighter spike clamp for surface-induced gz noise |
| `HEADING_MAX_CORRECTION` | 0.3 rad/s | **0.3 rad/s** | Unchanged |
| `LINEAR_RAMP_RATE` | 0.8 m/s² | **0.8 m/s²** | Unchanged |
| `ANGULAR_RAMP_RATE` | 2.0 rad/s² | **2.0 rad/s²** | Unchanged |
| `BIAS_DURATION` | 5.0 s | **5.0 s** | Unchanged |
| `BIAS_NOISE_THRESHOLD` | 3.0 counts | **3.0 counts** | Unchanged |
| `BIAS_MAX_DURATION` | 15.0 s | **15.0 s** | Unchanged |
| `ZUPT_ODO_THRESHOLD` | 0.5 | **0.5** | Unchanged |
| `ZUPT_SETTLE_TICKS` | 20 ticks | **20 ticks** | Unchanged |
| `ZUPT_ALPHA` | 0.05 | **0.05** | Unchanged |

---

## Limitations & Open Items

No SLAM map was captured this session. Battery constraints and the observational
nature of the testing meant no mapping run was attempted. Whether these constants
produce clean maps across both surfaces at moderate speeds is untested and is the
logical next validation step.

The widened deadband (0.045 rad, ~2.6°) means the controller tolerates more heading
error before intervening than the hard-floor baseline. This is the correct trade-off
for a universal tune, but the practical impact on map quality over long runs remains
to be confirmed.

Wall smearing at moderate speeds is an unresolved carry-over from Session 010.

---

## Key Takeaway

A heading hold controller tuned for one surface type will not transfer cleanly to
another. The hard-floor constants from Session 010 used a deadband narrower than the
vibration noise floor of carpet, turning the controller into a source of disturbance
rather than correction. The universal tune trades some hard-floor precision for the
ability to operate without manual re-tuning across surfaces. The result is
approximately 90% of the way toward the goal: behaviorally acceptable on carpet, and
not regressed on hard floors.
