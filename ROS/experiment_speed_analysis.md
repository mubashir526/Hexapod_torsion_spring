<!-- SUPERSEDED-BANNER -->
> [!IMPORTANT]
> **Superseded.** This document is kept as a working record. The authoritative
> analysis is [`ROS/report/experiment_report.md`](report/experiment_report.md),
> in which every quoted number is recomputed from the CSVs by
> `ROS/report/verify_claims.py`.
>
> Known-wrong values in this file:
>
> - No incorrect values identified. Its §5 data-integrity finding (that `experiment_speed_steps/run2` is mislabelled as a spring run) is confirmed independently in §8.3 of the report.

---
# Gait Speed Experiment — Frequency vs Step-Count Analysis

**Data**: `experiment_speed_freq/run{1,2,3}`, `experiment_speed_steps/run{1,2,3}` (all `spring_mode: none`, n=1 per config)
**Date**: 2026-07-30

---

## 1. What each change means and what it does

Cycle time is `NUM_DATA_POINTS / target_freq`. Two independent levers can change it:

**`target_freq`** (`kinematic_gait.py:33`, the "freq" experiment) — replays the same fixed 16-waypoint trajectory faster or slower. The trajectory *shape* and the *geometric size* of each command-to-command jump never change; only the time allowed between jumps (`dt = 1/target_freq`) does. Physically: same motion, played at a different speed, like a video's playback rate.

**`NUM_DATA_POINTS`** (`kinematics.py:18`, the "steps" experiment) — changes how many waypoints sample the *same* underlying trajectory geometry (a quadratic Bézier swing arc + linear stance sweep), replayed at a fixed 10 Hz. Fewer points should coarsen the sampling of the same curve; more points should refine it.

Both were expected to change cycle time while leaving *something else* fixed — speed only (freq), or resolution only (steps). The freq experiment behaves exactly as expected. The steps experiment does not, for a reason explained in §3.

---

## 2. Frequency experiment (`target_freq`, 16 waypoints fixed)

| Run | `target_freq` | Cycle time | Mean applied effort | RMS applied | Peak demand (unclipped) | Saturation (%) | Tracking error (mean/RMS/peak, deg) |
|---|---|---|---|---|---|---|---|
| run1 | 10 Hz | 1.6 s | 0.2292 | 0.2804 | 0.972 | 0.75% | 4.11 / 6.81 / 20.8 |
| run2 | 20 Hz | 0.8 s | 0.2424 (+5.8%) | 0.3265 (+16.5%) | 1.235 (+27.1%) | **4.88% (6.5×)** | 4.19 / 6.92 / 21.2 |
| run3 | 5 Hz | 3.2 s | 0.2101 (−8.3%) | 0.2535 (−9.6%) | 0.953 (−1.9%) | **0.19% (¼×)** | 3.76 / 6.58 / 21.0 |

*(Combined-average across FR/BR/BL/FL knees; % change relative to run1.)*

**Reading it**: this is the mechanism identified earlier in this project working exactly as predicted. The per-step geometric jump size is identical across all three runs (mean 2.989° everywhere — confirmed directly from the command log, table below), so *only* the time available to reach each jump changes. At 20 Hz the same jump must happen in half the time, so the derivative term reacts harder — saturation jumps 6.5× and peak demand +27%. At 5 Hz the controller gets twice as long, so saturation drops to a quarter and peak demand is essentially flat. Mean/RMS effort track the same direction but far more mildly (+6%/−8%), since they're dominated by the (unchanged) stance holding torque, not the transient spike. Tracking error moves the same direction but modestly (±0.2–0.35°) — expected, since a harder-to-track transient should show up somewhat in tracking error too, though most of that metric is still waypoint-spacing artifact (see the earlier kinematic_gait.py review).

No D-kick artifact flags (peak/p99 ratio) exceeded 1.4 in any run here — consistent with the original 111-run sweep, where this artifact was rare (2/111 cells) even though ever-present at the top-1% level.

**Per-step jump size (confirms the mechanism directly, not inferred):**

| Run | Mean step jump (deg) | Max step jump (deg) |
|---|---|---|
| freq/run1 | 2.989 | 22.23 |
| freq/run2 | 2.989 | 22.23 |
| freq/run3 | 2.989 | 22.23 |

Identical to 3 decimal places across all three — direct confirmation that only *speed*, not trajectory shape, changed.

---

## 3. Step-count experiment (`NUM_DATA_POINTS`, 10 Hz fixed)

| Run | `NUM_DATA_POINTS` | Cycle time | Mean applied effort | RMS applied | Peak demand | Saturation | Tracking error (mean/RMS/peak) | Mean step jump (deg) |
|---|---|---|---|---|---|---|---|---|
| run1 | 16 | 1.6 s | 0.2246 | 0.2756 | 0.965 | 0.63% | 4.05 / 6.83 / 21.4 | 2.989 |
| run2 | **8** | 0.8 s | **0.1749 (−22.1%)** | **0.2090 (−24.1%)** | **0.689 (−28.6%)** | **0.00%** | **1.00 / 1.26 / 2.97 (−75.3% mean)** | **0.483** |
| run3 | 32 | 3.2 s | 0.1994 (−11.2%) | 0.2419 (−12.2%) | 0.495 (−48.7%) | 0.00% | 2.92 / 3.63 / 10.7 (−28.0% mean) | 1.610 |

### 3.1 The problem with steps/run2: the swing phase has no lift at all

**This is the central defect in the steps experiment, and it fully explains why run2's numbers look dramatically "better" — they aren't a resolution effect, they're a broken trajectory.**

The swing foot-path is a quadratic Bézier curve through three control points: `P1=(-3,-7)` (swing start, at stance height), `P2=(0,-1)` (the *control point* pulling the curve upward), `P3=(3,-7)` (swing end, back at stance height). `generate_trajectory()` (`kinematics.py:118-137`) samples this curve at:

```python
n_swing = int(NUM_DATA_POINTS * SWING_FACTOR)      # SWING_FACTOR = 1/4
t = np.linspace(0, 1, n_swing, endpoint=True)
```

Two things matter here. First, **`P2=(0,-1)` is a control point, not a point the curve passes through** — for a quadratic Bézier the curve only reaches its own midpoint at `t=0.5`, which works out to `z(0.5) = -4.0`, i.e. **3 units of lift above stance** (not the 6 units you'd get by naively reading off `P2`'s height). Second, and decisively: **`np.linspace(0, 1, n)` only samples the curve's *interior* — where the lift actually happens — when `n ≥ 3`.** At `n=2` it returns exactly `t=[0, 1]`, the two endpoints, which are *both already at stance height by construction*. The entire lift-and-return arc sits between those two samples and is never evaluated.

At `NUM_DATA_POINTS = 8`: `n_swing = int(8 × 0.25) = 2`. This is exactly that degenerate case:

| `NUM_DATA_POINTS` | swing samples | sampled `t` | sampled heights (z) | lift reached |
|---|---|---|---|---|
| **8 (run2)** | **2** | **[0, 1]** | **−7.0, −7.0** | **0 units — 0% of max — no lift at all** |
| 12 (minimum viable) | 3 | [0, 0.5, 1] | −7.0, **−4.0**, −7.0 | 3.0 units — 100% of max, but a single sharp spike |
| 16 (run1) | 4 | [0, .33, .67, 1] | −7.0, −4.33, −4.33, −7.0 | 2.67 units — 89% of max, smoother shape |
| 32 (run3) | 8 | [0, .14, .29, ..., 1] | −7.0 → **−4.06** → −7.0 | 2.94 units — 98% of max |

**At 8 waypoints the foot is commanded to slide along the ground from one stance position straight to the next — it never lifts.** This is not a coarser version of the gait; it's a qualitatively different, much smaller-amplitude motion (a drag/shuffle instead of a step). Every number in run2's row is a *consequence* of that, not evidence that fewer steps improves anything:

- Lower effort/RMS/peak: a foot-drag needs far less torque than a lift-swing-descend cycle.
- Near-zero tracking error: a near-flat trajectory is trivial for the PID to track.
- Zero saturation: nothing demanding enough to approach the limit ever gets commanded.
- The step-jump collapse (2.989° → 0.483°, a 6.2× drop) is the direct fingerprint of this: reducing point count *should* increase per-step jump size (same distance, fewer steps) — instead it *collapsed*, because the "distance" itself collapsed (no more lift arc to traverse).

**Run2 is excluded from any step-count conclusion below.** It's retained in this report only as a documented artifact, not as a data point about speed or resolution.

### 3.1.1 Why `NUM_DATA_POINTS` must be ≥ 12 — the exact cliff

`n_swing = int(NUM_DATA_POINTS × 0.25)`, and Python's `int()` truncates. The curve's interior only gets sampled once `n_swing ≥ 3`, which requires `NUM_DATA_POINTS ≥ 12`:

| `NUM_DATA_POINTS` | `n_swing = int(N×0.25)` | lift reached | verdict |
|---|---|---|---|
| 8 | 2 | 0% | **degenerate — foot drags, no swing** |
| 9 | 2 | 0% | **degenerate** |
| 10 | 2 | 0% | **degenerate** |
| 11 | 2 | 0% | **degenerate — one short of viable** |
| **12** | **3** | **100%** | **minimum viable — but only a single sharp peak sample, not a smooth arc** |
| 13–15 | 3 | 100% | same as 12 (`int()` doesn't advance again until 16) |
| 16 (current baseline) | 4 | 89% | smoother — two samples straddle the peak instead of one |

The reason it's a hard cliff rather than a gradual falloff is exactly the `int()` truncation: `n_swing` stays frozen at 2 for `N=8` through `11`, then jumps straight to 3 at `N=12`. There is no value of `NUM_DATA_POINTS` between 8 and 11 that gives partial lift — it's binary: either the interior of the curve gets sampled (`N≥12`) or it doesn't (`N≤11`).

Note that `N=12` is a *floor*, not a good operating point: it captures the peak height exactly (100%) but only as one instantaneous spike between two ground-level samples — a triangular lift, not the rounded arc shape the Bézier curve actually describes. The current baseline of 16 (4 swing samples) was already a safety margin above the bare minimum, and `revised_experiment_plan.md`'s proposed 80 (20 swing samples) sits far above it — both are deliberately well clear of the `N=12` cliff edge, not just barely above it.

### 3.2 The valid step-count comparison: run1 (16) vs run3 (32)

Both sample a real lift arc; only resolution differs (10 Hz fixed in both). Going to 32 points (finer sampling of the *same* geometric curve, replayed at the *same* speed) gave:

- Peak demand **−48.7%** (0.965 → 0.495) — a much larger reduction than the freq experiment's speed-only levers produced for saturation, and directly consistent with the project's earlier finding that peak torque is dominated by a step-discontinuity artifact: finer sampling shrinks the discontinuity itself, at any replay speed.
- Saturation dropped from 0.63% to **0.00%**.
- Tracking error mean dropped 28% (4.05° → 2.92°) — makes sense, since a finer-sampled curve is smoother to track, independent of the waypoint-spacing confound noted elsewhere.
- Mean/RMS applied effort dropped more modestly (−11%/−12%), again dominated by the (largely resolution-independent) stance holding torque.

This is the cleanest evidence in either experiment for the peak-torque smoothing fix already scoped in `revised_experiment_plan.md` (raising `NUM_DATA_POINTS` 16→80): finer resolution reduces peak demand substantially, without needing a spring or any other change.

---

## 4. Freq vs steps, matched at the same nominal cycle time

Both experiments produced a run at ≈0.8s and ≈3.2s, via different levers. This isolates which lever is "safe" to use for changing speed alone.

**≈0.8s:** freq/run2 (20Hz, real trajectory) vs steps/run2 (8pt, degenerate — excluded from conclusions, shown for contrast only)

| | Mean effort | Peak demand | Saturation | Track err (mean) |
|---|---|---|---|---|
| freq/run2 (valid) | 0.2424 | 1.235 | 4.88% | 4.19° |
| steps/run2 (degenerate) | 0.1749 | 0.689 | 0.00% | 1.00° |

The gap between these two is almost entirely the degenerate-trajectory artifact, not a meaningful "which lever is better at 0.8s" comparison — steps/run2 looks better only because it's doing less.

**≈3.2s:** freq/run3 (5Hz) vs steps/run3 (32pt) — **both valid**, and the more informative comparison:

| | Mean effort | Peak demand | Saturation | Track err (mean) |
|---|---|---|---|---|
| freq/run3 (slower replay) | 0.2101 | 0.953 | 0.19% | 3.76° |
| steps/run3 (finer sampling) | 0.1994 | **0.495** | 0.00% | 2.92° |

At matched (roughly) cycle time, **finer sampling (steps) reduces peak demand far more than slower replay (freq)** — 0.495 vs 0.953, roughly half. Slowing replay gives the controller more time to chase the same discontinuity; finer sampling shrinks the discontinuity itself. If the goal is minimizing peak torque specifically, resolution is the more effective lever of the two — consistent with why the revised experiment plan chose to raise `NUM_DATA_POINTS` for that purpose rather than lowering `target_freq`.

---

## 5. Data-integrity note

`experiment_speed_steps/run2/run_info.txt` reports `spring_mode: unknown` with a spring_summary describing a native kx=0.5/θ₀=−50° spring — inconsistent with its siblings (all `spring_mode: none`). This is a metadata-reporting race identical to one found in 3/111 runs of the original spring sweep: `_spring_title_str()`'s fallback path execs the currently-installed `make_spring_models.py` and reports whatever `SPRING_CONFIG` happens to hold (stale values left over from the old sweep), when the latched `/gait/spring_config` topic didn't arrive in time. **Checked directly against the data**: steps/run2's signed per-knee effort means (−0.19, −0.12, +0.12, +0.23) sit in the same range and sign pattern as its three no-spring siblings, nowhere near the ~2× inflation a real kx=0.5/θ₀=−50° spring produced in the original sweep (that exact configuration gave −80.2% "reduction," i.e., roughly doubled effort). **No physical spring was active** — only the run_info.txt label is wrong. This does not affect §3.1's conclusion, which rests on the swing-height sampling math and the step-jump collapse, independent of spring state.

---

## 6. Summary

| Question | Answer |
|---|---|
| Does raising `target_freq` (faster replay, same trajectory) increase peak torque and saturation? | **Yes** — clearly and monotonically confirmed (20Hz: +27% peak, 6.5× saturation; 5Hz: −2% peak, ¼× saturation). |
| Does lowering `NUM_DATA_POINTS` safely speed up the gait? | **No, not below the swing-sampling floor.** At 8 points the swing arc's lift is not sampled at all — the result is a different, easier motion, not a faster version of the same gait. The floor for *any* lift to register is 3 swing samples, i.e. `NUM_DATA_POINTS ≥ 12` (`int(12×0.25)=3`). |
| Does raising `NUM_DATA_POINTS` (finer sampling, same speed) reduce peak torque? | **Yes, substantially** — −48.7% peak demand, saturation to 0%, more effective per this data than slowing down via `target_freq` alone. |
| Which lever isolates speed cleanly? | `target_freq`, as recommended in the earlier plan-mode decision — confirmed here by the identical 2.989° step-jump size across all three freq runs. |
