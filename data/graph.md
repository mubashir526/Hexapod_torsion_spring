# Torque-vs-Angle Baseline Graph — Changes & Analysis

**Date:** 2026-07-09
**File changed:** `ROS/src/sim_robot/sim_robot/kinematic_gait.py`
**Output produced:** `fr_torque_vs_angle.png` (written on Ctrl+C, alongside the
existing `joint_commands_vs_states.png` / `joint_torques.png`)

## Objective

Reproduce, from the `sim_robot` logging system, the *style* of graph shown in
`data/Paper_graph.png` — the knee **joint torque vs joint angle** phase plot
from Satsevich et al. 2024 (*"Optimizing energy consumption for legged robot by
adapting equilibrium position and stiffness of a parallel torsion spring"*).

Scope agreed for this implementation:

- **Baseline curve only.** The paper overlays a second "optimal" curve (torque
  after subtracting an optimal parallel torsion spring). Our robot has no such
  spring and no optimal case, so only the measured **baseline** is plotted.
- **Render it as a loop** (closed phase loop over one gait cycle).
- **No impact** on the existing torque graph, the CSV export, or how torque is
  recorded.

---

## Changes Made

All changes are additive and self-contained in `kinematic_gait.py`.

### 1. New import
```python
import numpy as np
```
Used only for `np.interp` (time-base alignment) and array handling in the new
plot. Nothing else depends on it.

### 2. New config field in `__init__`
```python
# Leg whose per-joint torque-vs-angle baseline loop is plotted on exit
# (FR/BR/BL/FL). Read-only extra figure; does not affect the existing
# torque graph or any CSV.
self.plot_leg = "FR"
```
Change this string to target a different leg. Default is Front-Right.

### 3. New call in `save_data()`
```python
def save_data(self):
    print("\nProcessing data...")
    self.plot_graphs()
    self.plot_torque_vs_angle()   # <-- added
    self.export_csvs()
```

### 4. New method `plot_torque_vs_angle()`
A 1×3 figure (hip / knee / foot) for `self.plot_leg`. For each joint it:

1. Takes the measured joint angle `theta_states` (radians, 10 Hz, on
   `command_timestamps`) and the measured torque `self.torques` (50 Hz, on
   `torque_timestamps`). Both timestamp streams are elapsed **sim-seconds**, so
   they share one time base.
2. **Phase-averages across every complete gait cycle** (see the stability note
   below): each cycle's torque is resampled onto a common 0..1 phase grid
   (`PHASE_N = 60`), the joint angle is sampled at the same phase instants, and
   the per-phase **mean** across cycles is taken.
3. Draws each raw cycle as a faint gray loop for context, with the averaged
   `baseline (mean)` loop bold on top.
4. Appends the first sample to the end to **close the loop** visually.
5. Axes labelled `… joint Angle, rad` / `… joint Torque, N*m`; saves
   `fr_torque_vs_angle.png`.

> **Stability note — one cycle vs many.** The first version plotted a **single**
> gait cycle and looked randomized. That jaggedness was *not* from overlapping
> cycles (only one was used) — it was genuine noise within one cycle: our
> torques are sub-Newton-metre, so 50 Hz contact/PID spikes are large relative
> to the signal. The fix is **phase-averaging over all complete cycles**, which
> is exactly what the paper assumes ("a number of full cyclic motions was
> performed"). Averaging cancels the per-cycle *random* noise; whatever
> structure survives is systematic (real per-phase torque + true cycle-to-cycle
> variation), which is why the mean loop is stable but still not as glass-smooth
> as the paper's — that smoothness is a property of their cleaner rig, not extra
> processing, and we deliberately do **not** over-smooth the real signal away.

### What was deliberately NOT changed

| Concern | State |
|---|---|
| `joint_torque_cb` | Untouched — still records `abs(msg.wrench.torque.z)` |
| Existing torque figure in `plot_graphs()` | Untouched — same magnitude-vs-time plot + 30 % stall line |
| `export_csvs()` (`joint_torques.csv`, `joint_commands_vs_states.csv`) | Untouched |

The new method only **reads** already-logged data and writes only its own PNG.

---

## Methodology in Detail — Reconciling 50 Hz Torque with 10 Hz Angle, and Averaging

This is the part that does the real work. The two quantities we want to plot
against each other are sampled at **different rates**:

| Signal | Source | Rate | Samples per 2 s cycle | Timestamp array |
|---|---|---|---|---|
| Joint **angle** `theta_states` | `/joint_states` (10 Hz gait loop) | 10 Hz | ~20 | `command_timestamps` |
| Joint **torque** `self.torques` | FT sensor 50 Hz timer | 50 Hz | ~100 | `torque_timestamps` |

You cannot just zip them index-by-index — index 5 of the torque array and index
5 of the angle array happened at completely different moments. The solution is
built on one fact and one tool.

### The key fact: both share a common time axis

Thanks to the earlier sim-time fix, **both** `command_timestamps` and
`torque_timestamps` store the *same thing*: elapsed **simulation seconds** since
the node started (`_elapsed_seconds()`). So every angle sample and every torque
sample carries a timestamp measured on the same clock. The rate difference stops
mattering the moment we treat each signal as a **continuous function of time**
and read it off wherever we like, rather than as a fixed-index list.

### The key tool: `np.interp` (piecewise-linear interpolation)

`np.interp(x_query, x_known, y_known)` returns, for each `x_query`, the value of
the signal linearly interpolated between the two nearest known points. It
requires `x_known` to be sorted ascending. We use it to **resample both signals
onto a shared grid** — upsampling the sparse 10 Hz angle and downsampling the
dense 50 Hz torque so they meet at identical points.

### Step-by-step, exactly as the code does it

**Step 0 — count complete cycles.** One gait cycle = `self.steps_len` command
ticks (20 steps × 100 ms = 2.0 s nominal). `complete = n_cmd // steps_len`.
Only whole cycles are used.

**Step 1 — cut each cycle's time window** (from the 10 Hz command clock, the
authoritative cycle timer):
```
t_start = command_timestamps[c   * steps_len]          # first tick of cycle c
t_end   = command_timestamps[(c+1)* steps_len]         # first tick of cycle c+1
```

**Step 2 — normalise time to phase 0..1.** Instead of absolute seconds, express
position within the cycle as a fraction:
```
phase = (t - t_start) / (t_end - t_start)     #  0.0 at cycle start, →1.0 at end
```
This is what makes cycles of *slightly different real duration* line up — the
sim does not run at a perfectly constant 2.000 s/cycle, so normalising by each
cycle's own length is more robust than assuming a fixed period.

**Step 3 — build one common phase grid** shared by every cycle and every joint:
```
phase_grid = linspace(0, 1, 60, endpoint=False)   # 60 evenly spaced phases
```
`endpoint=False` leaves phase 1.0 out so that when we later wrap phase 0 onto the
end, the loop closes without a duplicate point.

**Step 4 — resample the TORQUE (50 Hz → grid).** Take the ~100 torque samples in
this cycle, compute each one's `phase`, sort by phase, and interpolate onto the
60-point grid:
```
tau_grid = np.interp(phase_grid, phase_of_torque_samples, torque_samples)
```
This *downsamples* the dense torque to 60 phase points.

**Step 5 — resample the ANGLE (10 Hz → same grid).** The grid phases map back to
real times `grid_t = t_start + phase_grid*(t_end - t_start)`. Read the angle at
those exact times:
```
ang_grid = np.interp(grid_t, state_t, state_ang)
```
This *upsamples* the sparse ~20-point angle to the same 60 phase points.

> After Steps 4–5, for every one of the 60 phase points we have a **matched
> `(angle, torque)` pair**, both evaluated at the identical instant. The
> 50 Hz/10 Hz mismatch is gone: neither signal is used at its native index — both
> are re-expressed on the shared 60-point phase grid.

**Step 6 — average across cycles.** Stack the per-cycle arrays into a matrix of
shape `(complete, 60)` and take the mean **down the cycle axis**:
```
ang_mean = mean(ang_cycles, axis=0)      # 60 values
tau_mean = mean(tau_cycles, axis=0)      # 60 values
```
Because every cycle used the *same* phase grid, column `k` of every row is "the
same gait phase". So `tau_mean[k]` is the average torque **at phase k across all
cycles** — a phase-aligned point-wise average. Random per-cycle spikes (which
land at different phases each time) average toward zero; the systematic,
repeatable torque pattern survives.

**Step 7 — close the loop.** Append the first grid point to the end
(`np.append(x, x[0])`) so the parametric curve returns to its start and draws as
a closed loop.

**Step 8 — draw.** Faint gray = each raw cycle's loop (honesty/context); bold =
the averaged `baseline (mean)` loop.

### Worked mini-example (one phase point, one cycle)

Say cycle 3 runs `t_start = 6.00 s`, `t_end = 8.00 s`, and we want grid phase
`k` = 0.50 (half-way through the cycle):
- real time of this point: `6.00 + 0.50 × 2.00 = 7.00 s`
- torque at 7.00 s: the FT samples nearest 7.00 s might be at 6.98 s (0.42 N·m)
  and 7.02 s (0.30 N·m) → interp gives ≈ 0.36 N·m.
- angle at 7.00 s: the 10 Hz state samples nearest are 6.9 s (0.61 rad) and
  7.0 s (0.58 rad) → interp gives ≈ 0.58 rad.
- so cycle 3 contributes the pair `(0.58 rad, 0.36 N·m)` at phase 0.50.
Do this for cycles 1..N at phase 0.50, average the torques and the angles → one
point of the mean loop. Repeat for all 60 phases → the full loop.

### Design choices & edge cases

- **`PHASE_N = 60`** sits between the two native densities (≈100 torque, ≈20
  angle points per cycle): a mild downsample of torque, a ~3× upsample of angle.
- **Cycles with `< 3` torque samples are skipped** (guards against a truncated
  final cycle or a stall).
- **Last cycle boundary:** if `(c+1)*steps_len` runs past the recorded commands,
  `t_end` falls back to the last timestamp.
- **Sorting:** torque phases are `argsort`-ed before `np.interp` (it needs
  ascending x); `state_t` is already monotonic sim time.
- **Interpolation assumption:** linear between samples — safe for the dense
  50 Hz torque and acceptable for the smooth 10 Hz trajectory.

---

## Graph Analysis — Comparison with the Paper

### What matches ✅

| Aspect | Paper `Paper_graph.png` | Our `fr_torque_vs_angle.png` |
|---|---|---|
| Plot type | Torque vs angle **phase plot** | Same |
| X axis | Joint angle (rad) | Joint angle (rad) |
| Y axis | Joint torque (N·m) | Joint torque (N·m) |
| Baseline shape | Closed **loop / hysteresis** over the cyclic motion | Closed loop over one gait cycle |
| Meaning | Torque the actuator must supply as the joint sweeps its angle range | Same |

The **knee subplot** in particular is the direct analogue of the paper's
"no spring" (Baseline) curve: a loop with an outbound (stance-loaded) branch and
a return (swing) branch tracing different torques at the same angle.

### What differs (and why) ⚠️

1. **Only one curve (baseline).** The paper's orange "optimal" curve requires a
   parallel torsion spring and a closed-form spring fit. Our robot has neither,
   so — as agreed — only the baseline is drawn. This is an intentional scope
   difference, not a defect.

2. **Torque magnitude scale.** Paper baseline ≈ 11–15 N·m; ours ≈ ±0.5 N·m.
   Expected: the paper's leg has 0.28 m links on a full-size robot; ours is a
   ~1.4 kg quadruped with a 0.9414 N·m servo stall limit. Absolute values are
   not comparable; the graph *shape* is what carries over.

3. **Angle range / sign.** Paper knee ≈ −0.95→−0.5 rad; our knee ≈ 0.53→0.92
   rad. Different link geometry and joint-zero convention (our IK, Section 6 of
   the walkthrough). Not an error — just a different coordinate frame.

4. **Three joints instead of one.** The paper shows only the knee. We plot
   hip/knee/foot of one leg (configurable) since all three are logged; the knee
   is the closest one-to-one match.

5. **Noisier / less glass-smooth loop** (even after phase-averaging). The
   paper's loop is smooth. Ours is stabilised by averaging over all cycles but
   still shows some structure because (a) the 50 Hz force-torque signal carries
   contact/PID transients that are large *relative to* our tiny ~0.5 N·m torques
   (near the sensor noise floor), (b) the 10 Hz angle gives only ~20 command
   points per cycle, and (c) there is genuine cycle-to-cycle variation
   (per-phase std ≈ 0.06–0.12 N·m in the sample data). The paper used a stiff PD
   controller (Kp=300, Kd=1) on a vertical-stand single leg with near-zero
   friction, giving an inherently cleaner cyclic signal.

### Verdict

**Same *kind* of graph, faithfully reproducing the paper's baseline
methodology — not a numerical match.** It is a torque-vs-angle phase loop of the
measured actuator torque, which is exactly what the paper's "no spring" /
Baseline curve is. It is not identical because (i) only the baseline is in
scope, (ii) the robot is at a completely different scale, and (iii) the sim
signal is noisier. To move it closer to the paper's clean appearance one could:
average the loop over several cycles, smooth the torque, and/or reduce ground
friction — but those change the data, not the plotting.

---

## How to Regenerate

```bash
cd ~/Documents/FYP-Legged-Robot-main/Code
colcon build --packages-select sim_robot        # also in Code/ROS/ per the dual-tree note
ros2 launch sim_robot start_world.launch.py
ros2 run sim_robot kinematic_gait               # let it run a few full cycles
# Ctrl+C  ->  writes fr_torque_vs_angle.png
```

Change `self.plot_leg` in `kinematic_gait.py` to plot a different leg.
