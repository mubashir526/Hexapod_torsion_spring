# 2026-07-19 — Sample joint ANGLE at 50 Hz to match 50 Hz torque

**File changed:** `ROS/src/sim_robot/sim_robot/kinematic_gait.py` (only this file)
**Related prior work:** `torque_logging_fix.md` (the 50 Hz torque fix + sim-time fix)

---

## Problem

The torque-vs-angle graph compared two data streams recorded at **different
rates**:

- **Torque** was already sampled at **50 Hz** (the true FT-sensor rate, fixed
  earlier — see `torque_logging_fix.md`).
- **Joint angle** (`theta_states`) was only recorded at **10 Hz**, because
  `joint_state_cb` gated it 1:1 against the 10 Hz command list
  (`if len(theta_states) < len(theta_commands)`).

So `plot_torque_vs_angle` had to *interpolate* the coarse 10 Hz angle up onto
the 50 Hz torque phase grid. The angle axis only ever had 10 Hz resolution — the
mismatch the graph was fighting.

**Goal:** sample the measured angle at a genuine 50 Hz, in lockstep with the
torque samples, so the torque-vs-angle loops use real measured data at the same
rate on both axes — without touching the gait, the 10 Hz command/state logging,
or the other graphs.

## Why this was safe to do

The enabling pieces already existed:

- Physics runs at **1000 Hz sim** (`friction_world.sdf`, `max_step_size 0.001`)
  and the `JointStatePublisher` plugins have **no `<update_rate>`**, so
  `/joint_states` publishes far above 50 Hz. Measured live: **~3593 Hz wall** —
  it never starves a 50 Hz sampler.
- `joint_state_cb` already caches the latest positions in
  `self.latest_state_pos` (used by the settle check) — no new subscriber needed.
- A **50 Hz sim-time timer** (`torque_logging_loop`) already runs, already gated
  on `recording` + sensor-ready. We just piggy-back the angle snapshot onto it,
  so angle[i] and torque[i] are captured at the **same instant** and share
  `torque_timestamps[i]`.

## Changes (4)

### 1. New 50 Hz angle buffer (mirrors the torque buffer) — in `__init__`
```python
# Measured joint angle sampled at torque_freq (50Hz), 1:1 with self.torques,
# so torque[i] and angle[i] share torque_timestamps[i]. ... the 10Hz
# self.theta_states path is left completely untouched.
self.theta_states_hf = [ {"hip": [], "knee": [], "foot": []} for _ in range(4) ]
```

### 2. Snapshot the angle inside the existing 50 Hz loop — `torque_logging_loop`
Added a `state_ready` guard, and append the cached angle next to the torque:
```python
if not all(self.state_ready[i][j] for i in range(4) for j in self.joint_types):
    return
...
self.torques[leg_idx][joint_type].append(self.latest_torque[leg_idx][joint_type])
self.theta_states_hf[leg_idx][joint_type].append(self.latest_state_pos[leg_idx][joint_type])
```

### 3. `plot_torque_vs_angle` rewritten — all 4 legs, fed by the 50 Hz angle
- Now loops over **all four legs** (`fr/br/bl/fl_torque_vs_angle.png`) instead of
  one (`self.plot_leg`, which was removed as dead).
- Both angle and torque per cycle come from the **same** 50 Hz samples selected
  by the cycle's time window, then resampled onto the shared `phase_grid`. The
  old cross-rate `np.interp` of the 10 Hz `theta_states` is gone.
- Kept the phase-averaging (mean loop) + faint raw per-cycle loops.

### 4. New CSV `joint_torque_vs_angle.csv` — in `export_csvs`
A third CSV (existing two unchanged): `Time_Step, Time_s`, then per-joint
`<LEG>_<joint>_torque` and `<LEG>_<joint>_angle_deg` for all 12 joints, one row
per 50 Hz sample. `_write_run_info`'s manifest updated to list the new files.

**Not touched:** the gait loop, the 10 Hz `theta_states` / `command_timestamps`,
`joint_commands_vs_states.{png,csv}`, and `joint_torques.{png,csv}`.

## Build

```bash
cd ROS && source /opt/ros/humble/setup.bash
colcon build --packages-select sim_robot        # ~1.2 s, no errors
# copy-install workspace, so also synced the top-level install/ python copy
```

## How it was run & verified (headless)

`start_world.launch.py` starts Gazebo **paused with a GUI**; on this box the
software-rendered GUI is unstable (it exited on its own, freezing `/clock` and
hanging the sim-time-paced node). For a clean automated run we launched Gazebo
**headless server-only** and unpaused it — equivalent physics, no GUI:

```bash
gz sim -s -r -v4 <friction_world.sdf>          # -s server only, -r running
ros2 run ros_gz_sim create ... THex_Quadruped  # + Cube
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=<ros_gz_bridge.yaml> -p expand_gz_topic_names:=true
ros2 run sim_robot kinematic_gait              # settles, records 5 cycles, auto-saves
```
(For normal interactive use, `ros2 launch sim_robot start_world.launch.py` +
press Play, then `ros2 run sim_robot kinematic_gait` still works.)

### Results — `experiment/run1/`

| Check | Result |
|-------|--------|
| `/joint_states` publish rate | ~3593 Hz wall (≫ 50 Hz) ✅ |
| Run captured | 5 gait cycles, 16 steps/cycle |
| `joint_torque_vs_angle.csv` rows | **400** (= 5×16×5, i.e. 5× the 80 command rows) |
| New CSV `Time_s` step | **0.0200 s → exactly 50 Hz** ✅ |
| Angle "held-value" run length (`FR_hip_angle_deg`) | **mean 1.08, max 3** — fresh essentially every row (old zero-order-hold was 2–20) ✅ |
| Torque ↔ angle alignment | 1:1 (both 400 rows, shared timestamps) ✅ |
| 10 Hz path unchanged | `joint_commands_vs_states.csv` still 80 rows @ 0.1 s (10 Hz) ✅ |
| Graphs | `fr/br/bl/fl_torque_vs_angle.png` all rendered non-blank (~180–195 KB); command-vs-state & torque-magnitude PNGs unchanged ✅ |

The FR figure shows clean closed torque-vs-angle loops (bold phase-mean over 5
cycles, faint raw cycles behind) for hip/knee/foot, now with the angle axis at
full 50 Hz resolution instead of interpolated 10 Hz.

## Note / observation

At gait start a few angle rows repeat (max run = 3) — that is real physics (the
joint sits near a velocity zero-crossing and barely moves in 20 ms), not a
sampling artifact. The mean run length of 1.08 confirms the angle is genuinely
refreshed almost every 50 Hz tick.

---

# 2026-07-19 (update) — Switch the torque-vs-angle graph to SAMPLE-BY-SAMPLE averaging over the first 4 full cycles

**File changed:** `ROS/src/sim_robot/sim_robot/kinematic_gait.py`
(`plot_torque_vs_angle` only)
**Related:** `phase_averaging_explained.md`, and the "does it differ?" comparison
earlier today (`ROS/experiment/run1/phase_vs_sample_comparison.png`).

## Why change it

Earlier we compared phase-averaging vs naive sample-by-sample averaging on the
run-1 data and found:

- On **equal-length cycles**, the two methods are **identical** (torque diff
  ~0.0002 N·m, angle ~0.008°).
- The whole visible divergence (up to **0.25 N·m / ~14°**) came **only** from the
  **last cycle being truncated** to 75 samples instead of 80 — recording stops
  mid-stride, so the final cycle's time window loses one command step.

Key realisation: **only the last recorded cycle is ever short.** Every earlier
cycle spans a full `steps_len` (16-step) window and therefore holds the same
number of 50 Hz samples. So if we simply **average the first 4 cycles** (of the
default 5) and **drop the last one**, all averaged cycles are equal-length and we
can average **sample-by-sample by index** — no phase grid, no interpolation, and
the partial-cycle problem disappears entirely.

> On the "always 64": 4 cycles × 16 command steps = **64 command steps**. At the
> 50 Hz logging rate each full 1.6 s cycle holds **80 samples**, so the 4 cycles
> contribute 4×80 = 320 samples and average down to an **80-point** loop. (The
> "64" is the command-step count; the averaged loop itself is 80 points.)

## Change made

`plot_torque_vs_angle` rewritten:

- Added class attribute **`AVG_CYCLES = 4`**.
- Uses the **first `min(AVG_CYCLES, complete)` cycles** (cycles 0–3 by default),
  explicitly excluding the possibly-truncated final cycle.
- For each chosen cycle it takes the 50 Hz sample indices inside that cycle's
  time window, trims all to the **common length `L`** (= 80 in practice; a
  harmless safety net against ±1-sample jitter), then averages
  **sample-by-sample**: `mean(sample_i across cycles)` for both angle and torque.
- Phase grid / `np.interp` / `PHASE_N` removed. Faint raw per-cycle loops + bold
  mean loop and closed-loop wrap (`np.append(...,[0])`) kept.
- Figure title now reads *"sample-averaged over N full cycles, L samples, 50Hz"*.

Nothing else touched (gait, 10 Hz logging, the other graphs/CSVs, and the 50 Hz
`theta_states_hf` recording from the first update above are all unchanged).

## Build & re-run verification

```bash
cd ROS && colcon build --packages-select sim_robot   # OK, then synced top-level install/ copy
# headless: gz sim -s -r ... + create robot/cube + bridge, then:
ros2 run sim_robot kinematic_gait                     # -> experiment/run2/
```

Results (`experiment/run2/`):

| Check | Result |
|-------|--------|
| Node log | `sample-by-sample averaged over 4 full cycles, 80 samples/cycle @ 50Hz` ✅ |
| Cycles used | first **4** (5th/truncated cycle excluded) ✅ |
| Samples/cycle | **80** each (equal length → clean index average) ✅ |
| Graphs | `fr/br/bl/fl_torque_vs_angle.png` all rendered; FR shows clean closed loops (mean + faint raw) ✅ |
| Untouched outputs | command-vs-state & torque-magnitude PNGs/CSVs, and `joint_torque_vs_angle.csv` (still 400 rows @ 50 Hz) unchanged ✅ |

## Net effect

The torque-vs-angle baseline is now computed the simplest exact way for this
setup — a straight per-sample mean of the guaranteed-full leading cycles. It
gives the same loop phase-averaging would on these cycles, but without the phase
resample, and it is immune to the truncated final cycle that caused the earlier
phase-vs-sample discrepancy.
