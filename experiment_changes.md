# Experiment Harness — Changes Log

**File changed:** `ROS/src/sim_robot/sim_robot/kinematic_gait.py`
**Purpose:** Turn each `kinematic_gait` run into a fixed-length, self-documenting
experiment so many runs can be collected and compared cleanly.

This log records the changes that set up the experiment framework. Individual
run outputs live under `experiment/runN/` (see below); this file documents the
*mechanism*, not the results of any single run.

---

## What each run now does

1. **Runs exactly 5 gait cycles, then stops on its own.** No more manual Ctrl+C
   timing — every run is the same length, so runs are comparable.
2. **Writes all outputs into a fresh `experiment/runN/` folder.** `run1`, `run2`,
   … auto-increment; a new run never overwrites an old one.
3. **Drops a `run_info.txt` manifest** into each folder so the run is
   self-describing (cycle count, sample counts, rates, timestamp).

### Files produced per run (inside `experiment/runN/`)

| File | Contents |
|---|---|
| `joint_commands_vs_states.png` / `.csv` | Command vs measured angle, all 12 joints (10 Hz) |
| `joint_torques.png` / `.csv` | Torque magnitude vs time, all 12 joints (50 Hz) |
| `<leg>_torque_vs_angle.png` | Baseline torque-vs-angle phase loop for `plot_leg` (default FR), phase-averaged over the 5 cycles |
| `run_info.txt` | Per-run manifest |

---

## Code Changes (all in `kinematic_gait.py`)

### 1. New imports
```python
import os
from datetime import datetime
```
`os` for folder creation / path joining, `datetime` for the run timestamp.

### 2. New config in `__init__`
```python
# --- EXPERIMENT HARNESS ---
self.max_cycles = 5      # auto-stop after this many complete gait cycles
self.run_dir = None      # resolved lazily at save time to experiment/runN
```
Change `self.max_cycles` to run a different number of cycles.

### 3. Auto-stop in `timer_callback`
After a cycle completes (`current_step` wraps and `cycle_count` increments):
```python
if self.cycle_count >= self.max_cycles:
    self.get_logger().info(
        f"=== Completed {self.max_cycles} gait cycles — stopping run ===")
    raise KeyboardInterrupt
```
`raise KeyboardInterrupt` unwinds into `main()`'s existing `finally` block, which
already calls `save_data()` — so the stop path is identical to a manual Ctrl+C
and no data-saving logic had to be duplicated.

### 4. New helper `_make_run_dir()`
```python
base = "experiment"
os.makedirs(base, exist_ok=True)
n = 1
while os.path.exists(os.path.join(base, f"run{n}")):
    n += 1
run_dir = os.path.join(base, f"run{n}")
os.makedirs(run_dir)
return run_dir
```
Finds the next free `runN` and creates it. `experiment/` is created relative to
the **current working directory** where `ros2 run` is launched (same place the
PNGs/CSVs used to land).

### 5. New helper `_write_run_info()`
Writes `run_info.txt` with: run dir, wall-clock timestamp, cycles completed vs
target, steps/cycle, gait & torque rates, command-row and torque-sample counts,
and the phase-plot leg.

### 6. `save_data()` resolves the run dir first
```python
self.run_dir = self._make_run_dir()
print(f"Saving experiment outputs to: {self.run_dir}/")
self.plot_graphs()
self.plot_torque_vs_angle()
self.export_csvs()
self._write_run_info()
```

### 7. All output paths routed through `self.run_dir`
Every hard-coded filename now uses `os.path.join(self.run_dir, <name>)`:
- `plot_graphs()` → `joint_commands_vs_states.png`, `joint_torques.png`
- `plot_torque_vs_angle()` → `<leg>_torque_vs_angle.png`
- `export_csvs()` → `joint_commands_vs_states.csv`, `joint_torques.csv`

Nothing about *how* data is recorded, plotted, or the CSV columns changed — only
**where** the files are written.

---

## How to run an experiment

```bash
cd ~/Documents/FYP-Legged-Robot-main/Code        # experiment/ is created here
colcon build --packages-select sim_robot          # rebuild after the edit (also in Code/ROS/)
ros2 launch sim_robot start_world.launch.py
ros2 run sim_robot kinematic_gait                 # stops itself after 5 cycles
# -> writes experiment/run1/ (next run -> run2, ...)
```

Notes:
- The run auto-stops after 5 cycles; Ctrl+C earlier still saves whatever was
  collected (into the next `runN`).
- The `experiment/` folder appears wherever you launched `ros2 run` from — run it
  from the same directory each time to keep all runs together.
- To change run length, edit `self.max_cycles`; to change the phase-plot leg,
  edit `self.plot_leg`.

---

# Simulation Fidelity Fixes (Issues #1 & #4)

**File changed:** `ROS/src/sim_robot/models/THex_Quadruped/model.sdf`
**Date:** 2026-07-12
**Scope:** Only Issues #1 and #4 from `simulation_issues_analysis.md` /
`simulation_issues_deep_analysis.md`. Issues #2 (unit mismatch) and #3 (PID
gains) were deliberately left untouched.

## Physics-engine finding (why the parameters are what they are)

The sim runs on **Gazebo Harmonic 8.14** with `<physics type="ignored">` and no
`--physics-engine` override, so the engine is **DART** (`gz-physics-dartsim`).
DART honors joint damping/friction, surface friction (`<mu>`), and restitution,
but **ignores** the ODE penalty-contact params (`<kp>/<kd>/<soft_cfm>/<soft_erp>`)
that the analysis docs propose for softening foot contact. Those were therefore
**omitted** — including them would have been dead XML under DART. Genuine
contact-spike softening under DART would require switching engine or adding
sphere/capsule foot tips; both were out of scope for this minimal fix.

## Issue #1 — Joint damping & friction (all 12 joints)

Added `<damping>` and `<friction>` to every joint's `<dynamics>` block.

**Before** (×12, identical):
```xml
<dynamics>
  <spring_reference>0</spring_reference>
  <spring_stiffness>0</spring_stiffness>
</dynamics>
```
**After** (×12):
```xml
<dynamics>
  <damping>0.01</damping>
  <friction>0.005</friction>
  <spring_reference>0</spring_reference>
  <spring_stiffness>0</spring_stiffness>
</dynamics>
```

**Values / reasoning:** uniform across all joints (per-joint tuning is Issue #3,
excluded). `damping=0.01` N·m·s/rad ≈ 5.5% of stall torque at max joint speed
(0.01 × 5.23 rad/s ≈ 0.052 N·m) and ~17% (hip) to ~35% (foot) of critical
damping — a large improvement over the current 0% without over-damping the light
joints. `friction=0.005` N·m (~0.5% of stall) resists idle drift during stance.
These are conservative first values and are a natural knob to sweep across runs.

## Issue #4 — Foot contact surface (all 4 feet)

Added a `<surface>` block to each foot collision (`bl/br/fl/fr_foot_collision`),
which previously had **no** `<surface>` at all. Chose friction + no-bounce only
(the parts DART actually uses).

**Before** (×4, per foot):
```xml
        </geometry>
      </collision>
```
**After** (×4, per foot — inserted before `</collision>`):
```xml
        </geometry>
        <surface>
          <friction>
            <ode><mu>1.0</mu><mu2>1.0</mu2></ode>
          </friction>
          <bounce>
            <restitution_coefficient>0.0</restitution_coefficient>
            <threshold>0.01</threshold>
          </bounce>
        </surface>
      </collision>
```

**Values / reasoning:** foot `mu=mu2=1.0` (rubber-tip grip); combined with the
ground's `mu=0.7` the effective friction is ground-limited at ≈0.7, so the feet
now have an explicit, defined friction instead of relying on the engine default.
`restitution_coefficient=0.0` prevents the feet bouncing off the ground.

## Undo (easy revert)

Both changes are pure additions — reverting = deleting the added lines:

1. **Issue #1:** in all 12 `<dynamics>` blocks, delete the two lines
   `<damping>0.01</damping>` and `<friction>0.005</friction>`.
2. **Issue #4:** delete the four `<surface> … </surface>` blocks (the ones with
   `<mu>1.0</mu>`) from the foot collisions.

Or restore `model.sdf` from version control. After reverting (or applying),
rebuild so the installed model updates.

## Rebuild & verify

```bash
cd ~/Documents/FYP-Legged-Robot-main/Code
colcon build --packages-select sim_robot          # also in Code/ROS/ per the dual-tree note
ros2 launch sim_robot start_world.launch.py
ros2 run sim_robot kinematic_gait
```
Expected effect: calmer, less oscillatory joint torques (damping) and feet that
grip instead of sliding, with no ground bounce. Compare an `experiment/runN/`
before vs after.

**Change counts (sanity):** 12 `<damping>`, 12 `<friction>0.005`, 4 `<surface>`,
4 foot `<mu>1.0</mu>`, 4 `restitution 0.0`. SDF validated as well-formed XML.

---

# Homing / Settle Before Recording

**File changed:** `ROS/src/sim_robot/sim_robot/kinematic_gait.py` (only this file)
**Date:** 2026-07-12
**Goal:** Have the robot already sitting in the gait's starting configuration
(on its feet) before recording, instead of recording the spawn free-fall + the
step-0 "slam" from a random landed pose.

## Behaviour

On startup the node now runs a **homing/settle phase**: it publishes the gait's
**first waypoint** (the "home pose") to all 12 joints and holds it, recording
nothing. Once the robot has settled, it resets the clock and begins the normal
gait loop **and** recording. Every `experiment/runN` therefore starts from the
same, settled configuration.

**Settle trigger = convergence + timeout.** Recording starts when *either*:
- every joint is within `settle_pos_tol = 0.20` rad of its home target **and**
  moving slower than `settle_vel_tol = 0.10` rad/s (robot has reached the pose
  and stopped), **or**
- `settle_max_s = 4.0` s of sim time elapses (hard fallback, so a joint with
  steady-state droop can never stall the run).

## Code changes (`kinematic_gait.py`)

1. **New state in `__init__`:** `self.recording = False`; `self.home` (step-0
   angles per leg/joint from `theta_targets`); settle tolerances/timeout; and
   `latest_state_pos` / `latest_state_vel` / `state_ready` caches for the check.
2. **`timer_callback`:** if `not self.recording`, call `_settle_step()` and
   return (no command data appended during settle).
3. **New `_settle_step()`:** publishes the home pose each tick; when converged
   or timed out, resets `start_time` (so logged timestamps start at 0 at
   gait-start), `current_step`/`cycle_count`, and sets `recording = True`.
4. **`joint_state_cb`:** also caches latest position/velocity + `state_ready`
   per joint (used only by the settle check). State logging is unchanged and
   still stays empty until recording (it is gated by `theta_commands` length).
5. **`torque_logging_loop`:** added `if not self.recording: return` so torque is
   logged only during the gait.

No other files touched. `steps_len`, `max_cycles`, plotting, and CSV export are
all unchanged.

## Effect on torque analysis

**Improves it; does not bias the measured torques.** The robot, gait, PID, and
sensors are unchanged — only *when* recording starts changes, plus a home-pose
hold. Consequences:
- Removes the dominant artifacts (step-0 saturation slam; "joints stuck at 0 deg
  while free-falling"). Logged torques reflect walking loads.
- Phase-averaged torque-vs-angle loops converge better because all 5 recorded
  cycles start from a settled pose (less cycle-to-cycle spread).
- Runs become directly comparable (identical start pose each time).

**Caveats:** the gait is still open-loop with no balance, so the robot can drift
or tip over the 5 cycles — homing gives a clean *start*, not a guaranteed steady
limit cycle. The feet-plant contact transient during settle is excluded by only
recording after the settle window (make `settle_max_s` long enough if needed).

## Tunables / undo

- Tunables: `self.settle_pos_tol`, `self.settle_vel_tol`, `self.settle_max_s`.
- Undo: delete the `_settle_step` method and the homing state block; in
  `timer_callback` remove the `if not self.recording:` guard; in
  `torque_logging_loop` remove its `if not self.recording: return`; in
  `joint_state_cb` remove the pos/vel/ready caching lines. (Or restore the file
  from version control.)

## Verified
- `py_compile` OK.
- Convergence predicate tested offline: False on spawn (not ready), False when
  ready-but-far, False when at-home-but-moving, True only when at-home-and-stopped.

### Homing settle fix (velocity-based) — 2026-07-12

**Symptom:** the robot appeared to "only stand after the timeout" — the settle
phase always ran the full `settle_max_s` (4 s) instead of exiting early once the
robot was ready.

**Root cause:** the original settle test required **all 12 joints
simultaneously** within `settle_pos_tol = 0.20` rad of the home pose **and**
nearly zero velocity. This is an open-loop position-PID robot with no gravity
compensation, so the load-bearing joints (hips) droop and never sit within
0.20 rad of target at the same time → the `converged` branch was effectively
unreachable → the phase always fell through to the timeout. (The home pose was
being commanded the whole time; there was just no early exit, and no settle
logging, so it looked like nothing happened until the timeout.)

**Fix (in `_settle_step`, `kinematic_gait.py`):** stop gating on position;
settle when the robot has **stopped moving** instead.
- Joint speed is estimated from Δposition/Δt between ticks (bridge-independent —
  does not rely on `/joint_states` publishing velocity).
- Settle fires when, after `settle_min_s = 1.0` s, every joint stays slower than
  `settle_vel_tol = 0.10` rad/s **continuously** for `settle_still_s = 0.4` s;
  the dwell timer resets if any joint moves again.
- `settle_max_s = 4.0` s remains as a hard backstop.
- Worst `|state − home|` is now **logged for information only** (no longer gates).
- Added throttled diagnostics (~every 0.5 s): elapsed, worst pos-error (deg),
  max joint speed, and the current still-dwell — so the settle is visible and
  the tolerances can be tuned from real numbers.

**New tunables:** `settle_vel_tol`, `settle_min_s`, `settle_still_s`,
`settle_max_s` (all in `__init__`). `settle_pos_tol` is retained only for the
log line.

**Undo:** restore the previous `_settle_step` (position-AND-velocity `converged`
check) and the previous `__init__` settle block, or restore the file from
version control.

**Verified:** `py_compile` OK; settle state machine simulated offline — a robot
that reaches home and stops fires "stopped moving" ~2.1 s (well before timeout),
and a robot that never stops correctly falls back to the 4.0 s timeout.
