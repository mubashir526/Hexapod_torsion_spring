<!-- SUPERSEDED-BANNER -->
> [!IMPORTANT]
> **Superseded.** This document is kept as a working record. The authoritative
> analysis is [`ROS/report/experiment_report.md`](report/experiment_report.md),
> in which every quoted number is recomputed from the CSVs by
> `ROS/report/verify_claims.py`.
>
> Known-wrong values in this file:
>
> - The ≲5° heading-error threshold set here for using net forward displacement as the CoT denominator is **violated by every run**: measured heading error is 14.20° at baseline (median 12.41° across 91 runs). See §5.7 of the report for the resulting ±27% band on absolute CoT.

---
# Plan — Body Velocity, IMU, and Distance Covered Logging

**Date**: 2026-07-30
**Target**: `kinematic_gait` runs (the node used for the spring sweep and speed experiments)
**Status**: for review

---

## Context

Every experiment run so far logs joint-space data only — commanded/measured joint angles, force-torque, and commanded motor effort. Nothing records where the robot's **body** actually went. This means:

- There is no way to tell from data whether a run walked forward, shuffled in place, or fell over.
- In the 111-run spring sweep, the harmful corner (down to −122.7% effort "reduction") is uninterpretable — a fallen robot posts low knee torque for entirely the wrong reason.
- In the speed experiment, the 8-waypoint degenerate run's "improvements" were diagnosed from trajectory math (the foot never lifts), but could not be *confirmed* by measured displacement.

This plan adds three signals: **body pose** (→ distance covered), **body velocity**, and **IMU**.

### Interpretation of your answers

You selected the option that includes derived summary quantities, but then said **"no need to reason fall or success."** I've read that as:

- **Do** log raw pose/velocity/IMU, **and do** compute descriptive measurements (net displacement, path length, mean forward speed) into `run_info.txt`.
- **Do not** implement fall detection, success/failure verdicts, or any pass/fail thresholds.

This removes the need for any threshold constants. If you actually wanted the fall flag too, say so and I'll add it back — it's ~10 lines.

Also per your answer: **future runs only**, no backfill of the 6 existing speed runs.

---

## What already exists vs what's missing

Verified against your installed system (Gazebo Sim **8.14.0** Harmonic):

| Signal | Status | Evidence |
|---|---|---|
| **IMU sensor** | ✅ present | `model.sdf:39-44` — `imu_sensor` on `base_link`, `<update_rate>50</update_rate>`, gz topic `imu` |
| **IMU world system** | ✅ present | `gz-sim-imu-system` at `friction_world_cam.sdf:16` |
| **IMU survives generation** | ✅ yes | `imu_sensor` found in both `model_effort.sdf` and `model_spring_native.sdf` |
| **IMU ROS bridge** | ✅ present | `ros_gz_bridge_spring.yaml:93-97` → `/imu/data` as `sensor_msgs/msg/Imu` |
| **IMU consumed by kinematic_gait** | ❌ **no subscription at all** | `grep -n "imu\|Imu" kinematic_gait.py` returns nothing relevant |
| **Body pose / odometry plugin** | ❌ missing | no `OdometryPublisher`/`PosePublisher` in any model SDF |
| **Body pose ROS bridge** | ❌ missing | no `/odom`, `/tf`, or pose entry in any bridge yaml |
| **Odometry plugin available** | ✅ installed | `/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/libgz-sim8-odometry-publisher-system.so.8.14.0` |
| **Bridge supports Odometry** | ✅ yes | `nav_msgs/msg/Odometry` ↔ `gz.msgs.Odometry`, both directions, declared in `/opt/ros/humble/include/ros_gz_bridge/convert/nav_msgs.hpp:33-52` |
| **Joint velocity** | ⚠️ received but discarded | `kinematic_gait.py:343` caches `msg.velocity[i]` into `self.latest_state_vel`; nothing ever reads it (write-only) |

**So IMU needs no plumbing at all** — the sensor, world system, and bridge are all already in place. It only needs a subscriber and a logger. Body pose/velocity is the only part needing new plumbing.

---

## Design

### Signal sources

**1. Body pose + velocity — `gz-sim-odometry-publisher-system`**

Add the plugin to the model; it publishes pose *and* twist in one message on `/model/THex_Quadruped/odometry`, bridged to ROS `/odom` as `nav_msgs/msg/Odometry`.

Confirmed SDF parameters (extracted from the plugin binary): `odom_frame`, `robot_base_frame`, `odom_publish_frequency`, `odom_topic`, `dimensions`, `tf_topic`, `xyz_offset`, `rpy_offset`, `gaussian_noise`, `odom_covariance_topic`.

`<dimensions>3</dimensions>` is required — the default is 2, which gives planar x/y/yaw only and would silently drop vertical motion on a legged robot.

**Frame convention caveat**: by ROS `nav_msgs/Odometry` convention, `pose` is in the odom (world) frame but `twist` is in `child_frame_id` (body frame). Also, the plugin derives twist by **finite-differencing pose internally**, so it is not ground-truth velocity. Both are fine for gait analysis, but the plan logs pose *and* twist so world-frame velocity can be re-derived offline from pose with a proper filter as a cross-check.

**2. IMU — existing `/imu/data`**

Subscribe to the already-bridged topic. Gives orientation (quaternion), angular velocity, and linear acceleration — with Gazebo's sensor noise model applied, unlike odometry's clean pose. Keeping both odometry orientation *and* IMU orientation is deliberate: it gives a sim-truth vs noisy-sensor comparison for free, which matters if the RL pipeline ever needs to be validated against these runs.

**3. Distance covered — derived offline from logged pose**

The primary consumer of this is **Cost of Transport**, so the definition is chosen to serve that (see §"Cost of Transport" below):

```
d = Δy_net = y_end − y_start        # PRIMARY: forward axis, the commanded direction
```

with vertical (z) excluded entirely — this is level ground, and body bob is not transport.

Also computed, as **diagnostics only** (never in the CoT denominator):

| Quantity | Formula | Purpose |
|---|---|---|
| lateral drift | `Δx_net` | how far off-axis it ended up |
| net horizontal magnitude | `√(Δx² + Δy²)` | alternative `d`; sanity cross-check |
| heading error | `atan2(\|Δx\|, Δy)` | quantifies misdirection in degrees |
| yaw drift | `Δψ` | did the body rotate (vs translate off-axis) |
| path length | `Σ\|Δp_horiz\|` | reported for completeness, **not** used for CoT |
| straightness ratio | `Δy_net / path_length` | 1.0 = perfectly straight; low = wandered |

---

## Cost of Transport

```
CoT = E / (m · g · d)          dimensionless
```

### Measured constants for this robot

| Quantity | Value | Source |
|---|---|---|
| Total mass `m` | **1.39847 kg** | sum of all 13 link masses in `model.sdf` |
| `g` | **9.8 m/s²** | Gazebo default — no explicit `<gravity>` in either world SDF |
| `m·g` | **13.71 N** | |

So `CoT = E / (13.71 × d)`, `d` in metres.

**Scale estimate**: stance sweep is `T = 6` cm/cycle, so 5 cycles ≈ 0.30 m if there is no slip. With all-12-joint absolute work of 12.1 J (measured, below), that gives **CoT ≈ 2.9** — plausible for a small servo quadruped (ASIMO ≈ 3.2, ANYmal ≈ 1.2, passive walkers ≈ 0.2). Estimate only; no pose data exists yet to confirm 0.30 m.

### Why net displacement, not path length

**1. Path length rewards drift, which inverts the metric.** The gait is open-loop and designed to go straight in +y, so lateral/yaw deviation is a defect, not intended transport. Because `d` is in the *denominator*, a robot that wanders gets a larger `d` and therefore a **flatteringly lower** CoT than one walking perfectly straight.

**2. Path length is sampling-rate dependent; net displacement is not.** `Σ|Δp|` is a sum of magnitudes, so error can never cancel — the estimate is biased high and the bias *grows with sample rate* (the coastline paradox). Sampling the identical walk at 200 Hz instead of 50 Hz would report a longer "distance travelled." Three real contributors for a legged robot: per-step body bob (mitigated by excluding z), lateral sway where net motion per cycle ≈ 0 but `Σ|Δx|` accumulates anyway, and any odometry jitter. Net displacement uses two endpoints and is immune.

### How Δy accommodates drift in x

It does **not** credit x-drift — and that is precisely the mechanism by which drift is penalised. Energy spent drifting sideways stays in the **numerator** (the motors really did that work) while contributing **nothing to the denominator**, so CoT rises. The penalty is automatic; no extra term is needed.

Worked comparison — robot B drifts 10 cm sideways and spends 4% more energy doing it:

| | Δy | Δx | E | `d = Δy` → CoT | `d = √(Δx²+Δy²)` → CoT |
|---|---|---|---|---|---|
| A (straight) | 0.30 m | 0.00 m | 12.0 J | 12.0/(13.71×0.300) = **2.92** | **2.92** |
| B (drifts) | 0.30 m | 0.10 m | 12.5 J | 12.5/(13.71×0.300) = **3.04** ✅ worse | 12.5/(13.71×0.316) = **2.88** ❌ *better* |

With `Δy` the drifting robot is correctly ranked worse. With the net-magnitude denominator it is ranked *better* than the robot that walked straight — the wrong answer.

**The trade-off, stated honestly**: `Δy` conflates "inefficient" with "poorly aimed." A run that ends 45° off-axis has its CoT inflated by `1/cos45° = 1.41×` even if the locomotion itself was efficient. That is defensible for a gait *intended* to go straight, and it is the conservative direction (you can never accidentally flatter your result). But it means the heading-error diagnostic must be reported alongside so a reader can tell *why* a CoT is high.

**Decision rule**: if measured heading error stays small (≲ 5°), `Δy ≈ √(Δx²+Δy²)` and the choice is immaterial — use `Δy`. If runs turn out to curve substantially, `Δy` stops representing transport at all (imagine a quarter-circle path ending mostly in +x with `Δy ≈ 0`); in that case switch to net horizontal magnitude and state the change explicitly.

### Where start and end positions come from

**Start** = first row of `body_state.csv`. **End** = last row. Three things make this correct rather than arbitrary:

**1. The odom frame origin cancels.** Because `d` is a *difference* of two poses expressed in the same fixed frame, it does not matter where the `odom` frame's origin sits (world origin, spawn point, or an `<xyz_offset>`). Any constant offset subtracts out. This holds because the frame is fixed for the whole run — nothing resets it mid-run.

**2. Start is post-warmup, not the spawn pose.** `torque_logging_loop()` returns early while `not self.recording` (`:365-366`), and `recording` only becomes `True` after the warm-up cycles finish (`:306-314`, which also resets `start_time` and creates `run_dir`). So the first logged sample is taken with the robot **already walking in steady state** — the warm-up distance is deliberately excluded. Sequence is: wait for joint-state topics → warm-up gait (currently 1 cycle, 3 in the revised plan) → recording begins.

**3. ⚠️ The energy window must match the distance window exactly.** This is the easiest way to get CoT silently wrong. `E` must be integrated over the *same* sample range used for `d`. Both come from streams index-aligned on `torque_timestamps`, so taking first→last row of the same CSV set guarantees it. Using the *spawn* position as "start" while integrating energy only from post-warmup would put a longer window in the denominator than the numerator and **understate CoT**.

**Recommended: also report per-cycle CoT.** `_cycle_sample_ranges()` (`:731-762`) already maps gait-cycle indices → 50 Hz sample windows and is reused by both existing angle plots. Feeding it to CoT gives a per-cycle value, hence a mean ± sd **from a single run** — and lets the cycle-0 transient be dropped consistently from numerator and denominator together. Caveat: per-cycle displacement is only ~6 cm, so relative noise is higher than the whole-run figure.

### Numerator: three issues that matter more than the `d` choice

**1. Use all 12 joints, not just the knees.** Measured on `experiment_speed_freq/run1`: knees account for only **57%** of total mechanical work (6.88 J of 12.13 J). The existing `mechanical_work` metric in `generate_detailed_knee_analysis.py` is knee-only and would understate `E` by ~43%. All 12 joints are already in the CSVs.

**2. Absolute vs positive-only work is a 25% swing.** Measured on the same run:

| Definition | Energy |
|---|---|
| `Σ\|τ·Δθ\|` — absolute, braking also costs | **12.125 J** |
| `Σ max(0, τ·Δθ)` — positive only, negative work free | **9.686 J** |

For geared servos with no regeneration, absolute is the more honest choice (backdriving a geared motor genuinely dissipates). Either is acceptable — state which.

**3. ⚠️ Mechanical CoT will hide the spring's benefit — likely by ~10×.** From the earlier spring analysis: at the optimum, mean torque fell **34%** but mechanical work fell only **4.2%**, because the torque goes into *statically holding* the leg where `dθ ≈ 0` and so contributes almost nothing to `∫τ·ω`.

| Metric | Change at spring optimum |
|---|---|
| Mechanical work → mechanical CoT | **−4.2%** (reads as "spring does nothing") |
| Copper loss `∝ τ_rms²` → electrical CoT | **≈ −44%** (RMS 0.2856 → 0.2133, squared) |

A stalled servo holding a load draws current and does zero mechanical work. For a gravity-compensating spring, the entire benefit lives in the static holding term that mechanical work ignores. **Report electrical CoT as primary** (or at minimum a `∫τ²dt` copper-loss proxy), with mechanical CoT secondary — otherwise the headline understates the spring by roughly an order of magnitude.

Electrical needs two motor constants: `P_copper = (τ/k_t)² · R`. Without a datasheet, the `∫τ²dt` proxy (units N²·m²·s) still gives the correct *relative* comparison between configurations, which is what config selection depends on.

### Sampling and alignment

`torque_logging_loop()` (`kinematic_gait.py:363`) already runs at **50 Hz** (`torque_freq`) and is where `latest_torque` / `latest_effort` / `latest_state_pos` get snapshotted. IMU is 50 Hz and odometry will be configured to 50 Hz, so the new signals snapshot in the *same* loop, at the *same* tick, sharing `torque_timestamps` — 1:1 index alignment with every existing 50 Hz stream, no interpolation.

This also means the new logging is unaffected by `target_freq` changes (the gait speed lever), since `torque_freq` is independent.

---

## Files to change

### 1. `src/sim_robot/models/THex_Quadruped/model.sdf`

Add one model-level plugin (sibling of the existing `JointPositionController` blocks at `:922-1005`):

```xml
<plugin filename="gz-sim-odometry-publisher-system"
        name="gz::sim::systems::OdometryPublisher">
  <odom_frame>odom</odom_frame>
  <robot_base_frame>base_link</robot_base_frame>
  <odom_publish_frequency>50</odom_publish_frequency>
  <dimensions>3</dimensions>
</plugin>
```

**No change needed to `make_spring_models.py`.** Verified: `load_base()` (`:234-239`) does `ET.parse(BASE)` on `model.sdf` and carries the entire tree into both generated variants — which is exactly why `imu_sensor` already appears in both. The generator only *adds* (effort publisher, chase camera) and *modifies* (spring stiffness, initial positions); it never strips unknown plugins.

**But the generator must be re-run** to propagate the new plugin into the two files actually loaded at launch:

```bash
cd src/sim_robot/models/THex_Quadruped && python3 make_spring_models.py
```

⚠️ Note: `make_spring_models.py`'s `SPRING_CONFIG` currently holds stale values from the old sweep (`knee: kx=0.5, ref_deg=-50, enabled=True`). Running the generator as-is will bake that spring into `model_spring_native.sdf`. For baseline (`spring:=none`) runs this is irrelevant — that path loads `model_effort.sdf`, which has no spring — but set `SPRING_CONFIG` deliberately before generating if you intend to use `spring:=native`.

### 2. `src/sim_robot/config/ros_gz_bridge_spring.yaml`

Append one entry (matching the existing GZ_TO_ROS style at `:86-97`):

```yaml
# Body odometry (pose + twist)
- ros_topic_name: "/odom"
  gz_topic_name: "/model/THex_Quadruped/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
```

The gz topic name follows the plugin's default `/model/<model_name>/odometry`; the spawn name is `THex_Quadruped` (`spring_experiment.launch.py:76`).

### 3. `src/sim_robot/sim_robot/kinematic_gait.py` — the bulk of the work

| Location | Change |
|---|---|
| imports (top) | `from sensor_msgs.msg import Imu`, `from nav_msgs.msg import Odometry` |
| `__init__`, near `:122-139` | Add caches mirroring the existing `latest_*` pattern: `self.latest_odom_pos/quat/lin_vel/ang_vel`, `self.latest_imu_quat/ang_vel/lin_acc`, plus `self.odom_available` / `self.imu_available` flags |
| `__init__`, near `:151-175` | Two subscriptions, following the `rl_obs.py:44` precedent: `create_subscription(Odometry, '/odom', self.odom_cb, 1)` and `create_subscription(Imu, '/imu/data', self.imu_cb, 1)` |
| new callbacks, near `:357` | `odom_cb` and `imu_cb` — cache latest values and set the available flag (same shape as the existing `joint_effort_cb`) |
| `torque_logging_loop()` `:363-384` | Append the cached body/IMU values to new per-sample lists, inside the existing gating, so they stay index-aligned with `torque_timestamps` |
| `export_csvs()` `:867+` | New CSV writer → `body_state.csv` (schema below) |
| `_write_run_info()` near `:465-490` | Add derived lines (see CoT section for definitions): `forward_displacement_m` (= `Δy`, the CoT denominator), `lateral_drift_m`, `net_horizontal_m`, `heading_error_deg`, `yaw_drift_deg`, `path_length_m`, `straightness_ratio`, `mean_forward_speed_mps`, `odom_recorded`, `imu_recorded` |

**Note on the existing readiness-gate asymmetry**: `torque_logging_loop` gates on per-joint `torque_ready` and `state_ready` but has no equivalent for effort — a latent bug flagged in the earlier `kinematic_gait.py` review (it never fired in 111 runs, but is structurally unguarded). The new odom/IMU signals will have the *same* exposure: their caches initialise to zero, so a sample logged before the first message arrives would record a false 0. Recommend gating on `odom_available`/`imu_available` in the same `if` block rather than repeating that pattern — cheap to do correctly now.

### 4. `src/sim_robot/package.xml`

Add `<depend>nav_msgs</depend>`. Optional hygiene: `sensor_msgs`, `geometry_msgs`, and `std_msgs` are all already imported by the code but undeclared (only `rclpy` and `ros_gz_sim` are listed at `:10-11`) — it works because rclpy doesn't enforce declarations at runtime, but declaring them is correct.

### 5. New CSV: `experiment/runN/body_state.csv`

50 Hz, one row per `torque_timestamps` sample, index-aligned with all existing 50 Hz streams:

```
Time_Step, Time_s,
base_x, base_y, base_z,                                  # odom pose, world frame (m)
base_qx, base_qy, base_qz, base_qw,                      # odom orientation
base_roll, base_pitch, base_yaw,                         # derived from quaternion (rad), convenience
odom_vel_lin_x, odom_vel_lin_y, odom_vel_lin_z,          # odom twist, BODY frame (m/s)
odom_vel_ang_x, odom_vel_ang_y, odom_vel_ang_z,          # odom twist angular (rad/s)
imu_qx, imu_qy, imu_qz, imu_qw,                          # IMU orientation (noisy)
imu_ang_vel_x, imu_ang_vel_y, imu_ang_vel_z,             # IMU gyro (rad/s)
imu_lin_acc_x, imu_lin_acc_y, imu_lin_acc_z              # IMU accelerometer (m/s^2)
```

Written unconditionally when either source is available (mirroring how the effort CSV is gated on `effort_available` at `:973`).

---

## What does NOT need changing

- **No `colcon build` required.** Verified the full symlink chain: `install/share/sim_robot/{models,config}/… → build/sim_robot/… → src/sim_robot/…`. Editing `model.sdf`, the bridge yaml, or any `.py` takes effect immediately. (Python is editable via `sim-robot.egg-link`; data files are symlinked by `--symlink-install`.)
- **No `make_spring_models.py` code change** — only a re-run.
- **No world SDF change** — `gz-sim-imu-system` and `SceneBroadcaster` are already present in both worlds.
- **No launch file change** — the bridge yaml it already points at picks up the new entry.

---

## Verification

1. **Topic exists**: launch the sim, then `ros2 topic echo /odom --once` and `ros2 topic echo /imu/data --once`. If `/odom` is silent, the plugin didn't load — check `gz topic -l | grep odometry` to distinguish a plugin-load failure from a bridge misconfiguration.
2. **Rate check**: `ros2 topic hz /odom` should read ~50 Hz. If it reads ~2 Hz, `<odom_publish_frequency>` didn't take.
3. **3D check**: confirm `base_z` in `body_state.csv` actually varies (body bob during gait). If it's pinned at a constant, `<dimensions>3</dimensions>` was ignored and you're getting planar odometry.
4. **Alignment check**: `body_state.csv` row count must equal `joint_torques.csv` row count exactly (both driven by `torque_timestamps`). A mismatch means the append isn't inside the same gated block.
5. **Sanity on distance**: for the current 16-waypoint / 10 Hz baseline over 5 cycles (8 s), the stance sweep is `T=6` units per cycle in leg-local coordinates — confirm `forward_displacement_m` is positive, of plausible magnitude, and that `path_length ≥ net_displacement`.
6. **Cross-check velocity**: `mean_forward_speed_mps` computed from net Δy / duration should agree with the mean of the odom twist's forward component to within a few percent. A large disagreement points at the frame convention (body vs world) being mishandled.

---

## Open questions

1. **Forward axis**: the capstone report states forward motion is in **+y**, and `kinematics.py`'s stance sweeps y from `+T/2` to `−T/2`. I've assumed `forward_displacement = Δy` in the world frame. Worth confirming once run 1 produces data — if the robot actually advances in x, the CoT denominator is trivially re-pointed. Note the model is spawned unrotated (`-x 0 -y 0 -z 0.08`, no yaw argument at `spring_experiment.launch.py:76-77`), so body and world axes coincide at t=0.
4. **Motor constants for electrical CoT**: `k_t` (torque constant) and `R` (winding resistance) — do you have a datasheet or part number for the servos? Without them the `∫τ²dt` proxy still gives correct relative comparisons, but absolute electrical CoT needs both.
5. **CoT window**: whole recorded run (first→last sample) is the default. Per-cycle CoT via `_cycle_sample_ranges()` is recommended additionally for mean ± sd from a single run — confirm you want both.
2. **Fall/success gate**: excluded per your answer. If you later want the revised sweep plan's exclusion criterion to work automatically, this is the hook — `base_z` and `base_roll/pitch` are logged, so it becomes a pure offline computation with no re-runs needed.
3. **Backfill**: skipped per your answer. Consequence to note: the 8-waypoint degenerate-trajectory finding stays supported by trajectory math (foot height never leaves stance level) rather than by measured zero displacement.

---
---

# IMPLEMENTATION CHANGELOG — 2026-07-30

**Status**: implemented and smoke-tested end to end. All 6 changed files were git-tracked and clean beforehand, so every change is revertible.

## How to revert

Everything is a tracked-file modification — nothing was created or deleted in `src/`:

```bash
cd Code/ROS
git checkout -- src/sim_robot/models/THex_Quadruped/model.sdf \
                src/sim_robot/models/THex_Quadruped/model_effort.sdf \
                src/sim_robot/models/THex_Quadruped/model_spring_native.sdf \
                src/sim_robot/config/ros_gz_bridge_spring.yaml \
                src/sim_robot/sim_robot/kinematic_gait.py \
                src/sim_robot/package.xml
```

No rebuild needed after reverting (all paths are symlinked `install → build → src`). To revert only part of it, see the per-file sections below — each is independent except that `kinematic_gait.py`'s logging is inert without the SDF plugin + bridge entry (it just prints "skipping body_state.csv").

Diffstat:

```
 config/ros_gz_bridge_spring.yaml                   |   9 +
 models/THex_Quadruped/model.sdf                    |   6 +
 models/THex_Quadruped/model_effort.sdf             |   6 +
 models/THex_Quadruped/model_spring_native.sdf      |   6 +
 package.xml                                        |   4 +
 sim_robot/kinematic_gait.py                        | 242 ++++++-
 6 files changed, 272 insertions(+), 1 deletion(-)
```

## 1. `models/THex_Quadruped/model.sdf` — +6 lines

Added one model-level plugin immediately before `</model>`, after the last `JointStatePublisher` block:

```xml
<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
  <odom_frame>odom</odom_frame>
  <robot_base_frame>base_link</robot_base_frame>
  <odom_publish_frequency>50</odom_publish_frequency>
  <dimensions>3</dimensions>
</plugin>
```

`<dimensions>3</dimensions>` is essential — the default of 2 gives planar x/y/yaw only.

## 2. `model_effort.sdf` + `model_spring_native.sdf` — +6 lines each, GENERATED

Not hand-edited. Produced by re-running the generator:

```bash
cd src/sim_robot/models/THex_Quadruped && python3 make_spring_models.py
```

`make_spring_models.py` needed **no code change** — `load_base()` copies the whole `model.sdf` tree, so the new plugin propagated automatically (the same mechanism by which `imu_sensor` already appeared in both variants).

Verified idempotent w.r.t. springs: `model_spring_native.sdf` still carries `spring_reference −0.8727` / `spring_stiffness 0.5000` on the four knees, byte-identical to before. `git diff` on both generated files shows **only** the 6-line plugin insertion.

## 3. `config/ros_gz_bridge_spring.yaml` — +9 lines

Appended at end of file (bridge now has 42 entries):

```yaml
# Body odometry (pose in odom frame + twist in body frame), from the
# OdometryPublisher plugin on the model. Used for distance covered / body
# velocity / Cost of Transport.
- ros_topic_name: "/odom"
  gz_topic_name: "/model/THex_Quadruped/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
```

## 4. `package.xml` — +4 lines

Added `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs` as `<depend>`. Only `nav_msgs` is strictly new; the other three were already imported by the code but undeclared.

## 5. `sim_robot/kinematic_gait.py` — +242 lines

Six edits, in file order:

| # | Location | Change |
|---|---|---|
| 5a | imports (top) | `JointState` → `JointState, Imu`; new `from nav_msgs.msg import Odometry` |
| 5b | `__init__`, new block "2d. BODY STATE" after `theta_states_hf` | `odom_available` / `imu_available` flags, 7 `latest_*` caches, 7 per-sample history lists (`body_pos`, `body_quat`, `body_lin_vel`, `body_ang_vel`, `imu_quat`, `imu_ang_vel`, `imu_lin_acc`) |
| 5c | `__init__`, after the commanded-effort subscription loop | `create_subscription(Odometry, '/odom', …)` and `create_subscription(Imu, '/imu/data', …)` |
| 5d | after `joint_effort_cb` | new `odom_cb`, `imu_cb`, and static `_quat_to_rpy` (ZYX, with `asin` domain clamp) |
| 5e | end of `torque_logging_loop` | appends body/IMU snapshot each 50 Hz tick, inside the existing gate |
| 5f | end of `export_csvs`, plus new `_body_summary`, plus `_write_run_info` | writes `body_state.csv`; computes and emits the displacement summary |

### Deliberate design choice: NaN, not 0.0

`torque_logging_loop` appends **unconditionally** (preserving 1:1 alignment with `torque_timestamps`) but writes `NaN` — emitted as an empty CSV cell — until a source has actually reported. This avoids repeating the latent bug found in the effort path, where an uninitialised `0.0` cache would be logged as though it were real data ("robot at the origin, stationary and level" is indistinguishable from a genuine reading). Verified: 0 empty cells in the smoke-test run, i.e. both sources reported before sample 0.

### New output: `experiment/runN/body_state.csv`

28 columns, 50 Hz, index-aligned with every other 50 Hz CSV:
`Time_Step, Time_s, base_{x,y,z}, base_q{x,y,z,w}, base_{roll,pitch,yaw}, odom_vel_lin_{x,y,z}, odom_vel_ang_{x,y,z}, imu_q{x,y,z,w}, imu_ang_vel_{x,y,z}, imu_lin_acc_{x,y,z}`

### New `run_info.txt` fields

`odom_recorded`, `imu_recorded`, and a `body displacement (recorded window, warm-up excluded)` block: `forward_displacement_m` (the CoT denominator), `lateral_drift_m`, `net_horizontal_m`, `heading_error_deg`, `yaw_drift_deg`, `path_length_m`, `straightness_ratio`, `recorded_duration_s`, `mean_forward_speed_mps`.

**Not implemented, per your instruction** ("no need to reason fall or success"): no fall detection, no success/failure verdict, no thresholds. `base_z` and `base_roll/pitch` are logged, so adding it later is a pure offline computation needing no re-runs.

---

## Smoke test results

### Static validation — all pass
- XML well-formed: `model.sdf`, `model_effort.sdf`, `model_spring_native.sdf`, `package.xml`
- YAML parses; 42 bridge entries; `/odom` entry present and correct
- `python -m py_compile kinematic_gait.py` clean
- `git diff` on the two generated SDFs contains **only** the plugin insertion

### Unit tests of the new pure logic — all pass
- `_quat_to_rpy`: identity, yaw 45°, roll 45°, pitch 45° all exact; pitch 90° gimbal case correctly clamped (no `asin` domain error)
- `_body_summary` on a synthetic 0.30 m-forward / 0.10 m-lateral track: forward, lateral, net-horizontal, heading error, speed, and yaw drift all match hand-computed values to 1e-9
- **z correctly excluded** from `path_length` (verified: injected 0.08→0.081→0.079 body bob did not contribute)
- `path_length ≥ net_horizontal` invariant holds
- All-NaN pose input returns `{}` rather than crashing

### Live end-to-end run — pass
`spring:=none record:=false headless:=true`, 5 cycles, → `experiment/run1/`

- gz side: `/model/THex_Quadruped/odometry` present → **plugin loaded**
- ROS side: `/odom` and `/imu/data` both bridged
- `Body state (400 samples @ 50Hz) saved to body_state.csv [odom=True, imu=True]`
- **Row alignment exact**: `body_state` 400 = `joint_torques` 400 = `joint_commanded_effort` 400 = `joint_effort_vs_angle` 400
- **3D odometry confirmed**: `base_z` ranges 0.05856 → 0.06820 m (9.6 mm of body bob). Had `<dimensions>` defaulted to 2 this would have been constant.
- 0 empty cells → no leading-NaN gap
- `run_info.txt` summary matches an independent recomputation from the CSV exactly (forward 0.325898, lateral 0.077602, path 0.442004, heading 13.394°, speed 0.040839 m/s)

### First real Cost of Transport

| Quantity | Value |
|---|---|
| `d` (forward displacement) | **0.325898 m** |
| duration | 7.98 s |
| mean forward speed | 0.0408 m/s |
| `E` all 12 joints, absolute work | 12.3124 J |
| `E` all 12 joints, positive work | 9.8210 J |
| `m·g` | 13.7050 N |
| **CoT (absolute work)** | **2.757** |
| **CoT (positive work)** | **2.199** |

The pre-implementation estimate in the CoT section above predicted `d ≈ 0.30 m` and `CoT ≈ 2.9` — measured 0.326 m and 2.76. The estimate held.

Two things the real data now shows that were previously unknowable:
- **Lateral drift is not negligible**: 7.76 cm sideways over 32.6 cm forward, a **13.4° heading error**, and `straightness_ratio` 0.737. This is exactly the case where the net-vs-path choice matters — using `path_length` (0.442 m) as the denominator would have reported CoT 2.03 instead of 2.76, understating it by 26%.
- Heading error at 13.4° **exceeds the ≲5° threshold** in the decision rule above, so the `Δy`-vs-net-magnitude distinction is live rather than academic. `Δy` (0.3259) vs net-horizontal (0.3350) differ by 2.8%; `Δy` remains the recommended conservative choice, but the heading error must be reported with any CoT figure.

## Notes / cleanup

- The smoke test created **`experiment/run1/`** (a fresh `experiment/` directory, since the old one was deleted earlier). It is a throwaway artifact — safe to delete, and untracked so it won't affect git.
- **Sandbox gotcha discovered while testing**: `pkill -9 -f "gz sim"` self-matches, because the pattern appears verbatim in the invoking shell's own command line, so the script kills itself (silent exit 1, empty logs). Use the bracket trick — `pkill -9 -f "gz[ ]sim"`. `run_parameter_sweep.py:100-103` is *not* affected, since it invokes pkill via `subprocess.run([...])` and its own cmdline never contains the pattern.
- `/odom` reads ~21 Hz on `ros2 topic hz` while headless; that is wall-clock. The plugin's 50 Hz and the logger are both sim-time paced (`use_sim_time=True`), so they stay aligned — the gap is just real-time factor ≈ 0.43.
