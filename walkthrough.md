# Walkthrough: Kinematic Gait Improvements

## Files Modified

### [kinematic_gait.py](file:///home/mubashir/Pictures/FYP-Legged-Robot-main/Code/ROS/src/sim_robot/sim_robot/kinematic_gait.py)

#### Issue 1: Homing Ramp (lines ~191–420)

**Before**: The settle logic instantly commanded the home pose and only waited for the robot to stop moving (`max_speed < 0.10 rad/s`). Position error was logged but not gated — the robot "settled" with 32.9° error, then the first gait step caused a violent slam.

**After**: Two-phase homing sequence:
1. **RAMP** (1.5s) — Smooth cubic Hermite interpolation from the robot's actual current position → home pose. Joint positions are captured once all `/joint_states` feedback arrives, then smoothly interpolated.
2. **HOLD** — Commands the home pose and waits for **both** speed (`< 0.10 rad/s`) **and** position (`< 5°`) convergence before starting the gait.

The ramp uses `3α² - 2α³` (smoothstep) for jerk-free motion. The hard timeout increased from 4s → 6s to accommodate the ramp.

---

#### Issue 2: Graph Titles with Spring Config (lines ~44–53, ~483–525, all suptitle calls)

**Spring config subscription**: Subscribes to `/gait/spring_config` (latched `String` topic, JSON payload) published by the launch file. Caches `spring_mode` (`"none"` / `"native"`) and `spring_config` (the full `SPRING_CONFIG` dict).

**`_spring_title_str()` helper**: Builds a human-readable string like:
- `"Baseline (no spring)"`
- `"Spring (native) — knee: kx=0.15 N·m/rad, θ₀=-50.0°"`

**All 5 `fig.suptitle()` calls updated**:
- Command vs State
- Torque Magnitude
- Applied Motor Effort
- Torque vs Angle (per-leg)
- Effort vs Angle (per-leg)

**`run_info.txt`** now includes:
```
spring_mode:      native
spring_config:    {"hip": {...}, "knee": {...}, "foot": {...}}
spring_summary:   Spring (native) — knee: kx=0.15 N·m/rad, θ₀=-50.0°
```

---

#### Issue 3: Auto-Shutdown (lines ~1140–1150)

After `save_data()` and `rclpy.shutdown()`, sends `SIGINT` to the Gazebo process via `pkill -INT -f 'ruby.*gz.*sim'`. Since Gazebo is a `required` process in the launch system, its exit cascades to shut down bridge, camera_recorder, etc. in Terminal 1.

---

### [spring_experiment.launch.py](file:///home/mubashir/Pictures/FYP-Legged-Robot-main/Code/ROS/src/sim_robot/launch/spring_experiment.launch.py)

Added spring config publishing:
1. Reads `SPRING_CONFIG` from `make_spring_models.py` (via `exec()` since it's a standalone script, not a module).
2. Publishes `{"mode": "none|native", "config": {SPRING_CONFIG}}` as JSON on `/gait/spring_config` using `ros2 topic pub --once` with `transient_local` QoS, so `kinematic_gait` (started later) still receives it.

---

## Build Verification

All changes built successfully with `colcon build --packages-select sim_robot --symlink-install` — zero errors, zero warnings.

## How to Test

```bash
# Terminal 1: Launch with spring
cd FYP-Legged-Robot/Code/ROS
source install/setup.bash
ros2 launch sim_robot spring_experiment.launch.py spring:=native

# Terminal 2: Run gait
cd FYP-Legged-Robot/Code/ROS
source install/setup.bash
ros2 run sim_robot kinematic_gait
```

**What to observe**:
1. `[homing] Ramp started` log, then smooth interpolation diagnostics with decreasing position error
2. `[homing] Ramp complete` → `[settle]` logs showing pos + speed convergence
3. `=== Home pose reached (settled, ...)`  with `worst|pos-home|` now much lower than 32.9°
4. Graph titles include spring config info (e.g. `"Spring (native) — knee: kx=0.15 N·m/rad, θ₀=-50.0°"`)
5. `run_info.txt` includes `spring_mode`, `spring_config`, and `spring_summary`
6. After 5 cycles + data save: `"Shutting down Gazebo simulation..."` — Terminal 1 exits automatically
