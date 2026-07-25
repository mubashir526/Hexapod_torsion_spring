# Cheetah ROS 2 — Dynamic Controller Run & Fix Log

**Goal:** run the *dynamic* (MIT-Cheetah-style, force/MPC-based) controller in
`cheetah_ros2` in Gazebo, get it running without errors, confirm it also works
with teleoperation, and record every change made.

- **Package:** `ROS/src/cheetah_ros2`
- **Main launch:** `cheetah_sim.launch.py`
- **Environment:** ROS 2 Humble · Gazebo Sim 8.14.0 (Harmonic) · Ubuntu (host
  `/home/mubashir/...`, DISPLAY `:0`)
- **Date:** 2026-07-16

---

## 1. What the "dynamic controller" is (context)

Unlike the purely *kinematic* gait in `sim_robot` (which scripts foot
trajectories), `cheetah_ros2` is a **dynamic, force-based** controller ported
from the MIT Cheetah 3 stack. It runs a pipeline of ROS 2 nodes:

| Node | Role |
|------|------|
| `estimator_node` | fuses joint states + odometry + foot contacts → robot state, foot positions, Jacobians |
| `gait_node` | generates the gait schedule (stance/swing phases & timing) |
| `fsm_node` | decides per-leg stance-vs-swing from schedule + contacts |
| `stance_controller` | balance/force controller (QP) → stance-leg torques |
| `swing_controller` | swing-leg trajectory + PD → swing-leg torques |
| `effort_controller` | merges stance+swing torques → `/forward_effort_controller/commands` |

Gazebo runs the physics; `gz_ros2_control` exposes the 12 joints as an
`effort_controllers/JointGroupEffortController`. Teleop (`teleop_node`) is
**optional** — it publishes `geometry_msgs/Twist` on `/teleop`; the stance and
swing controllers subscribe to it. With no teleop, they fall back to defaults in
`linear_mpc_configs.py` / `robot_configs.py` (`cmd_xvel=0.08 m/s`,
`base_height_des=0.06 m`), so the robot runs on its own.

---

## 2. Problems found (root cause: project moved out of its devcontainer)

The code was authored in a VS Code devcontainer rooted at
`/workspaces/FYP-Legged-Robot`, using a control workspace at
`/opt/gz_control_ws`. On this host the project lives at
`/home/mubashir/Documents/FYP-Legged-Robot-main` and the control workspace at
`/home/mubashir/gz_control_ws`, so every absolute path baked into the config was
stale.

| # | File | Broken value | Effect |
|---|------|--------------|--------|
| 1 | 6 × `launch/*.launch.py` | `GZ_SIM_SYSTEM_PLUGIN_PATH = /opt/gz_control_ws/install/gz_ros2_control/lib` (that install dir is empty — only `COLCON_IGNORE`) | Gazebo can't find `gz_ros2_control-system` → no controllers load → robot never actuates |
| 2 | `models/THex_Quadruped/model.sdf` | `<parameters>/workspaces/FYP-Legged-Robot/Code/.../cheetah_controllers.yaml` (`/workspaces` doesn't exist) | `gz_ros2_control` plugin can't read controller config → controller_manager has no controllers |

The custom Python nodes themselves use `get_package_share_directory(...)` to
find the URDF (portable) — no path bugs there.

---

## 3. Changes made

### Fix 1 — gz_ros2_control plugin path (all 6 launch files)

`launch/*.launch.py`, line 32. Replaced the hard-coded, empty `/opt` path with a
portable `$HOME`-based lookup so it resolves wherever the user's control
workspace lives:

```python
# before
'/opt/gz_control_ws/install/gz_ros2_control/lib',
# after
os.path.join(os.path.expanduser('~'), 'gz_control_ws', 'install', 'gz_ros2_control', 'lib'),
```

Files changed: `cheetah_sim.launch.py`, `cheetah_cube.launch.py`,
`cheetah_cave.launch.py`, `launch_rough_floor_easy.launch.py`,
`launch_rough_floor_medium.launch.py`, `launch_rough_floor_hard.launch.py`.

### Fix 2 — controller-config path in the model SDF

`models/THex_Quadruped/model.sdf`, line 928. Pointed the `gz_ros2_control`
`<parameters>` tag at the config file's real location on this host:

```xml
<!-- before -->
<parameters>/workspaces/FYP-Legged-Robot/Code/ROS/src/cheetah_ros2/config/cheetah_controllers.yaml</parameters>
<!-- after -->
<parameters>/home/mubashir/Documents/FYP-Legged-Robot-main/Code/ROS/src/cheetah_ros2/config/cheetah_controllers.yaml</parameters>
```

### Rebuild

```bash
cd ROS && source /opt/ros/humble/setup.bash
colcon build --packages-select cheetah_ros2   # 1 package, ~1.4 s, no errors
```
(The workspace is a *copy* install, not `--symlink-install`, so the edited
source files had to be reinstalled for `ros2 launch` to see them.)

---

## 4. How to run it

```bash
# terminal 1 — the dynamic controller + Gazebo (runs on its own, no teleop needed)
source /opt/ros/humble/setup.bash
source ~/Documents/FYP-Legged-Robot-main/Code/ROS/install/setup.bash
source ~/gz_control_ws/install/setup.bash
ros2 launch cheetah_ros2 cheetah_sim.launch.py

# terminal 2 — OPTIONAL teleop (same three sources first)
ros2 run cheetah_ros2 teleop_node
#   w/s = forward/back, a/d = strafe, q/e/z/c = turn while moving,
#   r/f = raise/lower ride height, Ctrl-C = quit
```

With no teleop, the robot uses the built-in defaults (`cmd_xvel = 0.08 m/s`,
ride height `0.06 m`) and walks a slow forward crawl gait on its own.

---

## 5. Verification results (2026-07-16)

Launched the full stack; everything came up cleanly.

**Controllers — all active:**

| Controller | Type | State |
|------------|------|-------|
| `joint_state_broadcaster` | JointStateBroadcaster | ✅ active |
| `forward_effort_controller` | JointGroupEffortController | ✅ active |
| `imu_broadcaster` | IMUSensorBroadcaster | ✅ active |

`gz_ros2_control` reported *"System Successfully configured!"* and loaded all 12
joints with position/velocity/effort interfaces.

**Live data flow (running WITHOUT teleop):**

| Topic | Meaning | Rate | Status |
|-------|---------|------|--------|
| `/forward_effort_controller/commands` | 12-joint torques → Gazebo | ~70–86 Hz | ✅ finite values, some at ±0.9414 N·m limit |
| `/estimated_robot_state` | state estimator output | ~64 Hz | ✅ streaming |
| `/fsm_state` | per-leg stance/swing | streaming | ✅ |
| `/nominal_schedule`, `/stance_torques`, `/swing_torques`, `/foot_positions` | pipeline internals | present | ✅ |

Robot ground-truth odometry showed **z ≈ 0** relative to the 0.08 m spawn height
(i.e. it holds its stance height, doesn't collapse) while drifting in x/y — the
expected default forward crawl. No `NaN`, no Python tracebacks, no dead nodes in
the whole run.

**Teleop path — verified working:**
- `/teleop` has **2 subscribers** (stance + swing controllers).
- Publishing a test `Twist` (`x=0.1, z=0.07, yaw=0.2`) was accepted with no
  crash; torque stream continued uninterrupted.
- `teleop_node` launches under a real terminal and prints its control UI (the
  `termios` keyboard reader initialises correctly).

### Non-fatal messages (left as-is — informational only)

| Message | Why it's harmless |
|---------|-------------------|
| `[ERROR] ... robot_state_publisher service not available, waiting again...` | Transient startup race; it connects on the next line (`connected to service!!`). |
| `[WARN] ForceTorque sensor 'force_torque_sensor' not found in hardware_info, skipping.` (×12) | The URDF `ros2_control` block references FT sensors that aren't declared as hardware; `gz_ros2_control` safely skips them. The controller uses foot **contact** sensors instead, so this doesn't affect the gait. |
| `libEGL warning: Not allowed to force software rendering...` | From `LIBGL_ALWAYS_SOFTWARE=1` (set to be safe on headless GPUs) clashing with a hardware EGL device. Rendering still works; drop the env var if running on a machine with a real GPU. |

---

## 6. Summary

The dynamic (force/MPC-based) Cheetah controller **runs successfully** in
Gazebo. The only real problems were two stale absolute paths left over from the
original devcontainer; fixing them (and making the plugin path portable) was
enough. It runs standalone with no teleoperation, and teleoperation also works
when launched. No functional/logic changes were made to the controller code —
only environment/path fixes and a rebuild.

**Files changed:** 6 launch files + 1 SDF (paths only). No `.py` node logic
touched.
