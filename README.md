# T-Quad Legged Robot — Simulation with Passive Torsion Spring

A ROS 2 simulation of the **THex_Quadruped** (T-Quad) robot with **passive gravity compensation** via torsion springs. This project lets you run the robot's kinematic gait in Gazebo Harmonic and measure how a parallel elastic spring at each joint reduces the motor torque required to hold the stance.

The robot has **4 legs × 3 joints** (hip, knee, foot) = **12 revolute actuators**. You can selectively attach a native SDF spring to any combination of joint types (hip, knee, foot), configure stiffness and rest angle independently for each, and compare motor effort between sprung and unsprung runs.

---

## Table of Contents

1. [Prerequisites](#1-prerequisites)
2. [Installation](#2-installation)
3. [Running the Simulation](#3-running-the-simulation)
   - [Baseline (No Spring)](#31-baseline-no-spring)
   - [With Spring](#32-with-spring)
   - [Camera Recording](#33-camera-recording)
4. [Configuring Spring Parameters](#4-configuring-spring-parameters)
   - [SPRING_CONFIG Dictionary](#41-spring_config-dictionary)
   - [Parameter Reference](#42-parameter-reference)
   - [Configuration Examples](#43-configuration-examples)
5. [Rebuilding After Parameter Changes](#5-rebuilding-after-parameter-changes)
6. [Comparing Runs](#6-comparing-runs)
7. [Project Structure](#7-project-structure)

---

## 1. Prerequisites

You need a machine running **Ubuntu 22.04** with **ROS 2 Humble** already installed and sourced.

Verify your ROS 2 installation:

```bash
echo $ROS_DISTRO
# Should print: humble
```

If it prints nothing, source ROS first:

```bash
source /opt/ros/humble/setup.bash
```

---

## 2. Installation

Clone the repository and run the setup script. It installs **all** dependencies (Gazebo Harmonic, Python libraries, ROS 2 packages) and builds the workspace:

```bash
# Clone the repo
git clone https://github.com/mubashir526/Hexapod_torsion_spring.git
cd Hexapod_torsion_spring/Code

# Make the setup script executable and run it
chmod +x setup_env.sh
./setup_env.sh
```

The script will:
1. Verify your ROS 2 environment
2. Install system build tools (`colcon`, `rosdep`, `ffmpeg`, etc.)
3. Install ROS 2 Control and visualization packages
4. Install Python dependencies from `requirements.txt` (`numpy`, `matplotlib`, `scipy`, `pandas`, `opencv-python`, `onnxruntime`, `osqp`, `pin`)
5. Build `gz_ros2_control` underlay (if missing)
6. Resolve workspace dependencies via `rosdep`
7. Build the workspace packages (`sim_robot`, `cheetah_ros2`)

> **Note on `elevation_mapping`:** The `elevation_mapping` package is ignored by default (via `ROS/src/elevation_mapping/COLCON_IGNORE`) because it relies on external packages (`grid_map_core` and `kindr_ros`) that lack standard `rosdep` keys on ROS 2 Humble. Since `elevation_mapping` is not required for the torsion spring torque-reduction experiment, skipping it prevents build failures out of the box. If you need to build it, clone `kindr` into `ROS/src/` and remove `ROS/src/elevation_mapping/COLCON_IGNORE` (or use `colcon build --packages-skip elevation_mapping`).

After the script finishes, source the workspace:

```bash
cd ROS
source install/setup.bash
```

> **Note:** You must run `source install/setup.bash` every time you open a new terminal.

---

## 3. Running the Simulation

Every experiment uses **two terminals**. Terminal 1 launches Gazebo + the robot. Terminal 2 runs the gait controller.

### 3.1 Baseline (No Spring)

This runs the robot with **no spring** on any joint — pure motor control. Use this as the control group to compare against.

**Terminal 1 — Launch Gazebo:**

```bash
cd ROS
source install/setup.bash
ros2 launch sim_robot spring_experiment.launch.py spring:=none
```

**Terminal 2 — Run the gait:**

```bash
cd ROS
source install/setup.bash
ros2 run sim_robot kinematic_gait
```

The gait node will:
- Home the robot to its stance pose
- Run **5 gait cycles** at 50 Hz sampling
- Auto-stop and save data to `experiment/runN/` (where N increments automatically)

### 3.2 With Spring

This runs the robot with **native SDF springs** on the actuator types you've enabled in `SPRING_CONFIG` (see [§4](#4-configuring-spring-parameters) — by default, only **knees** have a spring).

**Terminal 1:**

```bash
ros2 launch sim_robot spring_experiment.launch.py spring:=native
```

**Terminal 2:**

```bash
ros2 run sim_robot kinematic_gait
```

### 3.3 Camera Recording

Add `record:=true` to record timestamped, torque-overlaid video from both a fixed and a chase camera:

```bash
ros2 launch sim_robot spring_experiment.launch.py spring:=native record:=true
```

This uses a special world file with the Sensors render system and starts a `camera_recorder` node that writes MP4s into the experiment's `runN/` folder.

> Without `record:=true` the simulation runs headless (faster, no rendering overhead).

---

## 4. Configuring Spring Parameters

All spring configuration lives in **one file**:

```
ROS/src/sim_robot/models/THex_Quadruped/make_spring_models.py
```

Open it and edit the `SPRING_CONFIG` dictionary near the top of the file (around line 67).

### 4.1 SPRING_CONFIG Dictionary

```python
SPRING_CONFIG = {
    "hip":  {"enabled": False, "kx": 0.20, "ref_mode": "data"},
    "knee": {"enabled": True,  "kx": 0.40, "ref_mode": "fixed", "ref_deg": -30.0},
    "foot": {"enabled": False, "kx": 0.35, "ref_mode": "data"},
}
```

Each key (`"hip"`, `"knee"`, `"foot"`) controls **all 4 joints** of that type (e.g. `"knee"` controls `fr_knee`, `br_knee`, `bl_knee`, `fl_knee`).

### 4.2 Parameter Reference

| Parameter | Type | Description |
|---|---|---|
| `enabled` | `True` / `False` | Whether this actuator type gets a spring. `False` = no spring (stiffness stays 0). |
| `kx` | float (N·m/rad) | **Spring stiffness.** How hard the spring pushes per radian of deflection. Higher = stiffer spring = more gravity assist, but risks destabilising the controller. Typical range: `0.10` – `0.50`. |
| `ref_mode` | `"fixed"` or `"data"` | How the spring's **rest angle** (θ₀) is determined. See below. |
| `ref_deg` | float (degrees) | *(Only when `ref_mode = "fixed"`)* The rest angle in **degrees**. The spring produces zero torque at this angle and pushes the joint toward it. |

**`ref_mode` explained:**

| Mode | What it does | When to use |
|---|---|---|
| `"fixed"` | Uses the angle you set in `ref_deg` as the rest angle for **all 4 joints** of that type (not mirror-aware). | Quick experiments: pick an angle and see what happens. |
| `"data"` | Computes the rest angle **per joint** from measured baseline torques (`HOLD` dict) so the spring assists in the **correct direction** for each leg (mirror-aware). Uses `ASSIST_FRAC` to control how much of the gravity load to cancel. | Recommended for serious tuning — automatically handles the left/right leg mirroring. |

**`ASSIST_FRAC`** (line 75): Only affects `ref_mode = "data"`. Controls what fraction of the measured DC holding torque the spring should cancel:
- `1.0` = cancel the full measured gravity load (aggressive)
- `0.5` = cancel half (conservative)
- Lower it if a joint becomes unstable

### 4.3 Configuration Examples

**Spring on knees only (fixed angle):**

```python
SPRING_CONFIG = {
    "hip":  {"enabled": False, "kx": 0.20, "ref_mode": "data"},
    "knee": {"enabled": True,  "kx": 0.40, "ref_mode": "fixed", "ref_deg": -30.0},
    "foot": {"enabled": False, "kx": 0.35, "ref_mode": "data"},
}
```

**Spring on all joints (data-driven):**

```python
SPRING_CONFIG = {
    "hip":  {"enabled": True,  "kx": 0.20, "ref_mode": "data"},
    "knee": {"enabled": True,  "kx": 0.25, "ref_mode": "data"},
    "foot": {"enabled": True,  "kx": 0.35, "ref_mode": "data"},
}
```

**Spring on knees and feet, different stiffnesses:**

```python
SPRING_CONFIG = {
    "hip":  {"enabled": False, "kx": 0.20, "ref_mode": "data"},
    "knee": {"enabled": True,  "kx": 0.50, "ref_mode": "data"},
    "foot": {"enabled": True,  "kx": 0.15, "ref_mode": "fixed", "ref_deg": 60.0},
}
```

**No spring at all (same as spring:=none, but via config):**

```python
SPRING_CONFIG = {
    "hip":  {"enabled": False, "kx": 0.20, "ref_mode": "data"},
    "knee": {"enabled": False, "kx": 0.40, "ref_mode": "data"},
    "foot": {"enabled": False, "kx": 0.35, "ref_mode": "data"},
}
```

---

## 5. Rebuilding After Parameter Changes

After editing `SPRING_CONFIG`, you must **regenerate** the SDF model files and **rebuild** the workspace. Here are the exact commands:

```bash
# Step 1: Regenerate the model SDF files
cd ROS/src/sim_robot/models/THex_Quadruped/
python3 make_spring_models.py
```

You will see output confirming which joints are sprung:

```
Generating spring model variants from model.sdf
  wrote model_effort.sdf
  wrote model_spring_native.sdf

Springs ENABLED on:  knee
Springs DISABLED on: hip, foot
```

```bash
# Step 2: Rebuild the workspace
cd ROS
colcon build --symlink-install

# Step 3: Source the workspace
source install/setup.bash

# Step 4: Launch and test
ros2 launch sim_robot spring_experiment.launch.py spring:=native
```

> **Important:** You must re-source `install/setup.bash` after every rebuild, or the old model files will be used.

---

## 6. Comparing Runs

After running a baseline and a spring experiment (each auto-saves to `experiment/runN/`), compare the motor torque reduction:

```bash
ros2 run sim_robot compare_runs experiment/run1 experiment/run2
```

This prints a per-joint table showing the **applied motor torque** and the percentage change. A positive `%` means the spring **reduced** motor effort on that joint.

---

## 7. Project Structure

```
Code/
├── setup_env.sh                     # One-click dependency installer
├── requirements.txt                 # Python dependencies
├── ROS/
│   └── src/
│       ├── sim_robot/               # Main simulation package
│       │   ├── launch/
│       │   │   └── spring_experiment.launch.py
│       │   ├── models/THex_Quadruped/
│       │   │   ├── model.sdf                  # Base robot (do NOT edit)
│       │   │   ├── model_effort.sdf           # Generated: baseline
│       │   │   ├── model_spring_native.sdf    # Generated: with springs
│       │   │   └── make_spring_models.py      # ← EDIT THIS for spring params
│       │   ├── sim_robot/
│       │   │   ├── kinematic_gait.py          # Gait controller
│       │   │   └── compare_runs.py            # Run comparison tool
│       │   └── config/
│       │       └── ros_gz_bridge_spring.yaml   # ROS↔Gazebo bridge config
│       ├── gz_joint_torsional_spring/         # Torsion spring C++ plugin (unused by default)
│       └── cheetah_ros2/                      # MPC-based controller (advanced)
└── torsion_spring_integration.md              # Detailed technical documentation
```

---

## Quick Reference

| Task | Command |
|---|---|
| Install everything | `./setup_env.sh` |
| Source workspace | `source ROS/install/setup.bash` |
| Run baseline (no spring) | `ros2 launch sim_robot spring_experiment.launch.py spring:=none` |
| Run with spring | `ros2 launch sim_robot spring_experiment.launch.py spring:=native` |
| Run with camera recording | Add `record:=true` to either launch command |
| Start the gait (separate terminal) | `ros2 run sim_robot kinematic_gait` |
| Edit spring parameters | Edit `SPRING_CONFIG` in `ROS/src/sim_robot/models/THex_Quadruped/make_spring_models.py` |
| Regenerate models after edit | `python3 make_spring_models.py` (from the model directory) |
| Rebuild after edit | `colcon build --symlink-install` (from `ROS/` directory) |
| Compare two runs | `ros2 run sim_robot compare_runs experiment/runA experiment/runB` |
