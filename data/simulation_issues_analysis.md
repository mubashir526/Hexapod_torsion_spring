# THex Quadruped Simulation — Issues Analysis

## Context
Running `ros2 launch sim_robot start_world.launch.py` with the `kinematic_gait` node. The professor noted: *"The simulation in its current state has issues (unrelated to your work but affecting it regardless)."*

This analysis identifies **simulation-level issues** — problems in the physics setup, model definition, and sim-to-real gap — rather than issues in the kinematic gait code itself.

---

## 🔴 Critical Issue #1: No Joint Damping or Friction in the SDF Model

**File:** [model.sdf](file:///home/mubashir/Documents/FYP-Legged-Robot-main/Code/ROS/src/sim_robot/models/THex_Quadruped/model.sdf#L58-L61)

Every joint in the robot model has:
```xml
<dynamics>
  <spring_reference>0</spring_reference>
  <spring_stiffness>0</spring_stiffness>
</dynamics>
```

**What's missing:** There is **no `<damping>` and no `<friction>` parameter** in any joint's `<dynamics>` tag.

**Why this matters:**
- Without joint damping, the joints have **zero resistance to motion** other than the PID controller. In reality, servo motors have internal friction and back-EMF that naturally damp oscillations.
- This creates an **underdamped system** where the PID controller fights against the joint's inertia with no natural energy dissipation, causing the oscillatory/jittery torque behavior visible in your data.
- The torque plot shows wild, chaotic oscillations (0 to 0.9414 N⋅m constantly) — this is the PID controller overshooting back and forth because there is nothing to damp the response.

**What it should look like:**
```xml
<dynamics>
  <damping>0.01</damping>
  <friction>0.005</friction>
  <spring_reference>0</spring_reference>
  <spring_stiffness>0</spring_stiffness>
</dynamics>
```

---

## 🔴 Critical Issue #2: Unit Mismatch — Kinematics in Centimeters, SDF in Meters

**Files:** [kinematics.py](file:///home/mubashir/Documents/FYP-Legged-Robot-main/Code/ROS/src/sim_robot/sim_robot/kinematics.py#L8-L15) vs [model.sdf](file:///home/mubashir/Documents/FYP-Legged-Robot-main/Code/ROS/src/sim_robot/models/THex_Quadruped/model.sdf#L80)

The kinematics module defines link lengths in **centimeters**:
```python
L1 = 2.845  # in cm
L2 = 5.439
L3 = 2.637
L4 = 9.265
```

And the trajectory parameters are similarly in cm-scale:
```python
X = 15      # ~15 cm reach
S = -7      # ~7 cm below
A = 3       # ~3 cm swing height
T = 6       # ~6 cm stride length
```

But the SDF model uses **meters** (standard for Gazebo/SDF):
- Hip link CoM at `0.015013` (i.e. ~1.5 cm = 0.015 m) ✓
- Joint offsets like `0.028451` m (2.8 cm) ✓

**The trajectory is computed in cm-space but the resulting angles are applied to a meter-scale model.** While the inverse kinematics outputs *angles* (which are unitless), the trajectory planning assumes foot positions in centimeters. The foot workspace in cm-space may not match the actual workspace of the meter-scale model, potentially causing the IK to command positions near singularities or at the edges of the workspace — leading to jerky, unstable motion.

> [!IMPORTANT]
> The IK outputs angles so it "works," but the trajectory shape is designed for a different scale of workspace. This can cause trajectories that are physically unrealizable or near-singular for the actual simulated robot.

---

## 🔴 Critical Issue #3: PID Gains Are Poorly Tuned (No Systematic Tuning)

**File:** [model.sdf](file:///home/mubashir/Documents/FYP-Legged-Robot-main/Code/ROS/src/sim_robot/models/THex_Quadruped/model.sdf#L862-L945)

All 12 joints use identical PID gains:
```xml
<p_gain>5</p_gain>
<i_gain>0.1</i_gain>
<d_gain>0.1</d_gain>
```

**Problems:**
- **Same gains for all joints:** Hip, knee, and foot joints have very different inertias and load profiles. The hip bears almost the full body weight while the foot is a lightweight distal link. Using identical gains for all means they can't all be well-tuned.
- **P=5 is aggressive for this scale robot:** The robot's total mass is ~0.3 kg body + ~0.15 kg × 4 hips + lighter knee/foot links ≈ **1.3 kg total**. A P-gain of 5 with this low inertia and zero damping causes aggressive overshoot.
- **I-gain accumulates error:** With `i_gain=0.1`, the integral term accumulates over time and contributes to the oscillatory/drifting behavior, especially during stance phases when the error should be near-zero.
- **Insufficient D-gain:** `d_gain=0.1` is too low to compensate for the zero-damping joints. The D-gain is the only source of velocity-dependent opposition, and it's not enough.

---

## 🔴 Critical Issue #4: No Foot Contact Model — Point/Mesh Feet with No Rubber Tips

**File:** [model.sdf](file:///home/mubashir/Documents/FYP-Legged-Robot-main/Code/ROS/src/sim_robot/models/THex_Quadruped/model.sdf#L227-L234)

The foot links use STL mesh collision geometry:
```xml
<collision name='bl_foot_collision'>
  <geometry>
    <mesh>
      <uri>model://THex_Quadruped/meshes/bl_foot_collision.STL</uri>
    </mesh>
  </geometry>
</collision>
```

**What's missing:**
- **No `<surface>` tag on any foot collision** — There is no friction definition on the robot's feet. The ground plane has `mu=0.7` but the foot-ground contact pair uses Gazebo defaults for the foot side.
- **No contact parameters** (`<bounce>`, `<soft_cfm>`, `<soft_erp>`, `<kp>`, `<kd>`) — The contact stiffness is fully default, which for mesh-mesh contact in Gazebo can be unrealistically stiff, causing "bouncing" off the ground.
- **Mesh collision for feet is expensive and imprecise** — Using STL meshes for foot-ground contact is computationally expensive and prone to collision detection artifacts. Simple sphere or capsule primitives at the foot tips would be more stable.

> [!WARNING]
> Without proper foot friction and contact parameters, the feet likely **slip on the ground** during stance phase, which directly undermines any walking gait. This is almost certainly a major contributor to the robot's poor walking performance.

---

## 🟡 Significant Issue #5: `self_collide` Is Enabled

**File:** [model.sdf](file:///home/mubashir/Documents/FYP-Legged-Robot-main/Code/ROS/src/sim_robot/models/THex_Quadruped/model.sdf#L3)

```xml
<self_collide>true</self_collide>
```

With self-collision enabled and STL mesh collision geometries on all links, adjacent links in the kinematic chain (hip↔knee, knee↔foot) will likely collide with each other, generating **parasitic contact forces** that the PID has to fight against. This:
- Adds random, unpredictable forces to the torque readings
- Can cause the simulation to become jittery
- Is especially problematic if the collision meshes aren't carefully designed with gaps between adjacent links

---

## 🟡 Significant Issue #6: No Gravity Compensation or Feedforward Terms

The joint position controllers are pure PID — they receive a desired angle and try to reach it. But for a legged robot under gravity:
- **Stance leg joints** must constantly fight gravity to keep the body elevated. The PID integral slowly accumulates to provide this, but responds sluggishly.
- **Swing leg joints** have different dynamics (low load, fast motion) than stance legs (high load, slow motion).
- A real walking controller needs **feedforward torques** (gravity compensation) or at minimum different PID regimes for stance vs. swing — the current pure kinematic approach can't handle this.

---

## 🟡 Significant Issue #7: Controller Rate vs. Physics Rate Mismatch

**Files:**
- [kinematic_gait.py](file:///home/mubashir/Documents/FYP-Legged-Robot-main/Code/ROS/src/sim_robot/sim_robot/kinematic_gait.py#L18) — `target_freq = 10` Hz (100ms per step)
- [friction_world.sdf](file:///home/mubashir/Documents/FYP-Legged-Robot-main/Code/ROS/src/sim_robot/worlds/friction_world.sdf#L6) — `max_step_size = 0.001` (1000 Hz physics)
- Force/torque sensors at 50 Hz, joint state publishers at default rate

The kinematic gait runs at **10 Hz** — commanding a new position every 100ms. Between commands, the PID controller has 100 physics steps (at 1ms each) to try to reach the target. This is very slow for a walking gait and means:
- Abrupt jumps between trajectory waypoints (only 16 points × 10Hz ≈ 1.6 seconds per gait cycle)
- The robot's legs make step-function position changes rather than smooth trajectories

---

## 📊 Torque Data Analysis

Looking at the [joint_torques.csv](file:///home/mubashir/Documents/FYP-Legged-Robot-main/Code/ROS/src/sim_robot/joint_torques.csv) and [joint_torques.png](file:///home/mubashir/Documents/FYP-Legged-Robot-main/Code/ROS/src/sim_robot/../../../joint_torques.png):

| Observation | Evidence | Root Cause |
|---|---|---|
| **Torques frequently hit 0.9414 N⋅m (max effort limit)** | Time steps 0, 3, 13, 14, 20, 33, 47, 54, 58, 59, 65... | PID overshoot + no damping → controller saturates |
| **Chaotic, non-periodic pattern** | No repeating waveform visible despite cyclic gait | System is unstable — robot is likely falling/stumbling |
| **Only 98 data points out of 380 rows** | Rows 98–378 are empty | Robot likely fell or simulation went unstable; node was stopped early |
| **Time step 0: all hip torques = 0.9414** | Initial command from rest causes max-effort response | No initial pose → joints start at 0° and slam to first waypoint |
| **All joints equally noisy** | Hip, knee, foot all show similar chaotic ranges | Systemic issue (simulation setup) not per-joint issue |

---

## 📋 Summary Table for Professor Meeting

| # | Issue | Category | Impact |
|---|---|---|---|
| 1 | No joint damping/friction | **Simulation physics** | Oscillatory, underdamped response |
| 2 | CM vs M unit mismatch in trajectory | **Sim-to-model gap** | Workspace mismatch, near-singularity motions |
| 3 | Untuned, uniform PID gains | **Simulation control** | Overshoot, instability, torque saturation |
| 4 | No foot contact model/friction | **Simulation physics** | Feet slip, can't generate traction |
| 5 | Self-collision enabled with mesh collisions | **Simulation physics** | Parasitic forces, jitter |
| 6 | No gravity compensation | **Control architecture** | PID alone can't handle stance loads |
| 7 | Controller rate too slow (10Hz) | **Control timing** | Jerky, step-function trajectories |

> [!TIP]
> The professor's hint — *"issues unrelated to your work but affecting it"* — most likely refers to issues **#1, #3, #4, and #5**, as these are all in the **simulation/model setup** (SDF files), not in your kinematic gait code. These are fundamental simulation fidelity problems that would affect *any* controller run on this model.
