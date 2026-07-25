# Porting a Parallel Elastic Actuator Plugin from Gazebo Classic / ROS 1 to gz-sim / ROS 2

## What this covers

Taking [`aminsung/gazebo_joint_torsional_spring_plugin`](https://github.com/aminsung/gazebo_joint_torsional_spring_plugin)
(classic Gazebo, ROS 1, catkin, BSD-3-Clause) and rewriting it as an
ament package for the new Gazebo. The resulting package is
`gz_joint_torsional_spring`.

The mechanism being simulated is a parallel torsion spring at a femur pitch
joint — a spring acting alongside the motor rather than in series with it:

```
tau_joint  = tau_motor + tau_spring
tau_spring = -k (theta - theta_0)
```

Nothing is added to the kinematic tree. This is a change to what happens at
an existing revolute joint, not a new body.

---

## The original plugin

The whole thing is about 80 lines. Reproduced in outline:

```cpp
class TorsionalSpringPlugin : public ModelPlugin
{
  physics::ModelPtr model;
  physics::JointPtr joint;
  double setPoint;
  double kx;
  event::ConnectionPtr updateConnection;

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    if (_model->GetJointCount() == 0) { /* bail out */ }
    this->model = _model;
    if (_sdf->HasElement("joint"))
      this->joint = _model->GetJoint(_sdf->Get<std::string>("joint"));
    this->kx       = _sdf->Get<double>("kx");
    this->setPoint = _sdf->Get<double>("set_point");
  }

  void Init()
  {
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&TorsionalSpringPlugin::OnUpdate, this));
  }

  void OnUpdate()
  {
    double current_angle = this->joint->GetAngle(0).Radian();
    this->joint->SetForce(0, this->kx * (this->setPoint - current_angle));
  }
};

GZ_REGISTER_MODEL_PLUGIN(TorsionalSpringPlugin)
```

Original usage:

```xml
<gazebo>
  <plugin name="knee_joint_torsional_spring"
          filename="libgazebo_joint_torsional_spring.so">
    <kx>0.1</kx>
    <set_point>0.5</set_point>
    <joint>knee_joint</joint>
  </plugin>
</gazebo>
```

Parameters: `kx` (spring coefficient), `set_point` (zero-torque angle in
radians), `joint` (joint name).

---

## Mapping table

| Original (classic Gazebo / ROS 1) | Port (gz-sim / ROS 2) |
|---|---|
| `ModelPlugin` + `GZ_REGISTER_MODEL_PLUGIN` | `System` + `ISystemConfigure` + `ISystemPreUpdate`, `GZ_ADD_PLUGIN` |
| `Load()` then `Init()` wiring a world-update callback | `Configure()` once; `PreUpdate()` called every step by the framework |
| `joint->GetAngle(0).Radian()` | read `components::JointPosition` off the entity-component manager |
| `joint->SetForce(0, tau)` | write `components::JointForceCmd` |
| catkin / `catkin_make` | ament_cmake / `colcon build` |
| One joint per plugin instance | Any number of `<spring>` blocks per instance |
| `SetForce` accumulated into a per-step buffer | torque explicitly **added** to `JointForceCmd` |
| Linear `kx` only | Linear `kx` **or** piecewise-linear torque-angle table |
| — | Optional viscous `damping` and `max_torque` saturation |

---

## The accumulation problem (the substantive bug)

This is the part a mechanical translation gets wrong.

Classic Gazebo's `Joint::SetForce` added into a per-step force accumulator.
Several callers could each call it in one step and the physics engine would
see the sum.

In gz-sim, `JointForceCmd` is a plain component sitting in the entity
component manager. The physics system reads whatever value is in it at the
end of the step. So the obvious line:

```cpp
forceComp->Data()[0] = tau;    // WRONG for a parallel spring
```

silently discards whatever `gz_ros2_control` wrote for that joint that
step — or gets discarded by it, depending on which system happens to
execute later. Two systems both assigning to the same component is a race,
and the loser's contribution simply vanishes.

Since the entire point of a *parallel* elastic actuator is that spring
torque and motor torque sum, the port must read-modify-write:

```cpp
auto forceComp = ecm.Component<components::JointForceCmd>(cfg.jointEntity);
if (!forceComp) {
  ecm.CreateComponent(cfg.jointEntity, components::JointForceCmd({tau}));
} else if (forceComp->Data().empty()) {
  forceComp->Data() = {tau};
} else {
  forceComp->Data()[0] += tau;   // accumulate
}
```

If overwrite behaviour is ever wanted, change the `+=` back to `=`.

---

## Component creation in `Configure`

gz-sim only populates `JointPosition` and `JointVelocity` if something has
asked for them. If nothing creates the component, `PreUpdate` has nothing
valid to read and the spring silently does nothing. So `Configure` creates
them up front:

```cpp
if (!ecm.Component<components::JointPosition>(cfg.jointEntity)) {
  ecm.CreateComponent(cfg.jointEntity, components::JointPosition());
}
if (cfg.damping != 0.0 &&
    !ecm.Component<components::JointVelocity>(cfg.jointEntity)) {
  ecm.CreateComponent(cfg.jointEntity, components::JointVelocity());
}
```

`PreUpdate` still guards against an empty `Data()` vector, because the
physics engine may not have filled it in on the very first step.

---

## Nonlinear spring support

A flat spiral torsion spring is linear only while its coils are free. Once
coils begin to close, stiffness rises sharply — in the FEA for this project
the stiffness roughly doubled at 180 degrees of rotation, from about
50 N·mm/rad to about 100 N·mm/rad, once coil contact engaged. A single `kx`
cannot represent that.

The port accepts a sampled torque-angle curve, which overrides `kx` and
`set_point` entirely when present:

```xml
<spring>
  <joint>RH_femur_joint</joint>
  <curve_angles>-1.57 -0.79 0.0 0.79 1.57 2.36 3.14</curve_angles>
  <curve_torques>19.6 9.9 0.0 -9.9 -19.6 -32.0 -50.0</curve_torques>
</spring>
```

Angles in radians and strictly increasing; torques in N·m; equal lengths;
at least two points. Interpolation is piecewise linear via `upper_bound`,
with clamping beyond either end point. Sample densely enough that the
linear segments track the real curve where it bends.

Evaluation:

```cpp
if (theta <= a.front())      tau = t.front();
else if (theta >= a.back())  tau = t.back();
else {
  auto it = std::upper_bound(a.begin(), a.end(), theta);
  std::size_t hi = it - a.begin();
  std::size_t lo = hi - 1;
  double span = a[hi] - a[lo];
  double frac = (span > 0.0) ? (theta - a[lo]) / span : 0.0;
  tau = t[lo] + frac * (t[hi] - t[lo]);
}
```

Damping and saturation are applied afterwards, so they work with either
the linear law or the curve.

---

## Syntax gotchas that bite when porting

1. **`filename` is now the bare library name.** `gz_joint_torsional_spring`,
   not `libgazebo_joint_torsional_spring.so`. No `lib` prefix, no `.so`
   suffix.

2. **`name` must be the fully qualified C++ class**, including namespace:
   `gz_joint_torsional_spring::TorsionalSpringSystem`. In classic Gazebo
   `name` was an arbitrary human-readable label; it is now load-bearing.

3. **The `sdf` parameter to `Configure` is const.** To walk child elements
   with the non-const API, call `sdf->Clone()` first.

4. **Plugin discovery.** The `.so` must be on `GZ_SIM_SYSTEM_PLUGIN_PATH`.
   An ament environment hook handles this so that sourcing the workspace is
   enough:

   ```
   prepend-non-duplicate;GZ_SIM_SYSTEM_PLUGIN_PATH;lib
   ```

---

## Package layout

```
gz_joint_torsional_spring/
├── CMakeLists.txt
├── package.xml
├── README.md
├── hooks/
│   └── gz_joint_torsional_spring.dsv.in
├── include/gz_joint_torsional_spring/
│   └── torsional_spring_system.hh
├── src/
│   └── torsional_spring_system.cc
└── examples/
    └── urdf_snippet.xacro
```

`CMakeLists.txt` auto-detects the Gazebo generation — it tries `gz-sim9`
(Ionic, pairs with Kilted/Rolling) first and falls back to `gz-sim8`
(Harmonic, pairs with Jazzy), selecting the matching `gz-plugin` version
alongside.

`package.xml` depends on the vendor packages (`gz_sim_vendor`,
`gz_plugin_vendor`, `gz_common_vendor`, `sdformat_vendor`) rather than
pinning specific Gazebo versions.

---

## Build

```bash
cd ~/ros2_ws/src
# place the package here
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select gz_joint_torsional_spring
source install/setup.bash
```

Requires ROS 2 Jazzy (Gazebo Harmonic) or newer.

**Not compile-tested** — the container used to write the package had no
gz-sim installed. Run `colcon build` before relying on it.

---

## Usage

```xml
<gazebo>
  <plugin filename="gz_joint_torsional_spring"
          name="gz_joint_torsional_spring::TorsionalSpringSystem">

    <spring>
      <joint>LF_femur_joint</joint>
      <kx>12.5</kx>                 <!-- N*m/rad -->
      <set_point>0.0</set_point>    <!-- rad -->
      <damping>0.05</damping>       <!-- N*m*s/rad, optional -->
      <max_torque>30.0</max_torque> <!-- N*m, optional -->
    </spring>

    <spring>
      <joint>RF_femur_joint</joint>
      <kx>12.5</kx>
      <set_point>0.0</set_point>
    </spring>

  </plugin>
</gazebo>
```

### Parameters, per `<spring>` block

| Tag | Units | Default | Meaning |
|---|---|---|---|
| `joint` | — | required | Joint name in the model |
| `kx` | N·m/rad | 0.0 | Stiffness |
| `set_point` | rad | 0.0 | Angle at which spring torque is zero |
| `damping` | N·m·s/rad | 0.0 | Viscous term, subtracts `damping * omega` |
| `max_torque` | N·m | disabled | Symmetric saturation |
| `curve_angles` | rad | — | Increasing list; overrides `kx`/`set_point` |
| `curve_torques` | N·m | — | Same length as `curve_angles` |

Sign convention matches the original: `tau = kx * (set_point - theta)`.

The flat `<joint>`/`<kx>`/`<set_point>` layout without a `<spring>` wrapper
is also accepted, so an existing ROS 1 plugin block ports over unchanged
apart from `filename` and `name`.

---

## Alternatives considered

**Native SDF joint dynamics.** URDF has no spring element, only `damping`
and `friction` inside `<dynamics>`. A `<gazebo reference="joint_name">`
block translates `<springStiffness>` / `<springReference>` into the SDF
elements `//axis/dynamics/spring_stiffness` and `spring_reference`:

```xml
<gazebo reference="LF_hip_joint">
  <springStiffness>12.5</springStiffness>
  <springReference>0.0</springReference>
</gazebo>
```

Zero code, but strictly linear, and backend-dependent — dartsim (the
default) honours it; bullet-featherstone did not as of the last tracked
feature request. Not suitable for a spiral spring with a nonlinear
torque-rotation curve.

**Custom `gz_ros2_control` `SystemInterface`.** Fold the spring torque into
the hardware interface itself, below the controller manager:

```cpp
double theta      = joint_position_[i];
double tau_spring = evaluateSpringCurve(theta);
joint_effort_command_[i] = commanded_effort_[i] + tau_spring;
```

Registered via `<hardware><plugin>your_pkg/YourSpringSystemInterface</plugin></hardware>`
in place of the stock `GazeboSimSystem`. This combines actuator and spring
torque in one place, with no cross-plugin ordering to reason about — the
most robust option if `gz_ros2_control` is already in use in effort mode.
More code, and it replaces the hardware interface.

**Where the standalone system plugin sits between them:** engine-independent
(it commands force explicitly rather than relying on the physics engine's
internal stiffness implementation) and nonlinear-capable, without requiring
the hardware interface to be replaced. Its cost is the accumulation and
ordering concern described above, which the `+=` addresses.

---

## Project context

This feeds the passive gravity compensation work for a sprawling-type
hexapod continuing operation as a quadruped after leg loss. The spring is a
3D-printed ABS spiral torsion spring at the femur pitch joint in a parallel
elastic actuator arrangement.

The nonlinear curve support exists specifically because the FEA on that
spring showed stiffness departing from the design target of 50 N·mm/rad
once coil contact engaged. Simulating it with a single linear `kx` would
misrepresent exactly the behaviour the FEA was run to characterise.

---

## License

BSD-3-Clause, same as the original plugin.
