# gz_joint_torsional_spring

Parallel torsion spring (parallel elastic actuator) for **gz-sim** and **ROS 2**.
A port of [`aminsung/gazebo_joint_torsional_spring_plugin`](https://github.com/aminsung/gazebo_joint_torsional_spring_plugin),
which targeted classic Gazebo and ROS 1.

## What changed in the port

| Original (classic Gazebo / ROS 1) | This port (gz-sim / ROS 2) |
|---|---|
| `ModelPlugin` + `GZ_REGISTER_MODEL_PLUGIN` | `System` + `ISystemConfigure` + `ISystemPreUpdate`, `GZ_ADD_PLUGIN` |
| `Load()` then `Init()` wiring a world-update callback | `Configure()` once, `PreUpdate()` called every step by the framework |
| `joint->GetAngle(0).Radian()` | read `components::JointPosition` off the entity-component manager |
| `joint->SetForce(0, tau)` | write `components::JointForceCmd` |
| catkin / `catkin_make` | ament_cmake / `colcon build` |
| One joint per plugin instance | Any number of `<spring>` blocks per instance |
| `SetForce` **replaced** the accumulated force | torque is **added** to `JointForceCmd` |
| Linear `kx` only | Linear `kx` **or** a piecewise-linear torque-angle table |
| — | Optional viscous `damping` and `max_torque` saturation |

### The accumulation fix matters

The original called `SetForce`, which in classic Gazebo added into a
per-step force accumulator. In gz-sim, `JointForceCmd` is a plain component
that the physics system reads at the end of the step. A naive port that
writes `forceComp->Data()[0] = tau` will silently clobber whatever
`gz_ros2_control` wrote for that joint that step, or be clobbered by it,
depending on system execution order. This plugin does `+=` instead, so the
spring genuinely acts *in parallel* with the actuator:

```
tau_joint = tau_motor + tau_spring
```

If for some reason you want the old overwrite behaviour, change the `+=` in
`PreUpdate` to `=`.

## Build

Requires ROS 2 Jazzy (Gazebo Harmonic, gz-sim8) or newer. gz-sim9 (Ionic) is
auto-detected.

```bash
cd ~/ros2_ws/src
# place this package here
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select gz_joint_torsional_spring
source install/setup.bash
```

Sourcing the workspace adds `install/lib` to `GZ_SIM_SYSTEM_PLUGIN_PATH`
via the environment hook, so gz-sim can find the library.

## Usage

```xml
<gazebo>
  <plugin filename="gz_joint_torsional_spring"
          name="gz_joint_torsional_spring::TorsionalSpringSystem">
    <spring>
      <joint>LF_femur_joint</joint>
      <kx>12.5</kx>
      <set_point>0.0</set_point>
    </spring>
  </plugin>
</gazebo>
```

Note that `filename` is now the bare library name (no `lib` prefix, no
`.so` suffix) and `name` is the fully qualified C++ class, not an arbitrary
label as it was in classic Gazebo.

### Parameters (per `<spring>` block)

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

### Nonlinear springs

A flat spiral torsion spring is only linear while its coils are free. Once
they begin to close, stiffness rises sharply. If you have a torque-rotation
curve from FEA or a test rig, feed the sampled points in directly:

```xml
<spring>
  <joint>RH_femur_joint</joint>
  <curve_angles>-1.57 -0.79 0.0 0.79 1.57 2.36 3.14</curve_angles>
  <curve_torques>19.6 9.9 0.0 -9.9 -19.6 -32.0 -50.0</curve_torques>
</spring>
```

Interpolation is piecewise linear; values beyond either end are clamped.
Sample densely enough that the linear segments track your real curve.

## Alternatives worth knowing about

- **Native SDF** `//axis/dynamics/spring_stiffness` and `spring_reference`.
  Zero code, but linear only and backend-dependent (dartsim honours it,
  bullet-featherstone historically did not).
- **Custom `gz_ros2_control` `SystemInterface`.** Folds spring torque in
  alongside the actuator command in one place, removing cross-plugin
  ordering concerns entirely. More code, but the most robust option if you
  are already using `gz_ros2_control` in effort mode.

This plugin sits between the two: engine-independent and nonlinear-capable
without requiring you to replace your hardware interface.

## License

BSD-3-Clause, same as the original.
