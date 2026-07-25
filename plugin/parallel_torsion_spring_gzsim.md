# Implementing a Parallel Torsion Spring in Gazebo (gz-sim) for a ROS2 Quadruped

## The concept

A parallel torsion spring sits at the same joint as your actuator and simply
adds torque to it:

```
τ_joint = τ_motor + τ_spring
τ_spring = -k(θ - θ₀)
```

This is different from a series elastic actuator, which needs an extra
compliant joint and a dummy link inserted between the motor output and the
leg segment. A parallel spring needs nothing added to the kinematic tree.
You are modifying what happens at an existing revolute joint, not adding a
body.

---

## Method 1: native SDF joint dynamics (quick, linear only)

URDF itself has no spring element, only `damping` and `friction` inside
`<dynamics>`. When Gazebo converts URDF to SDF, it looks for a
`<gazebo reference="joint_name">` extension block and translates
`<springReference>` / `<springStiffness>` into the SDF elements
`//axis/dynamics/spring_reference` and `//axis/dynamics/spring_stiffness`.

```xml
<gazebo reference="LF_hip_joint">
  <springStiffness>12.5</springStiffness>   <!-- N·m/rad -->
  <springReference>0.0</springReference>    <!-- rad, unloaded equilibrium -->
</gazebo>
```

Caveats:
- dartsim (the default gz-sim physics backend) honors these parameters.
- bullet-featherstone did not support them as of the last tracked feature
  request, worth verifying against your current gz-sim version.
- This is a strictly linear torque law, not suitable if your spring's
  torque-rotation curve is nonlinear by geometry (e.g. a flat spiral
  spring).

---

## Method 2: inject the torque yourself via ros2_control (handles nonlinearity)

If you need your simulated spring to match a real, characterized nonlinear
torque curve (from FEA or an optimization fit), build a custom
`gz_ros2_control` `SystemInterface` plugin. This sits below the controller
manager and adds the spring torque to whatever your locomotion controller
already commands, rather than fighting for the same command interface.

```cpp
double theta = joint_position_[i];
double tau_spring = evaluateSpringCurve(theta);   // your fitted poly or lookup table
double tau_total = commanded_effort_[i] + tau_spring;
joint_effort_command_[i] = tau_total;
```

Register it under `<hardware><plugin>your_pkg/YourSpringSystemInterface</plugin></hardware>`
inside the `<ros2_control>` block of your URDF, in place of the stock
`GazeboSimSystem`.

---

## The gz-sim equivalent of a classic Gazebo plugin (e.g. `gazebo_joint_torsional_spring_plugin`)

Classic Gazebo plugins (ModelPlugin, WorldPlugin, SensorPlugin) do not exist
in the new Gazebo. They were replaced by an entity component system, where
everything is a System plugin that reads and writes components on an entity
component manager rather than calling methods directly on joint objects.
There is no drop-in port, but the replacement is small.

### A few C++ ideas first, coming from an Arduino / basic embedded background

**Pointers.** In Arduino, `int sensorValue = analogRead(A0);` stores the
value itself. A pointer instead stores the *location* of a value, like a
locker number rather than what's inside the locker. If you have the number,
you can go open it and read or change what's inside.

**References.** A parameter like `gz::sim::EntityComponentManager &ecm`
(note the `&`) is a reference, a nickname for a locker rather than its
number. Using the nickname behaves exactly like touching the locker
directly. A function taking a reference is working with the real live data,
not a copy.

**Smart pointers.** In plain C, if your program creates something in
memory, it is responsible for freeing it later, and forgetting to do so
leaks memory. A `std::shared_ptr` keeps a running count of how many parts
of the program are using a piece of data, and cleans it up automatically
once that count hits zero. You can mostly treat it like a regular pointer
without worrying about cleanup.

**Classes and inheritance.** A class is a bit like a library you'd import
into a sketch, say `Servo`. It bundles data (which pin) with functions
(`write()`). Inheritance lets a new class start from an existing one and
add to or override its behavior. Our plugin class inherits from a few
gz-sim base classes purely so gz-sim recognizes it as loadable and knows
which functions to call.

**Angle bracket syntax**, like `sdf->Get<std::string>("joint")`. This tells
one shared function what type of value you expect back, similar to a print
function behaving differently depending on whether you hand it a number or
a string.

**How `PreUpdate` gets called.** No interrupt or timer is involved. Think
of it like Arduino's `loop()`. Once simulation starts, gz-sim calls
`PreUpdate()` over and over, once per simulation step, the same way `loop()`
runs repeatedly while the board is powered. You never call it yourself,
the framework calls it automatically once your class is registered as a
plugin.

### Header file

```cpp
// torsional_spring_system.hh
//
// This file just describes the shape of our class: what data it holds
// and which functions gz-sim is allowed to call on it. It does not
// contain the actual logic, that lives in the .cc file below. Think of
// it as similar to declaring a function before you define it, except
// for a whole bundle of functions and variables at once.

#include <gz/sim/System.hh>
// Pulls in the definitions for System and the interface classes we are
// about to inherit from below.

namespace torsional_spring_system
{
  // A namespace is just a labeled folder for your code, so a class
  // called TorsionalSpringSystem here does not collide with some other
  // class of the same name written by someone else. Not something you
  // would run into on a small Arduino sketch, but it matters once many
  // libraries are combined in one project.

  class TorsionalSpringSystem
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
  // The colon and everything after it means this class inherits from
  // three things at once, which is allowed in C++. System is the base
  // type every gz-sim plugin needs. The other two are essentially
  // promises: by inheriting from ISystemPreUpdate, we are telling
  // gz-sim, I will provide a PreUpdate function, please call it once
  // every simulation step.
  {
    public:
    // Functions and variables listed under public are the ones gz-sim
    // itself is allowed to reach in from outside and call.

    void Configure(const gz::sim::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   gz::sim::EntityComponentManager &ecm,
                   gz::sim::EventManager &eventMgr) override;
    // Runs once, when the plugin is first loaded. This is where we will
    // read our settings out of the SDF file and remember which joint we
    // are attached to.
    //
    // "override" tells the compiler, I am intentionally replacing a
    // function that already exists on the parent class, please double
    // check my spelling and parameter list match. It has no runtime
    // effect, it is purely there to catch mistakes early.

    void PreUpdate(const gz::sim::UpdateInfo &info,
                   gz::sim::EntityComponentManager &ecm) override;
    // Called automatically, over and over, once per simulation step,
    // the same way loop() runs over and over on an Arduino. This is
    // where the actual spring torque gets calculated and applied.

    private:
    // Only code inside this class can touch anything listed here. These
    // are the values the plugin needs to remember between one call to
    // PreUpdate and the next, similar to a variable declared outside
    // loop() on an Arduino so it keeps its value across iterations.

    gz::sim::Entity jointEntity;
    // Not an object carrying data around with it, just a plain number
    // that identifies which joint we are talking about. The actual
    // angle, torque, and so on for that joint live in separate lookup
    // tables elsewhere, and we use this number to find them.

    double kx{0.0};
    // Spring stiffness in newton metres per radian. Writing {0.0} here
    // just gives it a starting value of zero, the same as writing = 0.0
    // would.

    double setPoint{0.0};
    // The joint angle, in radians, where the spring pushes with zero
    // torque, your unloaded or neutral position.
  };
}
```

### Source file

```cpp
// torsional_spring_system.cc
//
// This is where the class actually does what the header promised.

#include "torsional_spring_system.hh"
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointForceCmd.hh>
// JointPosition is the lookup table entry holding a joint's current
// angle. JointForceCmd is the lookup table entry the physics engine
// checks each step to know how much torque to apply to that joint.

using namespace torsional_spring_system;
// Lets us write TorsionalSpringSystem below instead of spelling out
// torsional_spring_system::TorsionalSpringSystem every single time.
// Purely a convenience, changes nothing about how the code runs.

void TorsionalSpringSystem::Configure(const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager &)
    // Leaving this last parameter without a name just means gz-sim will
    // hand us an EventManager, but we have no use for it here.
{
  gz::sim::Model model(entity);
  // Wraps our raw joint locator number in a small helper object that
  // gives us convenience functions, like looking up a joint by its
  // name, so we are not stuck manually digging through lookup tables.

  auto jointName = sdf->Get<std::string>("joint");
  // "auto" just means, let the compiler work out the type on its own,
  // here it becomes a std::string. This line reads the <joint> tag out
  // of your plugin's block in the SDF file.

  this->jointEntity = model.JointByName(ecm, jointName);
  // Looks up the number that identifies the joint with that name, and
  // saves it for later use in PreUpdate. Writing "this->" in front just
  // makes clear we are updating one of the persistent variables we
  // declared in the header, rather than some throwaway local one.

  this->kx = sdf->Get<double>("kx");
  this->setPoint = sdf->Get<double>("set_point");
  // Reads the stiffness and equilibrium angle out of the SDF block the
  // same way.

  if (!ecm.Component<gz::sim::components::JointPosition>(this->jointEntity))
    ecm.CreateComponent(this->jointEntity, gz::sim::components::JointPosition());
  // ecm.Component<T>(entity) asks the big lookup table, does this joint
  // already have a JointPosition entry, and if so hand me a pointer to
  // it. If nothing has requested one yet, the answer comes back empty,
  // and the exclamation mark flips that into true, so we create one
  // ourselves. This just guarantees there is somewhere for the physics
  // engine to write the joint's current angle each step, otherwise
  // PreUpdate would have nothing valid to read from.
}

void TorsionalSpringSystem::PreUpdate(const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &ecm)
{
  auto posComp = ecm.Component<gz::sim::components::JointPosition>(this->jointEntity);
  // Fetches the current angle entry for our joint. This can still come
  // back empty if the physics engine has not filled it in yet this
  // particular step.

  if (!posComp || posComp->Data().empty())
    return;
  // A safety check before we try to use posComp. If it came back empty,
  // there is nothing valid to read yet, so we exit this call early
  // rather than crash trying to use data that is not there.

  double theta = posComp->Data()[0];
  // For a simple hip or knee joint with a single degree of freedom,
  // this is just the current joint angle in radians.

  double tau = -this->kx * (theta - this->setPoint);
  // Hooke's law for rotation. Torque equals minus stiffness times how
  // far the joint has moved from its equilibrium angle. Nothing about
  // this line is specific to gz-sim, it is the same equation you would
  // write anywhere.

  auto forceComp = ecm.Component<gz::sim::components::JointForceCmd>(this->jointEntity);
  if (!forceComp)
    ecm.CreateComponent(this->jointEntity, gz::sim::components::JointForceCmd({tau}));
  else
    forceComp->Data()[0] = tau;
  // Same lookup pattern as before. If there is no torque command entry
  // for this joint yet, create one seeded with our value. If there
  // already is one, most likely because another plugin is also
  // commanding this same joint, overwrite it. Whatever value is sitting
  // in this entry by the end of this step is what the physics engine
  // actually applies, which is exactly why the ordering caveat below
  // matters if you have another plugin touching the same joint.
}

GZ_ADD_PLUGIN(TorsionalSpringSystem,
              gz::sim::System,
              TorsionalSpringSystem::ISystemConfigure,
              TorsionalSpringSystem::ISystemPreUpdate)
// This line is what actually registers your class as something gz-sim
// can find and load. Without it, gz-sim has no way to locate your class
// inside the compiled file when your SDF asks for it by name.
```

### Usage in URDF

```xml
<gazebo>
  <plugin filename="libtorsional_spring_system.so"
          name="torsional_spring_system::TorsionalSpringSystem">
    <joint>knee_joint</joint>
    <kx>0.1</kx>
    <set_point>0.5</set_point>
  </plugin>
</gazebo>
```

---

## Caveat: plugin ordering with ros2_control

If your leg joints are also driven through `gz_ros2_control`'s actuator
interface (effort mode, or a position controller that computes torque
internally), that plugin is very likely also writing to `JointForceCmd` on
the same joint entity every step. Two systems both assigning to the same
component is a race, and whichever executes later in that step simply
overwrites the other rather than combining with it.

If using the standalone system plugin above, your spring system needs to
**read** whatever value is already sitting in `JointForceCmd` and **add**
to it, not replace it, and you need to verify execution order rather than
assume it. This is the strongest argument for folding the spring torque
calculation directly into a custom `gz_ros2_control` `SystemInterface`
(Method 2 above) instead, since there both the commanded actuator torque
and the spring torque are combined in one place, with no cross plugin
ordering to reason about.

The one advantage the standalone system plugin has over the native SDF
`spring_stiffness` tag (Method 1) is engine independence, since it commands
force explicitly rather than depending on the physics engine's own internal
stiffness implementation, so it behaves the same on dartsim or
bullet-featherstone.
