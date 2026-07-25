// commanded_effort_publisher.hh
//
// gz-sim (Harmonic+) System plugin that publishes each joint's *commanded
// actuation effort* — the value sitting in its JointForceCmd component — on a
// gz-transport topic, so it can be bridged to ROS 2 and logged.
//
// WHY THIS EXISTS
//   The per-joint force_torque sensor measures the TOTAL transmitted wrench
//   through the joint (clamped actuation + gravity/inertial/contact load). A
//   *parallel* spring does not change that total — it only shifts how much of
//   it the motor supplies vs. the spring — so the force_torque sensor barely
//   moves when a parallel spring is added. The quantity that actually drops is
//   the motor/PID effort, i.e. JointForceCmd. This plugin exposes exactly that.
//
// ORDERING (important)
//   Implemented as ISystemPreUpdate. gz-sim runs every system's PreUpdate in
//   plugin-load order. List this plugin AFTER the JointPositionController
//   blocks (so JointForceCmd already holds the PID command this step) but
//   BEFORE the TorsionalSpringSystem block (so the spring's += has not yet been
//   added). It then captures the pure motor effort in every configuration:
//   baseline (no spring), native passive spring, or the += plugin spring.

#ifndef GZ_JOINT_TORSIONAL_SPRING__COMMANDED_EFFORT_PUBLISHER_HH_
#define GZ_JOINT_TORSIONAL_SPRING__COMMANDED_EFFORT_PUBLISHER_HH_

#include <memory>
#include <string>
#include <vector>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>

namespace gz_joint_torsional_spring
{

class CommandedEffortPublisher
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  CommandedEffortPublisher() = default;
  ~CommandedEffortPublisher() override = default;

  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager & eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override;

private:
  struct JointPub
  {
    std::string name;
    gz::sim::Entity entity{gz::sim::kNullEntity};
    gz::transport::Node::Publisher pub;
  };

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::transport::Node node_;
  std::vector<JointPub> joints_;
};

}  // namespace gz_joint_torsional_spring

#endif  // GZ_JOINT_TORSIONAL_SPRING__COMMANDED_EFFORT_PUBLISHER_HH_
