// torsional_spring_system.hh
//
// ROS 2 / gz-sim (Gazebo Harmonic+) port of
// aminsung/gazebo_joint_torsional_spring_plugin (BSD-3-Clause).
//
// Differences from the classic-Gazebo original:
//   - Multiple joints per plugin instance (one <spring> block each), so a
//     whole leg set can be handled by one plugin tag.
//   - Torque is ACCUMULATED into JointForceCmd instead of overwriting it,
//     so it coexists with gz_ros2_control / other controllers.
//   - Optional viscous damping, torque saturation, and a piecewise-linear
//     lookup table for nonlinear springs (e.g. a flat spiral torsion spring
//     whose torque-rotation curve is not a straight line).

#ifndef GZ_JOINT_TORSIONAL_SPRING__TORSIONAL_SPRING_SYSTEM_HH_
#define GZ_JOINT_TORSIONAL_SPRING__TORSIONAL_SPRING_SYSTEM_HH_

#include <memory>
#include <string>
#include <vector>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>

namespace gz_joint_torsional_spring
{

/// \brief Per-joint spring configuration and cached state.
struct SpringConfig
{
  std::string jointName;
  gz::sim::Entity jointEntity{gz::sim::kNullEntity};

  double kx{0.0};          ///< stiffness [N*m/rad]
  double setPoint{0.0};    ///< zero-torque angle [rad]
  double damping{0.0};     ///< viscous damping [N*m*s/rad], 0 disables
  double maxTorque{-1.0};  ///< saturation [N*m], negative disables

  /// Piecewise-linear curve. If non-empty it REPLACES the kx/setPoint law.
  /// angles must be strictly increasing; torques[i] pairs with angles[i].
  std::vector<double> curveAngles;
  std::vector<double> curveTorques;

  bool warnedMissing{false};
};

class TorsionalSpringSystem
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  TorsionalSpringSystem() = default;
  ~TorsionalSpringSystem() override = default;

  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager & eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override;

private:
  /// \brief Evaluate spring torque for a given angle and velocity.
  double SpringTorque(const SpringConfig & cfg, double theta, double omega) const;

  /// \brief Parse a whitespace/comma separated list of doubles.
  static std::vector<double> ParseList(const std::string & text);

  gz::sim::Entity modelEntity_{gz::sim::kNullEntity};
  std::vector<SpringConfig> springs_;
};

}  // namespace gz_joint_torsional_spring

#endif  // GZ_JOINT_TORSIONAL_SPRING__TORSIONAL_SPRING_SYSTEM_HH_
