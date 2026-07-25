// torsional_spring_system.cc

#include "gz_joint_torsional_spring/torsional_spring_system.hh"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/Name.hh>

using namespace gz_joint_torsional_spring;

std::vector<double> TorsionalSpringSystem::ParseList(const std::string & text)
{
  std::vector<double> out;
  std::string cleaned = text;
  std::replace(cleaned.begin(), cleaned.end(), ',', ' ');
  std::istringstream ss(cleaned);
  double v;
  while (ss >> v) {
    out.push_back(v);
  }
  return out;
}

void TorsionalSpringSystem::Configure(
  const gz::sim::Entity & entity,
  const std::shared_ptr<const sdf::Element> & sdf,
  gz::sim::EntityComponentManager & ecm,
  gz::sim::EventManager & /*eventMgr*/)
{
  this->modelEntity_ = entity;
  gz::sim::Model model(entity);

  if (!model.Valid(ecm)) {
    gzerr << "[torsional_spring] Plugin must be attached to a model. "
          << "Not loading." << std::endl;
    return;
  }

  // sdf is const; clone so we can walk child elements with the non-const API.
  auto sdfClone = sdf->Clone();

  // Two accepted layouts:
  //   (a) one <spring> block per joint  (preferred, multi-joint)
  //   (b) flat <joint>/<kx>/<set_point> (backwards compatible with the
  //       original ROS 1 plugin's parameter names)
  std::vector<sdf::ElementPtr> blocks;
  if (sdfClone->HasElement("spring")) {
    for (auto e = sdfClone->GetElement("spring"); e; e = e->GetNextElement("spring")) {
      blocks.push_back(e);
    }
  } else {
    blocks.push_back(sdfClone);
  }

  for (const auto & block : blocks) {
    SpringConfig cfg;

    if (!block->HasElement("joint")) {
      gzerr << "[torsional_spring] Must specify <joint>. Skipping this spring."
            << std::endl;
      continue;
    }
    cfg.jointName = block->Get<std::string>("joint");

    cfg.jointEntity = model.JointByName(ecm, cfg.jointName);
    if (cfg.jointEntity == gz::sim::kNullEntity) {
      gzerr << "[torsional_spring] Joint [" << cfg.jointName
            << "] not found in model [" << model.Name(ecm)
            << "]. Skipping." << std::endl;
      continue;
    }

    if (block->HasElement("kx")) {
      cfg.kx = block->Get<double>("kx");
    } else {
      gzwarn << "[torsional_spring] <kx> not specified for joint ["
             << cfg.jointName << "]. Defaulting to 0.0." << std::endl;
    }

    if (block->HasElement("set_point")) {
      cfg.setPoint = block->Get<double>("set_point");
    }
    if (block->HasElement("damping")) {
      cfg.damping = block->Get<double>("damping");
    }
    if (block->HasElement("max_torque")) {
      cfg.maxTorque = block->Get<double>("max_torque");
    }

    // Optional nonlinear lookup table.
    if (block->HasElement("curve_angles") && block->HasElement("curve_torques")) {
      cfg.curveAngles = ParseList(block->Get<std::string>("curve_angles"));
      cfg.curveTorques = ParseList(block->Get<std::string>("curve_torques"));

      if (cfg.curveAngles.size() != cfg.curveTorques.size() ||
          cfg.curveAngles.size() < 2)
      {
        gzerr << "[torsional_spring] curve_angles/curve_torques for joint ["
              << cfg.jointName << "] must be equal length and >= 2 entries. "
              << "Falling back to linear kx." << std::endl;
        cfg.curveAngles.clear();
        cfg.curveTorques.clear();
      } else if (!std::is_sorted(cfg.curveAngles.begin(), cfg.curveAngles.end())) {
        gzerr << "[torsional_spring] curve_angles for joint [" << cfg.jointName
              << "] must be strictly increasing. Falling back to linear kx."
              << std::endl;
        cfg.curveAngles.clear();
        cfg.curveTorques.clear();
      }
    }

    // The physics system only populates JointPosition / JointVelocity if
    // something has asked for them. Create the components so they exist.
    if (!ecm.Component<gz::sim::components::JointPosition>(cfg.jointEntity)) {
      ecm.CreateComponent(cfg.jointEntity, gz::sim::components::JointPosition());
    }
    if (cfg.damping != 0.0 &&
        !ecm.Component<gz::sim::components::JointVelocity>(cfg.jointEntity))
    {
      ecm.CreateComponent(cfg.jointEntity, gz::sim::components::JointVelocity());
    }

    gzmsg << "[torsional_spring] joint=" << cfg.jointName
          << " kx=" << cfg.kx
          << " set_point=" << cfg.setPoint
          << " damping=" << cfg.damping
          << (cfg.curveAngles.empty() ? "" : " (nonlinear curve active)")
          << std::endl;

    this->springs_.push_back(std::move(cfg));
  }

  gzmsg << "[torsional_spring] Loaded " << this->springs_.size()
        << " spring(s)." << std::endl;
}

double TorsionalSpringSystem::SpringTorque(
  const SpringConfig & cfg, double theta, double omega) const
{
  double tau;

  if (!cfg.curveAngles.empty()) {
    // Piecewise-linear interpolation, clamped at the ends.
    const auto & a = cfg.curveAngles;
    const auto & t = cfg.curveTorques;

    if (theta <= a.front()) {
      tau = t.front();
    } else if (theta >= a.back()) {
      tau = t.back();
    } else {
      auto it = std::upper_bound(a.begin(), a.end(), theta);
      std::size_t hi = static_cast<std::size_t>(it - a.begin());
      std::size_t lo = hi - 1;
      const double span = a[hi] - a[lo];
      const double frac = (span > 0.0) ? (theta - a[lo]) / span : 0.0;
      tau = t[lo] + frac * (t[hi] - t[lo]);
    }
  } else {
    // Hooke's law, sign convention identical to the original plugin:
    // tau = kx * (set_point - theta), i.e. restoring toward set_point.
    tau = cfg.kx * (cfg.setPoint - theta);
  }

  if (cfg.damping != 0.0) {
    tau -= cfg.damping * omega;
  }

  if (cfg.maxTorque >= 0.0) {
    tau = std::clamp(tau, -cfg.maxTorque, cfg.maxTorque);
  }

  return tau;
}

void TorsionalSpringSystem::PreUpdate(
  const gz::sim::UpdateInfo & info,
  gz::sim::EntityComponentManager & ecm)
{
  if (info.paused) {
    return;
  }

  for (auto & cfg : this->springs_) {
    auto posComp =
      ecm.Component<gz::sim::components::JointPosition>(cfg.jointEntity);
    if (!posComp || posComp->Data().empty()) {
      if (!cfg.warnedMissing) {
        gzwarn << "[torsional_spring] No position data yet for joint ["
               << cfg.jointName << "]." << std::endl;
        cfg.warnedMissing = true;
      }
      continue;
    }

    const double theta = posComp->Data()[0];

    double omega = 0.0;
    if (cfg.damping != 0.0) {
      auto velComp =
        ecm.Component<gz::sim::components::JointVelocity>(cfg.jointEntity);
      if (velComp && !velComp->Data().empty()) {
        omega = velComp->Data()[0];
      }
    }

    const double tau = this->SpringTorque(cfg, theta, omega);

    // ACCUMULATE, do not overwrite. gz_ros2_control and other controllers
    // write to this same component; assigning would silently discard their
    // command depending on system execution order.
    auto forceComp =
      ecm.Component<gz::sim::components::JointForceCmd>(cfg.jointEntity);
    if (!forceComp) {
      ecm.CreateComponent(
        cfg.jointEntity,
        gz::sim::components::JointForceCmd({tau}));
    } else if (forceComp->Data().empty()) {
      forceComp->Data() = {tau};
    } else {
      forceComp->Data()[0] += tau;
    }
  }
}

GZ_ADD_PLUGIN(
  gz_joint_torsional_spring::TorsionalSpringSystem,
  gz::sim::System,
  gz_joint_torsional_spring::TorsionalSpringSystem::ISystemConfigure,
  gz_joint_torsional_spring::TorsionalSpringSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  gz_joint_torsional_spring::TorsionalSpringSystem,
  "gz_joint_torsional_spring::TorsionalSpringSystem")
