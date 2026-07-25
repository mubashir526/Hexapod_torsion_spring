// commanded_effort_publisher.cc

#include "gz_joint_torsional_spring/commanded_effort_publisher.hh"

#include <utility>

#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/msgs/double.pb.h>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/Name.hh>

using namespace gz_joint_torsional_spring;

void CommandedEffortPublisher::Configure(
  const gz::sim::Entity & entity,
  const std::shared_ptr<const sdf::Element> & sdf,
  gz::sim::EntityComponentManager & ecm,
  gz::sim::EventManager & /*eventMgr*/)
{
  this->model_ = gz::sim::Model(entity);
  if (!this->model_.Valid(ecm)) {
    gzerr << "[commanded_effort] Plugin must be attached to a model. "
          << "Not loading." << std::endl;
    return;
  }

  // sdf is const; clone so we can walk child elements with the non-const API.
  auto sdfClone = sdf->Clone();

  // Which joints to publish: explicit <joint_name> entries if given, else
  // every joint in the model.
  std::vector<std::string> names;
  if (sdfClone->HasElement("joint_name")) {
    for (auto e = sdfClone->GetElement("joint_name"); e;
         e = e->GetNextElement("joint_name"))
    {
      names.push_back(e->Get<std::string>());
    }
  } else {
    for (const auto & j : this->model_.Joints(ecm)) {
      auto n = ecm.Component<gz::sim::components::Name>(j);
      if (n) {
        names.push_back(n->Data());
      }
    }
  }

  const std::string prefix =
    "/model/" + this->model_.Name(ecm) + "/joint/";

  for (const auto & nm : names) {
    JointPub jp;
    jp.name = nm;
    jp.entity = this->model_.JointByName(ecm, nm);
    if (jp.entity == gz::sim::kNullEntity) {
      gzerr << "[commanded_effort] Joint [" << nm
            << "] not found in model [" << this->model_.Name(ecm)
            << "]. Skipping." << std::endl;
      continue;
    }
    const std::string topic = prefix + nm + "/commanded_effort";
    jp.pub = this->node_.Advertise<gz::msgs::Double>(topic);
    gzmsg << "[commanded_effort] joint=" << nm << " -> " << topic << std::endl;
    this->joints_.push_back(std::move(jp));
  }

  gzmsg << "[commanded_effort] Tracking " << this->joints_.size()
        << " joint(s)." << std::endl;
}

void CommandedEffortPublisher::PreUpdate(
  const gz::sim::UpdateInfo & info,
  gz::sim::EntityComponentManager & ecm)
{
  if (info.paused) {
    return;
  }

  for (auto & jp : this->joints_) {
    // JointForceCmd holds this step's commanded actuation torque. If nothing
    // has commanded the joint yet this step the component may be absent/empty,
    // in which case the effort is 0.
    double val = 0.0;
    auto fc = ecm.Component<gz::sim::components::JointForceCmd>(jp.entity);
    if (fc && !fc->Data().empty()) {
      val = fc->Data()[0];
    }

    gz::msgs::Double msg;
    msg.set_data(val);
    jp.pub.Publish(msg);
  }
}

GZ_ADD_PLUGIN(
  gz_joint_torsional_spring::CommandedEffortPublisher,
  gz::sim::System,
  gz_joint_torsional_spring::CommandedEffortPublisher::ISystemConfigure,
  gz_joint_torsional_spring::CommandedEffortPublisher::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  gz_joint_torsional_spring::CommandedEffortPublisher,
  "gz_joint_torsional_spring::CommandedEffortPublisher")
