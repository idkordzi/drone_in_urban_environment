#include <chrono>

#include <gz/transport/Node.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include "gz/sim/components/ParentLinkName.hh"
#include <gz/plugin/Register.hh>
#include <gz/msgs/pose.pb.h>


namespace gz {
namespace sim {
namespace systems{

class DroneStatePublisher
: public System,
  public ISystemConfigure,
  public ISystemPostUpdate
{

public:

  virtual void Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/) override
  {
    auto linkName = _sdf->Get<std::string>("link_name");
    auto model    = Model(_entity);
    this->linkEntity_ = model.LinkByName(_ecm, linkName);

    double updateFrequency = _sdf->Get<double>("update_frequency", -1).first;
    if (updateFrequency > 0)
    {
      std::chrono::duration<double> period{1 / updateFrequency};
      this->updatePeriod_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    }

    std::string poseTopic = _sdf->Get<std::string>("gz_topic");
    this->pub_ = this->node_.Advertise<msgs::Pose>(poseTopic);
  }

  virtual void PostUpdate(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm) override
  {
    if (_info.dt < std::chrono::steady_clock::duration::zero())
      gzwarn << "Detected jump back in time [" << std::chrono::duration<double>(_info.dt).count() << "s]. System may not work properly." << std::endl;
    if (_info.paused) return;

    bool publish = true;
    auto diff = _info.simTime - this->lastPosePubTime_;
    if ((diff > std::chrono::steady_clock::duration::zero()) && (diff < this->updatePeriod_))
      publish = false;
    if (!publish) return;

    this->publish(_ecm, convert<msgs::Time>(_info.simTime));
    this->lastPosePubTime_ = _info.simTime;

  }
  
private:

  Entity linkEntity_;
  transport::Node node_;
  transport::Node::Publisher pub_;
  std::chrono::steady_clock::duration updatePeriod_ {0};
  std::chrono::steady_clock::duration lastPosePubTime_ {0};

  void publish(
    const EntityComponentManager &_ecm,
    const msgs::Time &_stampMsg)
  {
    msgs::Pose msg;

    msg.mutable_header()->mutable_stamp()->CopyFrom(_stampMsg);
    msgs::Set(&msg, worldPose(this->linkEntity_, _ecm));

    this->pub_.Publish(msg);
  }

};

} // namespace systems
} // namespace sim
} // namespace gz

GZ_ADD_PLUGIN(gz::sim::systems::DroneStatePublisher,
              gz::sim::System,
              gz::sim::systems::DroneStatePublisher::ISystemConfigure,
              gz::sim::systems::DroneStatePublisher::ISystemPostUpdate)
 
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::DroneStatePublisher, "DroneStatePublisher")
