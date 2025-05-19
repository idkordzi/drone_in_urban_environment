#pragma once

#include <memory>
#include <array>
#include <vector>
#include <unordered_set>
#include <string>
#include <chrono>
#include <sstream>
#include <functional>
#include <algorithm>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Export.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/PID.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/Utility.hh>

// #include <gz/common/SignalHandler.hh>
// #include <gz/sim/components/CustomSensor.hh>
// #include <gz/sim/components/Imu.hh>
// #include <gz/sim/components/Sensor.hh>
// #include <gz/math/Filter.hh>

#include <sdf/sdf.hh>

#include "SocketUDP.hh"


namespace gz {
namespace sim {

inline namespace GZ_SIM_VERSION_NAMESPACE {

/// \brief Helper function to get an entity given its unscoped name.
///
/// \param[in] _name Entity's unscoped name.
/// \param[in] _ecm Immutable reference to ECM.
/// \param[in] _relativeTo Entity that the unscoped name is relative to.
/// If not provided, the unscoped name could be relative to any entity.
/// \return All entities that match the unscoped name and relative to
/// requirements, or an empty set otherwise.
std::unordered_set<Entity> EntitiesFromUnscopedName(const std::string &_name, const EntityComponentManager &_ecm, Entity _relativeTo = kNullEntity);

/// \brief Get the ID of a joint entity which is a descendent of this model.
///
/// A replacement for gz::sim::Model::JointByName which does not resolve
/// joints for nested models.
/// \param[in] _ecm Entity-component manager.
/// \param[in] _entity Model entity.
/// \param[in] _name Scoped joint name.
/// \return Joint entity.
Entity JointByName(EntityComponentManager &_ecm, Entity _modelEntity, const std::string &_name);

}

namespace systems {

#define MAX_MOTORS 255

struct servo_packet_16 {
  uint16_t magic;         // 18458 expected magic value
  uint16_t frame_rate;
  uint32_t frame_count;
  uint16_t pwm[16];
};

struct servo_packet_32 {
  uint16_t magic;         // 29569 expected magic value
  uint16_t frame_rate;
  uint32_t frame_count;
  uint16_t pwm[32];
};

template<typename TServoPacket>
ssize_t getServoPacket(
  SocketUDP &_sock,
  const char *&_soc_address,
  uint16_t &_soc_port,
  uint32_t _waitMs,
  const std::string &_modelName,
  TServoPacket &_pkt
)
{
  ssize_t recvSize = _sock.recv(&_pkt, sizeof(TServoPacket), _waitMs);
  _sock.get_client_address(_soc_address, _soc_port);

  int counter = 0;
  while (true) {
    TServoPacket last_pkt;
    auto recvSize_last = _sock.recv(&last_pkt, sizeof(TServoPacket), 0ul);
    if (recvSize_last == -1)
      break;
    counter++;
    _pkt = last_pkt;
    recvSize = recvSize_last;
  }
  if (counter > 0)
    gzwarn << "[" << _modelName << "] Drained n packets: " << counter << ".\n";
  return recvSize;
}

// Class responsible for controlling a joint
class Control {

public:

  Control() {
    this->pid.Init(0.1, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0);
  }

  ~Control() {}

  // Joint being controlled
  gz::sim::Entity joint;
  std::string jointName;

  // Publisher for sending commands
  gz::transport::Node::Publisher pub;

  // Name of the topic to forward commands
  std::string cmdTopic;

  // The controller type, valid types are:
  // - VELOCITY control velocity of joint
  // - POSITION control position of joint
  // - EFFORT   control effort of joint
  // - COMMAND  control sends command to topic
  std::string type;

  // Velocity PID for motor control
  gz::math::PID pid;

  // Next command to be applied to the joint
  double cmd = 0;

  // The PWM channel used to command this control
  int channel = 0;

  // Use force controller
  bool useForce = true;

  // A multiplier to scale the raw input command
  double multiplier = 1.0;

  //An offset to shift the zero-point of the raw input command
  double offset = 0.0;

  // Lower bound of PWM input, has default (1000).
  double servo_min = 1000.0;

  // Upper limit of PWM input, has default (2000).
  double servo_max = 2000.0;

  // Rotor slowdown coefficient
  double rotorVelocitySlowdownSim = 1.0;
};

class ArduPilotPluginPrivate {

public:

  // Model entity handler
  gz::sim::Model model{gz::sim::kNullEntity};
  std::string modelName;

  // World entity handler
  gz::sim::World world{gz::sim::kNullEntity};
  std::string worldName;

  // Array of controllers
  std::vector<Control> controls;

  // Keep track of controller update sim-time.
  std::chrono::steady_clock::duration lastControllerUpdateTime {0};

  // Keep track of the time the last servo packet was received.
  std::chrono::steady_clock::duration lastServoPacketRecvTime {0};

  // Socket manager
  SocketUDP sock = SocketUDP(true, true);

  // Address for the flight dynamics model (i.e. this plugin)
  std::string fdm_address{"127.0.0.1"};

  // Address for the flight controller - auto detected
  const char* fcu_address{nullptr};

  // Port for the flight dynamics model (i.e. this plugin)
  uint16_t fdm_port_in{9002};

  // Port for the flight controller - auto detected
  uint16_t fcu_port_out;

  // Set true if have 32 servo channels
  bool have32Channels{false};

  // Set to true when the ArduPilot flight controller is online
  //
  // Set to false when Gazebo starts to prevent blocking, true when 
  // the ArduPilot controller is detected and online, and false if 
  // the connection to the ArduPilot controller times out.
  bool arduPilotOnline{false};

  // Number of consecutive missed ArduPilot controller messages
  int connectionTimeoutCount{0};

  // Max number of consecutive missed ArduPilot controller messages before timeout
  int connectionTimeoutMaxCount;

  // Last received frame rate from the ArduPilot controller
  uint16_t fcu_frame_rate;

  // Last received frame count from the ArduPilot controller
  uint32_t fcu_frame_count = -1;

  // gz-transport Node for rotor msg publishing
  gz::transport::Node node;
};

/// \brief Interface ArduPilot from ardupilot stack
/// modeled after SITL/SIM_*
///
/// The plugin requires the following parameters:
/// <control>             control description block
///    <!-- inputs from Ardupilot -->
///    "channel"          attribute, ardupilot control channel
///    <multiplier>       command multiplier
///    <offset>           command offset
///    <servo_max>        upper limit for PWM input
///    <servo_min>        lower limit for PWM input
///    <!-- output to Gazebo -->
///    <type>             type of control, VELOCITY, POSITION, EFFORT or COMMAND
///    <useForce>         1 if joint forces are applied, 0 to set joint directly
///    <p_gain>           velocity pid p gain
///    <i_gain>           velocity pid i gain
///    <d_gain>           velocity pid d gain
///    <i_max>            velocity pid max integral correction
///    <i_min>            velocity pid min integral correction
///    <cmd_max>          velocity pid max command torque
///    <cmd_min>          velocity pid min command torque
///    <jointName>        motor joint, torque applied here
///    <cmd_topic>        topic to publish commands that are processed
///                       by other plugins
///
///    <turningDirection> rotor turning direction, 'cw' or 'ccw'
///    <frequencyCutoff>  filter incoming joint state
///    <samplingRate>     sampling rate for filtering incoming joint state
///    <rotorVelocitySlowdownSim> for rotor aliasing problem, experimental
///
/// <imuName>     scoped name for the imu sensor
/// <anemometer>  scoped name for the wind sensor
/// <connectionTimeoutMaxCount> timeout before giving up on
///                             controller synchronization
/// <have_32_channels>    set true if 32 channels are enabled
///
class GZ_SIM_VISIBLE ArduPilotPlugin
: public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemReset
{

public: 
  ArduPilotPlugin();
  ~ArduPilotPlugin();

  void Reset(const UpdateInfo &_info, EntityComponentManager &_ecm) final;

  // Load configuration from SDF on startup.
  void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventMgr) final;

  // Do the part of one update loop that involves making changes to simulation.
  void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) final;

  // Not used
  void OnUpdate();

  // Do the part of one update loop that involves reading results from simulation.
  void PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) final;

private:

  // Load control channels
  bool LoadControlChannels(sdf::ElementPtr _sdf, gz::sim::EntityComponentManager &_ecm);

  // Reset PID Joint controllers.
  void ResetPIDs();

  // Update the motor commands given servo PWM values
  void UpdateMotorCommands(const std::array<uint16_t, 32> &_pwm);

  // Update PID Joint controllers.
  void ApplyMotorForces(const double _dt, gz::sim::EntityComponentManager &_ecm);

  // Initialise flight dynamics model socket
  bool InitSockets(sdf::ElementPtr _sdf) const;

  // Receive a servo packet
  bool ReceiveServoPacket();

  std::unique_ptr<ArduPilotPluginPrivate> dataPtr;
};

}  // namespace systems
}  // namespace sim
}  // namespace gz
