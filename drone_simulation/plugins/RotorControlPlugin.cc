#include "RotorControlPlugin.hh"

GZ_ADD_PLUGIN(gz::sim::systems::ArduPilotPlugin,
              gz::sim::System,
              gz::sim::systems::ArduPilotPlugin::ISystemConfigure,
              gz::sim::systems::ArduPilotPlugin::ISystemPostUpdate,
              gz::sim::systems::ArduPilotPlugin::ISystemReset,
              gz::sim::systems::ArduPilotPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::ArduPilotPlugin, "RotorControlPlugin")

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {

std::unordered_set<Entity> EntitiesFromUnscopedName(const std::string &_name, const EntityComponentManager &_ecm, Entity _relativeTo) {
  // holds entities that match
  std::vector<Entity> entities;

  if (_relativeTo == kNullEntity)
    entities = _ecm.EntitiesByComponents(components::Name(_name));
  else {
    auto descendents = _ecm.Descendants(_relativeTo);
    for (const auto& descendent : descendents) {
      if (_ecm.EntityHasComponentType(descendent, gz::sim::components::Name::typeId)) {
        auto nameComp = _ecm.Component<gz::sim::components::Name>(descendent);
        if (nameComp->Data() == _name)
          entities.push_back(descendent);
      }
    }
  }
  if (entities.empty()) return {};
  return std::unordered_set<Entity>(entities.begin(), entities.end());
}

Entity JointByName(EntityComponentManager &_ecm, Entity _modelEntity, const std::string &_name) {

  auto entities = entitiesFromScopedName(_name, _ecm, _modelEntity);

  if (entities.empty()) {
    gzerr << "Joint with name [" << _name << "] not found. The joint will not respond to ArduPilot commands\n";
    return kNullEntity;
  }
  else if (entities.size() > 1)
    gzwarn << "Multiple joint entities with name[" << _name << "] found. Using the first one.\n";

  Entity joint = *entities.begin();

  // Validate
  if (!_ecm.EntityHasComponentType(joint, components::Joint::typeId)) {
    gzerr << "Entity with name[" << _name << "] is not a joint\n";
    return kNullEntity;
  }

  // Ensure the joint has a velocity component
  if (!_ecm.EntityHasComponentType(joint, components::JointVelocity::typeId))
    _ecm.CreateComponent(joint, components::JointVelocity());

  return joint;
}

}
}
}

gz::sim::systems::ArduPilotPlugin::ArduPilotPlugin() : dataPtr(new ArduPilotPluginPrivate()) {}

gz::sim::systems::ArduPilotPlugin::~ArduPilotPlugin() {}

void gz::sim::systems::ArduPilotPlugin::Reset(const UpdateInfo &_info, EntityComponentManager &_ecm) {

  // update velocity PID for controls and apply force to joint
  for (size_t i = 0; i < this->dataPtr->controls.size(); ++i) {
    gz::sim::components::JointForceCmd* jfcComp = nullptr;
    gz::sim::components::JointVelocityCmd* jvcComp = nullptr;
    if (this->dataPtr->controls[i].useForce || this->dataPtr->controls[i].type == "EFFORT") {
      jfcComp = _ecm.Component<gz::sim::components::JointForceCmd>(this->dataPtr->controls[i].joint);
      if (jfcComp == nullptr) {
        jfcComp = _ecm.CreateComponent(this->dataPtr->controls[i].joint, gz::sim::components::JointForceCmd({0}));
      }
    }
    else if (this->dataPtr->controls[i].type == "VELOCITY") {
      jvcComp = _ecm.Component<gz::sim::components::JointVelocityCmd>(this->dataPtr->controls[i].joint);
      if (jvcComp == nullptr) {
        jvcComp = _ecm.CreateComponent(this->dataPtr->controls[i].joint, gz::sim::components::JointVelocityCmd({0}));
      }
    }
  }
}

void gz::sim::systems::ArduPilotPlugin::Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                                                  gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventMgr) {
  // Make a clone so that we can call non-const methods
  sdf::ElementPtr sdfClone = _sdf->Clone();

  this->dataPtr->model = gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    gzerr << "Plugin not attached to a model entity. Failed to initialize.\n";
    return;
  }
  this->dataPtr->modelName = this->dataPtr->model.Name(_ecm);

  this->dataPtr->world = gz::sim::World(_ecm.EntityByComponents(components::World()));
  if (!this->dataPtr->world.Valid(_ecm)) {
    gzerr << "World entity not found.\n";
    return;
  }
  if (this->dataPtr->world.Name(_ecm).has_value())
    this->dataPtr->worldName = this->dataPtr->world.Name(_ecm).value();

  // Load control channel params
  if(!this->LoadControlChannels(sdfClone, _ecm)) {
    gzerr << "[" << this->dataPtr->modelName << "] Could not load control channels.\n";
    return;
  }

  // Initialise sockets
  if (!InitSockets(sdfClone)) {
    gzerr << "[" << this->dataPtr->modelName << "] Could not initialize web socket.\n";
    return;
  }

  // Missed update count before we declare arduPilotOnline status false
  this->dataPtr->connectionTimeoutMaxCount = sdfClone->Get("connectionTimeoutMaxCount", 10).first;

  // Set channels nubmer
  this->dataPtr->have32Channels = sdfClone->Get("have_32_channels", false).first;
}

/////////////////////////////////////////////////
bool gz::sim::systems::ArduPilotPlugin::LoadControlChannels(sdf::ElementPtr _sdf, gz::sim::EntityComponentManager &_ecm) {

  // per control channel
  sdf::ElementPtr controlSDF;
  if (_sdf->HasElement("control"))
    controlSDF = _sdf->GetElement("control");

  while (controlSDF) {
    Control control;

    if (controlSDF->HasAttribute("channel"))
      control.channel = atoi(controlSDF->GetAttribute("channel")->GetAsString().c_str());
    else {
      gzerr << "[" << this->dataPtr->modelName << "] Channel attribute was not specified.\n";
      return false;
    }

    if (controlSDF->HasElement("jointName"))
      control.jointName = controlSDF->Get<std::string>("jointName");
    else {
      gzerr << "[" << this->dataPtr->modelName << "] Control channel is not attached to any joint.\n";
      return false;
    }

    // Get the pointer to the joint
    control.joint = JointByName(_ecm, this->dataPtr->model.Entity(), control.jointName);
    if (control.joint == gz::sim::kNullEntity) {
      gzerr << "[" << this->dataPtr->modelName << "] " << "Couldn't find specified joint [" << control.jointName << "].\n";
      return false;
    }

    if (controlSDF->HasElement("type"))
      control.type = controlSDF->Get<std::string>("type");
    else {
      gzerr << "[" << this->dataPtr->modelName << "] " <<  "Control type not specified.\n";
      return false;
    }

    if (control.type != "VELOCITY" && control.type != "POSITION" && control.type != "EFFORT" && control.type != "COMMAND") {
      gzwarn << "[" << this->dataPtr->modelName << "] " << "Control type [" << control.type << "] not recognized.\n";
      return false;
    }

    if (controlSDF->HasElement("useForce"))
      control.useForce = controlSDF->Get<bool>("useForce");

    // Set up publisher if relaying the command
    if (control.type == "COMMAND") {
      if (controlSDF->HasElement("cmd_topic"))
        control.cmdTopic = controlSDF->Get<std::string>("cmd_topic");
      else {
        control.cmdTopic = "/world/" + this->dataPtr->worldName + "/model/" + this->dataPtr->modelName + "/joint/" + control.jointName + "/cmd";
        gzwarn << "[" << this->dataPtr->modelName << "] Control type [" << control.type << "] requires a valid <cmd_topic>. Using default.\n";
      }
      gzmsg << "[" << this->dataPtr->modelName << "] Advertising on []" << control.cmdTopic << "].\n";
      control.pub = this->dataPtr->node.Advertise<msgs::Double>(control.cmdTopic);
    }

    if (controlSDF->HasElement("multiplier"))
      control.multiplier = controlSDF->Get<double>("multiplier");
    else {
      gzerr << "[" << this->dataPtr->modelName << "] Channel [" << control.channel << "]: <multiplier> not specified.\n";
      return false;
    }
    if (controlSDF->HasElement("offset"))
      control.offset = controlSDF->Get<double>("offset");
    else {
      gzerr << "[" << this->dataPtr->modelName << "] Channel [" << control.channel << "]: <offset> not specified.\n";
      return false;
    }
    if (controlSDF->HasElement("servo_min"))
      control.servo_min = controlSDF->Get<double>("servo_min");
    else {
      gzerr << "[" << this->dataPtr->modelName << "] Channel [" << control.channel << "]: <servo_min> not specified.\n";
      return false;
    }
    if (controlSDF->HasElement("servo_max"))
      control.servo_max = controlSDF->Get<double>("servo_max");
    else {
      gzerr << "[" << this->dataPtr->modelName << "] Channel [" << control.channel << "]: <servo_max> not specified.\n";
      return false;
    }

    control.rotorVelocitySlowdownSim = controlSDF->Get("rotorVelocitySlowdownSim", 1.0).first;
    if (gz::math::equal(control.rotorVelocitySlowdownSim, 0.0)) {
      gzerr << "[" << this->dataPtr->modelName << "] Control for joint [" << control.jointName << "] rotorVelocitySlowdownSim cannot be zero.\n";
      return false;
    }

    // Overload the PID parameters if they are available.
    double param;

    param = controlSDF->Get("p_gain", control.pid.PGain()).first;
    control.pid.SetPGain(param);

    param = controlSDF->Get("i_gain", control.pid.IGain()).first;
    control.pid.SetIGain(param);

    param = controlSDF->Get("d_gain", control.pid.DGain()).first;
    control.pid.SetDGain(param);

    param = controlSDF->Get("i_max", control.pid.IMax()).first;
    control.pid.SetIMax(param);

    param = controlSDF->Get("i_min", control.pid.IMin()).first;
    control.pid.SetIMin(param);

    param = controlSDF->Get("cmd_max", control.pid.CmdMax()).first;
    control.pid.SetCmdMax(param);

    param = controlSDF->Get("cmd_min", control.pid.CmdMin()).first;
    control.pid.SetCmdMin(param);

    // set pid initial command
    control.pid.SetCmd(0.0);

    this->dataPtr->controls.push_back(control);
    controlSDF = controlSDF->GetNextElement("control");
  }
  return true;
}

void gz::sim::systems::ArduPilotPlugin::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) {

  if (!_info.paused && _info.simTime > this->dataPtr->lastControllerUpdateTime) {
    if (this->ReceiveServoPacket())
      this->dataPtr->lastServoPacketRecvTime = _info.simTime;
    if (this->dataPtr->arduPilotOnline) {
      double dt = std::chrono::duration_cast<std::chrono::duration<double> >(_info.simTime - this->dataPtr->lastControllerUpdateTime).count();
      this->ApplyMotorForces(dt, _ecm);
    }
  }
}

void gz::sim::systems::ArduPilotPlugin::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) {
  if (!_info.paused && _info.simTime > this->dataPtr->lastControllerUpdateTime && this->dataPtr->arduPilotOnline)
    this->dataPtr->lastControllerUpdateTime = _info.simTime;
}

void gz::sim::systems::ArduPilotPlugin::ResetPIDs() {
  for (size_t i = 0; i < this->dataPtr->controls.size(); ++i)
    this->dataPtr->controls[i].cmd = 0;
}

bool gz::sim::systems::ArduPilotPlugin::InitSockets(sdf::ElementPtr _sdf) const {
  // Get the fdm address if provided, otherwise default to localhost
  this->dataPtr->fdm_address = _sdf->Get("fdm_addr", static_cast<std::string>("127.0.0.1")).first;
  this->dataPtr->fdm_port_in = _sdf->Get("fdm_port_in", static_cast<uint32_t>(9002)).first;

  // Bind the socket
  if (!this->dataPtr->sock.bind(this->dataPtr->fdm_address.c_str(), this->dataPtr->fdm_port_in)) {
    gzerr << "[" << this->dataPtr->modelName << "] Failed to bind with [" 
          << this->dataPtr->fdm_address << ":" << this->dataPtr->fdm_port_in
          << "]. Aborting plugin.\n";
    return false;
  }
  gzlog << "[" << this->dataPtr->modelName << "] Flight dynamics model @" << this->dataPtr->fdm_address << ":" << this->dataPtr->fdm_port_in << ".\n";
  return true;
}

bool gz::sim::systems::ArduPilotPlugin::ReceiveServoPacket()
{
    // Added detection for whether ArduPilot is online or not.
    // If ArduPilot is detected (receive of fdm packet from someone),
    // then socket receive wait time is increased from 1ms to 1 sec
    // to accomodate network jitter.
    // If ArduPilot is not detected, receive call blocks for 1ms
    // on each call.
    // Once ArduPilot presence is detected, it takes this many
    // missed receives before declaring the FCS offline.

    uint32_t waitMs;
    if (this->dataPtr->arduPilotOnline) {
        // Increase timeout for recv once we detect a packet from ArduPilot FCS.
        // If this value is too high then it will block the main Gazebo
        // update loop and adversely affect the RTF.
        waitMs = 2.5;
    }
    else {
        // Otherwise skip quickly and do not set control force.
        waitMs = 1;
    }

    // 16 / 32 channel compatibility
    uint16_t pkt_magic{0};
    uint16_t pkt_frame_rate{0};
    uint16_t pkt_frame_count{0};
    std::array<uint16_t, 32> pkt_pwm;
    ssize_t recvSize{-1};
    if (this->dataPtr->have32Channels) {
      servo_packet_32 pkt;
      recvSize = getServoPacket(
        this->dataPtr->sock,
        this->dataPtr->fcu_address,
        this->dataPtr->fcu_port_out,
        waitMs,
        this->dataPtr->modelName,
        pkt);
      pkt_magic = pkt.magic;
      pkt_frame_rate = pkt.frame_rate;
      pkt_frame_count = pkt.frame_count;
      std::copy(std::begin(pkt.pwm), std::end(pkt.pwm), std::begin(pkt_pwm));
    }
    else {
      servo_packet_16 pkt;
      recvSize = getServoPacket(
        this->dataPtr->sock,
        this->dataPtr->fcu_address,
        this->dataPtr->fcu_port_out,
        waitMs,
        this->dataPtr->modelName,
        pkt);
      pkt_magic = pkt.magic;
      pkt_frame_rate = pkt.frame_rate;
      pkt_frame_count = pkt.frame_count;
      std::copy(std::begin(pkt.pwm), std::end(pkt.pwm), std::begin(pkt_pwm));
    }

    // Packet not received, increment timeout count if online, then return
    if (recvSize == -1) {
      if (this->dataPtr->arduPilotOnline) {
        if (++this->dataPtr->connectionTimeoutCount > this->dataPtr->connectionTimeoutMaxCount) {
          this->dataPtr->connectionTimeoutCount = 0;
          this->dataPtr->arduPilotOnline = false;
          gzwarn << "[" << this->dataPtr->modelName << "] Broken ArduPilot connection. Resetting motor control.\n";
          this->ResetPIDs();
        }
      }
      return false;
    }

    // Check magic, return if invalid
    constexpr uint16_t magic_16 = 18458;
    constexpr uint16_t magic_32 = 29569;
    uint16_t magic = this->dataPtr->have32Channels ? magic_32 : magic_16;
    if (magic != pkt_magic) {
      gzwarn << "[" << this->dataPtr->modelName << "] Incorrect protocol magic [" << pkt_magic << "] should be [" << magic << "].\n";
      return false;
    }

    // Controller is online
    if (!this->dataPtr->arduPilotOnline) {
      this->dataPtr->arduPilotOnline = true;

      gzlog << "[" << this->dataPtr->modelName << "] Connected to ArduPilot controller @"
            << this->dataPtr->fcu_address << ":" << this->dataPtr->fcu_port_out << ".\n";
    }

    // Update frame rate
    this->dataPtr->fcu_frame_rate = pkt_frame_rate;

    // Check for controller reset
    if (pkt_frame_count < this->dataPtr->fcu_frame_count) {
      // @TODO implement re-initialisation
    }

    // Check for duplicate frame
    else if (pkt_frame_count == this->dataPtr->fcu_frame_count) {
      gzwarn << "[" << this->dataPtr->modelName << "] Duplicate input frame.\n";
      return false;
    }

    // Check for skipped frames
    else if (pkt_frame_count != this->dataPtr->fcu_frame_count + 1 && this->dataPtr->arduPilotOnline) {
      gzwarn << "[" << this->dataPtr->modelName << "] Missed " << pkt_frame_count - this->dataPtr->fcu_frame_count << " input frames.\n";
    }

    // Update frame count
    this->dataPtr->fcu_frame_count = pkt_frame_count;

    // Reset the connection timeout so we don't accumulate
    this->dataPtr->connectionTimeoutCount = 0;

    this->UpdateMotorCommands(pkt_pwm);

    return true;
}

void gz::sim::systems::ArduPilotPlugin::UpdateMotorCommands(const std::array<uint16_t, 32> &_pwm) {
  int max_servo_channels = this->dataPtr->have32Channels ? 32 : 16;

  // Compute command based on requested motorSpeed
  for (unsigned i = 0; i < this->dataPtr->controls.size(); ++i) {
    // Enforce limit on the number of <control> elements
    if (i < MAX_MOTORS) {
      if (this->dataPtr->controls[i].channel < max_servo_channels) {
        // convert pwm to raw cmd: [servo_min, servo_max] => [0, 1],
        // default is: [1000, 2000] => [0, 1]
        const double pwm = _pwm[this->dataPtr->controls[i].channel];
        const double pwm_min = this->dataPtr->controls[i].servo_min;
        const double pwm_max = this->dataPtr->controls[i].servo_max;
        const double multiplier = this->dataPtr->controls[i].multiplier;
        const double offset = this->dataPtr->controls[i].offset;

        // bound incoming cmd between 0 and 1
        double raw_cmd = (pwm - pwm_min)/(pwm_max - pwm_min);
        raw_cmd = gz::math::clamp(raw_cmd, 0.0, 1.0);
        this->dataPtr->controls[i].cmd = multiplier * (raw_cmd + offset);
      }
      else {
        gzerr << "[" << this->dataPtr->modelName << "] Control[" << i << "] Channel [" << this->dataPtr->controls[i].channel 
              << "] is greater than the number of servo channels [" << max_servo_channels << "]. Control not applied.\n";
      }
    }
    else {
      gzerr << "[" << this->dataPtr->modelName << "] " << "too many motors, skipping [" << i << " > " << MAX_MOTORS << "].\n";
    }
  }
}

void gz::sim::systems::ArduPilotPlugin::ApplyMotorForces(const double _dt, gz::sim::EntityComponentManager &_ecm) {
  // Update velocity PID for controls and apply force to joint
  for (size_t i = 0; i < this->dataPtr->controls.size(); ++i) {
    // Publish commands to be relayed to other plugins
    if (this->dataPtr->controls[i].type == "COMMAND") {
      msgs::Double cmd;
      cmd.set_data(this->dataPtr->controls[i].cmd);
      this->dataPtr->controls[i].pub.Publish(cmd);
      continue;
    }

    gz::sim::components::JointForceCmd* jfcComp = nullptr;
    gz::sim::components::JointVelocityCmd* jvcComp = nullptr;
    if (this->dataPtr->controls[i].useForce || this->dataPtr->controls[i].type == "EFFORT") {
      jfcComp = _ecm.Component<gz::sim::components::JointForceCmd>(this->dataPtr->controls[i].joint);
      if (jfcComp == nullptr) {
        jfcComp = _ecm.CreateComponent(this->dataPtr->controls[i].joint, gz::sim::components::JointForceCmd({0}));
      }
    }
    else if (this->dataPtr->controls[i].type == "VELOCITY") {
      jvcComp = _ecm.Component<gz::sim::components::JointVelocityCmd>(this->dataPtr->controls[i].joint);
      if (jvcComp == nullptr) {
        jvcComp = _ecm.CreateComponent(this->dataPtr->controls[i].joint, gz::sim::components::JointVelocityCmd({0}));
      }
    }

    if (this->dataPtr->controls[i].useForce) {
      if (this->dataPtr->controls[i].type == "VELOCITY") {
        const double velTarget = this->dataPtr->controls[i].cmd / this->dataPtr->controls[i].rotorVelocitySlowdownSim;
        gz::sim::components::JointVelocity* vComp = _ecm.Component<gz::sim::components::JointVelocity>(this->dataPtr->controls[i].joint);
        if (vComp && !vComp->Data().empty()) {
          const double vel = vComp->Data()[0];
          const double error = vel - velTarget;
          const double force = this->dataPtr->controls[i].pid.Update(error, std::chrono::duration<double>(_dt));
          jfcComp->Data()[0] = force;
        }
      }
      else if (this->dataPtr->controls[i].type == "POSITION") {
        const double posTarget = this->dataPtr->controls[i].cmd;
        gz::sim::components::JointPosition* pComp = _ecm.Component<gz::sim::components::JointPosition>(this->dataPtr->controls[i].joint);
        if (pComp && !pComp->Data().empty()) {
          const double pos = pComp->Data()[0];
          const double error = pos - posTarget;
          const double force = this->dataPtr->controls[i].pid.Update(error, std::chrono::duration<double>(_dt));
          jfcComp->Data()[0] = force;
        }
      }
      else if (this->dataPtr->controls[i].type == "EFFORT") {
        const double force = this->dataPtr->controls[i].cmd;
        jfcComp->Data()[0] = force;
      }
      else {
        // do nothing
      }
    }
    else {
      if (this->dataPtr->controls[i].type == "VELOCITY") {
        jvcComp->Data()[0] = this->dataPtr->controls[i].cmd;
      }
      else if (this->dataPtr->controls[i].type == "POSITION") {
        // @TODO Figure out whether position control matters, and if so, how to use it.
        gzwarn << "Failed to do position control on joint " << i << " because there's no JointPositionCmd component.\n";
      }
      else if (this->dataPtr->controls[i].type == "EFFORT") {
        const double force = this->dataPtr->controls[i].cmd;
        jvcComp->Data()[0] = force;
      }
      else {
        // do nothing
      }
    }
  }
}
