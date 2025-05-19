#include "flight_controller_ros.hpp"


using std::placeholders::_1;

namespace DRONE_NAVIGATION {

FlightControllerROS::FlightControllerROS(rclcpp::NodeOptions options)
: Node("drone_flight_controller_node", options)
{
  this->declareRosParameters();
  this->initializeRosNodeConfig();
  this->initializeComponents();
  this->initializeSubscribers();
  this->initializePublishers();
  this->initializeExecutionThread();
}

FlightControllerROS::~FlightControllerROS() {
  this->execute_worker_.join();
}

void FlightControllerROS::poseCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr &msg) {

  float dt = (rclcpp::Duration(msg->header.stamp.sec, msg->header.stamp.nanosec) - 
              rclcpp::Duration(this->pose_cache_->header.stamp.sec, this->pose_cache_->header.stamp.nanosec)).seconds();

  geometry_msgs::msg::Pose drone_pose = msg->poses[this->config_.gz_idx];

  this->velocity_cache_->header = msg->header;
  this->velocity_cache_->twist.linear.x = (drone_pose.position.x - this->pose_cache_->pose.position.x) / dt;
  this->velocity_cache_->twist.linear.y = (drone_pose.position.y - this->pose_cache_->pose.position.y) / dt;
  this->velocity_cache_->twist.linear.z = (drone_pose.position.z - this->pose_cache_->pose.position.z) / dt;
  
  this->pose_cache_->header = msg->header;
  this->pose_cache_->pose = drone_pose;
}

void FlightControllerROS::controlCallback(const drone_navigation_msgs::msg::ControlVector::ConstSharedPtr &msg) {
  this->control_cache_->fr = msg->fr;
  this->control_cache_->fl = msg->fl;
  this->control_cache_->rr = msg->rr;
  this->control_cache_->rl = msg->rl;
}

void FlightControllerROS::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg) {
  this->imu_cache_->header = msg->header;
  this->imu_cache_->orientation = msg->orientation;
  this->imu_cache_->angular_velocity = msg->angular_velocity;
  this->imu_cache_->linear_acceleration = msg->linear_acceleration;
  // this->imu_cache_->orientation_covariance = msg->orientation_covariance;
  // this->imu_cache_->angular_velocity_covariance = msg->angular_velocity_covariance;
  // this->imu_cache_->linear_acceleration_covariance = msg->linear_acceleration_covariance;
}

void FlightControllerROS::goalCallback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &msg) {
  this->goal_cache_->header = msg->header;
  this->goal_cache_->vector = msg->vector;
}

void FlightControllerROS::publish() {

  this->msg_control_->header.stamp = this->get_clock()->now();
  this->msg_control_->header.frame_id = "odom";
  this->pub_drone_control_->publish(*this->msg_control_);

  this->msg_pose_->header.stamp = this->get_clock()->now();
  this->msg_pose_->header.frame_id = "odom";
  this->pub_drone_pose_->publish(*this->msg_pose_);

  this->msg_velocity_->header.stamp = this->get_clock()->now();
  this->msg_velocity_->header.frame_id = "odom";
  this->pub_drone_vel_->publish(*this->msg_velocity_);

  this->msg_acceleration_->header.stamp = this->get_clock()->now();
  this->msg_acceleration_->header.frame_id = "odom";
  this->pub_drone_acc_->publish(*this->msg_acceleration_);
}

void FlightControllerROS::executeThread() {
  while (rclcpp::ok()) {
    this->updateDroneState();
    this->updateContolData();
    this->sendControl();
    this->publish();
    if (this->execute_rate_) execute_rate_->sleep();
  }
}

void FlightControllerROS::updateDroneState() {

  // Use state estimator
  if (this->config_.use_state_internal) {
    // TODO get results from state estimator (NOT IMPLEMENTED YET)
  }
  // Use measurements from simulator
  else {

    this->msg_pose_->pose = this->pose_cache_->pose;
    this->msg_velocity_->twist.linear =  this->velocity_cache_->twist.linear;
    this->msg_acceleration_->accel.linear =  this->imu_cache_->linear_acceleration;
  }
}

void FlightControllerROS::updateContolData() {

  // Use controller
  if (this->config_.use_control_internal) {

    Eigen::Vector3f g_position = Eigen::Vector3f(this->goal_cache_->vector.x, this->goal_cache_->vector.y, this->goal_cache_->vector.z);
    Eigen::Vector3f c_position = Eigen::Vector3f(this->msg_pose_->pose.position.x, this->msg_pose_->pose.position.y, this->msg_pose_->pose.position.z);
    
    tf2::Quaternion q;
    tf2::fromMsg(this->pose_cache_->pose.orientation, q);
    double yaw, pitch, roll;
    tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
    Eigen::Vector3f c_orientation = Eigen::Vector3f(roll, pitch, yaw);

    Eigen::Vector3f c_velocity = Eigen::Vector3f(this->msg_velocity_->twist.linear.x, this->msg_velocity_->twist.linear.y, this->msg_velocity_->twist.linear.z);
    if (std::isnan(c_velocity.x()) || std::isnan(c_velocity.y()) || std::isnan(c_velocity.z()))
      c_velocity = Eigen::Vector3f::Zero();

    this->controller_->setGoal(g_position);
    auto ctrl = this->controller_->calculateControl(c_position, c_orientation, c_velocity);

    // RCUTILS_LOG_INFO("[DEBUG] G POS    (%0.2f, %0.2f, %0.2f)", g_position.x(), g_position.y(), g_position.z());
    // RCUTILS_LOG_INFO("[DEBUG] C POS    (%0.2f, %0.2f, %0.2f)", c_position.x(), c_position.y(), c_position.z());
    // RCUTILS_LOG_INFO("[DEBUG] C ORIENT (%0.2f, %0.2f, %0.2f)", c_orientation.x(), c_orientation.y(), c_orientation.z());
    // RCUTILS_LOG_INFO("[DEBUG] C VEL    (%0.2f, %0.2f, %0.2f)", c_velocity.x(), c_velocity.y(), c_velocity.z());
    // RCUTILS_LOG_INFO("[DEBUG] R CTRL   (%0.2f, %0.2f, %0.2f, %0.2f)", ctrl[0], ctrl[1], ctrl[2], ctrl[3]);

    this->msg_control_->control.fr = ctrl[0];
    this->msg_control_->control.fl = ctrl[1];
    this->msg_control_->control.rr = ctrl[2];
    this->msg_control_->control.rl = ctrl[3];

    // DebugStruct db = this->controller_->getDebug();
    // RCUTILS_LOG_INFO("[DEBUG] (%6.3f, %6.3f, %6.3f) (%6.3f, %6.3f, %6.3f) (%6.3f, %6.3f), (%6.3f)", 
    //                  db.pos_diff_[0], db.pos_diff_[1], db.pos_diff_[2], db.ang_diff_[0], db.ang_diff_[1], db.ang_diff_[2],
    //                  db.goal_ang_[0], db.goal_ang_[1], db.slope_);

  }
  // Use external data
  else {
    this->msg_control_->control.fr = this->control_cache_->fr;
    this->msg_control_->control.fl = this->control_cache_->fl;
    this->msg_control_->control.rr = this->control_cache_->rr;
    this->msg_control_->control.rl = this->control_cache_->rl;
  }
}

void FlightControllerROS::sendControl() {

  servo_packet_16 pkt;

  // rotors control (fr/rl/fl/rr rotor), ctrl (1100, 1900)
  pkt.pwm[0] = this->msg_control_->control.fr;
  pkt.pwm[1] = this->msg_control_->control.rl;
  pkt.pwm[2] = this->msg_control_->control.fl;
  pkt.pwm[3] = this->msg_control_->control.rr;

  // // gimbl setting (camera roll/pitch/yaw), ctrl (1100, 1900)
  // pkt.pwm[8]  = 1500; // (-30, 30) [deg], 0 in 1500
  // pkt.pwm[9]  = 1700; // (-135, 45) [deg], 0 in 1700
  // pkt.pwm[10] = 1500; // (-160, 160) [deg], 0 in 1500

  pkt.magic = this->config_.magic_number;
  pkt.frame_rate  = this->config_.frame_rate;
  pkt.frame_count = this->frame_count_;

  this->socket_->sendto(&pkt, sizeof(pkt), this->config_.address.c_str(), this->config_.port);

  this->frame_count_++;
}

void FlightControllerROS::declareRosParameters() {

  // ros node params
  this->declare_parameter("ros_node.subs.debug_pose", rclcpp::PARAMETER_STRING);
  this->declare_parameter("ros_node.subs.debug_control", rclcpp::PARAMETER_STRING);
  this->declare_parameter("ros_node.subs.drone_imu", rclcpp::PARAMETER_STRING);
  this->declare_parameter("ros_node.subs.drone_goal", rclcpp::PARAMETER_STRING);

  this->declare_parameter("ros_node.pubs.drone_control", rclcpp::PARAMETER_STRING);
  this->declare_parameter("ros_node.pubs.drone_pose", rclcpp::PARAMETER_STRING);
  this->declare_parameter("ros_node.pubs.drone_velocity", rclcpp::PARAMETER_STRING);
  this->declare_parameter("ros_node.pubs.drone_acceleration", rclcpp::PARAMETER_STRING);

  this->declare_parameter("ros_node.gz_model_idx", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("ros_node.thread_freq", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("ros_node.en_control_int", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("ros_node.en_state_int", rclcpp::PARAMETER_BOOL);

  // udp socket
  this->declare_parameter("udp_socket.address", rclcpp::PARAMETER_STRING);
  this->declare_parameter("udp_socket.port", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("udp_socket.magic", rclcpp::PARAMETER_INTEGER);

  // flight controller
  this->declare_parameter("flight_controller.hover_cmd", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("flight_controller.pid_mov_z", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("flight_controller.pid_rot_x", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("flight_controller.pid_rot_y", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("flight_controller.pid_rot_z", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("flight_controller.pid_lim", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("flight_controller.yaw_off_pos_margin", rclcpp::PARAMETER_DOUBLE);

  // state estimator
  // ...
}

void FlightControllerROS::initializeRosNodeConfig() {

  // subs
  this->config_.sub_debug_pose    = this->get_parameter("ros_node.subs.debug_pose").as_string();
  this->config_.sub_debug_control = this->get_parameter("ros_node.subs.debug_control").as_string();
  this->config_.sub_drone_imu     = this->get_parameter("ros_node.subs.drone_imu").as_string();
  this->config_.sub_drone_goal    = this->get_parameter("ros_node.subs.drone_goal").as_string();

  // pubs
  this->config_.pub_drone_control      = this->get_parameter("ros_node.pubs.drone_control").as_string();
  this->config_.pub_drone_pose         = this->get_parameter("ros_node.pubs.drone_pose").as_string();
  this->config_.pub_drone_velocity     = this->get_parameter("ros_node.pubs.drone_velocity").as_string();
  this->config_.pub_drone_acceleration = this->get_parameter("ros_node.pubs.drone_acceleration").as_string();

  // other params
  this->config_.gz_idx = (unsigned)(this->get_parameter("ros_node.gz_model_idx").as_int());
  this->config_.thread_hz = (float)(this->get_parameter("ros_node.thread_freq").as_double());
  this->config_.use_control_internal = this->get_parameter("ros_node.en_control_int").as_bool();
  this->config_.use_state_internal   = this->get_parameter("ros_node.en_state_int").as_bool();
}

void FlightControllerROS::initializeComponents() {

  // flight controller
  std::vector<double> hover_cmd = this->get_parameter("flight_controller.hover_cmd").as_double_array();
  std::vector<double> pid_mov_z = this->get_parameter("flight_controller.pid_mov_z").as_double_array();
  std::vector<double> pid_rot_x = this->get_parameter("flight_controller.pid_rot_x").as_double_array();
  std::vector<double> pid_rot_y = this->get_parameter("flight_controller.pid_rot_y").as_double_array();
  std::vector<double> pid_rot_z = this->get_parameter("flight_controller.pid_rot_z").as_double_array();
  std::vector<double> pid_lim   = this->get_parameter("flight_controller.pid_lim").as_double_array();
  float yaw_off_pos_margin = this->get_parameter("flight_controller.yaw_off_pos_margin").as_double();

  FlightControllerConfig flight_controller_config = {};
  flight_controller_config.hover_ctrl_fr = hover_cmd[0];
  flight_controller_config.hover_ctrl_fl = hover_cmd[1];
  flight_controller_config.hover_ctrl_rr = hover_cmd[2];
  flight_controller_config.hover_ctrl_rl = hover_cmd[3];
  
  flight_controller_config.reg_mov_z_Kp = pid_mov_z[0];
  flight_controller_config.reg_mov_z_Ki = pid_mov_z[1];
  flight_controller_config.reg_mov_z_Kd = pid_mov_z[2];

  flight_controller_config.reg_rot_x_Kp = pid_rot_x[0];
  flight_controller_config.reg_rot_x_Ki = pid_rot_x[1];
  flight_controller_config.reg_rot_x_Kd = pid_rot_x[2];

  flight_controller_config.reg_rot_y_Kp = pid_rot_y[0];
  flight_controller_config.reg_rot_y_Ki = pid_rot_y[1];
  flight_controller_config.reg_rot_y_Kd = pid_rot_y[2];

  flight_controller_config.reg_rot_z_Kp = pid_rot_z[0];
  flight_controller_config.reg_rot_z_Ki = pid_rot_z[1];
  flight_controller_config.reg_rot_z_Kd = pid_rot_z[2];

  flight_controller_config.pid_min = pid_lim[0];
  flight_controller_config.pid_max = pid_lim[1];

  flight_controller_config.yaw_off_pos_margin = yaw_off_pos_margin;

  // state estimator
  StateEstimatorConfig state_estimator_config = {};

  // udp socket
  this->config_.address = this->get_parameter("udp_socket.address").as_string();
  this->config_.port = (uint16_t)(this->get_parameter("udp_socket.port").as_int());
  this->config_.magic_number = (uint16_t)(this->get_parameter("udp_socket.magic").as_int());
  this->config_.frame_rate = (uint16_t)(this->config_.thread_hz);

  // initialize components
  this->controller_ = std::make_unique<FlightController>(flight_controller_config);
  this->estimator_ = std::make_unique<StateEstimator>(state_estimator_config);
  this->socket_ = std::make_unique<SocketUDP>(true, true);
  
}

void FlightControllerROS::initializeSubscribers() {

  // subscribers // debug
  this->sub_debug_pose_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    this->config_.sub_debug_pose, 1, std::bind(&FlightControllerROS::poseCallback, this, _1));
  this->sub_debug_control_ = this->create_subscription<drone_navigation_msgs::msg::ControlVector>(
    this->config_.sub_debug_control, 1, std::bind(&FlightControllerROS::controlCallback, this, _1));
  
  // subscribers
  this->sub_drone_goal_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    this->config_.sub_drone_goal, 1, std::bind(&FlightControllerROS::goalCallback, this, _1));
  this->sub_drone_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
    this->config_.sub_drone_imu, 1, std::bind(&FlightControllerROS::imuCallback, this, _1));

  // subscription msg cache // debug
  this->pose_cache_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
  this->velocity_cache_ = std::make_shared<geometry_msgs::msg::TwistStamped>();
  this->control_cache_ = std::make_shared<drone_navigation_msgs::msg::ControlVector>();

  this->pose_cache_->header.stamp = this->get_clock()->now();
  this->pose_cache_->pose.position.x = 0.0f;
  this->pose_cache_->pose.position.y = 0.0f;
  this->pose_cache_->pose.position.z = 0.19f;
  this->pose_cache_->pose.orientation.x = 0.0f;
  this->pose_cache_->pose.orientation.y = 0.0f;
  this->pose_cache_->pose.orientation.z = 0.0f;
  this->pose_cache_->pose.orientation.w = 1.0f;

  this->control_cache_->fr = 0.0f;
  this->control_cache_->fl = 0.0f;
  this->control_cache_->rr = 0.0f;
  this->control_cache_->rl = 0.0f;

  // subscription msg cache
  this->imu_cache_ = std::make_shared<sensor_msgs::msg::Imu>();
  this->goal_cache_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();

  this->goal_cache_->header.stamp = this->get_clock()->now();
  this->goal_cache_->vector.x = 0.0f;
  this->goal_cache_->vector.y = 0.0f;
  this->goal_cache_->vector.z = 0.195f;
}

void FlightControllerROS::initializePublishers() {

  // publishers
  this->pub_drone_control_ = this->create_publisher<drone_navigation_msgs::msg::ControlVectorStamped>(this->config_.pub_drone_control, 1);
  this->pub_drone_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(this->config_.pub_drone_pose, 1);
  this->pub_drone_vel_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(this->config_.pub_drone_velocity, 1);
  this->pub_drone_acc_ = this->create_publisher<geometry_msgs::msg::AccelStamped>(this->config_.pub_drone_acceleration, 1);

  // published msg cache
  this->msg_control_ = std::make_shared<drone_navigation_msgs::msg::ControlVectorStamped>();
  this->msg_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
  this->msg_velocity_ = std::make_shared<geometry_msgs::msg::TwistStamped>();
  this->msg_acceleration_ = std::make_shared<geometry_msgs::msg::AccelStamped>();
}

void FlightControllerROS::initializeExecutionThread() {
  this->execute_rate_   = std::make_unique<rclcpp::Rate>(this->config_.thread_hz);
  this->execute_worker_ = std::thread{&FlightControllerROS::executeThread, this};
}

} // namespace DRONE_NAVIGATION
