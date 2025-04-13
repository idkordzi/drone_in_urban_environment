#include "flight_controller_ros.hpp"


using std::placeholders::_1;

namespace DRONE_NAVIGATION {

FlightControllerROS::FlightControllerROS(rclcpp::NodeOptions options)
: Node("drone_flight_controller_node", options)
{
  this->controller_ = std::make_unique<FlightController>();
  this->estimator_  = std::make_unique<StateEstimator>();

  this->sub_debug_pose_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    config_.sub_debug_pose_, 1, std::bind(&FlightControllerROS::poseCallback, this, _1));
  this->sub_debug_control_ = this->create_subscription<drone_navigation_msgs::msg::ControlVectorStamped>(
    config_.sub_debug_control_, 1, std::bind(&FlightControllerROS::controlCallback, this, _1));
  this->sub_drone_goal_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    config_.sub_drone_goal_, 1, std::bind(&FlightControllerROS::goalCallback, this, _1));
  
  this->pub_drone_control_ = this->create_publisher<drone_navigation_msgs::msg::ControlVectorStamped>(config_.pub_drone_control_, 1);
  this->pub_drone_pose_    = this->create_publisher<geometry_msgs::msg::PoseStamped>(config_.pub_drone_pose_, 1);
  this->pub_drone_vel_     = this->create_publisher<geometry_msgs::msg::TwistStamped>(config_.pub_drone_velocity_, 1);
  // this->pub_drone_acc_     = this->create_publisher<geometry_msgs::msg::AccelStamped>(config_.pub_drone_acceeration_, 1);

  this->pose_cache_    = std::make_shared<geometry_msgs::msg::PoseStamped>();
  this->goal_cache_    = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
  this->control_cache_ = std::make_shared<drone_navigation_msgs::msg::ControlVectorStamped>();

  this->execute_rate_   = std::make_unique<rclcpp::Rate>(config_.thread_hz_);
  this->execute_worker_ = std::thread{&FlightControllerROS::executeThread, this};

  this->msg_control_        = std::make_shared<drone_navigation_msgs::msg::ControlVectorStamped>();
  this->msg_pose_           = std::make_shared<geometry_msgs::msg::PoseStamped>();
  this->msg_velocity_       = std::make_shared<geometry_msgs::msg::TwistStamped>();
  this->msg_position_debug_ = std::make_shared<geometry_msgs::msg::Vector3>();
  this->msg_velocity_debug_ = std::make_shared<geometry_msgs::msg::Vector3>();

  this->pose_cache_->header.stamp = this->get_clock()->now();
  this->pose_cache_->pose.position.x = 0.0f;
  this->pose_cache_->pose.position.y = 0.0f;
  this->pose_cache_->pose.position.z = 0.0f;
}

void FlightControllerROS::poseCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr &msg) {

  float dt = (rclcpp::Duration(msg->header.stamp.sec, msg->header.stamp.nanosec) - 
              rclcpp::Duration(this->pose_cache_->header.stamp.sec, this->pose_cache_->header.stamp.nanosec)).seconds();

  geometry_msgs::msg::Pose drone_pose = msg->poses[this->config_.gz_idx_];

  this->msg_position_debug_->x = drone_pose.position.x;
  this->msg_position_debug_->y = drone_pose.position.y;
  this->msg_position_debug_->z = drone_pose.position.z;

  this->msg_velocity_debug_->x = (drone_pose.position.x - this->pose_cache_->pose.position.x) / dt;
  this->msg_velocity_debug_->y = (drone_pose.position.y - this->pose_cache_->pose.position.y) / dt;
  this->msg_velocity_debug_->z = (drone_pose.position.z - this->pose_cache_->pose.position.z) / dt;

  this->pose_cache_->header = msg->header;
  this->pose_cache_->pose   = drone_pose;
}

void FlightControllerROS::goalCallback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &msg) {
  this->goal_cache_->header = msg->header;
  this->goal_cache_->vector = msg->vector;
}

void FlightControllerROS::controlCallback(const drone_navigation_msgs::msg::ControlVectorStamped::ConstSharedPtr &msg) {
  this->control_cache_->header = msg->header;
  this->control_cache_->control.fr = msg->control.fr;
  this->control_cache_->control.fl = msg->control.fl;
  this->control_cache_->control.rr = msg->control.rr;
  this->control_cache_->control.rl = msg->control.rl;
}

void FlightControllerROS::publish() {

  this->msg_control_->header.stamp = this->get_clock()->now();
  this->pub_drone_control_->publish(*this->msg_control_);

  this->msg_pose_->header.stamp = this->get_clock()->now();
  this->pub_drone_pose_->publish(*this->msg_pose_);

  this->msg_velocity_->header.stamp = this->get_clock()->now();
  this->pub_drone_vel_->publish(*this->msg_velocity_);

  // geometry_msgs::msg::AccelStamped msg_acceleration = geometry_msgs::msg::AccelStamped();
  // msg_acceleration.header.stamp = this->get_clock()->now();
  // this->pub_drone_acc_->publish(msg_acceleration);
}

void FlightControllerROS::executeThread() {
  while (rclcpp::ok()) {
    this->updateDroneState();
    this->updateContolData();
    this->publish();
    if (this->execute_rate_) execute_rate_->sleep();
  }
}

void FlightControllerROS::updateDroneState() {

  // Use state estimator
  if (this->config_.use_state_internal_) {
    // TODO get results from state estimator (NOT IMPLEMENTED YET)
  }
  // Use measurements from simulator
  else {
    this->msg_pose_->pose.position.x = this->msg_position_debug_->x;
    this->msg_pose_->pose.position.y = this->msg_position_debug_->y;
    this->msg_pose_->pose.position.z = this->msg_position_debug_->z;

    this->msg_velocity_->twist.linear.x = this->msg_velocity_debug_->x;
    this->msg_velocity_->twist.linear.y = this->msg_velocity_debug_->y;
    this->msg_velocity_->twist.linear.z = this->msg_velocity_debug_->z;
  }
}

void FlightControllerROS::updateContolData() {

  // Use controller
  if (this->config_.use_control_internal_) {
    // TODO
  }
  // Use external data
  else {
    this->msg_control_->control.fr = this->control_cache_->control.fr;
    this->msg_control_->control.fl = this->control_cache_->control.fl;
    this->msg_control_->control.rr = this->control_cache_->control.rr;
    this->msg_control_->control.rl = this->control_cache_->control.rl;
  }
}

} // namespace DRONE_NAVIGATION
