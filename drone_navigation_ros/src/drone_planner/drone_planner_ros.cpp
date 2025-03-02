#include "drone_planner_ros.hpp"


using std::placeholders::_1;

namespace DRONE_NAVIGATION {

DronePlannerROS::DronePlannerROS(rclcpp::NodeOptions options)
: Node("drone_planner_node", options)
{
  this->planner_ = std::make_unique<VFHPlanner>();

  this->sub_vision_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
    config_.sub_vision_cloud_, 1, std::bind(&DronePlannerROS::cloudCallback, this, _1));
  this->sub_vision_target_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    config_.sub_vision_target_, 1, std::bind(&DronePlannerROS::targetCallback, this, _1));
  this->sub_drone_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    config_.sub_drone_pose_, 1, std::bind(&DronePlannerROS::poseCallback, this, _1));
  this->sub_drone_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    config_.sub_drone_velocity_, 1, std::bind(&DronePlannerROS::velocityCallback, this, _1));
  
  this->pub_planner_goal_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(config_.pub_planner_goal_, 1);
  
  this->cloud_cache_    = std::make_shared<sensor_msgs::msg::PointCloud>();
  this->target_cache_   = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
  this->pose_cache_     = std::make_shared<geometry_msgs::msg::PoseStamped>();
  this->velocity_cache_ = std::make_shared<geometry_msgs::msg::TwistStamped>();

  this->execute_rate_   = std::make_unique<rclcpp::Rate>(config_.thread_hz_);
  this->execute_worker_ = std::thread{&DronePlannerROS::executeThread, this};

  this->current_goal_ = Eigen::Vector3f::Zero();
}

void DronePlannerROS::cloudCallback(const sensor_msgs::msg::PointCloud::ConstSharedPtr &msg) {

  std::lock_guard<std::mutex> lg(this->mtx_cloud_);

  this->cloud_cache_->header   = msg->header;
  this->cloud_cache_->points   = msg->points;
  this->cloud_cache_->channels = msg->channels;

  this->cloud_ready_ = true;
}

void DronePlannerROS::targetCallback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &msg) {

  std::lock_guard<std::mutex> lg(this->mtx_target_);

  this->target_cache_->header = msg->header;
  this->target_cache_->vector = msg->vector;

  this->target_ready_ = true;
}

void DronePlannerROS::poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {

  std::lock_guard<std::mutex> lg(this->mtx_pose_);

  this->pose_cache_->header = msg->header;
  this->pose_cache_->pose   = msg->pose;

  this->pose_ready_ = true;
}

void DronePlannerROS::velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg) {

  std::lock_guard<std::mutex> lg(this->mtx_velocity_);

  this->velocity_cache_->header = msg->header;
  this->velocity_cache_->twist  = msg->twist;

  this->velocity_ready_ = true;
}

void DronePlannerROS::publish() {

  geometry_msgs::msg::Vector3Stamped msg_goal = geometry_msgs::msg::Vector3Stamped();
  msg_goal.header.stamp = this->get_clock()->now();
  msg_goal.vector.x = this->current_goal_.x();
  msg_goal.vector.x = this->current_goal_.y();
  msg_goal.vector.x = this->current_goal_.z();
  this->pub_planner_goal_->publish(msg_goal);
}

void DronePlannerROS::executeThread() {
  
  while (rclcpp::ok()) {
    
    if (this->cloud_ready_ && this->target_ready_ && this->pose_ready_ && this->velocity_ready_) {
      std::lock_guard<std::mutex> lg_cloud(this->mtx_cloud_);
      std::lock_guard<std::mutex> lg_target(this->mtx_target_);
      std::lock_guard<std::mutex> lg_pose(this->mtx_pose_);
      std::lock_guard<std::mutex> lg_velocity(this->mtx_velocity_);
  
      this->updateGoalPosition();
      this->publish();
  
      this->cloud_ready_    = false;
      this->target_ready_   = false;
      this->pose_ready_     = false;
      this->velocity_ready_ = false;
    }
    if (execute_rate_) execute_rate_->sleep();
  }
}

void DronePlannerROS::updateGoalPosition() {

  Eigen::Vector3f current_position(this->pose_cache_->pose.position.x, this->pose_cache_->pose.position.y, this->pose_cache_->pose.position.z);
  Eigen::Quaternionf current_orientation_q(this->pose_cache_->pose.orientation.x, this->pose_cache_->pose.orientation.y,
                                           this->pose_cache_->pose.orientation.z, this->pose_cache_->pose.orientation.w);
  Eigen::Vector3f current_orientation_v = current_orientation_q.toRotationMatrix().eulerAngles(0, 1, 2);
  Eigen::Vector3f curent_velocity(this->velocity_cache_->twist.linear.x, this->velocity_cache_->twist.linear.y, this->velocity_cache_->twist.linear.z);
  Eigen::Vector3f current_target(this->target_cache_->vector.x, this->target_cache_->vector.y, this->target_cache_->vector.z);

  this->planner_->setPosition(current_position);
  this->planner_->setOrientation(current_orientation_v);
  this->planner_->setVelocity(curent_velocity);
  this->planner_->setGoal(current_target);

  this->planner_->updateStatus();

  // no trajectory available/ trajectory outdated -> new computation required/ current data required
  if (this->planner_->getStatus() == 1) {

    PointCloud<PointXYZ> new_cloud = PointCloud<PointXYZ>();
    std::vector<geometry_msgs::msg::Point32>& cloud_ptr = this->cloud_cache_->points;
    for (unsigned i = 0; i < cloud_ptr.size(); i++) {
      new_cloud.push_back(PointXYZ(cloud_ptr[i].x, cloud_ptr[i].y, cloud_ptr[i].z));
    }

    this->planner_->setPointCloud(new_cloud);
  }
  this->planner_->run();

  // TODO handle case of no next goal position available
  this->current_goal_ = this->planner_->getNextGoal();
}

} // namespace DRONE_NAVIGATION
