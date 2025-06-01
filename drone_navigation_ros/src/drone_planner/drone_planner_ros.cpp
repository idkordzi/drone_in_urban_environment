#include "drone_planner_ros.hpp"


using std::placeholders::_1;

namespace DRONE_NAVIGATION {

DronePlannerROS::DronePlannerROS(rclcpp::NodeOptions options)
: Node("drone_planner_node", options)
{
  this->declareRosParameters();
  this->initializeRosNodeConfig();
  this->initializeComponents();
  this->initializeSubscribers();
  this->initializePublishers();
  this->initializeExecutionThread();
}

DronePlannerROS::~DronePlannerROS() {
  this->execute_worker_.join();
}

void DronePlannerROS::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {

  std::lock_guard<std::mutex> lg(this->mtx_cloud_);

  this->cloud_cache_->header = msg->header;

  // meta data
  this->cloud_cache_->width        = msg->width;
  this->cloud_cache_->height       = msg->height;
  this->cloud_cache_->point_step   = msg->point_step;
  this->cloud_cache_->row_step     = msg->row_step;
  this->cloud_cache_->is_bigendian = msg->is_bigendian;
  this->cloud_cache_->is_dense     = msg->is_dense;

  this->cloud_cache_->fields = msg->fields;

  // point data
  this->cloud_cache_->data = msg->data;

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
  msg_goal.vector.y = this->current_goal_.y();
  msg_goal.vector.z = this->current_goal_.z();
  this->pub_planner_goal_->publish(msg_goal);

  this->msg_hist_image_.header.stamp = this->get_clock()->now();
  this->pub_planner_hist_image_->publish(this->msg_hist_image_);

  this->msg_cost_image_.header.stamp = this->get_clock()->now();
  this->pub_planner_cost_image_->publish(this->msg_cost_image_);
}

void DronePlannerROS::executeThread() {
  
  while (rclcpp::ok()) {
    
    if (this->pose_ready_ && this->velocity_ready_) {
  
      this->updatePlanner();
      this->runPlanner();

      this->getDebugImage();

      this->publish();
  
      this->cloud_ready_    = false;
      this->target_ready_   = false;
      this->pose_ready_     = false;
      this->velocity_ready_ = false;
    }
    if (execute_rate_) execute_rate_->sleep();
  }
}

void DronePlannerROS::updatePlanner() {
  
  std::lock_guard<std::mutex> lg_pose(this->mtx_pose_);
  std::lock_guard<std::mutex> lg_velocity(this->mtx_velocity_);
  std::lock_guard<std::mutex> lg_target(this->mtx_target_);
  std::lock_guard<std::mutex> lg_cloud(this->mtx_cloud_);

  Eigen::Vector3f current_position(this->pose_cache_->pose.position.x, this->pose_cache_->pose.position.y, this->pose_cache_->pose.position.z);

  Eigen::Quaternionf current_orientation_q(this->pose_cache_->pose.orientation.x, this->pose_cache_->pose.orientation.y,
                                           this->pose_cache_->pose.orientation.z, this->pose_cache_->pose.orientation.w);
  Eigen::Vector3f current_orientation_v = current_orientation_q.normalized().toRotationMatrix().eulerAngles(2, 1, 0);
  current_orientation_v.x() = -current_orientation_v.x();
  current_orientation_v.y() = -current_orientation_v.y();

  short cfx = current_orientation_v.x() > 0 ? -1 : 1;
  short cfy = current_orientation_v.y() > 0 ? -1 : 1;
  short cfz = current_orientation_v.z() > 0 ? -1 : 1;
  if (std::abs(current_orientation_v.x()) > PI_F2) {
    current_orientation_v.x() += cfx * PI_F;
  }
  if (std::abs(current_orientation_v.y()) > PI_F2) {
    current_orientation_v.y() += cfy * PI_F;
    current_orientation_v.z() += cfz * PI_F;
  }

  Eigen::Vector3f curent_velocity(this->velocity_cache_->twist.linear.x, this->velocity_cache_->twist.linear.y, this->velocity_cache_->twist.linear.z);

  Eigen::Vector3f current_target(this->target_cache_->vector.x, this->target_cache_->vector.y, this->target_cache_->vector.z);

  this->planner_->setState(current_position, current_orientation_v, curent_velocity);

  if (this->target_ready_)
    this->planner_->setGoal(current_target);

  if (this->cloud_ready_) {
    PointCloud<PointXYZ> new_cloud = PointCloud<PointXYZ>();
    sensor_msgs::PointCloud2Iterator<float> iter_xzy(*(this->cloud_cache_), "x");
    
    for (uint32_t i = 0; i < this->cloud_cache_->height; i++) {
      for (uint32_t j = 0; j < this->cloud_cache_->width; j++, iter_xzy += 1) {
        new_cloud.push_back(PointXYZ(iter_xzy[0], iter_xzy[1], iter_xzy[2]));
      }
    }
    this->planner_->setPointCloud(new_cloud);
  }

  // RCUTILS_LOG_INFO("[DEBUG] POS    (%0.2f, %0.2f, %0.2f)", current_position.x(), current_position.y(), current_position.z());
  // RCUTILS_LOG_INFO("[DEBUG] ORIENT (%0.2f, %0.2f, %0.2f)", current_orientation_v.x(), current_orientation_v.y(), current_orientation_v.z());
  // RCUTILS_LOG_INFO("[DEBUG] TARGET (%0.2f, %0.2f, %0.2f)", current_target.x(), current_target.y(), current_target.z());
}

void DronePlannerROS::runPlanner() {

  this->planner_->run();
  this->current_goal_ = this->planner_->getNext();

  // RCUTILS_LOG_INFO("[DEBUG] GOAL   (%0.2f, %0.2f, %0.2f)", this->current_goal_.x(), this->current_goal_.y(), this->current_goal_.z());
}

void DronePlannerROS::getDebugImage() {
  cv::Mat hist_image = this->planner_->getHistImage();
  this->msg_hist_image_ = *(cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", hist_image).toImageMsg());

  cv::Mat cost_image = this->planner_->getCostImage();
  this->msg_cost_image_ = *(cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", cost_image).toImageMsg());
}

void DronePlannerROS::declareRosParameters() {

  // ros node params
  this->declare_parameter("ros_node.subs.vision_cloud", rclcpp::PARAMETER_STRING);
  this->declare_parameter("ros_node.subs.vision_target", rclcpp::PARAMETER_STRING);
  this->declare_parameter("ros_node.subs.drone_pose", rclcpp::PARAMETER_STRING);
  this->declare_parameter("ros_node.subs.drone_velocity", rclcpp::PARAMETER_STRING);

  this->declare_parameter("ros_node.pubs.planner_goal", rclcpp::PARAMETER_STRING);
  this->declare_parameter("ros_node.pubs.planner_hist_image", rclcpp::PARAMETER_STRING);
  this->declare_parameter("ros_node.pubs.planner_cost_image", rclcpp::PARAMETER_STRING);

  this->declare_parameter("ros_node.thread_freq", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("planner.skip_planning", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("planner.sensor_min_range", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("planner.sensor_max_range", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("planner.camera_fov_h", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("planner.camera_fov_v", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("planner.histogram_alpha", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("planner.point_max_age", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("planner.goal_dev_margin", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("planner.goal_min_dist", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("planner.goal_min_alt_diff", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("planner.max_candidates_per_it", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("planner.drone_pos_margin", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("planner.planning_step", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("planner.yaw_cost_param", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("planner.pitch_block_distance", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("planner.pitch_cost_param", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("planner.velocity_cost_param", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("planner.obstacle_min_distance", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("planner.obstacle_cost_param", rclcpp::PARAMETER_DOUBLE);
}

void DronePlannerROS::initializeRosNodeConfig() {

  // subs
  this->config_.sub_vision_cloud   = this->get_parameter("ros_node.subs.vision_cloud").as_string();
  this->config_.sub_vision_target  = this->get_parameter("ros_node.subs.vision_target").as_string();
  this->config_.sub_drone_pose     = this->get_parameter("ros_node.subs.drone_pose").as_string();
  this->config_.sub_drone_velocity = this->get_parameter("ros_node.subs.drone_velocity").as_string();

  // pubs
  this->config_.pub_planner_goal = this->get_parameter("ros_node.pubs.planner_goal").as_string();
  this->config_.pub_planner_hist_image = this->get_parameter("ros_node.pubs.planner_hist_image").as_string();
  this->config_.pub_planner_cost_image = this->get_parameter("ros_node.pubs.planner_cost_image").as_string();

  // other params
  this->config_.thread_hz = (float)(this->get_parameter("ros_node.thread_freq").as_double());
}

void DronePlannerROS::initializeComponents() {

  // planner
  this->current_goal_ = Eigen::Vector3f::Zero();

  LocalPlannerConfig planner_config = {};
  planner_config.skip_planning = this->get_parameter("planner.skip_planning").as_bool();
  planner_config.sensor_min_range = (float)(this->get_parameter("planner.sensor_min_range").as_double());
  planner_config.sensor_max_range = (float)(this->get_parameter("planner.sensor_max_range").as_double());
  planner_config.camera_fov_h = (float)(this->get_parameter("planner.camera_fov_h").as_double());
  planner_config.camera_fov_v = (float)(this->get_parameter("planner.camera_fov_v").as_double());
  planner_config.alpha = (float)(this->get_parameter("planner.histogram_alpha").as_int());
  planner_config.point_max_age   = this->get_parameter("planner.point_max_age").as_double();
  planner_config.goal_dev_margin = (float)(this->get_parameter("planner.goal_dev_margin").as_double());
  planner_config.goal_min_dist     = (float)(this->get_parameter("planner.goal_min_dist").as_double());
  planner_config.goal_min_alt_diff = (float)(this->get_parameter("planner.goal_min_alt_diff").as_double());
  planner_config.max_candidates_per_it = this->get_parameter("planner.max_candidates_per_it").as_int();
  planner_config.drone_pos_margin = (float)(this->get_parameter("planner.drone_pos_margin").as_double());
  planner_config.planning_step = (float)(this->get_parameter("planner.planning_step").as_double());

  planner_config.yaw_cost_param = (float)(this->get_parameter("planner.yaw_cost_param").as_double());
  planner_config.pitch_block_distance = (float)(this->get_parameter("planner.pitch_block_distance").as_double());
  planner_config.pitch_cost_param = (float)(this->get_parameter("planner.pitch_cost_param").as_double());
  planner_config.velocity_cost_param = (float)(this->get_parameter("planner.velocity_cost_param").as_double());
  planner_config.obstacle_min_distance = (float)(this->get_parameter("planner.obstacle_min_distance").as_double());
  planner_config.obstacle_cost_param = (float)(this->get_parameter("planner.obstacle_cost_param").as_double());

  // initialize components
  this->planner_ = std::make_unique<LocalPlanner>(planner_config);
}

void DronePlannerROS::initializeSubscribers() {

  // subs
  this->sub_vision_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    this->config_.sub_vision_cloud, 1, std::bind(&DronePlannerROS::cloudCallback, this, _1));
  this->sub_vision_target_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    this->config_.sub_vision_target, 1, std::bind(&DronePlannerROS::targetCallback, this, _1));
  this->sub_drone_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->config_.sub_drone_pose, 1, std::bind(&DronePlannerROS::poseCallback, this, _1));
  this->sub_drone_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    this->config_.sub_drone_velocity, 1, std::bind(&DronePlannerROS::velocityCallback, this, _1));

  // subs msg cache
  this->cloud_cache_    = std::make_shared<sensor_msgs::msg::PointCloud2>();
  this->target_cache_   = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
  this->pose_cache_     = std::make_shared<geometry_msgs::msg::PoseStamped>();
  this->velocity_cache_ = std::make_shared<geometry_msgs::msg::TwistStamped>();
}

void DronePlannerROS::initializePublishers() {

  // pubs
  this->pub_planner_goal_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(this->config_.pub_planner_goal, 1);
  this->pub_planner_hist_image_ = this->create_publisher<sensor_msgs::msg::Image>(this->config_.pub_planner_hist_image, 1);
  this->pub_planner_cost_image_ = this->create_publisher<sensor_msgs::msg::Image>(this->config_.pub_planner_cost_image, 1);
}

void DronePlannerROS::initializeExecutionThread() {
  this->execute_rate_   = std::make_unique<rclcpp::Rate>(this->config_.thread_hz);
  this->execute_worker_ = std::thread(&DronePlannerROS::executeThread, this);
}

} // namespace DRONE_NAVIGATION
