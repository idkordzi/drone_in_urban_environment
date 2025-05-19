#pragma once

#include <string>
#include <mutex>
#include <thread>

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "local_planner.hpp"


namespace DRONE_NAVIGATION {

struct DronePlannerNodeConfig {
  std::string sub_vision_cloud   = "/drone/vision/cloud";
  std::string sub_vision_target  = "/drone/vision/target";
  std::string sub_drone_pose     = "/drone/controller/pose";
  std::string sub_drone_velocity = "/drone/controller/velocity";

  std::string pub_planner_goal = "/drone/planner/goal";
  std::string pub_planner_cost_image = "/drone/planner/cost_image";

  float thread_hz = 30.0f;
};

class DronePlannerROS : public rclcpp::Node
{

public:
  DronePlannerROS(rclcpp::NodeOptions options);
  ~DronePlannerROS();

private:

  // initialization
  void declareRosParameters();
  void initializeRosNodeConfig();
  void initializeComponents();
  void initializeSubscribers();
  void initializePublishers();
  void initializeExecutionThread();

  // callbacks
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void targetCallback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg);

  void publish();

  // execution thread
  void executeThread();

  void updatePlanner();
  void runPlanner();

  void getCostImage();

  DronePlannerNodeConfig config_ = {};

  std::unique_ptr<LocalPlanner> planner_;

  // subscription msg cache
  sensor_msgs::msg::PointCloud2::SharedPtr      cloud_cache_;
  geometry_msgs::msg::Vector3Stamped::SharedPtr target_cache_;
  geometry_msgs::msg::PoseStamped::SharedPtr    pose_cache_;
  geometry_msgs::msg::TwistStamped::SharedPtr   velocity_cache_;

  // subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr      sub_vision_cloud_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vision_target_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    sub_drone_pose_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr   sub_drone_velocity_;

  // publishers msg cache
  sensor_msgs::msg::Image msg_cost_image_;

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_planner_goal_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_planner_cost_image_;

  // execution thread
  std::unique_ptr<rclcpp::Rate> execute_rate_;
  std::thread execute_worker_;

  std::mutex mtx_cloud_    = {};
  std::mutex mtx_target_   = {};
  std::mutex mtx_pose_     = {};
  std::mutex mtx_velocity_ = {};

  bool cloud_ready_    = false;
  bool target_ready_   = false;
  bool pose_ready_     = false;
  bool velocity_ready_ = false;

  Eigen::Vector3f current_goal_ = {};
};

}

