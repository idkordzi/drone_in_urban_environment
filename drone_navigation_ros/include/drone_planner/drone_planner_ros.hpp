#pragma once

#include <string>
#include <mutex>
#include <thread>
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "vfh_planner.hpp"


namespace DRONE_NAVIGATION {

struct DronePlannerNodeConfig {
  std::string sub_vision_cloud_    = "/drone/vision/cloud";
  std::string sub_vision_target_   = "/drone/vision/target";
  std::string sub_drone_pose_      = "/drone/controller/pose";
  std::string sub_drone_velocity_ = "/drone/controller/velocity";

  std::string pub_planner_goal_ = "/drone/planner/goal";

  float thread_hz_ = 1.0f;

  float drone_pos_margin_ = 0.0f; // [m]
};

class DronePlannerROS : public rclcpp::Node
{

public:
  DronePlannerROS(rclcpp::NodeOptions options);
  ~DronePlannerROS() = default;

private:

  DronePlannerNodeConfig config_ = {};
  std::unique_ptr<VFHPlanner> planner_;

  sensor_msgs::msg::PointCloud::SharedPtr       cloud_cache_;
  geometry_msgs::msg::Vector3Stamped::SharedPtr target_cache_;
  geometry_msgs::msg::PoseStamped::SharedPtr    pose_cache_;
  geometry_msgs::msg::TwistStamped::SharedPtr   velocity_cache_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr       sub_vision_cloud_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vision_target_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    sub_drone_pose_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr   sub_drone_velocity_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_planner_goal_;

  std::mutex mtx_cloud_    = {};
  std::mutex mtx_target_   = {};
  std::mutex mtx_pose_     = {};
  std::mutex mtx_velocity_ = {};

  bool cloud_ready_    = false;
  bool target_ready_   = false;
  bool pose_ready_     = false;
  bool velocity_ready_ = false;

  std::unique_ptr<rclcpp::Rate> execute_rate_;
  std::thread execute_worker_;

  Eigen::Vector3f current_goal_ = {};

  void cloudCallback(const sensor_msgs::msg::PointCloud::ConstSharedPtr &msg);
  void targetCallback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg);
  void publish();

  void executeThread();
  void updateGoalPosition();
};

}

