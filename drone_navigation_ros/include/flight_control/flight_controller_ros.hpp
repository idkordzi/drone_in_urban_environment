#pragma once

#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "drone_navigation_msgs/msg/control_vector_stamped.hpp"
#include "flight_controller.hpp"
#include "state_estimator.hpp"


namespace DRONE_NAVIGATION {

struct FlightControllerNodeConfig {
  std::string sub_debug_pose_    = "/gazebo/pose";
  std::string sub_debug_control_ = "/drone/debug/control";
  std::string sub_drone_goal_    = "/drone/planner/goal";

  unsigned gz_idx_ = 2; // index of drone model in gazebo in models pose vector

  std::string pub_drone_control_     = "/drone/controller/control";
  std::string pub_drone_pose_        = "/drone/controller/pose";
  std::string pub_drone_velocity_    = "/drone/controller/velocity";
  std::string pub_drone_acceeration_ = "/drone/controller/acceleration";

  float thread_hz_ = 1.0f; // [Hz]

  bool use_state_internal_   = false; // use internal/external state measurement data
  bool use_control_internal_ = false; // use internal/external control data
};

class FlightControllerROS : public rclcpp::Node
{

public:
  FlightControllerROS(rclcpp::NodeOptions options);
  ~FlightControllerROS() = default;

private:

  FlightControllerNodeConfig config_ = {};
  std::unique_ptr<FlightController> controller_;
  std::unique_ptr<StateEstimator> estimator_;

  geometry_msgs::msg::PoseStamped::SharedPtr    pose_cache_;
  geometry_msgs::msg::Vector3Stamped::SharedPtr goal_cache_;

  drone_navigation_msgs::msg::ControlVectorStamped::SharedPtr control_cache_;

  drone_navigation_msgs::msg::ControlVectorStamped::SharedPtr msg_control_;

  geometry_msgs::msg::PoseStamped::SharedPtr  msg_pose_;
  geometry_msgs::msg::TwistStamped::SharedPtr msg_velocity_;
  geometry_msgs::msg::Vector3::SharedPtr      msg_position_debug_;
  geometry_msgs::msg::Vector3::SharedPtr      msg_velocity_debug_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr      sub_debug_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_drone_goal_;

  rclcpp::Subscription<drone_navigation_msgs::msg::ControlVectorStamped>::SharedPtr sub_debug_control_;

  rclcpp::Publisher<drone_navigation_msgs::msg::ControlVectorStamped>::SharedPtr pub_drone_control_;
  
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pub_drone_pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_drone_vel_;
  // rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr pub_drone_acc_;

  std::unique_ptr<rclcpp::Rate> execute_rate_;
  std::thread execute_worker_;

  void poseCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr &msg);
  void goalCallback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &msg);
  void controlCallback(const drone_navigation_msgs::msg::ControlVectorStamped::ConstSharedPtr &msg);
  void publish();

  void executeThread();

  void updateDroneState();
  void updateContolData();
};

} // namespace DRONE_NAVIGATION
