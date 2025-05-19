#pragma once

#include <string>
#include <vector>
#include <thread>
#include <memory>

#include "Eigen/Dense"

#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "drone_navigation_msgs/msg/control_vector.hpp"
#include "drone_navigation_msgs/msg/control_vector_stamped.hpp"

#include "flight_controller.hpp"
#include "state_estimator.hpp"
#include "socket_udp.hpp"


namespace DRONE_NAVIGATION {

struct FlightControllerNodeConfig {
  std::string sub_debug_pose    = "/gazebo/pose";
  std::string sub_debug_control = "/drone/debug/control";

  std::string sub_drone_imu  = "/gazebo/imu";
  std::string sub_drone_goal = "/drone/planner/goal";

  std::string pub_drone_control      = "/drone/controller/control";
  std::string pub_drone_pose         = "/drone/controller/pose";
  std::string pub_drone_velocity     = "/drone/controller/velocity";
  std::string pub_drone_acceleration = "/drone/controller/acceleration";

  unsigned gz_idx = 1; // index of drone model in gazebo in models pose vector

  float thread_hz = 50.0f; // [Hz]

  bool use_control_internal = true; // use internal/external control data
  bool use_state_internal   = false; // use internal/external state measurement data

  // udp socket config
  std::string address {"127.0.0.1"};
  uint16_t port {9002};
  uint16_t magic_number {18458};
  uint16_t frame_rate {50}; // should be the same as thread call freq
};

class FlightControllerROS : public rclcpp::Node
{

public:
  FlightControllerROS(rclcpp::NodeOptions options);
  ~FlightControllerROS();

private:

  // initialization
  void declareRosParameters();
  void initializeRosNodeConfig();
  void initializeComponents();
  void initializeSubscribers();
  void initializePublishers();
  void initializeExecutionThread();

  // callbacks // debug
  void poseCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr &msg);
  void controlCallback(const drone_navigation_msgs::msg::ControlVector::ConstSharedPtr &msg);

  // callbacks
  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);
  void goalCallback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &msg);
  
  void publish();

  // execution thread
  void executeThread();

  void updateDroneState();
  void updateContolData();
  void sendControl();

  FlightControllerNodeConfig config_ = {};

  std::unique_ptr<FlightController> controller_;
  std::unique_ptr<StateEstimator> estimator_;
  std::unique_ptr<SocketUDP> socket_;

  // subscription msg cache // debug
  geometry_msgs::msg::PoseStamped::SharedPtr pose_cache_;
  geometry_msgs::msg::TwistStamped::SharedPtr velocity_cache_;
  drone_navigation_msgs::msg::ControlVector::SharedPtr control_cache_;

  // subscription msg cache
  sensor_msgs::msg::Imu::SharedPtr imu_cache_;
  geometry_msgs::msg::Vector3Stamped::SharedPtr goal_cache_;

  // published msg cache
  drone_navigation_msgs::msg::ControlVectorStamped::SharedPtr msg_control_;
  geometry_msgs::msg::PoseStamped::SharedPtr msg_pose_;
  geometry_msgs::msg::TwistStamped::SharedPtr msg_velocity_;
  geometry_msgs::msg::AccelStamped::SharedPtr msg_acceleration_;

  // subscribers // debug
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_debug_pose_;
  rclcpp::Subscription<drone_navigation_msgs::msg::ControlVector>::SharedPtr sub_debug_control_;

  // subscribers
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_drone_goal_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_drone_imu_;

  // publishers
  rclcpp::Publisher<drone_navigation_msgs::msg::ControlVectorStamped>::SharedPtr pub_drone_control_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_drone_pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_drone_vel_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr pub_drone_acc_;

  // execution thread
  std::unique_ptr<rclcpp::Rate> execute_rate_;
  std::thread execute_worker_;

  // UDP socket 
  uint32_t frame_count_ {0};
};

} // namespace DRONE_NAVIGATION
