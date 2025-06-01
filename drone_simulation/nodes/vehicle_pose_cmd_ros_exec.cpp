#include "vehicle_pose_cmd_ros.cpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DRONE_SIMULATION::PickupPoseCmd>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
