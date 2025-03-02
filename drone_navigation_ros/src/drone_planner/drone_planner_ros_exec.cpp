#include "drone_planner_ros.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DRONE_NAVIGATION::DronePlannerROS>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
