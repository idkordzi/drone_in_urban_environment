#include "drone_test_class.cpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DRONE_NAVIGATION::DroneTest>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}