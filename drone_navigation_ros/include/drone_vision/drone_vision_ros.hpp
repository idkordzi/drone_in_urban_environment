#pragma once

#include <string>
#include <mutex>
#include <thread>
#include "opencv2/opencv.hpp"
#include "Eigen/Dense"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
// #include "cv_bridge/cv_bridge.h"
#include "camera_wrapper.hpp"
#include "yolo_wrapper.hpp"


namespace DRONE_NAVIGATION {

struct DroneVisionNodeConfig {
  std::string sub_camera_color_ = "/gazebo/camera/color";
  std::string sub_camera_depth_ = "/gazebo/camera/depth";

  std::string pub_vision_cloud_  = "/drone/vision/cloud";
  std::string pub_vision_target_ = "/drone/vision/target";

  float thread_hz_ = 10.0f;

  unsigned im_width_  = 0;
  unsigned im_height_ = 0;
};

class DroneVisionROS : public rclcpp::Node
{

public:
  DroneVisionROS(rclcpp::NodeOptions options);
  ~DroneVisionROS() = default;

private:

  DroneVisionNodeConfig config_ = {};
  std::unique_ptr<CameraWrapper> camera_wrapper_;
  std::unique_ptr<YOLOWrapper>   yolo_wrapper_;

  sensor_msgs::msg::Image::SharedPtr im_color_cache_;
  sensor_msgs::msg::Image::SharedPtr im_depth_cache_;

  sensor_msgs::msg::PointCloud::SharedPtr       cloud_cache_;
  geometry_msgs::msg::Vector3Stamped::SharedPtr target_cache_;

  std::mutex mtx_im_color_ = {};
  std::mutex mtx_im_depth_ = {};

  bool im_color_ready_ = false;
  bool im_depth_ready_ = false;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_color_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_depth_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr       pub_vision_cloud_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_vision_target_;

  std::unique_ptr<rclcpp::Rate> execute_rate_;
  std::thread execute_worker_;

  void cameraColorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void cameraDepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void publish();

  void executeThread();
  void detectTarget();
  void convertToCloud();
};

} // namespace DRONE_NAVIGATION
