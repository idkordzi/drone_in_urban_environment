#pragma once

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <memory>

#include "opencv2/opencv.hpp"

#include "Eigen/Dense"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "cv_bridge/cv_bridge.hpp"

#include "camera_wrapper.hpp"
#include "yolo_wrapper.hpp"


namespace DRONE_NAVIGATION {

struct DroneVisionNodeConfig {
  std::string sub_camera_color = "/gazebo/camera/color";
  std::string sub_camera_depth = "/gazebo/camera/depth";
  std::string sub_camera_cloud = "/gazebo/camera/points";

  std::string pub_vision_target = "/drone/vision/target";
  std::string pub_vision_cloud  = "/drone/vision/cloud";

  float thread_hz = 10.0f;

  int im_width  = 640;
  int im_height = 360;

  bool use_ext_camera = true; // use data from external (simulation) camera instead of internal wrapper

  int mean_circ_radi = 3; // radius in pixels
};

class DroneVisionROS : public rclcpp::Node
{

public:
  DroneVisionROS(rclcpp::NodeOptions options);
  ~DroneVisionROS();

private:

  //initialization
  void declareRosParameters();
  void initializeRosNodeConfig();
  void initializeComponents();
  void initializeSubscribers();
  void initializePublishers();
  void initializeExecutionThread();

  // callbacks
  void cameraColorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void cameraDepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

  void publish();

  // execution thread
  void executeThread();

  void getDataFromSimulation();
  void getDataFromCamera();
  void detectTarget();

  DroneVisionNodeConfig config_ = {};

  std::unique_ptr<CameraWrapper> camera_wrapper_;
  std::unique_ptr<YOLOWrapper> yolo_wrapper_;

  // subscription msg cache
  sensor_msgs::msg::Image::SharedPtr im_color_cache_;
  sensor_msgs::msg::Image::SharedPtr im_depth_cache_;
  sensor_msgs::msg::PointCloud2::SharedPtr pt_cloud_cache_;

  // published msg cache
  geometry_msgs::msg::Vector3Stamped::SharedPtr msg_target_;
  sensor_msgs::msg::PointCloud2::SharedPtr msg_cloud_;

  // subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_color_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_depth_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_camera_cloud_;

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_vision_target_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_vision_cloud_;

  // execution thread
  std::unique_ptr<rclcpp::Rate> execute_rate_;
  std::thread execute_worker_;

  std::mutex mtx_im_color_ = {};
  std::mutex mtx_im_depth_ = {};
  std::mutex mtx_pt_cloud_ = {};

  bool im_color_ready_ = false;
  bool im_depth_ready_ = false;
  bool pt_cloud_ready_ = false;

  cv_bridge::CvImagePtr cv_ptr_ = {};

  cv::Mat color_frame_ = {};
  cv::Mat depth_frame_ = {};

  bool target_found_ = false;
};

} // namespace DRONE_NAVIGATION
