#include "drone_vision_ros.hpp"


using std::placeholders::_1;

namespace DRONE_NAVIGATION {

DroneVisionROS::DroneVisionROS(rclcpp::NodeOptions options)
: Node("drone_vision_node", options)
{
  this->camera_wrapper_ = std::make_unique<CameraWrapper>();
  this->yolo_wrapper_   = std::make_unique<YOLOWrapper>();

  this->sub_camera_color_ = this->create_subscription<sensor_msgs::msg::Image>(
    config_.sub_camera_color_, 1, std::bind(&DroneVisionROS::cameraColorCallback, this, _1));
  this->sub_camera_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
    config_.sub_camera_depth_, 1, std::bind(&DroneVisionROS::cameraDepthCallback, this, _1));
  
  this->pub_vision_cloud_  = this->create_publisher<sensor_msgs::msg::PointCloud>(config_.pub_vision_cloud_, 1);
  this->pub_vision_target_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(config_.pub_vision_target_, 1);
  
  this->im_color_cache_ = std::make_shared<sensor_msgs::msg::Image>();
  this->im_depth_cache_ = std::make_shared<sensor_msgs::msg::Image>();

  this->cloud_cache_  = std::make_shared<sensor_msgs::msg::PointCloud>();
  this->target_cache_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();

  this->execute_rate_   = std::make_unique<rclcpp::Rate>(config_.thread_hz_);
  this->execute_worker_ = std::thread{&DroneVisionROS::executeThread, this};
}

void DroneVisionROS::cameraColorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {

  std::lock_guard<std::mutex> lg(this->mtx_im_color_);

  this->im_color_cache_->header       = msg->header;
  this->im_color_cache_->height       = msg->height;
  this->im_color_cache_->width        = msg->width;
  this->im_color_cache_->encoding     = msg->encoding;
  this->im_color_cache_->is_bigendian = msg->is_bigendian;
  this->im_color_cache_->step         = msg->step;
  this->im_color_cache_->data         = msg->data;

  this->im_color_ready_ = true;
}

void DroneVisionROS::cameraDepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  
  std::lock_guard<std::mutex> lg(this->mtx_im_depth_);

  this->im_depth_cache_->header       = msg->header;
  this->im_depth_cache_->height       = msg->height;
  this->im_depth_cache_->width        = msg->width;
  this->im_depth_cache_->encoding     = msg->encoding;
  this->im_depth_cache_->is_bigendian = msg->is_bigendian;
  this->im_depth_cache_->step         = msg->step;
  this->im_depth_cache_->data         = msg->data;

  this->im_depth_ready_ = true;
}

void DroneVisionROS::publish() {
  this->cloud_cache_->header.stamp = this->get_clock()->now();
  this->pub_vision_cloud_->publish(*(this->cloud_cache_));

  this->target_cache_->header.stamp = this->get_clock()->now();
  this->pub_vision_target_->publish(*(this->target_cache_));
}

void DroneVisionROS::executeThread() {
  
  while (rclcpp::ok()) {
    
    if (this->im_color_ready_ && this->im_depth_ready_) {
      std::lock_guard<std::mutex> lg_color(this->mtx_im_color_);
      std::lock_guard<std::mutex> lg_depth(this->mtx_im_depth_);
  
      this->detectTarget();
      this->convertToCloud();
      this->publish();
  
      this->im_color_ready_ = false;
      this->im_depth_ready_ = false;
    }
    if (execute_rate_) execute_rate_->sleep();
  }
}

void DroneVisionROS::detectTarget() {

  cv::Mat image = cv::Mat(this->config_.im_height_, this->config_.im_width_, CV_8UC3, cv::Scalar(0, 0, 0));
  
  // Use 8UC3 encoding (uint8 3x channel)

  // cv_bridge::CvImagePtr cv_ptr
  // cv_ptr = cv_bridge::toCvCopy(*this->im_color_cache_, this->im_color_cache_->encoding);

  {
    unsigned plen = 3;
    unsigned step = this->im_color_cache_->step;
    std::vector<uint8_t>& data = this->im_color_cache_->data;

    if (data.size() != this->config_.im_height_ * this->config_.im_width_ * plen)
      RCUTILS_LOG_INFO("[WARN] Incoming color image does not meet expected frame size (byte size: %ld)", data.size());

    for (unsigned i = 0; i < this->config_.im_height_; i++) {
      for (unsigned j = 0; j < this->config_.im_width_; j++) {
        cv::Vec3b& pixel = image.at<cv::Vec3b>(j, i);
        pixel[0] = data[i*step+j*plen];
        pixel[1] = data[i*step+j*plen+1];
        pixel[2] = data[i*step+j*plen+2];
      }
    }

    cv::Size im_size = image.size();
    if (im_size.empty()) {
      RCUTILS_LOG_INFO("[WARN] Could not retrive CV image size");
    } else if (im_size.height != this->config_.im_height_ || im_size.width != this->config_.im_width_) {
      RCUTILS_LOG_INFO("[WARN] CV image does not meet expected frame size");
    }
  }

  this->yolo_wrapper_->setInput(image);
  this->yolo_wrapper_->run();
  
  Eigen::Vector2i target_loc = this->yolo_wrapper_->getOutput();

  if (target_loc.x() == -1 || target_loc.y() == -1) { // target was not found
    this->target_cache_->vector.x = NAN;
    this->target_cache_->vector.y = NAN;
    this->target_cache_->vector.z = NAN;
  } else {
    // Use 32FC1 enconding (float32 single channel)
    unsigned plen = 4;
    unsigned step = this->im_depth_cache_->step;
    std::vector<uint8_t>& data = this->im_depth_cache_->data;

    // Target location from YOLO detector is in CV standard, while ROS Image is in C standard
    unsigned loch = target_loc.x();
    unsigned locv = target_loc.y();
    unsigned loc  = locv * step + loch * plen;

    uint32_t raw = data[loc] + (data[loc+1] << 8) + (data[loc+2] << 16) + (data[loc+3] << 24);
    float converted = 0.0f;
    std::memcpy(&converted, &raw, sizeof(float));

    // TODO convert from polar to cartesian and write to Vector3
    Eigen::Vector3f point = Eigen::Vector3f::Zero();

    this->target_cache_->vector.x = point.x();
    this->target_cache_->vector.y = point.y();
    this->target_cache_->vector.z = point.z();
  }
}

void DroneVisionROS::convertToCloud() {
  
  // Use 32FC1 enconding (float32 single channel)
  unsigned plen = 4;
  // unsigned step = this->im_depth_cache_->step;
  std::vector<uint8_t>& data = this->im_color_cache_->data;

  this->cloud_cache_->points.clear();
  this->cloud_cache_->channels.clear();

  sensor_msgs::msg::ChannelFloat32 channel = sensor_msgs::msg::ChannelFloat32();
  channel.name = "distance";

  for (unsigned i = 0; i < data.size() / plen; i += plen) {
    uint32_t raw = data[i] + (data[i+1] << 8) + (data[i+2] << 16) + (data[i+3] << 24);
    float converted = 0.0f;
    std::memcpy(&converted, &raw, sizeof(float));

    // TODO convert from polar to cartesian and write to Vector3
    geometry_msgs::msg::Point32 point = geometry_msgs::msg::Point32();

    this->cloud_cache_->points.push_back(point);
    channel.values.push_back(converted);
  }
  this->cloud_cache_->channels.push_back(channel);

}

} // namespace DRONE_NAVIGATION
