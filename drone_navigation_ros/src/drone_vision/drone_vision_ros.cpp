#include "drone_vision_ros.hpp"


using std::placeholders::_1;

namespace DRONE_NAVIGATION {

DroneVisionROS::DroneVisionROS(rclcpp::NodeOptions options)
: Node("drone_vision_node", options)
{
  this->declareRosParameters();
  this->initializeRosNodeConfig();
  this->initializeComponents();
  this->initializeSubscribers();
  this->initializePublishers();
  this->initializeExecutionThread();
}

DroneVisionROS::~DroneVisionROS() {
  this->execute_worker_.join();
}

void DroneVisionROS::cameraColorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  if (!this->config_.use_ext_camera) return;

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
  if (!this->config_.use_ext_camera) return;

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

void DroneVisionROS::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  if (!this->config_.use_ext_camera) return;

  std::lock_guard<std::mutex> lg(this->mtx_pt_cloud_);

  this->pt_cloud_cache_->header = msg->header;

  // meta data
  this->pt_cloud_cache_->width        = msg->width;
  this->pt_cloud_cache_->height       = msg->height;
  this->pt_cloud_cache_->point_step   = msg->point_step;
  this->pt_cloud_cache_->row_step     = msg->row_step;
  this->pt_cloud_cache_->is_bigendian = msg->is_bigendian;
  this->pt_cloud_cache_->is_dense     = msg->is_dense;

  this->pt_cloud_cache_->fields = msg->fields;

  // point data
  this->pt_cloud_cache_->data = msg->data;

  this->pt_cloud_ready_ = true;
}

void DroneVisionROS::publish() {

  if (this->target_found_) {
    this->msg_target_->header.stamp = this->get_clock()->now();
    this->msg_target_->header.frame_id = "camera";
    this->pub_vision_target_->publish(*(this->msg_target_));
  }

  this->msg_cloud_->header.stamp = this->get_clock()->now();
  this->msg_cloud_->header.frame_id = "camera";
  this->pub_vision_cloud_->publish(*(this->msg_cloud_));
}

void DroneVisionROS::executeThread() {
  
  while (rclcpp::ok()) {
    
    if (this->im_color_ready_ && this->im_depth_ready_ && this->pt_cloud_ready_) {
      std::lock_guard<std::mutex> lg_color(this->mtx_im_color_);
      std::lock_guard<std::mutex> lg_depth(this->mtx_im_depth_);
      std::lock_guard<std::mutex> lg_cloud(this->mtx_pt_cloud_);

      if (this->config_.use_ext_camera)
        this->getDataFromSimulation();
      else
        this->getDataFromCamera();
      this->detectTarget();
      this->publish();
  
      this->im_color_ready_ = false;
      this->im_depth_ready_ = false;
      this->pt_cloud_ready_ = false;
    }
    if (execute_rate_) execute_rate_->sleep();
  }
}

void DroneVisionROS::getDataFromSimulation() {
  if (!this->config_.use_ext_camera) return;

  this->cv_ptr_ = cv_bridge::toCvCopy(*(this->im_color_cache_));
  this->color_frame_ = cv_ptr_->image;

  this->cv_ptr_ = cv_bridge::toCvCopy(*(this->im_depth_cache_));
  this->depth_frame_ = cv_ptr_->image;

  this->msg_cloud_ = this->pt_cloud_cache_;
}

void DroneVisionROS::getDataFromCamera() {
  if (this->config_.use_ext_camera) return;

  this->color_frame_ = this->camera_wrapper_->getColorImage();
  this->depth_frame_ = this->camera_wrapper_->getDepthImage();

  PointCloudFlatten pc_flat = this->camera_wrapper_->getPointCloud();

  this->msg_cloud_->data.clear();
  
  sensor_msgs::PointCloud2Iterator<float> iter_x(*(this->msg_cloud_), "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*(this->msg_cloud_), "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*(this->msg_cloud_), "z");
  sensor_msgs::PointCloud2Iterator<float> iter_d(*(this->msg_cloud_), "distance");

  for (unsigned i = 0, p = 0; i < pc_flat.height; i++) {
    for (unsigned j = 0; j < pc_flat.width; j++, iter_x +=1, iter_y +=1, iter_z +=1, iter_d +=1) {
      p = i*pc_flat.width + j;
      *iter_x = pc_flat.cloud[p].x;
      *iter_y = pc_flat.cloud[p].y;
      *iter_z = pc_flat.cloud[p].z;
      *iter_d = pc_flat.cloud[p].distance;
    }
  }
}

void DroneVisionROS::detectTarget() {

  cv::Size im_size = this->color_frame_.size();
  if (im_size.empty()) {
    RCUTILS_LOG_INFO("[WARN] Could not retrive CV image size");
  } else if (im_size.width != this->config_.im_width || im_size.height != this->config_.im_height) {
    RCUTILS_LOG_INFO("[WARN] CV image does not meet expected frame size (size: %d x %x)", im_size.width, im_size.height);
  }

  this->yolo_wrapper_->setInput(this->color_frame_);
  this->yolo_wrapper_->run();
  
  Eigen::Vector2i target_loc = this->yolo_wrapper_->getTarget();

  if (target_loc.x() == -1 || target_loc.y() == -1) { // target was not found
    this->msg_target_->vector.x = NAN;
    this->msg_target_->vector.y = NAN;
    this->msg_target_->vector.z = NAN;
    this->target_found_ = false;
  } else {
    // target location from YOLO detector is in CV standard, while ROS Image is in C standard

    int loch = target_loc.x();
    int locv = target_loc.y();
    // int loc = locv * this->config_.im_width + loch;

    sensor_msgs::PointCloud2Iterator<float> iter_x(*(this->msg_cloud_), "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*(this->msg_cloud_), "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*(this->msg_cloud_), "z");

    this->msg_target_->vector.x = 0.0f;
    this->msg_target_->vector.y = 0.0f;
    this->msg_target_->vector.z = 0.0f;

    int cnt = 0;
    for (int i = -this->config_.mean_circ_radi; i <= this->config_.mean_circ_radi; i++) {
      for (int j = -this->config_.mean_circ_radi + std::abs(i); j <= this->config_.mean_circ_radi - std::abs(i); j++) {
        int ii = locv + i;
        int jj = loch + j;
        if (ii >= 0 && jj >= 0 && ii <= this->config_.im_height && jj <= this->config_.im_width) {
          int off = ii * this->config_.im_width + jj;
          this->msg_target_->vector.x += *(iter_x+off);
          this->msg_target_->vector.y += *(iter_y+off);
          this->msg_target_->vector.z += *(iter_z+off);
          cnt++;
        }
      }
    }

    // iter_x += loc; iter_y += loc; iter_z += loc;

    // this->msg_target_->vector.x = *iter_x;
    // this->msg_target_->vector.y = *iter_y;
    // this->msg_target_->vector.z = *iter_z;

    this->msg_target_->vector.x /= cnt;
    this->msg_target_->vector.y /= cnt;
    this->msg_target_->vector.z /= cnt;

    this->target_found_ = true;
  }
}

void DroneVisionROS::declareRosParameters() {

  // ros node params
  this->declare_parameter("ros_node.subs.camera_color", rclcpp::PARAMETER_STRING);
  this->declare_parameter("ros_node.subs.camera_depth", rclcpp::PARAMETER_STRING);
  this->declare_parameter("ros_node.subs.camera_cloud", rclcpp::PARAMETER_STRING);

  this->declare_parameter("ros_node.pubs.vision_target", rclcpp::PARAMETER_STRING);
  this->declare_parameter("ros_node.pubs.vision_cloud", rclcpp::PARAMETER_STRING);

  this->declare_parameter("ros_node.thread_freq", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("ros_node.im_width", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("ros_node.im_height", rclcpp::PARAMETER_INTEGER);

  this->declare_parameter("ros_node.use_ext_camera", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("ros_node.mean_circ_radi", rclcpp::PARAMETER_INTEGER);

  // camera wrapper
  // ...

  // yolo wrapper
  this->declare_parameter("yolo_wrapper.yolo_cls_idx", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("yolo_wrapper.yolo_min_conf", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("yolo_wrapper.yolo_model_path", rclcpp::PARAMETER_STRING);
  this->declare_parameter("yolo_wrapper.yolo_labels_path", rclcpp::PARAMETER_STRING);
}

void DroneVisionROS::initializeRosNodeConfig() {

  // subs
  this->config_.sub_camera_color = this->get_parameter("ros_node.subs.camera_color").as_string();
  this->config_.sub_camera_depth = this->get_parameter("ros_node.subs.camera_depth").as_string();
  this->config_.sub_camera_cloud = this->get_parameter("ros_node.subs.camera_cloud").as_string();

  // pubs
  this->config_.pub_vision_target = this->get_parameter("ros_node.pubs.vision_target").as_string();
  this->config_.pub_vision_cloud = this->get_parameter("ros_node.pubs.vision_cloud").as_string();

  // other params
  this->config_.thread_hz = (float)(this->get_parameter("ros_node.thread_freq").as_double());
  this->config_.im_width  = (unsigned)(this->get_parameter("ros_node.im_width").as_int());
  this->config_.im_height = (unsigned)(this->get_parameter("ros_node.im_height").as_int());

  this->config_.use_ext_camera = this->get_parameter("ros_node.use_ext_camera").as_bool();
  this->config_.mean_circ_radi = this->get_parameter("ros_node.mean_circ_radi").as_int();
}

void DroneVisionROS::initializeComponents() {

  // camera wrapper
  CameraWrapperConfig camera_wrapper_config = {};
  camera_wrapper_config.im_width = (unsigned)(this->config_.im_width);
  camera_wrapper_config.im_height = (unsigned)(this->config_.im_height);

  // yolo wrapper
  YOLOWrapperConfig yolo_wrapper_config = {};
  yolo_wrapper_config.yolo_in_width = (unsigned)(this->config_.im_width);
  yolo_wrapper_config.yolo_in_height = (unsigned)(this->config_.im_height);
  yolo_wrapper_config.yolo_class = this->get_parameter("yolo_wrapper.yolo_cls_idx").as_int();
  yolo_wrapper_config.yolo_min_conf = (float)(this->get_parameter("yolo_wrapper.yolo_min_conf").as_double());
  yolo_wrapper_config.model_path  = this->get_parameter("yolo_wrapper.yolo_model_path").as_string();
  yolo_wrapper_config.labels_path = this->get_parameter("yolo_wrapper.yolo_labels_path").as_string();

  // initialize components
  this->camera_wrapper_ = std::make_unique<CameraWrapper>(camera_wrapper_config);
  this->yolo_wrapper_   = std::make_unique<YOLOWrapper>(yolo_wrapper_config);
}

void DroneVisionROS::initializeSubscribers() {

  // subscribers (use when camera data are send from external module/node, e.g. gazebo)
  if (this->config_.use_ext_camera) {
    this->sub_camera_color_ = this->create_subscription<sensor_msgs::msg::Image>(
      this->config_.sub_camera_color, 1, std::bind(&DroneVisionROS::cameraColorCallback, this, _1));
    this->sub_camera_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
      this->config_.sub_camera_depth, 1, std::bind(&DroneVisionROS::cameraDepthCallback, this, _1));
    this->sub_camera_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      this->config_.sub_camera_cloud, 1, std::bind(&DroneVisionROS::pointCloudCallback, this, _1));
  }

  // subscription msg cache
  this->im_color_cache_ = std::make_shared<sensor_msgs::msg::Image>();
  this->im_depth_cache_ = std::make_shared<sensor_msgs::msg::Image>();
  this->pt_cloud_cache_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
}

void DroneVisionROS::initializePublishers() {

  // publishers
  this->pub_vision_target_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(this->config_.pub_vision_target, 1);
  this->pub_vision_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->config_.pub_vision_cloud, 1);

  // published msg cache
  this->msg_target_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
  this->msg_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

  // fill in point cloud general info
  if (!this->config_.use_ext_camera) {
    this->msg_cloud_->width = this->config_.im_width;
    this->msg_cloud_->height = this->config_.im_height;
    
    this->msg_cloud_->point_step = 16; // 16 bytes (4x float32)
    this->msg_cloud_->row_step = this->config_.im_width * 16;

    this->msg_cloud_->is_bigendian = false;
    this->msg_cloud_->is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(*(this->msg_cloud_));
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                     "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                     "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                     "distance", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(this->msg_cloud_->width * this->msg_cloud_->height);
    this->msg_cloud_->data.clear();
  }
}

void DroneVisionROS::initializeExecutionThread() {
  this->execute_rate_   = std::make_unique<rclcpp::Rate>(this->config_.thread_hz);
  this->execute_worker_ = std::thread{&DroneVisionROS::executeThread, this};
}

} // namespace DRONE_NAVIGATION
