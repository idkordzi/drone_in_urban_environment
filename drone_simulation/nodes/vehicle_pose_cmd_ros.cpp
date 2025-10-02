#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <array>
#include <chrono>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Quaternion.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"


using std::placeholders::_1;

namespace DRONE_SIMULATION {

struct PickupConfig {
  std::string sub_gazebo_poses    = "/debug/pickup/cmd_pose";
  std::string sub_pickup_pose_cmd = "/debug/pickup/cmd_pose";
  std::string pub_pickup_pose_cmd = "/model/pickup/cmd_pose";
  bool en_input = false;
  float thread_freq = 10.0f;

  int model_idx = 2;
  float path_pub_period = 0.1f;
  std::string path_file = "";
};

class PickupPoseCmd : public rclcpp::Node {

public:
  PickupPoseCmd(rclcpp::NodeOptions options) : Node("pickup_pose_cmd_node", options) {
    this->declareRosParameters();
    this->initializeRosNodeConfig();
    this->initializeSubscribers();
    this->initializePublishers();
    this->initializeExecutionThread();

    if (!this->config_.en_input)
      this->readPath();
  }

  ~PickupPoseCmd() {
    this->execute_worker_.join();
  }

private:
  void declareRosParameters() {
    this->declare_parameter("ros_node.subs.gazebo_poses", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.subs.pickup_pose_cmd", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.pubs.pickup_pose_cmd", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.en_input", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("ros_node.thread_freq", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("ros_node.path_pub_period", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("ros_node.model_idx", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("ros_node.path_file", rclcpp::PARAMETER_STRING);
  }

  void initializeRosNodeConfig() {
    this->config_.sub_gazebo_poses    = this->get_parameter("ros_node.subs.gazebo_poses").as_string();
    this->config_.sub_pickup_pose_cmd = this->get_parameter("ros_node.subs.pickup_pose_cmd").as_string();
    this->config_.pub_pickup_pose_cmd = this->get_parameter("ros_node.pubs.pickup_pose_cmd").as_string();
    this->config_.en_input            = this->get_parameter("ros_node.en_input").as_bool();
    this->config_.thread_freq         = (float)this->get_parameter("ros_node.thread_freq").as_double();
    this->config_.path_pub_period     = (float)this->get_parameter("ros_node.path_pub_period").as_double();
    this->config_.model_idx           = this->get_parameter("ros_node.model_idx").as_int();
    this->config_.path_file           = this->get_parameter("ros_node.path_file").as_string();
  }

  void initializeSubscribers() {
    if (this->config_.en_input) {
      this->sub_pose_cmd_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->config_.sub_pickup_pose_cmd, 1, std::bind(&PickupPoseCmd::poseCallback, this, _1));

      this->pose_cache_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
    }

    this->sub_gazebo_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      this->config_.sub_gazebo_poses, 1, std::bind(&PickupPoseCmd::gazeboCallback, this, _1));

    this->model_cache_ = std::make_shared<geometry_msgs::msg::Pose>();
  }

  void initializePublishers() {
    this->pub_pose_cmd_ = this->create_publisher<geometry_msgs::msg::PoseArray>(this->config_.pub_pickup_pose_cmd, 1);
    this->pose_msg_ = std::make_shared<geometry_msgs::msg::PoseArray>();

    geometry_msgs::msg::Pose pose = geometry_msgs::msg::Pose();
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.x = 1.0;
    this->pose_msg_->poses.clear();
    this->pose_msg_->poses.push_back(pose);
  }

  void initializeExecutionThread() {
    this->execute_rate_   = std::make_unique<rclcpp::Rate>(this->config_.thread_freq);
    this->execute_worker_ = std::thread(&PickupPoseCmd::executeThread, this);
  }

  void readPath() {
    std::ifstream file(this->config_.path_file);
    if (!file)
      RCUTILS_LOG_ERROR("[ERROR] Could not open fiel from \"%s\"", this->config_.path_file.c_str());
    
    std::string fline;
    std::getline(file, fline);
    int plen = std::stoi(fline);
    this->path_.reserve(plen);

    std::string subs;
    char del = ' ';
    while(std::getline(file, fline)) {
      std::array<float, 6> ppoint;
      std::stringstream ss(fline);
      for (int i = 0; i < 6; i++) {
        std::getline(ss, subs, del);
        ppoint[i] = std::stof(subs);
      }
      this->path_.push_back(ppoint);
    }

    file.close();

    // std::array<float, 6> ppoint = {};
    // ppoint[0] = 1.0f;
    // this->path_.push_back(ppoint);

    geometry_msgs::msg::Pose n_pose = this->getPoint(0);
    this->pose_msg_->poses.clear();
    this->pose_msg_->poses.push_back(n_pose);

    this->ts_ = std::chrono::system_clock::now();
  }

  void gazeboCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr &msg) {
    this->model_cache_->position    = msg->poses[this->config_.model_idx].position;
    this->model_cache_->orientation = msg->poses[this->config_.model_idx].orientation;
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
    if (!this->config_.en_input) return;
    std::lock_guard<std::mutex> lg(this->mtx_pose_);
    this->pose_cache_->header = msg->header;
    this->pose_cache_->pose   = msg->pose;
  }

  void publish() {
    this->pose_msg_->header.stamp = this->get_clock()->now();
    this->pose_msg_->header.frame_id = "";
    this->pub_pose_cmd_->publish(*this->pose_msg_);
  }

  void executeThread() {
    while (rclcpp::ok()) {
      if (this->config_.en_input)
        this->getInputCmd();
      else
        this->getPointFromPath();
      this->publish();
      if (this->execute_rate_) execute_rate_->sleep();
    }
  }

  void getInputCmd() {
    if (!this->config_.en_input) return;
    std::lock_guard<std::mutex> lg(this->mtx_pose_);
    this->pose_msg_->poses.clear();
    this->pose_msg_->poses.push_back(this->pose_cache_->pose);
  }

  geometry_msgs::msg::Pose getPoint(int idx) {
    geometry_msgs::msg::Pose n_pose = geometry_msgs::msg::Pose();

    n_pose.position.x = this->path_[idx][0];
    n_pose.position.y = this->path_[idx][1];
    n_pose.position.z = this->path_[idx][2];

    tf2::Quaternion q;
    q.setRPY(this->path_[idx][3], this->path_[idx][4], this->path_[idx][5]);

    n_pose.orientation.x = q.x();
    n_pose.orientation.y = q.y();
    n_pose.orientation.z = q.z();
    n_pose.orientation.w = q.w();

    return n_pose;
  }

  void getPointFromPath() {
    if (this->config_.en_input) return;
    static long unsigned pidx = 1;

    std::chrono::duration<double> time_passed = std::chrono::system_clock::now() - this->ts_;
    double elapsed = time_passed.count();

    if (elapsed < 60.0) return; // 60.0

    if (pidx < this->path_.size()) {
      float xc = this->pose_msg_->poses[0].position.x;
      float yc = this->pose_msg_->poses[0].position.y;
      float zc = this->pose_msg_->poses[0].position.z;

      float xp = this->model_cache_->position.x;
      float yp = this->model_cache_->position.y;
      float zp = this->model_cache_->position.z;

      auto sqr = [](float a) -> float {return (a)*(a);};
      if (std::sqrt(sqr(xc-xp) + sqr(yc-yp) + sqr(zc-zp)) < 1.0) {
        geometry_msgs::msg::Pose n_pose = this->getPoint(pidx);
        this->pose_msg_->poses.clear();
        this->pose_msg_->poses.push_back(n_pose);
        pidx++;
      }
    }
  }

  PickupConfig config_ = {};

  geometry_msgs::msg::Pose::SharedPtr model_cache_;
  geometry_msgs::msg::PoseStamped::SharedPtr pose_cache_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_gazebo_poses_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_cmd_;

  geometry_msgs::msg::PoseArray::SharedPtr pose_msg_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pose_cmd_;

  std::unique_ptr<rclcpp::Rate> execute_rate_;
  std::thread execute_worker_;

  std::mutex mtx_pose_ = {};

  std::vector<std::array<float, 6>> path_;
  std::chrono::system_clock::time_point ts_;
};

} // namespace DRONE_SIMULATION
