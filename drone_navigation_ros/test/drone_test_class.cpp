#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "drone_navigation_msgs/msg/control_vector.hpp"

#include "socketUDP.cpp"


using std::placeholders::_1;

namespace DRONE_NAVIGATION {

struct servo_packet_16 {
  uint16_t magic {18458};
  uint16_t frame_rate {0};
  uint32_t frame_count {0};
  uint16_t pwm[16] = {0};
};


class DroneTest : public rclcpp::Node {

public:

  DroneTest(rclcpp::NodeOptions options) : Node("drone_test_node", options) {

    this->sub_ = this->create_subscription<drone_navigation_msgs::msg::ControlVector>("/test/control", 1, std::bind(&DroneTest::testControl, this, _1));

    this->execute_rate_   = std::make_unique<rclcpp::Rate>((double)(this->frame_rate_));
    this->execute_worker_ = std::thread{&DroneTest::executeThread, this};

    this->socket_ = std::make_unique<SocketUDP>(true, true);
    this->socket_->bind(this->address_, this->port_);
  }
  ~DroneTest() = default;

protected:

  rclcpp::Subscription<drone_navigation_msgs::msg::ControlVector>::SharedPtr sub_;

  std::unique_ptr<SocketUDP> socket_;

  std::unique_ptr<rclcpp::Rate> execute_rate_;
  std::thread execute_worker_;

  std::vector<uint16_t> ctrl_ {1100, 1100, 1100, 1100}; // fr: 1563.0, fl: 1566.0, rr: 1560.0, rl: 1562.0

  char address_[10] {"127.0.0.1"};
  uint16_t port_ {9002};

  uint16_t magic_number_ {18458};
  uint16_t frame_rate_ {10};
  uint32_t frame_count_ {0};

  void testControl(const drone_navigation_msgs::msg::ControlVector::ConstSharedPtr& msg) {
    this->ctrl_[0] = (uint16_t)(msg->rl); // rear-left
    this->ctrl_[1] = (uint16_t)(msg->fr); // front-right
    this->ctrl_[2] = (uint16_t)(msg->rr); // rear-right
    this->ctrl_[3] = (uint16_t)(msg->fl); // front-left
    RCUTILS_LOG_INFO("[INFO] Received cmd: {fl: %d, fr: %d, rl: %d, rr: %d}", this->ctrl_[3], this->ctrl_[1], this->ctrl_[0], this->ctrl_[2]);
  }

  void executeThread() {
    while (rclcpp::ok()) {
      
      servo_packet_16 pkt;

      pkt.pwm[0] = this->ctrl_[0];
      pkt.pwm[1] = this->ctrl_[1];
      pkt.pwm[2] = this->ctrl_[2];
      pkt.pwm[3] = this->ctrl_[3];

      pkt.magic = this->magic_number_;
      pkt.frame_rate  = this->frame_rate_;
      pkt.frame_count = this->frame_count_;

      this->socket_->sendto(&pkt, sizeof(pkt), this->address_, this->port_);

      this->frame_count_++;

      if (this->execute_rate_) execute_rate_->sleep();
    }
  }

};

} // namespace DRONE_NAVIGATION
