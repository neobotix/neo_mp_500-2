/*********************************************************************
MIT License

Copyright (c) 2022 neobotix gmbh

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 *********************************************************************/
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include <neo_srvs2/srv/relay_board_set_relay.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

class ConfigureRelays : public rclcpp::Node
{
public:
  ConfigureRelays()
  : Node("configure_relays")
  {
    // declare parameters
    this->declare_parameter<double>("slow_speed", 1.0);
    this->declare_parameter<double>("medium_speed", 2.0);
    this->declare_parameter<double>("faster_speed", 2.7);

    // get parameters
    this->get_parameter("slow_speed", slow_speed_);
    this->get_parameter("medium_speed", medium_speed_);
    this->get_parameter("faster_speed", faster_speed_);

    // seperate client node for spin safety
    client_node_ = std::make_shared<rclcpp::Node>("set_relays_client");

    // Subscribers, clients and timers
    odom_sub_ =
      this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 1,
      std::bind(&ConfigureRelays::odomCallback, this, _1));

    set_relays_client_ =
      client_node_->create_client<neo_srvs2::srv::RelayBoardSetRelay>("set_relay");

    set_relays2_client_ =
      this->create_client<neo_srvs2::srv::RelayBoardSetRelay>("set_relay");

    relay2 = std::make_shared<neo_srvs2::srv::RelayBoardSetRelay::Request>();
    relay2->id = 2;

    relay3 = std::make_shared<neo_srvs2::srv::RelayBoardSetRelay::Request>();
    relay3->id = 3;
  }

private:
  // send request
  void send_request(
    std::shared_ptr<neo_srvs2::srv::RelayBoardSetRelay::Request> relay2,
    std::shared_ptr<neo_srvs2::srv::RelayBoardSetRelay::Request> relay3)
  {
    auto relay2_result = set_relays2_client_->async_send_request(relay2);
    auto relay3_result = set_relays_client_->async_send_request(relay3);
  }

  // Odom callback
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    odom_vel_ = odom->twist.twist;
    // 4 possible cases
    if (odom_vel_.linear.x <= slow_speed_) {
      relay2->state = false;
      relay3->state = false;
    } else if (odom_vel_.linear.x > slow_speed_ && odom_vel_.linear.x <= medium_speed_) {
      relay2->state = false;
      relay3->state = true;
    } else if (odom_vel_.linear.x > medium_speed_ && odom_vel_.linear.x <= faster_speed_) {
      relay2->state = true;
      relay3->state = false;
    } else if (odom_vel_.linear.x > faster_speed_) {
      relay2->state = true;
      relay3->state = true;
    }
    send_request(relay2, relay3);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Client<neo_srvs2::srv::RelayBoardSetRelay>::SharedPtr set_relays_client_;
  rclcpp::Client<neo_srvs2::srv::RelayBoardSetRelay>::SharedPtr set_relays2_client_;

  geometry_msgs::msg::Twist odom_vel_;

  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<neo_srvs2::srv::RelayBoardSetRelay::Request> relay2;
  std::shared_ptr<neo_srvs2::srv::RelayBoardSetRelay::Request> relay3;

  double slow_speed_;
  double medium_speed_;
  double faster_speed_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConfigureRelays>();

  // setting loop rate to 100 hz
  rclcpp::Rate loop_rate(100);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}
