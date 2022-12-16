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
    this->declare_parameter<double>("medium_speed", 1.3);
    this->declare_parameter<double>("faster_speed", 1.8);
    this->declare_parameter<double>("scan_switch_delay", 0.2);

    // get parameters
    this->get_parameter("slow_speed", slow_speed_);
    this->get_parameter("medium_speed", medium_speed_);
    this->get_parameter("faster_speed", faster_speed_);
    this->get_parameter("scan_switch_delay", scan_switch_delay_);

    // seperate client node for spin safety
    client_node_ = std::make_shared<rclcpp::Node>("set_relays_client");

    // Subscribers, clients and timers
    odom_sub_ =
      this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 1,
      std::bind(&ConfigureRelays::odomCallback, this, _1));

    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    set_relays_client_ =
      client_node_->create_client<neo_srvs2::srv::RelayBoardSetRelay>("set_relay3");

    set_relays2_client_ =
      this->create_client<neo_srvs2::srv::RelayBoardSetRelay>("set_relay");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&ConfigureRelays::helper_thread, this),
      timer_cb_group_);

    relay2 = std::make_shared<neo_srvs2::srv::RelayBoardSetRelay::Request>();
    relay2->id = 2;

    relay3 = std::make_shared<neo_srvs2::srv::RelayBoardSetRelay::Request>();
    relay3->id = 3;
  }

private:
  inline double convert_to_double(builtin_interfaces::msg::Time stamp)
  {
    return stamp.sec + stamp.nanosec / 1e9;
  }
  // Thread where relays are set
  void helper_thread()
  {
    // 4 possible cases
    if (odom_.twist.twist.linear.x <= slow_speed_) {
      relay3->state = false;
    } else if (odom_.twist.twist.linear.x > slow_speed_ &&
      odom_.twist.twist.linear.x <= medium_speed_)
    {
      relay3->state = true;
    } else if (odom_.twist.twist.linear.x > medium_speed_ &&
      odom_.twist.twist.linear.x <= faster_speed_)
    {
      relay3->state = false;
    } else if (odom_.twist.twist.linear.x > faster_speed_) {
      relay3->state = true;
    }

    if (slow_down_) {
      if (temp_relay3_state != relay3->state &&
        !time_stored_relay_3_)
      {
        temp_odom_relay_3_ = odom_;
        time_stored_relay_3_ = true;
      }

      if (!time_stored_relay_3_) {
        temp_odom_relay_3_ = odom_;
      }

      if (convert_to_double(odom_.header.stamp) -
        convert_to_double(temp_odom_relay_3_.header.stamp) >= scan_switch_delay_)
      {
        auto relay3_result = set_relays_client_->async_send_request(relay3);
        temp_odom_relay_3_ = odom_;
        time_stored_relay_3_ = false;
      }
    } else {
      if (!time_stored_relay_3_) {
        auto relay3_result = set_relays_client_->async_send_request(relay3);
      }
    }
    temp_relay3_state = relay3->state;
    last_odom_ = odom_;
  }

  // Odom callback
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    odom_ = *odom;

    // 4 possible cases
    if (odom_.twist.twist.linear.x <= slow_speed_) {
      relay2->state = false;
    } else if (odom_.twist.twist.linear.x > slow_speed_ &&
      odom_.twist.twist.linear.x <= medium_speed_)
    {
      relay2->state = false;
    } else if (odom_.twist.twist.linear.x > medium_speed_ &&
      odom_.twist.twist.linear.x <= faster_speed_)
    {
      relay2->state = true;
    } else if (odom_.twist.twist.linear.x > faster_speed_) {
      relay2->state = true;
    }

    if (last_odom_.twist.twist.linear.x > odom_.twist.twist.linear.x) {
      slow_down_ = true;
      if (temp_relay2_state != relay2->state && !time_stored_relay_2_) {
        temp_odom_relay_2_ = odom_;
        time_stored_relay_2_ = true;
      }
    } else {
      slow_down_ = false;
    }

    if (!time_stored_relay_2_) {
      temp_odom_relay_2_ = odom_;
    }

    if (slow_down_) {
      if (convert_to_double(odom_.header.stamp) -
        convert_to_double(temp_odom_relay_2_.header.stamp) >= scan_switch_delay_)
      {
        auto relay2_result = set_relays2_client_->async_send_request(relay2);
        temp_odom_relay_2_ = odom_;
        time_stored_relay_2_ = false;
        return;
      }
    } else {
      if (!time_stored_relay_2_) {
        auto relay2_result = set_relays2_client_->async_send_request(relay2);
      }
    }
    temp_relay2_state = relay2->state;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Client<neo_srvs2::srv::RelayBoardSetRelay>::SharedPtr set_relays_client_;
  rclcpp::Client<neo_srvs2::srv::RelayBoardSetRelay>::SharedPtr set_relays2_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

  nav_msgs::msg::Odometry odom_;
  nav_msgs::msg::Odometry last_odom_;
  nav_msgs::msg::Odometry temp_odom_relay_2_;
  nav_msgs::msg::Odometry temp_odom_relay_3_;

  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<neo_srvs2::srv::RelayBoardSetRelay::Request> relay2;
  std::shared_ptr<neo_srvs2::srv::RelayBoardSetRelay::Request> relay3;

  double slow_speed_;
  double medium_speed_;
  double faster_speed_;
  double scan_switch_delay_;

  bool slow_down_ = false;
  bool time_stored_relay_2_ = false;
  bool time_stored_relay_3_ = false;

  bool temp_relay2_state = false;
  bool temp_relay3_state = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConfigureRelays>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // setting loop rate to 50 hz
  rclcpp::Rate loop_rate(50);

  while (rclcpp::ok()) {
    executor.spin_some();
    loop_rate.sleep();
  }

  return 0;
}
