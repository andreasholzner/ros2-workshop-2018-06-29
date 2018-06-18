/**
 * Copyright 2018 TNG Technology Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <chrono>
#include <cmath>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "leap_msgs/msg/hand_array.hpp"

using namespace std::chrono_literals;

class HandPublisher : public rclcpp::Node {
public:
  HandPublisher(std::string topic, std::string frame)
  : Node("hand_publisher"), frame_(frame), counter_(0), period_(50), time_lapse_(100ms)
  {
    publisher_ = create_publisher<leap_msgs::msg::HandArray>(topic);

    timer_ = this->create_wall_timer(time_lapse_, [this]() -> void {
      publish_hands();
    });
  }

private:
  void publish_hands()
  {
    std::cout << "publishing hands..." << std::endl;
    auto message = create_msg((++counter_ % period_) * 0.005);
    publisher_->publish(message);
  }

  leap_msgs::msg::HandArray create_msg(double elongation)
  {
    auto message = leap_msgs::msg::HandArray();
    message.header = std_msgs::msg::Header();
    message.header.frame_id = frame_;
    message.header.stamp = rclcpp::Clock().now();

    leap_msgs::msg::Hand left, right;

    left.position.z = 0; right.position.z = 0;
    left.position.y = 0.02; right.position.y = 0.02;
    left.position.x =  +elongation; right.position.x = -elongation;

    left.orientation.w = 0; right.orientation.w = M_PI;
    left.orientation.x = 0; right.orientation.x = 0;
    left.orientation.y = 1; right.orientation.y = 1;
    left.orientation.z = 0; right.orientation.z = 0;

    message.hands.push_back(left);
    message.hands.push_back(right);
    return message;
  }

  std::string frame_;
  size_t counter_;
  size_t period_;
  std::chrono::milliseconds time_lapse_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<leap_msgs::msg::HandArray>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  std::string topic = "hands";
  if (rcutils_cli_option_exist(argv, argv + argc, "-t")) {
    topic = std::string(rcutils_cli_get_option(argv, argv + argc, "-t"));
  }
  std::string frame = "leap_frame";
  if (rcutils_cli_option_exist(argv, argv + argc, "-f")) {
    frame = std::string(rcutils_cli_get_option(argv, argv + argc, "-f"));
  }

  rclcpp::init(0, nullptr);
  rclcpp::spin(std::make_shared<HandPublisher>(topic, frame));
  rclcpp::shutdown();
  return 0;
}
