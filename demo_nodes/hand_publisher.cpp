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
#include "leap_msgs/msg/leap_data.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class HandPublisher : public rclcpp::Node {
public:
  HandPublisher(std::string topic, std::string frame)
      : Node("hand_publisher"), frame_(frame), counter_(0), period_(20), time_lapse_(100ms) 
  {
    publisher_ = create_publisher<leap_msgs::msg::LeapData>(topic);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(topic + "_markers");

    timer_ = this->create_wall_timer(time_lapse_, [this]() -> void {
      publish_hands();
    });
  }

private:
  void publish_hands()
  {
    std::cout << "publishing hands..." << std::endl;
    auto message = create_msg((++counter_ % period_) * 1.0 / period_ * M_PI * 2);
    auto marker_message = create_markers_from_leap_data(message);
    publisher_->publish(message);
    marker_publisher_->publish(marker_message);
  }

  leap_msgs::msg::LeapData create_msg(double elongation)
  {
    auto message = leap_msgs::msg::LeapData();
    message.header = std_msgs::msg::Header();
    message.header.frame_id = frame_;
    message.header.stamp = rclcpp::Clock().now();

    leap_msgs::msg::Hand left, right;

    left.position.z = 0;
    right.position.z = 0;
    left.position.y = 0.02;
    right.position.y = 0.02;
    left.position.x = (std::sin(elongation)+1) / 4;
    right.position.x = (-std::sin(elongation)-1) / 4;

    left.orientation.w = 0;
    right.orientation.w = M_PI;
    left.orientation.x = 0;
    right.orientation.x = 0;
    left.orientation.y = 1;
    right.orientation.y = 1;
    left.orientation.z = 0;
    right.orientation.z = 0;

    message.hands.push_back(left);
    message.hands.push_back(right);
    return message;
  }

  visualization_msgs::msg::MarkerArray create_markers_from_leap_data(leap_msgs::msg::LeapData leap_frame)
  {
    auto message = visualization_msgs::msg::MarkerArray();

    int id = 0;
    for (const auto &hand : leap_frame.hands)
    {
      auto hand_marker = visualization_msgs::msg::Marker();
      hand_marker.header = std_msgs::msg::Header();
      hand_marker.header.frame_id = frame_;
      hand_marker.header.stamp = leap_frame.header.stamp;

      hand_marker.id = ++id;
      hand_marker.ns = "hand_markers";
      hand_marker.type = visualization_msgs::msg::Marker::SPHERE;
      hand_marker.action = visualization_msgs::msg::Marker::ADD;
      hand_marker.lifetime = rclcpp::Duration(1, 0);

      hand_marker.scale.x = 0.1;
      hand_marker.scale.y = 0.1;
      hand_marker.scale.z = 0.1;

      hand_marker.color.a = 1.0;
      hand_marker.color.r = 1.0;
      hand_marker.color.g = 0.0;
      hand_marker.color.b = 0.0;

      hand_marker.pose.position = hand.position;
      hand_marker.pose.orientation = hand.orientation;

      message.markers.emplace_back(hand_marker);
    }

    return message;
  }

  std::string frame_;
  size_t counter_;
  size_t period_;
  std::chrono::milliseconds time_lapse_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<leap_msgs::msg::LeapData>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

int main(int argc, char **argv)
{
  std::string topic = "leap_data";
  if (rcutils_cli_option_exist(argv, argv + argc, "-t"))
  {
    topic = std::string(rcutils_cli_get_option(argv, argv + argc, "-t"));
  }
  std::string frame = "leap_frame";
  if (rcutils_cli_option_exist(argv, argv + argc, "-f"))
  {
    frame = std::string(rcutils_cli_get_option(argv, argv + argc, "-f"));
  }

  rclcpp::init(0, nullptr);
  rclcpp::spin(std::make_shared<HandPublisher>(topic, frame));
  rclcpp::shutdown();
  return 0;
}
