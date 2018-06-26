/**
 * Copyright 2018 TNG Technology Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DEMO_NODES__HAND_PUBLISHER_NODE_HPP_
#define DEMO_NODES__HAND_PUBLISHER_NODE_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "leap_msgs/msg/leap_data.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


class HandPublisherNode : public rclcpp::Node {
public:
  HandPublisherNode(std::string topic, std::string frame);

private:
  void publish_hands();
  leap_msgs::msg::LeapData create_msg(double elongation);
  visualization_msgs::msg::MarkerArray
  create_markers_from_leap_data(leap_msgs::msg::LeapData leap_frame);

  std::string frame_;
  size_t counter_;
  size_t period_;
  std::chrono::milliseconds time_lapse_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<leap_msgs::msg::LeapData>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

#endif /* DEMO_NODES__HAND_PUBLISHER_NODE_HPP_ */