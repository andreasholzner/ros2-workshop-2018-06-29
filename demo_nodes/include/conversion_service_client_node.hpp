/**
 * Copyright 2018 TNG Technology Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DEMO_NODES__CONVERSION_SERVICE_CLIENT_NODE_HPP_
#define DEMO_NODES__CONVERSION_SERVICE_CLIENT_NODE_HPP_

#include <chrono>
#include <future>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "leap_msgs/msg/leap_data.hpp"
#include "leap_srvs/srv/hand_to_pose.hpp"

using ServiceResponse = rclcpp::Client<leap_srvs::srv::HandToPose>::SharedFuture;

class ConversionServiceClientNode : public rclcpp::Node {
  using ServiceResponse = rclcpp::Client<leap_srvs::srv::HandToPose>::SharedFuture;

public:
  ConversionServiceClientNode(
    std::string service_name, std::string input_topic, std::string output_topic);
private:
  void onHandMessage(leap_msgs::msg::LeapData::UniquePtr msg);
  void onServiceResponse(ServiceResponse future, std::string frame_id);

  rclcpp::Client<leap_srvs::srv::HandToPose>::SharedPtr converter_client_;
  rclcpp::Subscription<leap_msgs::msg::LeapData>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
};

#endif /* DEMO_NODES__CONVERSION_SERVICE_CLIENT_NODE_HPP_ */