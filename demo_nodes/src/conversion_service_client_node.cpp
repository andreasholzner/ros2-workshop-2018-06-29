/**
 * Copyright 2018 TNG Technology Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "conversion_service_client_node.hpp"

#include <chrono>
#include <future>
#include <iostream>

using namespace std::chrono_literals;

ConversionServiceClientNode::ConversionServiceClientNode(
  std::string service_name, std::string input_topic, std::string output_topic)
: Node("conversion_service_client")
{
  converter_client_ = create_client<leap_srvs::srv::HandToPose>(service_name);
  publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(output_topic);
  subscription_ = create_subscription<leap_msgs::msg::LeapData>(
    input_topic,
    [this](leap_msgs::msg::LeapData::UniquePtr msg) -> void { onHandMessage(std::move(msg)); });

  while (!converter_client_->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      std::cerr << "Service '" << service_name << "' was not found!" << std::endl;
      exit(1);
    }
    std::cout << "waiting for service '" << service_name << "' ..." << std::endl;
  }
}

void ConversionServiceClientNode::onHandMessage(leap_msgs::msg::LeapData::UniquePtr msg) {
  auto request = std::make_shared<leap_srvs::srv::HandToPose::Request>();
  request->hands = msg->hands;
  auto frame_id = msg->header.frame_id;
  auto result_future = converter_client_->async_send_request(request,
    [frame_id, this](ServiceResponse future){
      onServiceResponse(future, frame_id);
    });
}

void ConversionServiceClientNode::onServiceResponse(ServiceResponse future, std::string frame_id)
{
  auto result = future.get();
  auto message = geometry_msgs::msg::PoseArray();
  message.header.frame_id = frame_id;
  message.header.stamp = rclcpp::Clock().now();
  message.poses = result->poses.poses;
  publisher_->publish(message);
}
