/**
 * Copyright 2018 TNG Technology Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <chrono>
#include <future>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "leap_msgs/msg/leap_data.hpp"
#include "leap_srvs/srv/hand_to_pose.hpp"

using namespace std::chrono_literals;

using ServiceResponse = rclcpp::Client<leap_srvs::srv::HandToPose>::SharedFuture;

class ConversionServiceClientNode : public rclcpp::Node {
public:
  ConversionServiceClientNode(
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

private:
  void onHandMessage(leap_msgs::msg::LeapData::UniquePtr msg) {
    auto request = std::make_shared<leap_srvs::srv::HandToPose::Request>();
    request->hands = msg->hands;
    auto frame_id = msg->header.frame_id;
    auto result_future = converter_client_->async_send_request(request,
      [this](ServiceResponse future){
        onServiceResponse(future);
      });
  }

  void onServiceResponse(ServiceResponse future)
  {
    auto result = future.get();
    auto message = geometry_msgs::msg::PoseArray();
    message.header.frame_id = result->poses.header.frame_id;
    message.header.stamp = rclcpp::Clock().now();
    message.poses = result->poses.poses;
    publisher_->publish(message);
  }

  rclcpp::Client<leap_srvs::srv::HandToPose>::SharedPtr converter_client_;
  rclcpp::Subscription<leap_msgs::msg::LeapData>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;

};

int main(int argc, char ** argv)
{
  std::string service_name = "hands2pose_service";
  if (rcutils_cli_option_exist(argv, argv + argc, "-n")) {
    service_name = std::string(rcutils_cli_get_option(argv, argv + argc, "-n"));
  }
  std::string input_topic = "leap_data";
  if (rcutils_cli_option_exist(argv, argv + argc, "-i")) {
    input_topic = std::string(rcutils_cli_get_option(argv, argv + argc, "-i"));
  }
  std::string output_topic = "hand_poses";
  if (rcutils_cli_option_exist(argv, argv + argc, "-o")) {
    output_topic = std::string(rcutils_cli_get_option(argv, argv + argc, "-o"));
  }

  rclcpp::init(0, nullptr);
  rclcpp::spin(
    std::make_shared<ConversionServiceClientNode>(service_name, input_topic, output_topic));
  rclcpp::shutdown();
  return 0;
}
