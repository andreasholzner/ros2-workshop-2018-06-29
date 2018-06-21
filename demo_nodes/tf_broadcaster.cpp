/**
 * Copyright 2018 TNG Technology Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cmath>
#include <iostream>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "tf2_ros/transform_broadcaster.h"

#include "leap_msgs/msg/leap_data.hpp"

using namespace std::chrono_literals;

class TFBroadcaster {
public:
  TFBroadcaster(rclcpp::Node::SharedPtr node, std::string topic, std::string frame) 
  : node_(node), topic_(topic), frame_(frame)
  {
    
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    subscription_ = node_->create_subscription<leap_msgs::msg::LeapData>(
      topic_,
      [this](leap_msgs::msg::LeapData::UniquePtr msg) -> void { onHandMessage(std::move(msg)); });
  }

private:
  void onHandMessage(leap_msgs::msg::LeapData::UniquePtr msg)
  {
    int id = 0;
    for (const auto &hand : msg->hands) {
      auto hand_transform = geometry_msgs::msg::TransformStamped();
      hand_transform.header.stamp = rclcpp::Clock().now();
      hand_transform.header.frame_id = frame_;
      hand_transform.child_frame_id = "hands/" + std::to_string(++id);

      hand_transform.transform.translation.x = hand.position.x;
      hand_transform.transform.translation.y = hand.position.y;
      hand_transform.transform.translation.z = hand.position.z;
      hand_transform.transform.rotation.w = hand.orientation.w;
      hand_transform.transform.rotation.x = hand.orientation.x;
      hand_transform.transform.rotation.y = hand.orientation.y;
      hand_transform.transform.rotation.z = hand.orientation.z;

      tf_broadcaster_->sendTransform(hand_transform);
    }
    std::cout << "Published hand transforms" << std::endl;
  }

  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  std::string frame_;
  rclcpp::Subscription<leap_msgs::msg::LeapData>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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
  auto node = std::make_shared<rclcpp::Node>("tf_broadcaster");
  auto broadcaster = std::make_shared<TFBroadcaster>(node, topic, frame);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}