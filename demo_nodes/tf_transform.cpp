/**
 * Copyright 2018 TNG Technology Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <chrono>
#include <cmath>
#include <iostream>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "leap_msgs/msg/leap_data.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace tf2
{
template<>
inline
void doTransform(
  const tf2::Stamped<leap_msgs::msg::Hand> & t_in,
  tf2::Stamped<leap_msgs::msg::Hand> & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  t_out = t_in;
  t_out.stamp_ = tf2_ros::fromMsg(transform.header.stamp);
  t_out.frame_id_ = transform.header.frame_id;

  geometry_msgs::msg::PointStamped position;
  position.header.stamp = tf2_ros::toMsg(t_in.stamp_);
  position.header.frame_id = t_in.frame_id_;
  geometry_msgs::msg::PointStamped transformed_position;

  tf2::doTransform(position, transformed_position, transform);

  t_out.position = transformed_position.point;
}
}  // namespace tf2

using namespace std::chrono_literals;

class HandTransformer : public rclcpp::Node
{
public:
  HandTransformer(std::string topic)
    : Node("hand_transformer")
  {
    subscription_ = create_subscription<leap_msgs::msg::LeapData>(
      topic,
      [this](leap_msgs::msg::LeapData::UniquePtr msg) -> void { onHandMessage(std::move(msg)); });

    buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  }

private:
  void onHandMessage(leap_msgs::msg::LeapData::UniquePtr msg)
  {
    if (msg->hands.size() != 2) {
      std::cerr << "This node requires two hands to work" << std::endl;
      return;
    }
    auto left_hand = msg->hands[0];
    auto right_hand = msg->hands[1];
    try {
      if(!buffer_->canTransform("hands/2", "hands/1", tf2::TimePointZero)) {
        std::cerr << "Transform between hands does not exist" << std::endl;
        return;
      }

      std::cout << "Old position of right hand: ("
                << right_hand.position.x << ", "
                << right_hand.position.y << ", "
                << right_hand.position.z << ")"
                << std::endl;

      auto transformed_hand = buffer_->transform(
        tf2::Stamped<leap_msgs::msg::Hand>(right_hand, tf2::TimePointZero, "hands/2"), "hands/1");

      std::cout << "Transformed position of right hand: ("
                << transformed_hand.position.x << ", "
                << transformed_hand.position.y << ", "
                << transformed_hand.position.z << ")"
                << std::endl;

    } catch (tf2::LookupException & e) {
      std::cerr << "No hand transforms found..." << std::endl;
    }
  }

  rclcpp::Subscription<leap_msgs::msg::LeapData>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  std::string topic = "leap_data";
  if (rcutils_cli_option_exist(argv, argv + argc, "-t")) {
    topic = std::string(rcutils_cli_get_option(argv, argv + argc, "-t"));
  }

  rclcpp::init(0, nullptr);
  rclcpp::spin(std::make_shared<HandTransformer>(topic));
  rclcpp::shutdown();
  return 0;
}
