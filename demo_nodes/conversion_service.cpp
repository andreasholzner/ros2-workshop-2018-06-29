/**
 * Copyright 2018 TNG Technology Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "leap_msgs/msg/leap_data.hpp"
#include "leap_srvs/srv/hand_to_pose.hpp"

class ConversionServiceNode : public rclcpp::Node {
public:
  ConversionServiceNode(std::string service_name) : Node("hands_to_poses_service")
  {
    service_ = create_service<leap_srvs::srv::HandToPose>(service_name,
    [this](std::shared_ptr<rmw_request_id_t> request_header,
      leap_srvs::srv::HandToPose::Request::SharedPtr request,
      leap_srvs::srv::HandToPose::Response::SharedPtr response) -> void
    { convert(request_header, request, response); });
  }

private:
  void convert(
    std::shared_ptr<rmw_request_id_t> request_header,
    leap_srvs::srv::HandToPose::Request::SharedPtr request,
    leap_srvs::srv::HandToPose::Response::SharedPtr response)
  {
    (void)request_header;
    for (const auto & hand : request->hands) {
      auto pose = geometry_msgs::msg::Pose();
      pose.position = hand.position;
      pose.orientation = hand.orientation;
      response->poses.poses.push_back(pose);

      for (const auto & finger : hand.fingers) {
        auto finger_pose = geometry_msgs::msg::Pose();
        finger_pose.position = finger.tip_position;
        finger_pose.orientation = finger.tip_orientation;
        response->poses.poses.push_back(finger_pose);
      }
    }
  }
  rclcpp::Service<leap_srvs::srv::HandToPose>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  std::string service_name = "hands2pose_service";
  if (rcutils_cli_option_exist(argv, argv + argc, "-n")) {
    service_name = std::string(rcutils_cli_get_option(argv, argv + argc, "-n"));
  }

  rclcpp::init(0, nullptr);
  auto node = std::make_shared<ConversionServiceNode>(service_name);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
