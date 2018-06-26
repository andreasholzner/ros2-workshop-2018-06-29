/**
 * Copyright 2018 TNG Technology Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "hand_publisher_node.hpp"
#include "conversion_service_client_node.hpp"
#include "conversion_service_node.hpp"

int main(int argc, char **argv)
{
  std::string frame = "leap_frame";
  if (rcutils_cli_option_exist(argv, argv + argc, "-f"))
  {
    frame = std::string(rcutils_cli_get_option(argv, argv + argc, "-f"));
  }
  std::string hand_topic = "leap_data";
  std::string pose_topic = "hand_poses";
  std::string service_name = "conversion_service";

  rclcpp::init(0, nullptr);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto hand_publisher = std::make_shared<HandPublisherNode>(hand_topic, frame);
  auto conversion_service = std::make_shared<ConversionServiceNode>(service_name);
  auto conversion_service_client =
    std::make_shared<ConversionServiceClientNode>(service_name, hand_topic, pose_topic);
  exec.add_node(conversion_service);
  exec.add_node(conversion_service_client);
  exec.add_node(hand_publisher);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
