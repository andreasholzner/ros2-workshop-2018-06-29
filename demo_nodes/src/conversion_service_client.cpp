/**
 * Copyright 2018 TNG Technology Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "conversion_service_client_node.hpp"

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
