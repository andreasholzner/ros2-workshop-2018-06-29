/**
 * Copyright 2018 TNG Technology Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "hand_publisher_node.hpp"

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
  rclcpp::spin(std::make_shared<HandPublisherNode>(topic, frame));
  rclcpp::shutdown();
  return 0;
}
