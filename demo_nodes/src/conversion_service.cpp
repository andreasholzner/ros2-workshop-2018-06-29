/**
 * Copyright 2018 TNG Technology Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "conversion_service_node.hpp"


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
