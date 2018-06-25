/**
 * Copyright 2018 TNG Technology Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <chrono>
#include <iostream>
#include <thread>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;
using namespace std::this_thread;

int main(int argc, char **argv)
{
  std::string frame = "leap_frame";
  if (rcutils_cli_option_exist(argv, argv + argc, "-f"))
  {
    frame = std::string(rcutils_cli_get_option(argv, argv + argc, "-f"));
  }

  rclcpp::init(0, nullptr);

  auto buffer = std::make_shared<tf2_ros::Buffer>();
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*buffer);

  while(rclcpp::ok()) {
    try {
      auto tf_hand_1 = buffer->lookupTransform("hands/1", frame, tf2::TimePointZero);         

      std::cout << "Left hand translation: ("
                << tf_hand_1.transform.translation.x << ", " 
                << tf_hand_1.transform.translation.y << ", "
                << tf_hand_1.transform.translation.z << ")"
                << std::endl;
    } catch (tf2::LookupException & e) {
      std::cout << "No hand transforms found..." << std::endl;
    }
    
    sleep_for(200ms);
  }

  rclcpp::shutdown();
  return 0;
}