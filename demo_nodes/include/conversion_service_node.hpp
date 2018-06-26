/**
 * Copyright 2018 TNG Technology Consulting GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DEMO_NODES__CONVERSION_SERVICE_NODE_HPP_
#define DEMO_NODES__CONVERSION_SERVICE_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "leap_srvs/srv/hand_to_pose.hpp"

class ConversionServiceNode : public rclcpp::Node {
public:
  explicit ConversionServiceNode(std::string service_name);

private:
  void convert(
    std::shared_ptr<rmw_request_id_t> request_header,
    leap_srvs::srv::HandToPose::Request::SharedPtr request,
    leap_srvs::srv::HandToPose::Response::SharedPtr response);

  rclcpp::Service<leap_srvs::srv::HandToPose>::SharedPtr service_;
};

#endif /* DEMO_NODES__CONVERSION_SERVICE_NODE_HPP_ */