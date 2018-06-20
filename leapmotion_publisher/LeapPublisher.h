#ifndef LEAPMOTION_POC_LEAPLISTENER_H
#define LEAPMOTION_POC_LEAPLISTENER_H

#include <Leap.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "leap_msgs/msg/leap_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class LeapPublisher : public Leap::Listener  {
public:
    LeapPublisher();

    void onConnect(const Leap::Controller & controller) override;
    void onFrame(const Leap::Controller & controller) override;

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::Publisher<leap_msgs::msg::LeapData>> leap_data_publisher_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> marker_publisher_;
};


#endif //LEAPMOTION_POC_LEAPLISTENER_H
