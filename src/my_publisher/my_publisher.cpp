//
// Created by helge on 6/29/18.
//
\
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyPublisher : public rclcpp::Node
{
public:
    MyPublisher() : Node("my_publisher") {

    };

};


int main(int argc, char **argv)
{
    std::cout << "Hello, I'm a publisher" << std::endl;

    rclcpp::init(argc, argv);

    MyPublisher my_publisher;
    auto publisher = my_publisher.create_publisher<std_msgs::msg::String>("myTopic");
//    publisher->publish(std::make_shared<std_msgs::msg::String>("hi"));

    rclcpp::shutdown();

    return 0;
}