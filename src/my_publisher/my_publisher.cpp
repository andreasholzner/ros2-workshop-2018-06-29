//
// Created by helge on 6/29/18.
//
\
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include <thread>
#include <chrono>

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
    auto publisher = my_publisher.create_publisher<std_msgs::msg::String>("first_demo");
    auto msg = std_msgs::msg::String();
	for(int i = 0; true; i++){
	    msg.data = std::string("Testdaten") + std::to_string(i);
	    publisher->publish(msg);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

    rclcpp::shutdown();

    return 0;
}
