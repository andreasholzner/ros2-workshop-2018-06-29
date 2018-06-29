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
#include <memory>

class MyPublisher : public rclcpp::Node
{
public:
    MyPublisher() : Node("my_publisher") {

    };

};


int main(int argc, char **argv)
{
	int i = 0;
    std::cout << "Hello, I'm a publisher" << std::endl;

    rclcpp::init(argc, argv);

	auto my_publisher = std::make_shared<MyPublisher>();
    auto publisher = my_publisher->create_publisher<std_msgs::msg::String>("first_demo");
    auto timer = my_publisher->create_wall_timer(std::chrono::seconds(1),[publisher, &i](){
		auto msg = std_msgs::msg::String();
	    msg.data = std::string("Testdaten ") + std::to_string(i);
	    publisher->publish(msg);
		i++;
	});

	rclcpp::spin(my_publisher);

    rclcpp::shutdown();

    return 0;
}
