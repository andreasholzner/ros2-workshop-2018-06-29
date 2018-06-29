#include <iostream>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode() : Node("Subscriber")
    {
        auto sub = create_subscription<std_msgs::msg::String>("first_demo", [] (std::shared_ptr<std_msgs::msg::String> msg) {
            std::cout << msg->data << std::endl;
        });
    }   
};

int main(int argc, char const *argv[])
{
    SubscriberNode n();
}
