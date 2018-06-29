#include <iostream>
#include <memory>
#include <string>
#include "messages/msg/custom_message.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode() : Node("Subscriber")
    {
        subscription = create_subscription<std_msgs::msg::String>("first_demo", [] (std::shared_ptr<std_msgs::msg::String> msg) {
            std::cout << msg->data << std::endl;
        });
        customSubscription = create_subscription<messages::msg::CustomMessage>("first_demo", [] (std::shared_ptr<messages::msg::CustomMessage> msg) {
                    std::cout << msg->foo << std::endl;
        });
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
    rclcpp::Subscription<messages::msg::CustomMessage>::SharedPtr customSubscription;
};


int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
