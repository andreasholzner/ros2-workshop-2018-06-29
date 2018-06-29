#include <iostream>
#include <memory>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include "leap_msgs/msg/leap_data.hpp"
#include "rclcpp/rclcpp.hpp"

class BroadcastNode
{
public:
    BroadcastNode() : node(std::make_shared<rclcpp::Node>("broadcaster")), broadcaster(node)
    {
        //broadcaster = tf2_ros::TransformBroadcaster::TransformBroadcaster(node);
        subscription = node->create_subscription<leap_msgs::msg::LeapData>("leap_data", [this] (std::shared_ptr<leap_msgs::msg::LeapData> msg) {
            send_message(msg);
        });
    }
    rclcpp::Node::SharedPtr node;
    tf2_ros::TransformBroadcaster broadcaster;
    rclcpp::Subscription<leap_msgs::msg::LeapData>::SharedPtr subscription;

    void send_message(std::shared_ptr<leap_msgs::msg::LeapData> msg) {
        std::cout << msg->header.frame_id << std::endl;

        for(unsigned int i = 0; i < msg->hands.size(); i++){
            auto custom_msg = geometry_msgs::msg::TransformStamped();
            custom_msg.header.stamp = msg->header.stamp;
            custom_msg.header.frame_id = msg->header.frame_id;

            custom_msg.child_frame_id = std::string("hand") + std::to_string(i+1);

            custom_msg.transform.translation.x = msg->hands[i].position.x;
            custom_msg.transform.translation.y = msg->hands[i].position.y;
            custom_msg.transform.translation.z = msg->hands[i].position.z;
            custom_msg.transform.rotation.x = msg->hands[i].orientation.x;
            custom_msg.transform.rotation.y = msg->hands[i].orientation.y;
            custom_msg.transform.rotation.z = msg->hands[i].orientation.z;
            custom_msg.transform.rotation.w = msg->hands[i].orientation.w;

            broadcaster.sendTransform(custom_msg);
        }
    }
};


int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    BroadcastNode broadcast_node = BroadcastNode();
    rclcpp::spin(broadcast_node.node);
    rclcpp::shutdown();
    return 0;
}
