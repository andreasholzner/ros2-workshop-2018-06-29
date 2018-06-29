#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "leap_msgs/msg/leap_data.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include  <cmath>

namespace tf2{
    template <>
    inline
    void doTransform(const Stamped<leap_msgs::msg::Hand>& t_in, Stamped<leap_msgs::msg::Hand>& t_out, const geometry_msgs::msg::TransformStamped& transform){
        geometry_msgs::msg::PointStamped point_in;
        point_in.header.stamp = tf2_ros::toMsg(t_in.stamp_) ;
        point_in.header.frame_id = t_in.frame_id_;
        point_in.point = t_in.position;

        geometry_msgs::msg::PointStamped point_out;
        
        doTransform(point_in, point_out, transform);

        t_out.position = point_out.point;
        t_out.stamp_ = tf2_ros::fromMsg(point_out.header.stamp);
        t_out.frame_id_ = point_out.header.frame_id;


    }

}

class TfListener : public rclcpp::Node
{
public:
    TfListener() : Node("TfListener"), buffer(), tfListener(buffer)
    {
            std::function<void()> function = [this] (){
                
                try
                {
                    if(buffer.canTransform("hand2", "leap_frame", tf2::TimePointZero)){
                    geometry_msgs::msg::TransformStamped t = buffer.lookupTransform("hand2", "leap_frame", tf2::TimePointZero);
                    std::cout << "Transform zu hand 2 vorhanden " << t.transform.translation.x << std::endl;
                }

                if(buffer.canTransform("hand1", "leap_frame", tf2::TimePointZero)){
                    geometry_msgs::msg::TransformStamped t = buffer.lookupTransform("hand1", "leap_frame", tf2::TimePointZero);
                    std::cout << "Transform zu hand 1 vorhanden " << t.transform.translation.x << std::endl;
                }

                if(buffer.canTransform("hand1", "hand2", tf2::TimePointZero)){
                    auto t = buffer.lookupTransform("hand1", "hand2", tf2::TimePointZero);
                    std::cout << "Abstand:" << sqrt(t.transform.translation.x*t.transform.translation.x+t.transform.translation.y*t.transform.translation.y+t.transform.translation.z*t.transform.translation.z) << std::endl;
                }

                }
                catch(const tf2::LookupException& e)
                {
                    std::cerr << e.what() << '\n';
                }
                
                
            };

            //timer = create_wall_timer(std::chrono::milliseconds(200), function);

            subscription = create_subscription<leap_msgs::msg::LeapData>("/leap_data", [this] (std::shared_ptr<leap_msgs::msg::LeapData> msg) {
                if(msg->hands.size() != 2)
                    return;
                     try
                {
                auto left = msg->hands[0];
                auto right = msg->hands[1];
                auto left_transform = buffer.transform(tf2::Stamped<leap_msgs::msg::Hand>(left, tf2_ros::fromMsg(msg->header.stamp), "hand1"), "hand2", tf2::TimePointZero);

                std::cout << left.position.x << std::endl;
                std::cout << left_transform.position.x << std::endl;}
                 catch(const tf2::LookupException& e)
                {
                    std::cerr << e.what() << '\n';
                }
            });
    }; 
private:
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tfListener;
    rclcpp::WallTimer<std::function<void()>>::SharedPtr timer;
    rclcpp::Subscription<leap_msgs::msg::LeapData>::SharedPtr subscription;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfListener>());
    rclcpp::shutdown();
    return 0;
}
