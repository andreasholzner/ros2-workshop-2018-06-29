#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "geometry_msgs/msg/transform_stamped.hpp"

#include  <cmath>

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

            timer = create_wall_timer(std::chrono::milliseconds(200), function);
    };
private:
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tfListener;
    rclcpp::WallTimer<std::function<void()>>::SharedPtr timer;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfListener>());
    rclcpp::shutdown();
    return 0;
}
