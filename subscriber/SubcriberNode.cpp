#include <iostream>
#include <memory>
#include <string>

#include "SubscriberNode.hpp"

class SubcriberNode
{
    SubsriberNode::SubsriberNode()
    {
        auto sub = create_subscription("first_demo", [] (std::shared_ptr<std_msgs::String> msg) {
            std::cout << msg->data << std::cout;
        }
    }   
}

int main(int argc, char const *argv[])
{
    SubcriberNode n();
}
