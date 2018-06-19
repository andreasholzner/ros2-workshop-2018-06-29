#include <iostream>
#include "Leap.h"
#include "LeapPublisher.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    Leap::Controller controller;
    LeapPublisher listener;
    controller.addListener(listener);

    // Keep this process running until Enter is pressed
    std::cout << "Press Enter to quit..." << std::endl;
    std::cin.get();

    controller.removeListener(listener);

    std::cout << "Done!" << std::endl;

    rclcpp::shutdown();
    return 0;
}
