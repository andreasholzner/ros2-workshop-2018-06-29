#include "LeapPublisher.h"
#include "leap_msgs/msg/leap_frame.hpp"

using namespace Leap;

const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};

LeapPublisher::LeapPublisher()
{
  node_ = rclcpp::Node::make_shared("leapmotion_publisher");
  leap_frame_publisher_ = node_->create_publisher<leap_msgs::msg::LeapFrame>("leap_frames");
  marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("hand_array");
}

void LeapPublisher::onConnect(const Leap::Controller & controller)
{
  std::cout << "Connected" << std::endl;
}

geometry_msgs::msg::Point toPoint(Leap::Vector vector)
{
  auto position = geometry_msgs::msg::Point();
  position.x = -.001 * vector.z;
  position.y = -.001 * vector.x;
  position.z = .001 * vector.y;
  return position;
}

geometry_msgs::msg::Quaternion toQuaternion(double pitch, double roll, double yaw)
{
  geometry_msgs::msg::Quaternion q;
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);

  q.w = cy * cr * cp + sy * sr * sp;
  q.x = cy * sr * cp - sy * cr * sp;
  q.y = cy * cr * sp + sy * sr * cp;
  q.z = sy * cr * cp - cy * sr * sp;
  return q;
}

void LeapPublisher::onFrame(const Leap::Controller & controller)
{
  const Frame frame = controller.frame();
  std::cout << "Frame id: " << frame.id()
            << ", timestamp: " << frame.timestamp()
            << ", hands: " << frame.hands().count()
            << ", fingers: " << frame.fingers().count()
            << ", extended fingers: " << frame.fingers().extended().count()
            << ", tools: " << frame.tools().count()
            << ", gestures: " << frame.gestures().count()
            << std::endl;

  auto frame_msg = leap_msgs::msg::LeapFrame();
  frame_msg.header = std_msgs::msg::Header();
  frame_msg.header.frame_id = "leap_frame";
  frame_msg.header.stamp = rclcpp::Clock().now();

  auto marker_array_msg = visualization_msgs::msg::MarkerArray();

  for (auto && leap_hand : frame.hands()) {
    std::string handType = leap_hand.isLeft() ? "Left hand" : "Right hand";
    std::cout << std::string(2, ' ') << handType
              << ", id: " << leap_hand.id()
              << ", palm position: " << leap_hand.palmPosition()
              << ", fingers: " << leap_hand.fingers().count()
              << ", extended fingers: " << leap_hand.fingers().extended().count()
              << std::endl;

    const Vector normal = leap_hand.palmNormal();
    const Vector direction = leap_hand.direction();

    auto hand = leap_msgs::msg::Hand();
    hand.position = toPoint(leap_hand.palmPosition());
    hand.orientation =
      toQuaternion(-1 * direction.pitch(), -1 * normal.roll(), -1 * direction.yaw());

    auto hand_marker = visualization_msgs::msg::Marker();
    hand_marker.header = std_msgs::msg::Header();
    hand_marker.header.frame_id = "leap_frame";
    hand_marker.header.stamp = frame_msg.header.stamp;

    hand_marker.id = leap_hand.isLeft() ? 1 : 2;
    hand_marker.ns = "hand_markers";
    hand_marker.type = visualization_msgs::msg::Marker::SPHERE;
    hand_marker.action = visualization_msgs::msg::Marker::ADD;
    hand_marker.lifetime = rclcpp::Duration(1, 0);

    hand_marker.scale.x = 0.1;
    hand_marker.scale.y = 0.1;
    hand_marker.scale.z = 0.1;

    hand_marker.color.a = 1.0;
    hand_marker.color.r = 1.0;
    hand_marker.color.g = 0.0;
    hand_marker.color.b = 0.0;

    hand_marker.pose.position = hand.position;
    hand_marker.pose.orientation = hand.orientation;

    marker_array_msg.markers.emplace_back(hand_marker);

    for (auto && leap_finger : leap_hand.fingers().extended()) {
      std::cout << std::string(4, ' ') << fingerNames[leap_finger.type()]
                << " finger, id: " << leap_finger.id()
                << ", length: " << leap_finger.length()
                << "mm, width: " << leap_finger.width()
                << ", tip: " << leap_finger.tipPosition()
                << ", direction: " << leap_finger.direction()
                << std::endl;

      auto finger = leap_msgs::msg::Finger();
      finger.length = leap_finger.length();
      finger.width = leap_finger.width();
      finger.tip_position = toPoint(leap_finger.tipPosition());
      finger.tip_orientation = toQuaternion(
        -1 * leap_finger.direction().pitch(), -1 * leap_finger.direction().roll(),
        -1 * leap_finger.direction().yaw());

      hand.fingers.push_back(finger);

      auto finger_marker = visualization_msgs::msg::Marker();
      finger_marker.header = std_msgs::msg::Header();
      finger_marker.header.frame_id = "leap_frame";
      finger_marker.header.stamp = frame_msg.header.stamp;

      finger_marker.id = hand_marker.id * 10 + leap_finger.type();
      finger_marker.ns = "finger_markers";
      finger_marker.type = visualization_msgs::msg::Marker::ARROW;
      finger_marker.action = visualization_msgs::msg::Marker::ADD;
      finger_marker.lifetime = rclcpp::Duration(1, 0);

      finger_marker.scale.x = 0.1;
      finger_marker.scale.y = 0.01;
      finger_marker.scale.z = 0.01;

      finger_marker.color.a = 1.0;
      finger_marker.color.r = 0.0;
      finger_marker.color.g = 1.0;
      finger_marker.color.b = 1.0;

      finger_marker.pose.position = finger.tip_position;
      finger_marker.pose.orientation = finger.tip_orientation;

      marker_array_msg.markers.emplace_back(finger_marker);
    }

    frame_msg.hands.push_back(hand);
  }

  leap_frame_publisher_->publish(frame_msg);
  marker_publisher_->publish(marker_array_msg);
  rclcpp::spin_some(node_);
}
