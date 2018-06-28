# ROS 2.0 workshop

## Preparations

1. Install a recent ROS 2 either from source or as binary according to the [official instructions](https://github.com/ros2/ros2/wiki/Installation).
1. Clone this repository and checkout master.
1. Source the environment of your ROS 2 installation, e.g. `. $PATH_TO_YOUR_ROS_INSTALLATION/setup.bash`, and build this repositories content.
   ```
   cd $REPO_ROOT # directory that contains README.md
   colcon build --merge-install --cmake-args " -DCMAKE_BUILD_TYPE=Debug"
   ```
1. Source the local workshop environment and run the test program
   ```
   . install/setup.bash
   demo
   ```

## Links

- [Feature Overview](https://github.com/ros2/ros2/wiki/Features)
- [Design Overview](http://design.ros2.org/)
- [Tutorials](https://github.com/ros2/ros2/wiki/Tutorials)
- [Basic Examples](https://github.com/ros2/examples)
- [Colcon - build tool](https://colcon.readthedocs.io/en/released/)
- [Official Core Documentation](http://docs.ros2.org)
  - [rclcpp API docs](http://docs.ros2.org/ardent/api/rclcpp/index.html)
- TF2 - ROS Bindings
  - This is mostly ROS1 but the API works almost the same for ROS 2.0
  - [Wiki](http://wiki.ros.org/tf2_ros)
  - [API docs](http://docs.ros.org/latest/api/tf2_ros/html/c++/namespacetf2__ros.html)
  - [ROS 2.0 tf2_ros Github project](https://github.com/ros2/geometry2/tree/ros2/tf2_ros)

## ROS 2.0 CLI tools

- `ros2 topic list`
- `ros2 topic echo <TOPIC>`
  Subscribes to topic <TOPIC> and prints every received message.
- `ros2 topic pub ...`
  Creates an adhoc publisher and publishes periodically given data.
  ```
  ros2 topic pub topic1 std_msgs/String "data: Hello World"
  ```
