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
