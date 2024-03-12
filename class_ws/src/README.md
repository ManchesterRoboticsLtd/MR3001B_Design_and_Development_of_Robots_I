# Puzzlebot Sim + Dashgo
### Mini challenge 4

By [afr2903](https://github.com/afr2903/)

[Challenge instructions](https://github.com/afr2903/MR3001B_Design_and_Development_of_Robots_I/blob/main/Week%204/Challenge/)

The instructions and execution were added to the `src` folder, as this challenge involves several ros packages.

# Puzzlebot Sim

### Initial setup

In order to run the initial `roslaunch` command to test the environment setup, after cloning the 3 ros packages to the `workspace`, a similar error may appear:

```bash
CMake Error at /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
  Could not find a package configuration file provided by
  "controller_manager" with any of the following names:

    controller_managerConfig.cmake
    controller_manager-config.cmake
```

To solve this issue, additional ros packages have to be installed through Ubuntu's `apt` repository. Run the following commands:
```bash
# ROS Controller manager
sudo apt-get install ros-noetic-controller-manager
sudo apt-get install ros-noetic-diff-drive-controller
sudo apt-get install ros-noetic-transmission-interface
sudo apt-get install ros-noetic-gazebo-ros-control

# ROS Joint state controller
sudo apt-get install ros-noetic-joint-state-controller
sudo apt-get install ros-noetic-effort-controllers
sudo apt-get install ros-indigo-position-controllers
```

