# Open Loop Controller (Puzzlebot Sim + Dashgo)
### Mini challenge 4

By [afr2903](https://github.com/afr2903/)

[Challenge instructions](https://github.com/afr2903/MR3001B_Design_and_Development_of_Robots_I/blob/main/Week%204/Challenge/)

The instructions and execution were added to the `src` folder, as this challenge involves several ros packages.

**Summary:** The challenge required an implementation of an open-loop controller for the Puzzlebot or the Dashgo B1, using simple diff-drive kinematics to estimate the time required for a robot to reach a certain distance.

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
sudo apt-get install ros-noetic-position-controllers
```

### Characterization process

Inside the Gazebo environment, the following commands were used in separate terminals to obtain the maximum values for the input and the output:

```bash
# Publish linear and angular speed to the robot
rostopic pub -r 10 /puzzlebot_1/base_controller/cmd_vel geometry_msgs/Twist 
"linear:
  x: -2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
```

Changing the `linear x` value, different inputs were tested to find the maximum.

```bash
# Print the wheel speeds to the terminal
rostopic echo /puzzlebot_1/wl # or wr
```

By testing different inputs, and reviewing the wheels' speed output, it was found the input range is `[-1, 1]` and the output range (approximately) for each wheel is `[-11.9, 11.9]`.

### Open Loop

I realised the characterization is not needed for this challenge but for the next and final challenge. So in order to create an open loop controller node for the Gazebo simulation, the script `open_loop.py` was created inside the `puzzlebot_gazebo` package.

This script is structured with the main class `Puzzlebot`, containing the methods needed to execute the state machine to draw the path of a square.

The following states were defined:
- **IDLE:** Checks for the number of iterations (turn + movement) and serves as a hard stop or middle step during the execution.
- **TURN:** Indicates the robot to turn for `target_angle_time` at a speed of `target_angular_velocity`.
- **MOVE:** Indicates the robot to move for `target_distance_time` at a speed of `target_linear_velocity`.

For the target times in each state of the whole path, it was observed that an open-loop control (based on time) it's really unreliable and unstable. In order to achieve as much precision as possible, an array with custom times for each segment of the path was created. This change turned the script less scalable for more iterations of the path. But for the purposes of the challenge, it improved the precission in reaching the waypoints in the world.

[Video demonstration Puzzlebot Gazebo](https://github.com/afr2903/MR3001B_Design_and_Development_of_Robots_I/assets/25570636/b4cd25be-75d0-410d-b3fb-f8792980964d)
