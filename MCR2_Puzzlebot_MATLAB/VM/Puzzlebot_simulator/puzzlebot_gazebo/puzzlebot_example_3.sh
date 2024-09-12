#!/bin/sh

export SVGA_VGPU10=0
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/src/GazeboPlugin/export
export ROS_IP=$(hostname -I | tr -d [:blank:])
export ROS_MASTER_URI=http://$ROS_IP:11311

# Launch Gazebo world with TurtleBot3 Waffle Pi
gnome-terminal --title="Gazebo Puzzlebot CoSim" -- /bin/bash -c 'source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash ; roslaunch puzzlebot_gazebo puzzlebot_gazebo.launch' 



