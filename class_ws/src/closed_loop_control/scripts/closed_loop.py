#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math

# Puzzlebot class to calculate current state with basic kinematics and an open-loop routine

class Puzzlebot:
    def __init__(self):
        # Initialise the node 
        rospy.init_node("puzzlebot_open_loop")
        self.loop_rate = rospy.Rate(400)
        self.sample_time = 0.005

        # Robot description
        self.wheel_radius = 0.05
        self.wheel_base = 0.19

        # Variables/Parameters to be used
        self.target_iterations = 4

        # Closed Loop
        self.target_distance = -100 # meters
        self.target_angle = -250 # degrees
        self.distance_tolerance = 0.5
        self.angle_tolerance = 0.4

        # Kinematics
        self.right_wheel = 0.0
        self.left_wheel = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.distance = 0.0
        self.angle = 0.0
        self.last_time = 0.0

        self.task_states = {
            "IDLE": 0,
            "TURN": 1,
            "MOVE": 2
        }
        self.current_task = self.task_states["IDLE"]
        self.current_iteration = 0

        # PID Parameters
        self.rotation_pid = {
            "kp": 0.01,
            "ki": 0.0,
            "kd": 0.0,
            "integral": 0.0,
            "last_error": 0.0
        }

        self.translation_pid = {
            "kp": 0.5,
            "ki": 0.0,
            "kd": 0.0,
            "integral": 0.0,
            "last_error": 0.0
        }

        # Setup Publishers and Subscribers
        self.pub_cmd_vel = rospy.Publisher('/puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=1)
        self.sub_right_wheel = rospy.Subscriber('/puzzlebot_1/wr', Float32, self.right_wheel_callback)
        self.sub_left_wheel = rospy.Subscriber('/puzzlebot_1/wl', Float32, self.left_wheel_callback)

    # Right wheel callback in rad/s
    def right_wheel_callback(self, msg):
        self.right_wheel = msg.data

    # Left wheel callback in rad/s
    def left_wheel_callback(self, msg):
        self.left_wheel = msg.data

    # CMD_VEL publisher
    def set_twist(self, linear, angular):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        self.pub_cmd_vel.publish(cmd_vel)
        #rospy.loginfo(f"Linear: {linear}, Angular: {angular}")

    def rotation_control(self, error):

        return self.rotation_pid["kp"] * error
    
    def translation_control(self, error):
        return self.translation_pid["kp"] * error

    """
    Open-loop control for rotation
    """
    def rotate(self, dt):
        self.angle += self.angular_velocity * dt
        rospy.loginfo(f"Angle: {self.angle}, Vel: {self.angular_velocity}")

        # Check if the angle has been reached
        if abs(self.target_angle - self.angle) < self.angle_tolerance:
            self.current_task = self.task_states["IDLE"]
            self.stop()
            rospy.loginfo("Rotation complete")
            rospy.sleep(10.0)

            self.last_time = rospy.Time.now().to_sec()
            return

        self.set_twist(0.0, self.rotation_control(self.target_angle - self.angle))

    def move(self, dt):
        """
        Open-loop control for movement
        """
        self.distance += (self.linear_velocity ) * dt
        rospy.loginfo(f"Distance: {self.distance}, Vel: {self.linear_velocity}")

        # Check if the distance has been reached
        if abs(self.target_distance - self.distance) < self.distance_tolerance:
            self.current_task = self.task_states["IDLE"]
            self.stop()
            rospy.loginfo("Movement complete")
            rospy.sleep(5.0)
            
            self.current_iteration += 1
            return
        
        self.set_twist(self.translation_control(self.target_distance - self.distance), 0.0)

    def stop(self):
        self.set_twist(0.0, 0.0)


    def run(self):
        # Linear and angular velocity calculation with basic kinematics
        self.loop_rate.sleep()

        self.linear_velocity = self.wheel_radius * (self.right_wheel + self.left_wheel) / 2
        self.angular_velocity = self.wheel_radius * (self.right_wheel - self.left_wheel) / (self.wheel_base)

        dt = rospy.Time.now().to_sec() - self.last_time
        if dt < self.sample_time:
            return

        if self.current_task == self.task_states["IDLE"]:
            self.stop()
            if self.current_iteration == self.target_iterations:
                rospy.loginfo("Open-loop routine complete")
                rospy.signal_shutdown("Routine complete")
            else:
                self.current_task = self.task_states["TURN"]
                self.last_time = rospy.Time.now().to_sec()
        
        elif self.current_task == self.task_states["TURN"]:
            self.rotate(dt)

        elif self.current_task == self.task_states["MOVE"]:
            self.move(dt)        


if __name__ == "__main__":
    puzzlebot = Puzzlebot()
    try:
        while not rospy.is_shutdown():
            puzzlebot.run()
    except rospy.ROSInterruptException:
        pass