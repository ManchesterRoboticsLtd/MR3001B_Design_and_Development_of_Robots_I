#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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
        self.distance_step = -1300
        self.angle_step = -4.12
        self.move_angle_step = -1.67

        self.target_distance = -1500
        self.target_angle = -4.12
        self.move_angle = 0 # radians
        self.distance_tolerance = 0.5
        self.angle_tolerance = 0.05

        # Kinematics
        self.right_wheel = 0.0
        self.left_wheel = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.distance = 0.0
        self.angle = 0.0
        self.imu_angle = 0.0
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
            "kp": 0.15,
            "ki": 0.005,
            "kd": 0.001,
            "integral": 0.0,
            "last_error": 0.0
        }

        self.translation_pid = {
            "kp": 2.0,
            "ki": 0.005,
            "kd": 0.0,
            "integral": 0.0,
            "last_error": 0.0
        }

        # Setup Publishers and Subscribers
        self.pub_cmd_vel = rospy.Publisher('/puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=1)
        self.sub_right_wheel = rospy.Subscriber('/puzzlebot_1/wr', Float32, self.right_wheel_callback)
        self.sub_left_wheel = rospy.Subscriber('/puzzlebot_1/wl', Float32, self.left_wheel_callback)
        self.sub_odom = rospy.Subscriber('/puzzlebot_1/base_controller/odom', Odometry, self.odom_callback)

    # Right wheel callback in rad/s
    def right_wheel_callback(self, msg):
        self.right_wheel = msg.data

    # Left wheel callback in rad/s
    def left_wheel_callback(self, msg):
        self.left_wheel = msg.data

    # Odometry callback
    def odom_callback(self, msg):
        orientation_quaternion = msg.pose.pose.orientation
        self.imu_angle = 2 * math.atan2(orientation_quaternion.z, orientation_quaternion.w)
        #rospy.loginfo(f"IMU Angle: {self.imu_angle}")

    # CMD_VEL publisher
    def set_twist(self, linear, angular):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        self.pub_cmd_vel.publish(cmd_vel)
        #rospy.loginfo(f"Linear: {linear}, Angular: {angular}")

    def rotation_control(self, error, dt):
        self.rotation_pid["integral"] += error * dt
        derivative = (error - self.rotation_pid["last_error"]) / dt
        self.rotation_pid["last_error"] = error
        return self.rotation_pid["kp"] * error + self.rotation_pid["ki"] * self.rotation_pid["integral"] - self.rotation_pid["kd"] * derivative
    
    def translation_control(self, error, dt):
        self.translation_pid["integral"] += error
        derivative = error - self.translation_pid["last_error"]
        self.translation_pid["last_error"] = error
        return self.translation_pid["kp"] * error + self.translation_pid["ki"] * self.translation_pid["integral"] - self.translation_pid["kd"] * derivative

    """
    Open-loop control for rotation
    """
    def rotate(self, dt):
        self.angle += self.angular_velocity * dt
        rospy.loginfo(f"Angle: {self.angle}, Vel: {self.angular_velocity}")

        # Check if the angle has been reached
        if abs(self.target_angle - self.angle) < self.angle_tolerance:
            self.current_task = self.task_states["MOVE"]
            self.stop()
            rospy.loginfo("Rotation complete")
            self.angle = 0.0
            rospy.sleep(4.0)

            self.last_time = rospy.Time.now().to_sec()
            return

        self.set_twist(0.0, self.rotation_control(self.target_angle - self.angle, dt))

    """
    Open-loop control for movement
    """
    def move(self, dt):
        self.distance += (self.linear_velocity ) * dt
        rospy.loginfo(f"Distance: {self.distance}, Vel: {self.linear_velocity}")

        # Check if the distance has been reached
        if abs(self.target_distance - self.distance) < self.distance_tolerance:
            self.current_task = self.task_states["IDLE"]
            self.stop()
            rospy.loginfo("Movement complete")
            self.distance = 0.0
            rospy.sleep(5.0)
            
            self.current_iteration += 1
            return
        
        self.set_twist(self.translation_control(self.target_distance - self.distance, dt), 0.0) #self.rotation_control(self.imu_angle - self.move_angle, dt))

    def stop(self):
        self.set_twist(0.0, 0.0)


    def run(self):
        # Linear and angular velocity calculation with basic kinematics
        self.loop_rate.sleep()

        self.linear_velocity = self.wheel_radius * (self.right_wheel + self.left_wheel) / 2
        self.angular_velocity = self.wheel_radius * (self.right_wheel - self.left_wheel) / (self.wheel_base) * math.pi / 180

        dt = rospy.Time.now().to_sec() - self.last_time
        if dt < self.sample_time:
            return

        if self.current_task == self.task_states["IDLE"]:
            self.stop()
            if self.current_iteration == self.target_iterations:
                rospy.loginfo("Open-loop routine complete")
                rospy.signal_shutdown("Routine complete")
            else:
                #self.target_distance += self.distance_step
                #self.target_angle += self.angle_step
                self.move_angle += self.move_angle_step
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