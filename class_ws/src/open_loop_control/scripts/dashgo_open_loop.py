#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

# Puzzlebot class to calculate current state with basic kinematics and an open-loop routine

class Puzzlebot:
    def __init__(self):
        # Initialise the node 
        rospy.init_node("puzzlebot_open_loop")
        self.loop_rate = rospy.Rate(50)
        self.sample_time = 0.1

        # Variables/Parameters to be used
        self.target_iterations = 4

        # Open Loop
        self.target_distance_time = [7, 7, 7, 7]
        self.target_angle_time = [6.5, 6.5, 6.5, 6.5]
        self.target_linear_velocity = 0.6
        self.target_angular_velocity = -0.6

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

        # Setup Publishers and Subscribers
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        #self.sub_right_wheel = rospy.Subscriber('/puzzlebot_1/wr', Float32, self.right_wheel_callback)
        #self.sub_left_wheel = rospy.Subscriber('/puzzlebot_1/wl', Float32, self.left_wheel_callback)

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

    """
    Open-loop control for rotation
    """
    def rotate(self, dt):
        #self.angle += self.angular_velocity * dt

        # Check if the angle has been reached
        if rospy.Time.now().to_sec() - self.last_time > self.target_angle_time[self.current_iteration]:
            self.current_task = self.task_states["MOVE"]
            self.stop()
            rospy.loginfo("Rotation complete")
            rospy.sleep(1.0)

            self.last_time = rospy.Time.now().to_sec()
            return

        self.set_twist(0.0, self.target_angular_velocity)

    def move(self, dt):
        """
        Open-loop control for movement
        """
        #self.distance += (self.linear_velocity ) * dt

        # Check if the distance has been reached
        if rospy.Time.now().to_sec() - self.last_time > self.target_distance_time[self.current_iteration]:
            self.current_task = self.task_states["IDLE"]
            self.stop()
            rospy.loginfo("Movement complete")
            rospy.sleep(1.0)
            
            self.current_iteration += 1
            return
        
        self.set_twist(self.target_linear_velocity, 0.0)

    def stop(self):
        self.set_twist(0.0, 0.0)


    def run(self):
        # Linear and angular velocity calculation with basic kinematics
        self.loop_rate.sleep()

        # self.linear_velocity = (self.right_wheel + self.left_wheel) / 2
        # self.angular_velocity = (self.right_wheel - self.left_wheel) / 2

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