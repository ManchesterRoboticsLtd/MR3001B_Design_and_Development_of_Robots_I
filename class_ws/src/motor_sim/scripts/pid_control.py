#!/usr/bin/env python

import rospy

#Declare libraries to be used (if needed)
from std_msgs.msg import Float32
import numpy as np
#Declare Messages to be used


#Declare Variables/Parameters to be used

#Define callback functions (if required)

#Define other functions (if required)

class Controller:
    def __init__(self):
        #Initialise and Setup node
        rospy.init_node("Control")
        self.loop_rate = rospy.Rate(200)
        self.sample_time = rospy.get_param("~sample_time",0.01)

        # Variables/Parameters to be used
        self.kP = rospy.get_param("~kP",1.0)
        self.kI = rospy.get_param("~kI",0.0)
        self.kD = rospy.get_param("~kD",0.0)

        self.setpoint = 0.0
        self.feedback = 0.0
        self.integral = 0.0
        self.last_error = 0.0

        self.last_time = rospy.Time.now().to_sec()

        # Setup Publishers and Subscribers
        self.sub_set_point = rospy.Subscriber('/set_point', Float32, self.set_point_callback)
        
        self.pub_motor_input = rospy.Publisher('/motor_input', Float32, queue_size=1)
        self.sub_motor_output = rospy.Subscriber('/motor_output', Float32, self.motor_output_callback)

    # Set Point Callback
    def set_point_callback(self, msg):
        self.setpoint = msg.data

    # Motor output callback
    def motor_output_callback(self, msg):
        self.feedback = msg.data

    def run(self):
        dt = rospy.Time.now().to_sec() - self.last_time
        if dt >= self.sample_time:
            error = self.setpoint - self.feedback 
            self.integral += error * dt
            derivative = (error - self.last_error) / dt
            out = self.kP * error + self.kI * self.integral + self.kD * derivative
            out_msg = Float32()
            out_msg.data = out
            self.pub_motor_input.publish(out)

            self.last_error = error
            self.last_time = rospy.Time.now().to_sec()

        self.loop_rate.sleep()

if __name__=='__main__':
    control = Controller()

    #Run the node
    try:
        while not rospy.is_shutdown():
           control.run() 
            
    except rospy.ROSInterruptException:
        pass    