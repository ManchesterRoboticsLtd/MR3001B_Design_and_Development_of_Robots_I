#!/usr/bin/env python3

# ROS Node with a GUI to control the motor input using a slider from [-1, 1]

import rospy

#Declare libraries to be used
import math
from std_msgs.msg import Float32

# Libraty for a slider
from tkinter import Tk, Scale, HORIZONTAL, Label, Button
root = Tk()

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("Motor_Input_GUI")
    loop_rate = rospy.Rate(10)
    
    #Intialise slider
    root.title("Motor Input")
    root.geometry("300x200")

    label = Label(root, text="Motor Input")
    label.pack()

    slider = Scale(root, from_=-1, to=1, orient=HORIZONTAL, resolution=0.01)
    slider.pack()

    #Setup Publishers and Subscribers
    pub_motor_input = rospy.Publisher('/motor_input', Float32, queue_size=1)

    while not rospy.is_shutdown():
        msg_motor_input = Float32()
        msg_motor_input.data = slider.get()
        pub_motor_input.publish( msg_motor_input ) 

        loop_rate.sleep()
        root.update_idletasks()
        root.update()
