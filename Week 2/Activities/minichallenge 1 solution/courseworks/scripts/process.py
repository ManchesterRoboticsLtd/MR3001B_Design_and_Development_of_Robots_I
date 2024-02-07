#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32

signal_data = 0
time_data = 0
angle = np.pi/2.0
offset = 1.0
amplitude = 0.5

def callback(msg):
    global signal_data
    signal_data = msg.data


def callback_time(msg):
    global time_data
    time_data = msg.data

# Wrap to Pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta+np.pi),(2*np.pi))
    if (result<0):
        result += 2 * np.pi
    return result - np.pi


if __name__=='__main__':
    rospy.init_node("process")
    rospy.Subscriber("/signal", Float32, callback)
    rospy.Subscriber("/time", Float32, callback_time)
    pub=rospy.Publisher("proc_signal",Float32, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        if (wrap_to_Pi(time_data) >= -np.pi/2 and wrap_to_Pi(time_data)<= np.pi/2):
            signal = signal_data * np.cos(angle) + np.sqrt(1-np.power((signal_data),2)) * np.sin(angle) 
        else:
            signal = signal_data * np.cos(angle) - np.sqrt(1-np.power((signal_data),2)) * np.sin(angle)

        processed_signal = amplitude * (signal + offset)
        rospy.loginfo("The signal value is: %f at a time %f", processed_signal, time_data)

        pub.publish(processed_signal)
        rate.sleep()