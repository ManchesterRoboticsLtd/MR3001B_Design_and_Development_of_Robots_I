#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32

c_time = 0
signal = 0

def time_cb(msg):
    global c_time
    c_time = msg.data

def signal_cb(msg):
    global signal
    signal = msg.data


if __name__ == '__main__':
    rospy.init_node('process')
    phase_shift = rospy.get_param('~phase_shift', 0.0)
    rospy.loginfo("Phase shift: %f", phase_shift)

    sub_time = rospy.Subscriber('/time', Float32, time_cb)
    sub_signal = rospy.Subscriber('/signal', Float32, signal_cb)

    pub = rospy.Publisher('/proc_signal', Float32, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg_proc_signal = Float32()
        msg_proc_signal.data = (1/2) * math.sin(c_time + phase_shift) + 1 
        pub.publish(msg_proc_signal)
        rate.sleep()
        #rospy.loginfo("Processed Signal: %f", msg_proc_signal.data)
