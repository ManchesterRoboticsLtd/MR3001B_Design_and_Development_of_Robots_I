#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32

if __name__=='__main__':
    signal_pub=rospy.Publisher("signal",Float32, queue_size=10)
    time_pub=rospy.Publisher("time",Float32, queue_size=10)
    rospy.init_node("signal_generator")
    rate = rospy.Rate(10)
    init_time = rospy.get_time()

    while not rospy.is_shutdown():
        time = rospy.get_time()-init_time
        signal = np.sin(time)
        time_pub.publish(time)
        signal_pub.publish(signal)
        rospy.loginfo("The signal value is: %f at a time %f", signal, time)

        rate.sleep()