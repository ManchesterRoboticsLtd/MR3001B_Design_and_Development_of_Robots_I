#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64

if __name__=='__main__':
    signal_pub=rospy.Publisher("signal",Float64, queue_size=1)
    time_pub=rospy.Publisher("time",Float64, queue_size=1)
    rospy.init_node("signal_generator")
    rate = rospy.Rate(10)
    init_time = rospy.get_time()

    while not rospy.is_shutdown():
        time = rospy.get_time()-init_time
        signal = np.sin(time)
        time_pub.publish(time)
        signal_pub.publish(signal)

        rate.sleep()