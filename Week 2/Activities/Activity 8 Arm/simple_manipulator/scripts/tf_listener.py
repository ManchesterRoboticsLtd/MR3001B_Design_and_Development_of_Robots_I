#!/usr/bin/env python  
import rospy
import tf2_ros
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
import tf_conversions


# Setup parameters, transforms, variables and callback functions here (if required)



#Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")

if __name__ == '__main__':
    #Initialise and Setup node
    rospy.init_node('transform_listener')

    # Configure the Node
    loop_rate = rospy.Rate(10)
    rospy.on_shutdown(stop)

    #Setup the messages and transforms

    #Setup Publishers, subscribers and transform broadcasters here


    print("The tf listener is ready")

    try:

        while not rospy.is_shutdown():

            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass