#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf_conversions

# Setup parameters, transforms, variables and callback functions here (if required)
# Declare message or transform.






#Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")

if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("Robot")
 
    # Configure the Node
    loop_rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup the messages and transforms


    #Setup Publishers, subscribers and transform broadcasters here

  
 
    print("The tf's are ready")

    try:
        #Run the node
        while not rospy.is_shutdown(): 
            
            t = rospy.Time.now().to_sec()
            

            loop_rate.sleep()
 
    except rospy.ROSInterruptException:
        pass
