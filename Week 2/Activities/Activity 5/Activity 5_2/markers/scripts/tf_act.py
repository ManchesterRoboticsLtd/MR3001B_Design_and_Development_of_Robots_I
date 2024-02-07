#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf_conversions

# Setup parameters, transforms, variables and callback functions here (if required)
# Declare message or transform

# Initialise Sun Transform Message
def init_sunTransform():   


# Initialise Planet Transform Message
def init_planetTransform():



#Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")

if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("RVIZ_marker")
 
    # Configure the Node
    loop_rate = rospy.Rate(10)
    rospy.on_shutdown(stop)

    #Setup the messages
    init_sunTransform()
    init_planetTransform()

    #Setup Publishers, subscribers and transform broadcasters here

 
    print("The tf's are ready")

    try:
        #Run the node
        while not rospy.is_shutdown(): 
            
            #Get the Simulation time
            t = rospy.Time.now().to_sec()
            
            #Update time stamp for fixed transformations
            

            # Rotate around y-axis


	    #Update transformation


            # Publish the transforms


            loop_rate.sleep()
 
    except rospy.ROSInterruptException:
        pass
