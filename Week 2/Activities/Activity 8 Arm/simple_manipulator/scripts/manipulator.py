#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf_conversions

# Setup parameters, transforms, variables and callback functions here (if required)

# Declare Marker messages


#Declare Transform Messages


#################### Functions to Initialise Markers #######################################################

#Functions to initialise markers

#####################################################################################
    
############################ Initialise Transform Messages ###########################

#Functions to initialise transforms

#############################################################################

#Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")

if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("manipulator_ex")
 
    # Configure the Node
    loop_rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Functions to initialise markers and transforms

    #Setup Publishers, subscribers 

    #Setup transform broadcasters here


    print("The tf's are ready")

    try:
        #Run the node
        while not rospy.is_shutdown(): 
            
            #Declare time variable
            t = rospy.Time.now().to_sec()

            #Update the markers time stamp      

            # Define the transform movements

            #Update the transforms time stamps and movements
 
            #Broadcast transforms

            #Publish markers

            loop_rate.sleep()
 
    except rospy.ROSInterruptException:
        pass