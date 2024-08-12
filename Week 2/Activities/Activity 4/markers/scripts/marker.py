#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Time

#Setup parameters, vriables and callback functions here (if required)
# Message Declaration 

#Function to initialise a message
def init_sun():





#Stop Condition
def stop():
    print("Stopping")


if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("RVIZ_marker")

    # Configure the Node
    loop_rate = rospy.Rate(10)
    rospy.on_shutdown(stop)
    print("The Sun is ready")

    #Setup the messages
    init_sun()

    #Setup Publishers and subscribers here


    try:
    #Run the node
        while not rospy.is_shutdown():
           #Update time stamp


            #Publish the marker


            loop_rate.sleep()
 
    except rospy.ROSInterruptException:
        pass
