#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from first_order_sys_sim.msg import system_output


# Declare the output Messages


#Declare Variables/Parameters to be used



# Declare the output Messages


#Define the callback functions



    #Stop Condition


if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("DC_Motor_Joints")
 
    # Configure the Node


    #Init joints


    #Setup publishers and subscribers


    try:
    #Run the node
        while not rospy.is_shutdown(): 


            loop_rate.sleep()
 
    except rospy.ROSInterruptException:
        pass
