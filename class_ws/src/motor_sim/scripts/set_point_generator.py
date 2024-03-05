#!/usr/bin/env python

import rospy

#Declare libraries to be used
import math
from std_msgs.msg import Float32

#Declare Messages to be used

#Declare Variables/Parameters to be used
signal = 0


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SetPoint_Generator")
    amplitude = rospy.get_param('~amplitude', 1.0)
    frequency = rospy.get_param('~frequency', 1.0)

    # Configure the Node
    rate = rospy.Rate(100)

    #Declare other functions if required

    #Setup Publishers and Subscribers
    pub_setpoint = rospy.Publisher('/set_point', Float32, queue_size=10)
    
    try:
        while not rospy.is_shutdown():
            
            ######## Write your node here ##############
            signal = amplitude * math.sin(2 * math.pi * frequency * rospy.Time.now().to_sec())

            msg_setpoint = Float32()
            msg_setpoint.data = signal
            pub_setpoint.publish(msg_setpoint)

            rate.sleep()

     
    except rospy.ROSInterruptException:
        pass