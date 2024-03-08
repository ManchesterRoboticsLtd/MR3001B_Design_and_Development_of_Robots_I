#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32


#Declare Variables/Parameters to be used
Amplitude = rospy.get_param("/setPoint_Amplitude",8.0)
Omega = rospy.get_param("/setPoint_Freq",0.1)

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SetPoint_Generator")

    # Configure the Node
    rate = rospy.Rate(rospy.get_param("/setpointRate",200))

    #Setup Publishers and Subscribers
    signal_pub=rospy.Publisher("/set_point",Float32, queue_size=10)
    
    #Declare initial time
    init_time = rospy.get_time()

    while not rospy.is_shutdown():

        #Get time
        time = rospy.get_time()-init_time
        
        #Define the Set Point
        signal = Amplitude*np.sin(Omega*time)

        #Publish the Set Point
        signal_pub.publish(signal)

        #Wait and repeat
        rate.sleep()