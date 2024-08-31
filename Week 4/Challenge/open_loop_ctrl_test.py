#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock

# Declare the  process output message

clock_msg = None

#Define the callback functions
def clock_callback(msg):
    global clock_msg
    clock_msg = msg

#Stop Condition
def stop():
  #Setup the stop message (can be the same as the control message)
    controlOutput.linear.x = 0.0
    controlOutput.angular.z = 0.0
    control_pub.publish(controlOutput)
    print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("open_loop_controller")  

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))
    rospy.on_shutdown(stop)

    #Setup de publishers


    #Setup Subscribers
    clock_sub = rospy.Subscriber('/clock', Clock, clock_callback, queue_size=1)

    print("The controller is Running")

    # Setup Variables to be used
    first = True   
    last_time = None
    current_time = None

    try:

        while clock_msg == None:
            loop_rate.sleep()
           
        #Run the node
        while not rospy.is_shutdown(): 

            if first == True:
                last_time = rospy.Time.now().to_sec()
                current_time = rospy.Time.now().to_sec()
                first = False
            
            else:
                current_time = rospy.Time.now().to_sec()
                time = current_time - last_time


            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node
