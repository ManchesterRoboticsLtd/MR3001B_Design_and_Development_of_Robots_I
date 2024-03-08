#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32

#Declare Parameters to be used
kp = rospy.get_param("/control_kp",0.095)
ki = rospy.get_param("/control_ki",0.05)
sample_time = rospy.get_param("/control_sample_time",0.02)

#Declare Variables to be used
first = True
start_time = 0.0 
last_time = 0.0
current_time = 0.0
error = 0.0
Error_Int = 0.0
Error_Der = 0.0

#Initialise message to be published
u = Float32()
u.data = 0.0

#Initialise messages for the subscribers
angularVelocity = Float32()
setPoint = Float32()

#Define callback functions
def callback_omega(msg):
    global angularVelocity
    angularVelocity = msg


def callback_set_point(msg):
    global setPoint
    setPoint = msg

#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")
  u.data = 0.0
  controlInput.publish(u.data)

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("Control")
    rate = rospy.Rate(rospy.get_param("/pidControlRate",100))
    rospy.on_shutdown(stop)

    #Setup Publishers and Subscribers
    rospy.Subscriber("/motor_output", Float32, callback_omega)
    rospy.Subscriber("/set_point", Float32, callback_set_point)
    controlInput=rospy.Publisher("/motor_input",Float32, queue_size=10)
    
    print("The Controller is Running")

    #Run the node
    while not rospy.is_shutdown():
        
        #Setup the variables (run only one time)
        if first == True:
            start_time = rospy.get_time() 
            last_time = rospy.get_time()
            current_time = rospy.get_time()
            first = False

        #Controller
        else:
            current_time = rospy.get_time()
            dt = current_time - last_time       #Get the sampling time

            #Get the control output, using the motor angular speed and set point
            if dt >= sample_time:

                error = setPoint.data - angularVelocity.data        #Get the error
                Error_Int += (error) * dt                           #Integrate the error
                u.data  = kp * error +ki *Error_Int                 #PI Controller

                #Publish data
                controlInput.publish(u.data )             

                #Get Previous time          
                last_time = rospy.get_time()

        #Wait and repeat
        rate.sleep()