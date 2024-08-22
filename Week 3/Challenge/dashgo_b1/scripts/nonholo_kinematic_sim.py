#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from nav_msgs.msg import Odometry
import tf_conversions

# Setup Variables to be used

# Declare Messages to be used
pose_tf = TransformStamped()

#Initialise messages (if required)

#Initialise TF's
def init_poseTF(pos_x, pos_y, pos_th):
    #Transform the angle into a quaternion for the orientation
    pos_orient = tf_conversions.transformations.quaternion_from_euler(0,0,pos_th)
    pose_tf.header.frame_id = "odom"
    pose_tf.child_frame_id = "base_link"
    pose_tf.header.stamp = rospy.Time.now()
    pose_tf.transform.translation.x = pos_x
    pose_tf.transform.translation.y = pos_y
    pose_tf.transform.translation.z = 0.0
    pose_tf.transform.rotation.x = pos_orient[0]
    pose_tf.transform.rotation.y = pos_orient[1]
    pose_tf.transform.rotation.z = pos_orient[2]
    pose_tf.transform.rotation.w = pos_orient[3]

#Define the callback functions (if required)

#Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")

if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("B1_sim")
 
    #Set the parameters of the system

    #Set initial conditions of the system (Initial position of the robot )
    pos_x = 0.00
    pos_y = 0.00
    pos_th = 0.00

    # Configure the Node
    loop_rate = rospy.Rate(200)
    rospy.on_shutdown(stop)

    #Init messages to be used
    init_poseTF(pos_x, pos_y, pos_th)

    # Setup the Subscribers

    #Setup de publishers

    #Setup TF Broadcasters
    pose_tf_bc = TransformBroadcaster()

    #Node Running
    print("The Robot Simulator is Running")

    try:
    #Run the node
        while not rospy.is_shutdown():

                    ####### WRITE YOUR CODE HERE #############
            
            #Fill the transformation message witht the pose of the robot
            pose_tf.header.stamp = rospy.Time.now()
            #pose_tf.transform.translation.x = ...
            #pose_tf.transform.translation.y = ...
            #...            
        
            #Broadcast TF's
            pose_tf_bc.sendTransform(pose_tf)

            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass