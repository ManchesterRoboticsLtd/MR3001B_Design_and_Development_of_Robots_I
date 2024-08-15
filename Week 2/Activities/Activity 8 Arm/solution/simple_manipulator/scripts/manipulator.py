#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf_conversions

# Setup parameters, transforms, variables and callback functions here (if required)

# Declare Marker messages
base_marker = Marker()
arm1_marker = Marker()
arm2_marker = Marker()
arm3_marker = Marker()
arm4_marker = Marker()
arm5_marker = Marker()

#Declare Transform Messages
baseLink_l0_tf = TransformStamped()
l0_l1_tf = TransformStamped()
l1_l2_tf = TransformStamped()
l2_l3_tf = TransformStamped()
l3_l4_tf = TransformStamped()
l4_l5_tf = TransformStamped()

#################### Functions to Initialise Markers #######################################################

#Functions to initialise markers
def init_base_marker():
    # Declare the link0 Marker Message
    base_marker.header.frame_id = "link0"
    base_marker.header.stamp = rospy.Time.now()
    base_marker.id = 0
    base_marker.type = 3
    base_marker.action = 0
    base_marker.pose.position.x = 0.0
    base_marker.pose.position.y = 0.0
    base_marker.pose.position.z = (0.123-0.05)/2
    base_marker.pose.orientation.x = 0
    base_marker.pose.orientation.y = 0
    base_marker.pose.orientation.z = 0
    base_marker.pose.orientation.w = 1
    base_marker.scale.x = 0.1
    base_marker.scale.y = 0.1
    base_marker.scale.z = 0.123-0.05
    base_marker.color.r = 0.0
    base_marker.color.g = 1.0
    base_marker.color.b = 0.0
    base_marker.color.a = 1.0
    base_marker.lifetime = rospy.Duration(0)

def init_arm1_marker():
    # Declare the link1 Marker Message
    q_m1 = tf_conversions.transformations.quaternion_from_euler(0, np.pi/2, 0)     #Rotate Marker on the y axis
    arm1_marker.header.frame_id = "link2"
    arm1_marker.header.stamp = rospy.Time.now()
    arm1_marker.id = 0
    arm1_marker.type = 3
    arm1_marker.action = 0
    arm1_marker.pose.position.x = 0.0
    arm1_marker.pose.position.y = 0.0
    arm1_marker.pose.position.z = 0.0
    arm1_marker.pose.orientation.x = q_m1[0]
    arm1_marker.pose.orientation.y = q_m1[1]
    arm1_marker.pose.orientation.z = q_m1[2]
    arm1_marker.pose.orientation.w = q_m1[3]
    arm1_marker.scale.x = 0.1
    arm1_marker.scale.y = 0.1
    arm1_marker.scale.z = 0.1
    arm1_marker.color.r = 0.0
    arm1_marker.color.g = 1.0
    arm1_marker.color.b = 0.0
    arm1_marker.color.a = 1.0
    arm1_marker.lifetime = rospy.Duration(0)

def init_arm2_marker():
    # Declare the link2 Marker Message
    arm2_marker.header.frame_id = "link2"
    arm2_marker.header.stamp = rospy.Time.now()
    arm2_marker.id = 0
    arm2_marker.type = 3
    arm2_marker.action = 0
    arm2_marker.pose.position.x = 0.0
    arm2_marker.pose.position.y = 0.0
    arm2_marker.pose.position.z = (0.185-0.1)/2+0.05
    arm2_marker.pose.orientation.x = 0
    arm2_marker.pose.orientation.y = 0
    arm2_marker.pose.orientation.z = 0
    arm2_marker.pose.orientation.w = 1
    arm2_marker.scale.x = 0.1
    arm2_marker.scale.y = 0.1
    arm2_marker.scale.z = 0.185-0.1
    arm2_marker.color.r = 0.0
    arm2_marker.color.g = 1.0
    arm2_marker.color.b = 0.0
    arm2_marker.color.a = 1.0
    arm2_marker.lifetime = rospy.Duration(0)

def init_arm3_marker():
    # Declare the link3 Marker Message
    q_m3 = tf_conversions.transformations.quaternion_from_euler(0, np.pi/2, 0)     #Rotate Marker on the y axis
    arm3_marker.header.frame_id = "link3"
    arm3_marker.header.stamp = rospy.Time.now()
    arm3_marker.id = 0
    arm3_marker.type = 3
    arm3_marker.action = 0
    arm3_marker.pose.position.x = 0.0
    arm3_marker.pose.position.y = 0.0
    arm3_marker.pose.position.z = 0.0
    arm3_marker.pose.orientation.x = q_m3[0]
    arm3_marker.pose.orientation.y = q_m3[1]
    arm3_marker.pose.orientation.z = q_m3[2]
    arm3_marker.pose.orientation.w = q_m3[3]
    arm3_marker.scale.x = 0.1
    arm3_marker.scale.y = 0.1
    arm3_marker.scale.z = 0.1
    arm3_marker.color.r = 0.0
    arm3_marker.color.g = 1.0
    arm3_marker.color.b = 0.0
    arm3_marker.color.a = 1.0
    arm3_marker.lifetime = rospy.Duration(0)

def init_arm4_marker():
    # Declare the link4 Marker Message
    q_m4 = tf_conversions.transformations.quaternion_from_euler(np.pi/2, 0, 0)     #Rotate Marker on the y axis
    arm4_marker.header.frame_id = "link3"
    arm4_marker.header.stamp = rospy.Time.now()
    arm4_marker.id = 0
    arm4_marker.type = 3
    arm4_marker.action = 0
    arm4_marker.pose.position.x = 0.0
    arm4_marker.pose.position.y = (0.2-0.1)/2+0.05
    arm4_marker.pose.position.z = 0.0
    arm4_marker.pose.orientation.x = q_m4[0]
    arm4_marker.pose.orientation.y = q_m4[1]
    arm4_marker.pose.orientation.z = q_m4[2]
    arm4_marker.pose.orientation.w = q_m4[3]
    arm4_marker.scale.x = 0.1
    arm4_marker.scale.y = 0.1
    arm4_marker.scale.z = 0.2-0.1
    arm4_marker.color.r = 0.0
    arm4_marker.color.g = 1.0
    arm4_marker.color.b = 0.0
    arm4_marker.color.a = 1.0
    arm4_marker.lifetime = rospy.Duration(0)

# Declare the arm5 Marker Message
def init_arm5_marker():
    arm5_marker.header.frame_id = "link4"
    arm5_marker.header.stamp = rospy.Time.now()
    arm5_marker.id = 0
    arm5_marker.type = 3
    arm5_marker.action = 0
    arm5_marker.pose.position.x = 0.0
    arm5_marker.pose.position.y = 0.0
    arm5_marker.pose.position.z = 0.0
    arm5_marker.pose.orientation.x = 0
    arm5_marker.pose.orientation.y = 0
    arm5_marker.pose.orientation.z = 0
    arm5_marker.pose.orientation.w = 1
    arm5_marker.scale.x = 0.1
    arm5_marker.scale.y = 0.1
    arm5_marker.scale.z = 0.1
    arm5_marker.color.r = 0.0
    arm5_marker.color.g = 1.0
    arm5_marker.color.b = 0.0
    arm5_marker.color.a = 1.0
    arm5_marker.lifetime = rospy.Duration(0)

#####################################################################################
    
############################ Initialise Transform Messages ###########################

#Functions to initialise transforms
def init_baseLink_l0_tf():
    baseLink_l0_tf.header.frame_id = "base_link"
    baseLink_l0_tf.child_frame_id = "link0"
    baseLink_l0_tf.header.stamp = rospy.Time.now()
    baseLink_l0_tf.transform.translation.x = 0
    baseLink_l0_tf.transform.translation.y = 0
    baseLink_l0_tf.transform.translation.z = 0.0
    baseLink_l0_tf.transform.rotation.x = 0
    baseLink_l0_tf.transform.rotation.y = 0
    baseLink_l0_tf.transform.rotation.z = 0
    baseLink_l0_tf.transform.rotation.w = 1

def init_l0_l1_tf():
    l0_l1_tf.header.frame_id = "link0"
    l0_l1_tf.child_frame_id = "link1"
    l0_l1_tf.header.stamp = rospy.Time.now()
    l0_l1_tf.transform.translation.x = 0
    l0_l1_tf.transform.translation.y = 0
    l0_l1_tf.transform.translation.z = 0.123
    l0_l1_tf.transform.rotation.x = 0
    l0_l1_tf.transform.rotation.y = 0
    l0_l1_tf.transform.rotation.z = 0
    l0_l1_tf.transform.rotation.w = 1


def init_l1_l2_tf():
    l1_l2_tf.header.frame_id = "link1"
    l1_l2_tf.child_frame_id = "link2"
    l1_l2_tf.header.stamp = rospy.Time.now()
    l1_l2_tf.transform.translation.x = 0
    l1_l2_tf.transform.translation.y = 0
    l1_l2_tf.transform.translation.z = 0.0
    l1_l2_tf.transform.rotation.x = 0
    l1_l2_tf.transform.rotation.y = 0
    l1_l2_tf.transform.rotation.z = 0
    l1_l2_tf.transform.rotation.w = 1

def init_l2_l3_tf():
    l2_l3_tf.header.frame_id = "link2"
    l2_l3_tf.child_frame_id = "link3"
    l2_l3_tf.header.stamp = rospy.Time.now()
    l2_l3_tf.transform.translation.x = 0
    l2_l3_tf.transform.translation.y = 0
    l2_l3_tf.transform.translation.z = 0.185
    l2_l3_tf.transform.rotation.x = 0
    l2_l3_tf.transform.rotation.y = 0
    l2_l3_tf.transform.rotation.z = 0
    l2_l3_tf.transform.rotation.w = 1

def init_l3_l4_tf():
    l3_l4_tf.header.frame_id = "link3"
    l3_l4_tf.child_frame_id = "link4"
    l3_l4_tf.header.stamp = rospy.Time.now()
    l3_l4_tf.transform.translation.x = 0
    l3_l4_tf.transform.translation.y = 0.2
    l3_l4_tf.transform.translation.z = 0
    l3_l4_tf.transform.rotation.x = 0
    l3_l4_tf.transform.rotation.y = 0
    l3_l4_tf.transform.rotation.z = 0
    l3_l4_tf.transform.rotation.w = 1


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
    init_base_marker()
    init_arm1_marker()
    init_arm2_marker()
    init_arm3_marker()
    init_arm4_marker()
    init_arm5_marker()

    init_baseLink_l0_tf()
    init_l0_l1_tf()
    init_l1_l2_tf()
    init_l2_l3_tf()
    init_l3_l4_tf()

    #Setup Publishers, subscribers 
    pub_link0 = rospy.Publisher('/link0', Marker, queue_size=1)
    pub_link1 = rospy.Publisher('/link1', Marker, queue_size=1)
    pub_link2 = rospy.Publisher('/link2', Marker, queue_size=1)
    pub_link3 = rospy.Publisher('/link3', Marker, queue_size=1)
    pub_link4 = rospy.Publisher('/link4', Marker, queue_size=1)
    pub_link5 = rospy.Publisher('/link5', Marker, queue_size=1)

    bc_baselink = StaticTransformBroadcaster()
    bc_l0 = TransformBroadcaster()
    bc_l1 = TransformBroadcaster()
    bc_l2 = TransformBroadcaster()
    bc_l3 = TransformBroadcaster()
    bc_l4 = TransformBroadcaster()
    bc_l5 = TransformBroadcaster()

    #Setup transform broadcasters here


    print("The tf's are ready")

    try:
        #Run the node
        while not rospy.is_shutdown(): 
            
            #Declare time variable
            t = rospy.Time.now().to_sec()

            #Update the markers time stamp
            base_marker.header.stamp = rospy.Time.now()
            arm1_marker.header.stamp = rospy.Time.now()
            arm2_marker.header.stamp = rospy.Time.now()
            arm3_marker.header.stamp = rospy.Time.now()
            arm4_marker.header.stamp = rospy.Time.now()
            arm5_marker.header.stamp = rospy.Time.now()          

            # Define the transform movements
            q_l1 = tf_conversions.transformations.quaternion_from_euler(0, 0, 0.5*np.sin(0.5*t))
            q_l2 = tf_conversions.transformations.quaternion_from_euler(0.5*np.sin(t), 0, 0)
            q_l3 = tf_conversions.transformations.quaternion_from_euler(0.5*np.cos(t), 0, 0)
            q_l4 = tf_conversions.transformations.quaternion_from_euler(0, 0.5*np.cos(t), 0)


            #Update the transforms time stamps and movements
            baseLink_l0_tf.header.stamp = rospy.Time.now()

            l0_l1_tf.header.stamp = rospy.Time.now()
            l0_l1_tf.transform.rotation.x = q_l1[0]
            l0_l1_tf.transform.rotation.y = q_l1[1]
            l0_l1_tf.transform.rotation.z = q_l1[2]
            l0_l1_tf.transform.rotation.w = q_l1[3]
            
            l1_l2_tf.header.stamp = rospy.Time.now()
            l1_l2_tf.transform.rotation.x = q_l2[0]
            l1_l2_tf.transform.rotation.y = q_l2[1]
            l1_l2_tf.transform.rotation.z = q_l2[2]
            l1_l2_tf.transform.rotation.w = q_l2[3]

            l2_l3_tf.header.stamp = rospy.Time.now()
            l2_l3_tf.transform.rotation.x = q_l3[0]
            l2_l3_tf.transform.rotation.y = q_l3[1]
            l2_l3_tf.transform.rotation.z = q_l3[2]
            l2_l3_tf.transform.rotation.w = q_l3[3]

            l3_l4_tf.header.stamp = rospy.Time.now()
            l3_l4_tf.transform.rotation.x = q_l4[0]
            l3_l4_tf.transform.rotation.y = q_l4[1]
            l3_l4_tf.transform.rotation.z = q_l4[2]
            l3_l4_tf.transform.rotation.w = q_l4[3]
            
 
            #Broadcast transforms
            bc_baselink.sendTransform(baseLink_l0_tf)
            bc_l0.sendTransform(l0_l1_tf)
            bc_l1.sendTransform(l1_l2_tf)
            bc_l2.sendTransform(l2_l3_tf)
            bc_l3.sendTransform(l3_l4_tf)

            #Publish markers
            pub_link0.publish(base_marker)
            pub_link1.publish(arm1_marker)
            pub_link2.publish(arm2_marker)
            pub_link3.publish(arm3_marker)
            pub_link4.publish(arm4_marker)
            pub_link5.publish(arm5_marker)

            loop_rate.sleep()
 
    except rospy.ROSInterruptException:
        pass