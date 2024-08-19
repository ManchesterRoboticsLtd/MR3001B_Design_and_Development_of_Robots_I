#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf_conversions

# Setup parameters, transforms, variables and callback functions here (if required)
# Declare message or transform.
link0_marker = Marker()
link1_marker = Marker()
link2_marker = Marker()
link3_marker = Marker()
link4_marker = Marker()
link5_marker = Marker()
link6_marker = Marker()


baseLink_l0_tf = TransformStamped()
l0_l1_tf = TransformStamped()
l1_l2_tf = TransformStamped()
l2_l3_tf = TransformStamped()
l3_l4_tf = TransformStamped()
l4_l5_tf = TransformStamped()
l5_l6_tf = TransformStamped()

####################  Initialise Markers #######################################################

def init_link0_marker():
    # Declare the link0 Marker Message
    link0_marker.header.frame_id = "link0"
    link0_marker.header.stamp = rospy.Time.now()
    link0_marker.id = 0
    link0_marker.type = 10
    link0_marker.action = 0
    link0_marker.pose.position.x = 0.0
    link0_marker.pose.position.y = 0.0
    link0_marker.pose.position.z = 0.0
    link0_marker.pose.orientation.x = 0
    link0_marker.pose.orientation.y = 0
    link0_marker.pose.orientation.z = 0
    link0_marker.pose.orientation.w = 1
    link0_marker.scale.x = 0.001
    link0_marker.scale.y = 0.001
    link0_marker.scale.z = 0.001
    link0_marker.color.r = 0.0
    link0_marker.color.g = 1.0
    link0_marker.color.b = 0.0
    link0_marker.color.a = 1.0
    link0_marker.mesh_resource = "package://abb_irb_1010/models/link0.stl"
    link0_marker.lifetime = rospy.Duration(0)

def init_link1_marker():
    # Declare the link1 Marker Message
    link1_marker.header.frame_id = "link1"
    link1_marker.header.stamp = rospy.Time.now()
    link1_marker.id = 0
    link1_marker.type = 10
    link1_marker.action = 0
    link1_marker.pose.position.x = 0.0
    link1_marker.pose.position.y = 0.0
    link1_marker.pose.position.z = -0.123
    link1_marker.pose.orientation.x = 0
    link1_marker.pose.orientation.y = 0
    link1_marker.pose.orientation.z = 0
    link1_marker.pose.orientation.w = 1
    link1_marker.scale.x = 0.001
    link1_marker.scale.y = 0.001
    link1_marker.scale.z = 0.001
    link1_marker.color.r = 0.0
    link1_marker.color.g = 1.0
    link1_marker.color.b = 0.0
    link1_marker.color.a = 1.0
    link1_marker.mesh_resource = "package://abb_irb_1010/models/link1.stl"
    link1_marker.lifetime = rospy.Duration(0)

def init_link2_marker():
    # Declare the link2 Marker Message
    link2_marker.header.frame_id = "link2"
    link2_marker.header.stamp = rospy.Time.now()
    link2_marker.id = 0
    link2_marker.type = 10
    link2_marker.action = 0
    link2_marker.pose.position.x = 0.0
    link2_marker.pose.position.y = 0.0
    link2_marker.pose.position.z = -0.19
    link2_marker.pose.orientation.x = 0
    link2_marker.pose.orientation.y = 0
    link2_marker.pose.orientation.z = 0
    link2_marker.pose.orientation.w = 1
    link2_marker.scale.x = 0.001
    link2_marker.scale.y = 0.001
    link2_marker.scale.z = 0.001
    link2_marker.color.r = 0.0
    link2_marker.color.g = 1.0
    link2_marker.color.b = 0.0
    link2_marker.color.a = 1.0
    link2_marker.mesh_resource = "package://abb_irb_1010/models/link2.stl"
    link2_marker.lifetime = rospy.Duration(0)

def init_link3_marker():
    # Declare the link3 Marker Message
    link3_marker.header.frame_id = "link3"
    link3_marker.header.stamp = rospy.Time.now()
    link3_marker.id = 0
    link3_marker.type = 10
    link3_marker.action = 0
    link3_marker.pose.position.x = 0.0
    link3_marker.pose.position.y = 0.0
    link3_marker.pose.position.z = -0.375
    link3_marker.pose.orientation.x = 0
    link3_marker.pose.orientation.y = 0
    link3_marker.pose.orientation.z = 0
    link3_marker.pose.orientation.w = 1
    link3_marker.scale.x = 0.001
    link3_marker.scale.y = 0.001
    link3_marker.scale.z = 0.001
    link3_marker.color.r = 0.0
    link3_marker.color.g = 1.0
    link3_marker.color.b = 0.0
    link3_marker.color.a = 1.0
    link3_marker.mesh_resource = "package://abb_irb_1010/models/link3.stl"
    link3_marker.lifetime = rospy.Duration(0)

def init_link4_marker():
    # Declare the link4 Marker Message
    link4_marker.header.frame_id = "link4"
    link4_marker.header.stamp = rospy.Time.now()
    link4_marker.id = 0
    link4_marker.type = 10
    link4_marker.action = 0
    link4_marker.pose.position.x = 0.0
    link4_marker.pose.position.y = -0.0593
    link4_marker.pose.position.z = -0.375
    link4_marker.pose.orientation.x = 0
    link4_marker.pose.orientation.y = 0
    link4_marker.pose.orientation.z = 0
    link4_marker.pose.orientation.w = 1
    link4_marker.scale.x = 0.001
    link4_marker.scale.y = 0.001
    link4_marker.scale.z = 0.001
    link4_marker.color.r = 0.0
    link4_marker.color.g = 1.0
    link4_marker.color.b = 0.0
    link4_marker.color.a = 1.0
    link4_marker.mesh_resource = "package://abb_irb_1010/models/link4.stl"
    link4_marker.lifetime = rospy.Duration(0)

def init_link5_marker():
    # Declare the link2 Marker Message
    link5_marker.header.frame_id = "link5"
    link5_marker.header.stamp = rospy.Time.now()
    link5_marker.id = 0
    link5_marker.type = 10
    link5_marker.action = 0
    link5_marker.pose.position.x = 0.0
    link5_marker.pose.position.y = -0.185
    link5_marker.pose.position.z = -0.375
    link5_marker.pose.orientation.x = 0
    link5_marker.pose.orientation.y = 0
    link5_marker.pose.orientation.z = 0
    link5_marker.pose.orientation.w = 1
    link5_marker.scale.x = 0.001
    link5_marker.scale.y = 0.001
    link5_marker.scale.z = 0.001
    link5_marker.color.r = 0.0
    link5_marker.color.g = 1.0
    link5_marker.color.b = 0.0
    link5_marker.color.a = 1.0
    link5_marker.mesh_resource = "package://abb_irb_1010/models/link5.stl"
    link5_marker.lifetime = rospy.Duration(0)


def init_link6_marker():
    # Declare the link6 Marker Message
    link6_marker.header.frame_id = "link6"
    link6_marker.header.stamp = rospy.Time.now()
    link6_marker.id = 0
    link6_marker.type = 10
    link6_marker.action = 0
    link6_marker.pose.position.x = 0.0
    link6_marker.pose.position.y = -0.2278
    link6_marker.pose.position.z = -0.375
    link6_marker.pose.orientation.x = 0
    link6_marker.pose.orientation.y = 0
    link6_marker.pose.orientation.z = 0
    link6_marker.pose.orientation.w = 1
    link6_marker.scale.x = 0.001
    link6_marker.scale.y = 0.001
    link6_marker.scale.z = 0.001
    link6_marker.color.r = 0.0
    link6_marker.color.g = 1.0
    link6_marker.color.b = 0.0
    link6_marker.color.a = 1.0
    link6_marker.mesh_resource = "package://abb_irb_1010/models/link6.stl"
    link6_marker.lifetime = rospy.Duration(0)
#####################################################################################
    
############################ Initialise Transform Messages ###########################
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
    l1_l2_tf.transform.translation.z = 0.067
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
    l3_l4_tf.transform.translation.y = 0.0593
    l3_l4_tf.transform.translation.z = 0
    l3_l4_tf.transform.rotation.x = 0
    l3_l4_tf.transform.rotation.y = 0
    l3_l4_tf.transform.rotation.z = 0
    l3_l4_tf.transform.rotation.w = 1

def init_l4_l5_tf():
    l4_l5_tf.header.frame_id = "link4"
    l4_l5_tf.child_frame_id = "link5"
    l4_l5_tf.header.stamp = rospy.Time.now()
    l4_l5_tf.transform.translation.x = 0
    l4_l5_tf.transform.translation.y = 0.1257
    l4_l5_tf.transform.translation.z = 0
    l4_l5_tf.transform.rotation.x = 0
    l4_l5_tf.transform.rotation.y = 0
    l4_l5_tf.transform.rotation.z = 0
    l4_l5_tf.transform.rotation.w = 1

def init_l5_l6_tf():
    l5_l6_tf.header.frame_id = "link5"
    l5_l6_tf.child_frame_id = "link6"
    l5_l6_tf.header.stamp = rospy.Time.now()
    l5_l6_tf.transform.translation.x = 0
    l5_l6_tf.transform.translation.y = 0.0428
    l5_l6_tf.transform.translation.z = 0
    l5_l6_tf.transform.rotation.x = 0
    l5_l6_tf.transform.rotation.y = 0
    l5_l6_tf.transform.rotation.z = 0
    l5_l6_tf.transform.rotation.w = 1

#############################################################################

#Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")

if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("ABB_IRB_1010")
 
    # Configure the Node
    loop_rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup the messages and transforms
    init_link0_marker()
    init_link1_marker()
    init_link2_marker()
    init_link3_marker()
    init_link4_marker()
    init_link5_marker()
    init_link6_marker()

    init_baseLink_l0_tf()
    init_l0_l1_tf()
    init_l1_l2_tf()
    init_l2_l3_tf()
    init_l3_l4_tf()
    init_l4_l5_tf()
    init_l5_l6_tf()

    #Setup Publishers, subscribers and transform broadcasters here
    bc_baselink = StaticTransformBroadcaster()
    bc_l0 = TransformBroadcaster()
    bc_l1 = TransformBroadcaster()
    bc_l2 = TransformBroadcaster()
    bc_l3 = TransformBroadcaster()
    bc_l4 = TransformBroadcaster()
    bc_l5 = TransformBroadcaster()

    #Setup Publishers, subscribers and transform broadcasters here
    pub_link0 = rospy.Publisher('/link0', Marker, queue_size=1)
    pub_link1 = rospy.Publisher('/link1', Marker, queue_size=1)
    pub_link2 = rospy.Publisher('/link2', Marker, queue_size=1)
    pub_link3 = rospy.Publisher('/link3', Marker, queue_size=1)
    pub_link4 = rospy.Publisher('/link4', Marker, queue_size=1)
    pub_link5 = rospy.Publisher('/link5', Marker, queue_size=1)
    pub_link6 = rospy.Publisher('/link6', Marker, queue_size=1)

    print("The tf's are ready")

    try:
        #Run the node
        while not rospy.is_shutdown(): 
            
            t = rospy.Time.now().to_sec()

            #Update the markers
            link0_marker.header.stamp = rospy.Time.now()
            link1_marker.header.stamp = rospy.Time.now()
            link2_marker.header.stamp = rospy.Time.now()
            link3_marker.header.stamp = rospy.Time.now()
            link4_marker.header.stamp = rospy.Time.now()
            link5_marker.header.stamp = rospy.Time.now()
            link6_marker.header.stamp = rospy.Time.now()


            # Rotation of the transforms
            q_l1 = tf_conversions.transformations.quaternion_from_euler(0, 0, 0.5*np.sin(0.5*t))
            q_l2 = tf_conversions.transformations.quaternion_from_euler(0.5*np.sin(t), 0, 0)
            q_l3 = tf_conversions.transformations.quaternion_from_euler(0.5*np.cos(t), 0, 0)
            q_l4 = tf_conversions.transformations.quaternion_from_euler(0, 0.5*np.cos(t), 0)
            q_l5 = tf_conversions.transformations.quaternion_from_euler(0.3*np.cos(t), 0, 0)
            q_l6 = tf_conversions.transformations.quaternion_from_euler(0, 0.5*np.cos(t), 0)


            #Update the transforms
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

            l4_l5_tf.header.stamp = rospy.Time.now()
            l4_l5_tf.transform.rotation.x = q_l5[0]
            l4_l5_tf.transform.rotation.y = q_l5[1]
            l4_l5_tf.transform.rotation.z = q_l5[2]
            l4_l5_tf.transform.rotation.w = q_l5[3]

            l5_l6_tf.header.stamp = rospy.Time.now()
            l5_l6_tf.transform.rotation.x = q_l6[0]
            l5_l6_tf.transform.rotation.y = q_l6[1]
            l5_l6_tf.transform.rotation.z = q_l6[2]
            l5_l6_tf.transform.rotation.w = q_l6[3]

            #Broadcast transforms
            bc_baselink.sendTransform(baseLink_l0_tf)
            bc_l0.sendTransform(l0_l1_tf)
            bc_l1.sendTransform(l1_l2_tf)
            bc_l2.sendTransform(l2_l3_tf)
            bc_l3.sendTransform(l3_l4_tf)
            bc_l4.sendTransform(l4_l5_tf)
            bc_l5.sendTransform(l5_l6_tf)

            #Publish markers
            pub_link0.publish(link0_marker)
            pub_link1.publish(link1_marker)
            pub_link2.publish(link2_marker)
            pub_link3.publish(link3_marker)
            pub_link4.publish(link4_marker)
            pub_link5.publish(link5_marker)
            pub_link6.publish(link6_marker)


            loop_rate.sleep()
 
    except rospy.ROSInterruptException:
        pass
