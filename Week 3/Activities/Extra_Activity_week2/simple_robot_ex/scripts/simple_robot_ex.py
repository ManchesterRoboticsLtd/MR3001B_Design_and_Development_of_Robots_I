#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf_conversions

# Setup parameters, transforms, variables and callback functions here (if required)
# Declare message or transform.
lidarMarker = Marker()
chassis = Marker()
chassis2 = Marker()
caster = Marker()
caster2 = Marker()
wheelR = Marker()
wheelL = Marker()

base_link_tf = TransformStamped()
chassis_link_tf = TransformStamped()
lidar_tf = TransformStamped()
wheelR_tf = TransformStamped()
wheelL_tf = TransformStamped()

q_wR = tf_conversions.transformations.quaternion_from_euler(1.57, 0.0, 0)
q_wL = tf_conversions.transformations.quaternion_from_euler(1.57, 0.0, 0)
q_Lidar = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, np.pi)


#init Sun Marker
def init_chassisMarker():
    chassis.header.frame_id = "chassis"
    chassis.header.stamp = rospy.Time.now()
    chassis.id = 0
    chassis.type = 1
    chassis.action = 0
    chassis.pose.position.x = 0.205/2
    chassis.pose.position.y = 0.0
    chassis.pose.position.z = 0.0
    chassis.pose.orientation.x = 0.0
    chassis.pose.orientation.y = 0.0
    chassis.pose.orientation.z = 0.0
    chassis.pose.orientation.w = 1.0
    chassis.scale.x = 0.205
    chassis.scale.y = 0.41
    chassis.scale.z = 0.2
    chassis.color.r = 1.0
    chassis.color.g = 1.0
    chassis.color.b = 0.0
    chassis.color.a = 1.0
    chassis.lifetime = rospy.Duration(0)

def init_chassisMarker2():
    chassis2.header.frame_id = "chassis"
    chassis2.header.stamp = rospy.Time.now()
    chassis2.id = 0
    chassis2.type = 3
    chassis2.action = 0
    chassis2.pose.position.x = 0.0
    chassis2.pose.position.y = 0.0
    chassis2.pose.position.z = 0.0
    chassis2.pose.orientation.x = 0.0
    chassis2.pose.orientation.y = 0.0
    chassis2.pose.orientation.z = 0.0
    chassis2.pose.orientation.w = 1.0
    chassis2.scale.x = 0.410
    chassis2.scale.y = 0.410
    chassis2.scale.z = 0.2
    chassis2.color.r = 1.0
    chassis2.color.g = 1.0
    chassis2.color.b = 0.0
    chassis2.color.a = 1.0
    chassis2.lifetime = rospy.Duration(0)

def init_lidarMarker():
    lidarMarker.header.frame_id = "laser"
    lidarMarker.header.stamp = rospy.Time.now()
    lidarMarker.id = 0
    lidarMarker.type = 3
    lidarMarker.action = 0
    lidarMarker.pose.position.x = 0
    lidarMarker.pose.position.y = 0.0
    lidarMarker.pose.position.z = -0.025
    lidarMarker.pose.orientation.x = 0.0
    lidarMarker.pose.orientation.y = 0.0
    lidarMarker.pose.orientation.z = 0.0
    lidarMarker.pose.orientation.w = 1.0
    lidarMarker.scale.x = 0.1
    lidarMarker.scale.y = 0.1
    lidarMarker.scale.z = 0.05
    lidarMarker.color.r = 1.0
    lidarMarker.color.g = 0.0
    lidarMarker.color.b = 0.0
    lidarMarker.color.a = 1.0
    lidarMarker.lifetime = rospy.Duration(0)

def init_casterMarker1():
    caster.header.frame_id = "chassis"
    caster.header.stamp = rospy.Time.now()
    caster.id = 0
    caster.type = 2
    caster.action = 0
    caster.pose.position.x = -0.102
    caster.pose.position.y = 0.08
    caster.pose.position.z = -0.08625
    caster.pose.orientation.x = 0.0
    caster.pose.orientation.y = 0.0
    caster.pose.orientation.z = 0.0
    caster.pose.orientation.w = 1.0
    caster.scale.x = 0.1
    caster.scale.y = 0.1
    caster.scale.z = 0.1
    caster.color.r = 1.0
    caster.color.g = 1.0
    caster.color.b = 0.0
    caster.color.a = 1.0
    caster.lifetime = rospy.Duration(0)

def init_casterMarker2():
    caster2.header.frame_id = "chassis"
    caster2.header.stamp = rospy.Time.now()
    caster2.id = 0
    caster2.type = 2
    caster2.action = 0
    caster2.pose.position.x = -0.102
    caster2.pose.position.y = -0.08
    caster2.pose.position.z = -0.08625
    caster2.pose.orientation.x = 0.0
    caster2.pose.orientation.y = 0.0
    caster2.pose.orientation.z = 0.0
    caster2.pose.orientation.w = 1.0
    caster2.scale.x = 0.1
    caster2.scale.y = 0.1
    caster2.scale.z = 0.1
    caster2.color.r = 1.0
    caster2.color.g = 1.0
    caster2.color.b = 0.0
    caster2.color.a = 1.0
    caster2.lifetime = rospy.Duration(0)


def init_wheelRMarker():
    # Declare the planet 1 Marker Message
    wheelR.header.frame_id = "wheelR"
    wheelR.header.stamp = rospy.Time.now()
    wheelR.id = 0
    wheelR.type = 3
    wheelR.action = 0
    wheelR.pose.position.x = 0.0
    wheelR.pose.position.y = 0.0
    wheelR.pose.position.z = 0.0
    wheelR.pose.orientation.x = 0.0
    wheelR.pose.orientation.y = 0.0
    wheelR.pose.orientation.z = 0.0
    wheelR.pose.orientation.w = 1.0
    wheelR.scale.x = 0.124
    wheelR.scale.y = 0.124
    wheelR.scale.z = 0.038
    wheelR.color.r = 0.0
    wheelR.color.g = 1.0
    wheelR.color.b = 0.0
    wheelR.color.a = 1.0
    wheelR.lifetime = rospy.Duration(0)

def init_wheelLMarker():
    # # Declare the moon 1 Marker Message
    wheelL.header.frame_id = "wheelL"
    wheelL.header.stamp = rospy.Time.now()
    wheelL.id = 1
    wheelL.type = 3
    wheelL.action = 0
    wheelL.pose.position.x = 0.0
    wheelL.pose.position.y = 0.0
    wheelL.pose.position.z = 0.0
    wheelL.pose.orientation.x = 0.0
    wheelL.pose.orientation.y = 0.0
    wheelL.pose.orientation.z = 0.0
    wheelL.pose.orientation.w = 1.0
    wheelL.scale.x = 0.124
    wheelL.scale.y = 0.124
    wheelL.scale.z = 0.05
    wheelL.color.r = 0.0
    wheelL.color.g = 0.0
    wheelL.color.b = 1.0
    wheelL.color.a = 1.0
    wheelL.lifetime = rospy.Duration(0)
  

# Initialise Planet Transform Message
def init_base_linkTransform():
    base_link_tf.header.frame_id = "world"
    base_link_tf.child_frame_id = "base_link"
    base_link_tf.header.stamp = rospy.Time.now()
    base_link_tf.transform.translation.x = 0
    base_link_tf.transform.translation.y = 0
    base_link_tf.transform.translation.z = 0.0
    base_link_tf.transform.rotation.x = 0
    base_link_tf.transform.rotation.y = 0
    base_link_tf.transform.rotation.z = 0
    base_link_tf.transform.rotation.w = 1

def init_chassis_linkTransform():
    chassis_link_tf.header.frame_id = "base_link"
    chassis_link_tf.child_frame_id = "chassis"
    chassis_link_tf.header.stamp = rospy.Time.now()
    chassis_link_tf.transform.translation.x = 0
    chassis_link_tf.transform.translation.y = 0
    chassis_link_tf.transform.translation.z = 0.127
    chassis_link_tf.transform.rotation.x = 0
    chassis_link_tf.transform.rotation.y = 0
    chassis_link_tf.transform.rotation.z = 0
    chassis_link_tf.transform.rotation.w = 1

def init_lidarTransform():
    lidar_tf.header.frame_id = "chassis"
    lidar_tf.child_frame_id = "laser"
    lidar_tf.header.stamp = rospy.Time.now()
    lidar_tf.transform.translation.x = -0.0498
    lidar_tf.transform.translation.y = 0
    lidar_tf.transform.translation.z = 0.15
    lidar_tf.transform.rotation.x = q_Lidar[0]
    lidar_tf.transform.rotation.y = q_Lidar[1]
    lidar_tf.transform.rotation.z = q_Lidar[2]
    lidar_tf.transform.rotation.w = q_Lidar[3]


def init_wheelRTransform():
    wheelR_tf.header.frame_id = "base_link"
    wheelR_tf.child_frame_id = "wheelR"
    wheelR_tf.header.stamp = rospy.Time.now()
    wheelR_tf.transform.translation.x = 0.11
    wheelR_tf.transform.translation.y = 0.146
    wheelR_tf.transform.translation.z = 0.062
    wheelR_tf.transform.rotation.x = q_wR[0]
    wheelR_tf.transform.rotation.y = q_wR[1]
    wheelR_tf.transform.rotation.z = q_wR[2]
    wheelR_tf.transform.rotation.w = q_wR[3]

def init_wheelLTransform():
    wheelL_tf.header.frame_id = "base_link"
    wheelL_tf.child_frame_id = "wheelL"
    wheelL_tf.header.stamp = rospy.Time.now()
    wheelL_tf.transform.translation.x = 0.11
    wheelL_tf.transform.translation.y = -0.146
    wheelL_tf.transform.translation.z = 0.062
    wheelL_tf.transform.rotation.x = q_wL[0]
    wheelL_tf.transform.rotation.y = q_wL[1]
    wheelL_tf.transform.rotation.z = q_wL[2]
    wheelL_tf.transform.rotation.w = q_wL[3]


#Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")

if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("EAI_Robot")
 
    # Configure the Node
    loop_rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup the messages and transforms
    init_chassisMarker()
    init_chassisMarker2()
    init_casterMarker1()
    init_casterMarker2()
    init_lidarMarker()
    init_wheelRMarker()
    init_wheelLMarker()
    init_base_linkTransform()
    init_lidarTransform()
    init_chassis_linkTransform()
    init_wheelRTransform()
    init_wheelLTransform()

    #Setup Publishers, subscribers and transform broadcasters here
    pub_chassis = rospy.Publisher('/chassis', Marker, queue_size=1)
    pub_chassis2 = rospy.Publisher('/chassis2', Marker, queue_size=1)
    pub_lidar = rospy.Publisher('/lidar_marker', Marker, queue_size=1)
    pub_caster1 = rospy.Publisher('/caster1', Marker, queue_size=1)
    pub_caster2 = rospy.Publisher('/caster2', Marker, queue_size=1)
    pub_wheelR = rospy.Publisher('/wheelR', Marker, queue_size=1)
    pub_wheelL = rospy.Publisher('/WheelL', Marker, queue_size=1)



    bc_baselink = TransformBroadcaster()
    bc_chassis = StaticTransformBroadcaster()
    bc_lidar = StaticTransformBroadcaster()
    bc_wheelR = TransformBroadcaster()
    bc_wheelL = TransformBroadcaster()      
 
    print("The tf's are ready")

    try:
        #Run the node
        while not rospy.is_shutdown(): 
            
            t = rospy.Time.now().to_sec()
            
            chassis.header.stamp = rospy.Time.now()
            chassis2.header.stamp = rospy.Time.now()
            caster.header.stamp = rospy.Time.now()
            caster2.header.stamp = rospy.Time.now()
            lidarMarker.header.stamp = rospy.Time.now()
            wheelR.header.stamp = rospy.Time.now()
            wheelL.header.stamp = rospy.Time.now()

            #Update Transformations
            q_w = tf_conversions.transformations.quaternion_from_euler(0, 0.0, -0.05*t)
            base_link_tf.header.stamp = rospy.Time.now()
            base_link_tf.transform.translation.x = 3*np.sin(0.05*t)
            base_link_tf.transform.translation.y = 3*np.cos(0.05*t)
            base_link_tf.transform.translation.z = 0.0
            base_link_tf.transform.rotation.x = q_w[0]
            base_link_tf.transform.rotation.y = q_w[1]
            base_link_tf.transform.rotation.z = q_w[2]
            base_link_tf.transform.rotation.w = q_w[3]


            q_wR = tf_conversions.transformations.quaternion_from_euler(1.57, t*(3 + 0.33 * -0.05 / 2.0) / 0.124, 0.0)
            q_wL = tf_conversions.transformations.quaternion_from_euler(1.57, t*(3 - 0.33 * -0.05 / 2.0) / 0.124, 0.0)

            wheelR_tf.header.stamp = rospy.Time.now()
            wheelR_tf.transform.rotation.x = q_wR[0]
            wheelR_tf.transform.rotation.y = q_wR[1]
            wheelR_tf.transform.rotation.z = q_wR[2]
            wheelR_tf.transform.rotation.w = q_wR[3]


            wheelL_tf.header.stamp = rospy.Time.now()
            wheelL_tf.transform.rotation.x = q_wL[0]
            wheelL_tf.transform.rotation.y = q_wL[1]
            wheelL_tf.transform.rotation.z = q_wL[2]
            wheelL_tf.transform.rotation.w = q_wL[3]

            lidar_tf.header.stamp = rospy.Time.now()
            chassis_link_tf.header.stamp = rospy.Time.now()



            bc_baselink.sendTransform(base_link_tf)
            bc_chassis.sendTransform(chassis_link_tf)
            bc_lidar.sendTransform(lidar_tf)
            bc_wheelL.sendTransform(wheelR_tf)
            bc_wheelR.sendTransform(wheelL_tf)

            pub_wheelL.publish(wheelL)
            pub_wheelR.publish(wheelR)
            pub_chassis.publish(chassis)
            pub_chassis2.publish(chassis2)
            pub_caster1.publish(caster)
            pub_caster2.publish(caster2)
            pub_lidar.publish(lidarMarker)

            loop_rate.sleep()
 
    except rospy.ROSInterruptException:
        pass
