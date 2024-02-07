#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf_conversions

# Setup parameters, transforms, variables and callback functions here (if required)
# Declare message or transform
sun = Marker()
planet1 = Marker()
moon1 = Marker()
arrow = Marker()

inertial_tf = TransformStamped()
planet1_tf = TransformStamped()
moon1_tf = TransformStamped()


#init Sun Marker
def init_sunMarker():
    sun.header.frame_id = "sun"
    sun.header.stamp = rospy.Time.now()
    sun.id = 0
    sun.type = 2
    sun.action = 0
    sun.pose.position.x = 0.0
    sun.pose.position.y = 0.0
    sun.pose.position.z = 0.0
    sun.pose.orientation.x = 0.0
    sun.pose.orientation.y = 0.0
    sun.pose.orientation.z = 0.0
    sun.pose.orientation.w = 1.0
    sun.scale.x = 2.0
    sun.scale.y = 2.0
    sun.scale.z = 2.0
    sun.color.r = 1.0
    sun.color.g = 1.0
    sun.color.b = 0.0
    sun.color.a = 1.0
    sun.lifetime = rospy.Duration(0)

def init_planet1Marker():
        # Declare the planet 1 Marker Message
    planet1.header.frame_id = "sun"
    planet1.header.stamp = rospy.Time.now()
    planet1.id = 0
    planet1.type = 2
    planet1.action = 0
    planet1.pose.position.x = 0.0
    planet1.pose.position.y = 0.0
    planet1.pose.position.z = 0.0
    planet1.pose.orientation.x = 0.0
    planet1.pose.orientation.y = 0.0
    planet1.pose.orientation.z = 0.0
    planet1.pose.orientation.w = 1.0
    planet1.scale.x = 1.0
    planet1.scale.y = 1.0
    planet1.scale.z = 1.0
    planet1.color.r = 0.0
    planet1.color.g = 1.0
    planet1.color.b = 0.0
    planet1.color.a = 1.0
    planet1.lifetime = rospy.Duration(0)

def init_moon1Marker():
    # # Declare the moon 1 Marker Message
    moon1.header.frame_id = "planet"
    moon1.header.stamp = rospy.Time.now()
    moon1.id = 1
    moon1.type = 2
    moon1.action = 0
    moon1.pose.position.x = 0.0
    moon1.pose.position.y = 0.0
    moon1.pose.position.z = 0.0
    moon1.pose.orientation.x = 0.0
    moon1.pose.orientation.y = 0.0
    moon1.pose.orientation.z = 0.0
    moon1.pose.orientation.w = 1.0
    moon1.scale.x = 0.5
    moon1.scale.y = 0.5
    moon1.scale.z = 0.5
    moon1.color.r = 0.0
    moon1.color.g = 0.0
    moon1.color.b = 1.0
    moon1.color.a = 1.0
    moon1.lifetime = rospy.Duration(0)

def init_arrowMarker():
    # Declare the Arrow Marker
    q = tf_conversions.transformations.quaternion_from_euler(0.0, np.pi/4, np.pi/4)
    arrow.header.frame_id = "moon"
    arrow.header.stamp = rospy.Time.now()
    arrow.id = 0
    arrow.type = 0
    arrow.action = 0
    arrow.pose.position.x = -0.55
    arrow.pose.position.y = -0.55
    arrow.pose.position.z = 0.55
    arrow.pose.orientation.x = q[0]
    arrow.pose.orientation.y = q[1]
    arrow.pose.orientation.z = q[2]
    arrow.pose.orientation.w = q[3]
    arrow.scale.x = 0.7
    arrow.scale.y = 0.1
    arrow.scale.z = 0.1
    arrow.color.r = 1.0
    arrow.color.g = 0.0
    arrow.color.b = 0.0
    arrow.color.a = 1.0
    arrow.lifetime = rospy.Duration(0)     

# Initialise Planet Transform Message
def init_inertialTransform():
    inertial_tf.header.frame_id = "inertial_frame"
    inertial_tf.child_frame_id = "sun"
    inertial_tf.header.stamp = rospy.Time.now()
    inertial_tf.transform.translation.x = 0
    inertial_tf.transform.translation.y = 0
    inertial_tf.transform.translation.z = 0.0
    inertial_tf.transform.rotation.x = 0
    inertial_tf.transform.rotation.y = 0
    inertial_tf.transform.rotation.z = 0
    inertial_tf.transform.rotation.w = 1

def init_planet1Transform():
    planet1_tf.header.frame_id = "sun"
    planet1_tf.child_frame_id = "planet"
    planet1_tf.header.stamp = rospy.Time.now()
    planet1_tf.transform.translation.x = 0
    planet1_tf.transform.translation.y = 0
    planet1_tf.transform.translation.z = 0.0
    planet1_tf.transform.rotation.x = 0
    planet1_tf.transform.rotation.y = 0
    planet1_tf.transform.rotation.z = 0
    planet1_tf.transform.rotation.w = 1

def init_moon1Transform():
    moon1_tf.header.frame_id = "planet"
    moon1_tf.child_frame_id = "moon"
    moon1_tf.header.stamp = rospy.Time.now()
    moon1_tf.transform.translation.x = 0
    moon1_tf.transform.translation.y = 0
    moon1_tf.transform.translation.z = 0.0
    moon1_tf.transform.rotation.x = 0
    moon1_tf.transform.rotation.y = 0
    moon1_tf.transform.rotation.z = 0
    moon1_tf.transform.rotation.w = 1


#Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")

if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("RVIZ_marker")
 
    # Configure the Node
    loop_rate = rospy.Rate(10)
    rospy.on_shutdown(stop)

    #Setup the messages and transforms
    init_sunMarker()
    init_planet1Marker()
    init_moon1Marker()
    init_arrowMarker()
    init_inertialTransform()
    init_planet1Transform()
    init_moon1Transform()

    #Setup Publishers, subscribers and transform broadcasters here
    pub_sun = rospy.Publisher('/sun', Marker, queue_size=1)
    pub_planet1 = rospy.Publisher('/planet1', Marker, queue_size=1)
    pub_moon1 = rospy.Publisher('/moon1', Marker, queue_size=1)
    pub_arrow = rospy.Publisher('/arrow', Marker, queue_size=1)

    bc_planet = TransformBroadcaster()
    bc_moon = TransformBroadcaster()
    bc_inertial = StaticTransformBroadcaster()      
 
    print("The tf's are ready")

    try:
        #Run the node
        while not rospy.is_shutdown(): 
            t = rospy.Time.now().to_sec()
            
            planet1.header.stamp = rospy.Time.now()
            planet1.pose.position.x = 3*np.sin(t)
            planet1.pose.position.y = 3*np.cos(t)

            moon1.header.stamp = rospy.Time.now()
            moon1.pose.position.x = np.sin(1.8*t)
            moon1.pose.position.y = np.cos(1.8*t)

            arrow.header.stamp = rospy.Time.now()
            sun.header.stamp = rospy.Time.now()

            #Update Transformations
            inertial_tf.header.stamp = rospy.Time.now()

            planet1_tf.header.stamp = rospy.Time.now()
            planet1_tf.transform.translation.x = 3*np.sin(t)
            planet1_tf.transform.translation.y = 3*np.cos(t)
            planet1_tf.transform.translation.z = 0.0
            planet1_tf.transform.rotation.x = 0
            planet1_tf.transform.rotation.y = 0
            planet1_tf.transform.rotation.z = 0
            planet1_tf.transform.rotation.w = 1

            moon1_tf.header.stamp = rospy.Time.now()
            moon1_tf.transform.translation.x = np.sin(1.8*t)
            moon1_tf.transform.translation.y = np.cos(1.8*t)
            moon1_tf.transform.translation.z = 0.0
            moon1_tf.transform.rotation.x = 0
            moon1_tf.transform.rotation.y = 0
            moon1_tf.transform.rotation.z = 0
            moon1_tf.transform.rotation.w = 1

            bc_inertial.sendTransform(inertial_tf)
            bc_planet.sendTransform(planet1_tf)
            bc_moon.sendTransform(moon1_tf)

            pub_sun.publish(sun)
            pub_planet1.publish(planet1)
            pub_moon1.publish(moon1)
            pub_arrow.publish(arrow)

            loop_rate.sleep()
 
    except rospy.ROSInterruptException:
        pass