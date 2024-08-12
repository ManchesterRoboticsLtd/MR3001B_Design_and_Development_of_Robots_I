#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf_conversions

# Setup parameters, transforms, variables and callback functions here (if required)
# Declare message or transform
sun_tf = TransformStamped()
planet_tf = TransformStamped()    

# Initialise Sun Transform Message
def init_sunTransform():   
    sun_tf.header.frame_id = "inertial_frame"
    sun_tf.child_frame_id = "sun"
    sun_tf.header.stamp = rospy.Time.now()
    sun_tf.transform.translation.x = 1
    sun_tf.transform.translation.y = 1
    sun_tf.transform.translation.z = 1.0
    sun_tf.transform.rotation.x = 0
    sun_tf.transform.rotation.y = 0
    sun_tf.transform.rotation.z = 0
    sun_tf.transform.rotation.w = 1

# Initialise Planet Transform Message
def init_planetTransform():
    planet_tf.header.frame_id = "sun"
    planet_tf.child_frame_id = "planet"
    planet_tf.header.stamp = rospy.Time.now()
    planet_tf.transform.translation.x = 0
    planet_tf.transform.translation.y = 0
    planet_tf.transform.translation.z = 0.0
    planet_tf.transform.rotation.x = 0
    planet_tf.transform.rotation.y = 0
    planet_tf.transform.rotation.z = 0
    planet_tf.transform.rotation.w = 1


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

    #Setup the messages
    init_sunTransform()
    init_planetTransform()

    #Setup Publishers, subscribers and transform broadcasters here
    bc_sun = StaticTransformBroadcaster() 
    bc_planet = TransformBroadcaster()
 
    print("The tf's are ready")

    try:
        #Run the node
        while not rospy.is_shutdown(): 
            
            #Get the Simulation time
            t = rospy.Time.now().to_sec()
            
            #Update Transformations
            sun_tf.header.stamp = rospy.Time.now()

            # Rotate around y-axis
            q = tf_conversions.transformations.quaternion_from_euler(0.0, np.pi*t, 0.0)

            planet_tf.header.stamp = rospy.Time.now()
            planet_tf.transform.translation.x = 1.2*np.sin(t)
            planet_tf.transform.translation.y = 1.2*np.cos(t)
            planet_tf.transform.translation.z = 0.0
            planet_tf.transform.rotation.x = q[0]
            planet_tf.transform.rotation.y = q[1]
            planet_tf.transform.rotation.z = q[2]
            planet_tf.transform.rotation.w = q[3]

            # Publish the transforms
            bc_sun.sendTransform(sun_tf)
            bc_planet.sendTransform(planet_tf)

            loop_rate.sleep()
 
    except rospy.ROSInterruptException:
        pass