#!/usr/bin/env python  
import rospy
import tf2_ros
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
import tf_conversions


# Setup parameters, transforms, variables and callback functions here (if required)

marker_msg = PoseStamped()

def marker_callback(msg):
    global marker_msg
    marker_msg.header = msg.header
    marker_msg.pose = msg.pose

#Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")

if __name__ == '__main__':
    #Initialise and Setup node
    rospy.init_node('transform_listener')

    # Configure the Node
    loop_rate = rospy.Rate(10)
    rospy.on_shutdown(stop)

    #Setup the messages and transforms

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    #Setup Publishers, subscribers and transform broadcasters here
    rospy.Subscriber("/link5", Marker, marker_callback)

    print("The tf listener is ready")

    try:

        while not rospy.is_shutdown():
            try:
                trans = tfBuffer.lookup_transform('base_link', 'link4', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                loop_rate.sleep()
                continue

            translation = trans.transform.translation
            rotation = trans.transform.rotation

            translation_matrix = np.array([
                [1, 0, 0, translation.x],
                [0, 1, 1, translation.y],
                [0, 0, 1, translation.z],
                [0, 0, 0, 1]
            ])

            rotation_matrix = tf_conversions.transformations.quaternion_matrix([
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w
            ])

            trans_matrix = np.dot(translation_matrix, rotation_matrix)

            marker_transformed = tf2_geometry_msgs.do_transform_pose(marker_msg,trans)    
            
            print(trans_matrix)
            print(trans)
            print(marker_transformed)

            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass