#!/usr/bin/env python  
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2 as pc2
import tf2_sensor_msgs
from sensor_msgs import point_cloud2

# Setup parameters, transforms, variables and callback functions here (if required)

PC_out = pc2()

def PC_callback(msg):
    global PC_out
    PC_out = msg

#Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")

if __name__ == '__main__':
    #Initialise and Setup node
    rospy.init_node('planet_listener')

    # Configure the Node
    loop_rate = rospy.Rate(10)
    rospy.on_shutdown(stop)

    #Setup the messages and transforms

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    #Setup Publishers, subscribers and transform broadcasters here
    rospy.Subscriber("/laserPointCloud", pc2, PC_callback)
    pub_PC = rospy.Publisher("/laserPointCloud2", pc2, queue_size=1)

    print("The tf listener is ready")

    try:

        while not rospy.is_shutdown():
            try:
                trans = tfBuffer.lookup_transform('world', 'laser', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                loop_rate.sleep()
                continue

            point_cloud_transformed = tf2_sensor_msgs.do_transform_cloud(PC_out, trans)

            pub_PC.publish(point_cloud_transformed)
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass