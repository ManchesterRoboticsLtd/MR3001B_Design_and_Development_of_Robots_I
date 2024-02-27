#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection

# Setup parameters, transforms, variables and callback functions here (if required)
# Declare message or transform.

def L2PC_callback(msg):
    PC_out = laserProj.projectLaser(msg)
    pub_PC.publish(PC_out)

if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("Laser2PointCloud_node")
 
    # Configure the Node
    loop_rate = rospy.Rate(10)
    laserProj = LaserProjection()

    #Setup Publishers, subscribers and transform broadcasters here
    pub_PC = rospy.Publisher("/laserPointCloud", pc2, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, L2PC_callback)
    print("Laser2PC_ready")
    rospy.spin()
