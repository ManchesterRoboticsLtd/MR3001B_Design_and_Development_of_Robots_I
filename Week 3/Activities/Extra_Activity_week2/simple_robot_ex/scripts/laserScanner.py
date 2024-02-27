#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan


if __name__=='__main__':

    rospy.init_node('laser_scan_publisher')

    scan_pub = rospy.Publisher('scan', LaserScan, queue_size=1)

    num_readings = 1081
    laser_frequency = 40

    count = 0
    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        scan = LaserScan()

        scan.header.stamp = current_time
        scan.header.frame_id = 'laser'
        scan.angle_min = -3.1416
        scan.angle_max = 3.1416
        scan.angle_increment = 2*3.14 / num_readings
        scan.time_increment = (1.0 / laser_frequency) / (num_readings)
        scan.range_min = 0.0
        scan.range_max = 2.0

        scan.ranges = []
        scan.intensities = []
        for i in range(0, num_readings):
            scan.ranges.append(2.0  + np.random.normal(0,0.5))  # fake data
            scan.intensities.append(1)  # fake data

        scan_pub.publish(scan)
        count += 1
        r.sleep()