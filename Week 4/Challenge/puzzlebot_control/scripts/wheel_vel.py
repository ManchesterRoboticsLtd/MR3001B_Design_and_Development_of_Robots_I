#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

class WheelVel:
    def __init__(self):
        rospy.Subscriber("joint_states", JointState, self.callback)
        #Setup de publishers
        self.wr_pub = rospy.Publisher("wr", Float32, queue_size=1)
        self.wl_pub = rospy.Publisher("wl", Float32, queue_size=1)

    def main(self):
        rospy.spin()


    def callback(self, data):
        try: 

            wr_idx = data.name.index("wheel_right_joint")
            wl_idx = data.name.index("wheel_left_joint")

            #Publish messages
            self.wr_pub.publish(data.velocity[wr_idx])
            self.wl_pub.publish(data.velocity[wl_idx])

        except:
            pass


if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("Wheel's velocity")
    wheel_vel = WheelVel()
    print("Wheel Speed Publishing...")
    wheel_vel.main()