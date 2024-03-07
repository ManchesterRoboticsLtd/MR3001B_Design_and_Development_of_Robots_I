#!/usr/bin/env python
import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class StateGatherer():
    def __init__(self):

        self.br = tf2_ros.TransformBroadcaster()
        rospy.init_node("pose_tf")
        #rospy.wait_for_service('/gazebo/model_states')
        print("HERE")
        rospy.Subscriber("puzzlebot_1/real_pose", Odometry, self.callback)

    def main(self):
        rospy.spin()


    def callback(self, data):
        try: 

            #aux_idx = data.name.index("puzzlebot_1")
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = "puzzlebot_1/base_link"
            t.transform.translation = data.pose.pose.position
            t.transform.rotation = data.pose.pose.orientation
            self.br.sendTransform(t)

        except:
            pass

if __name__ == '__main__':
    aux = StateGatherer()
    print("World TF publishing...")
    aux.main()