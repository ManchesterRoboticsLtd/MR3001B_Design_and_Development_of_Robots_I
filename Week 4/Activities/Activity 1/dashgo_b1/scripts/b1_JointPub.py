#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState



#Declare variables to be used
first = True
start_time = 0.0
last_time =  0.0
current_time = 0.0

wr = 0.0
wl = 0.0
right_wheel_angle = 0.0
left_wheel_angle = 0.0


# Declare the output Messages
contJoints = JointState()
robot_odom = Odometry()

# Declare the output Messages
def init_joints():
    contJoints.header.frame_id = "base_link"
    contJoints.header.stamp = rospy.Time.now()
    contJoints.name.extend(["rightWheel", "leftWheel"])
    contJoints.position.extend([0.0, 0.0])
    contJoints.velocity.extend([0.0, 0.0])
    contJoints.effort.extend([0.0, 0.0])

#Right Wheel Callback Function
def odom_callback(msg):
    global robot_odom
    robot_odom = msg

# Wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

# Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")

if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("Robot_JointPub")

    #Set the parameters of the system
    _wheel_r = rospy.get_param("~wheel_radius",0.062)
    _robot_l = rospy.get_param("~robot_wheelbase",0.330)
    _sample_time = rospy.get_param("~sim_sample_time",0.01)
    _node_rate = rospy.get_param("~sim_node_rate",500)

    # Configure the Node
    loop_rate = rospy.Rate(_node_rate)
    rospy.on_shutdown(stop)

    #Init joints
    init_joints()

    #Setup Transform Broadcasters
    joint_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
    rospy.Subscriber("odom", Odometry, odom_callback)

    print("The Robot Joint Publisher is Running")
    try:
    #Run the node
        while not rospy.is_shutdown(): 
                
            if first == True:
                start_time = rospy.get_time()
                last_time =  rospy.get_time()
                current_time = rospy.get_time()
                first = False
            else: 
                current_time = rospy.get_time()
                dt = current_time - last_time

                if dt >= _sample_time:

                    #Get the wheel velocities
                    wr = (1/_wheel_r)*robot_odom.twist.twist.linear.x + (_robot_l/(2*_wheel_r)) * robot_odom.twist.twist.angular.z
                    wl = (1/_wheel_r)*robot_odom.twist.twist.linear.x - (_robot_l/(2*_wheel_r)) * robot_odom.twist.twist.angular.z

                    #Get the angle of the wheels 
                    right_wheel_angle += dt * wr
                    left_wheel_angle += dt * wl

                    #Fill the Joint Message
                    contJoints.header.stamp = rospy.Time.now()
                    contJoints.position[0] = wrap_to_Pi(right_wheel_angle)
                    contJoints.velocity[0] = wr
                    contJoints.position[1] = wrap_to_Pi(left_wheel_angle)
                    contJoints.velocity[0] = wl

                    #Publish messages
                    joint_pub.publish(contJoints)

                    #Update last time
                    last_time = rospy.get_time()

            loop_rate.sleep()
 
    except rospy.ROSInterruptException:
        pass