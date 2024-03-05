#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
#Declare libraries to be used (if needed)
#Declare Messages to be used


#Declare Variables/Parameters to be used


# Main class
class MotorJointTransformer:
    def __init__(self):
        #Initialise and Setup node
        rospy.init_node("DC_MotorJoints")
        self.loop_rate = rospy.Rate(100)

        #Variables/Parameters to be used
        self.motor_rads = 0.0
        self.wheel_angle = 0.0
        self.joint_states = JointState()

        self.last_time = rospy.Time.now().to_sec()

        #Declare other functions if required
        self.init_joint_state()

        #Setup Publishers and Subscribers
        self.sub_motor_output = rospy.Subscriber('/motor_output', Float32, self.motor_output_callback)
        self.pub_joint_states = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.pub_wheel_angle = rospy.Publisher('/wheel_angle', Float32, queue_size=1)

    # Motor output callback in rad/s
    def motor_output_callback(self, msg):
        self.motor_rads = msg.data

    def init_joint_state(self):
        self.joint_states.header.frame_id = "Motor"
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.name.extend(["motor_wheel_joint"])
        self.joint_states.position.extend([0.0])
        self.joint_states.velocity.extend([0.0])
        self.joint_states.effort.extend([0.0])

    def wrap_to_pi(self, theta):
        result = theta % (2 * 3.14159)
        if result < 0:
            result += 2 * 3.14159
        return result - 3.14159
    
    def run(self):
        dt = rospy.Time.now().to_sec() - self.last_time
        self.last_time = rospy.Time.now().to_sec()

        self.wheel_angle += self.motor_rads * dt
        self.pub_wheel_angle.publish(self.wheel_angle)
        #rospy.loginfo(f"Wheel Angle: {self.wheel_angle} rad")

        # Update and publish joints
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.position[0] = self.wrap_to_pi(self.wheel_angle)
        self.pub_joint_states.publish(self.joint_states)

        self.loop_rate.sleep()


if __name__=='__main__':

    #Initialise and Setup node
    transformer = MotorJointTransformer()

    try:
        while not rospy.is_shutdown(): 
            transformer.run()
    except rospy.ROSInterruptException:
        pass
