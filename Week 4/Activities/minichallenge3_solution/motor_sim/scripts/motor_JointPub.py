#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

#Declare Variables/Parameters to be used
sample_time = rospy.get_param("/motorJointSampleTime",0.01)
motorAngle = 0.0
first = True
start_time = 0.0
current_time = 0.0
last_time = 0.0

# Declare the inpit and output Messages
contJoints = JointState()
motorOutput = Float32()


# Declare/initialise the output Messages (Joint names in URDF file)
def init_joints():
    contJoints.header.frame_id = "joint1"
    contJoints.header.stamp = rospy.Time.now()
    contJoints.name.append("joint2")
    contJoints.position.append(0.0)
    contJoints.velocity.append(0.0)
    contJoints.effort.append(0.0)


#Define the callback functions
def motor_outputCallback(msg):
    global motorOutput
    motorOutput = msg


#wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

    #Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")
    contJoints.header.stamp = rospy.Time.now()
    contJoints.position[0] = wrap_to_Pi(motorAngle)
    joint_pub.publish(contJoints)

if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("DC_MotorJoints")
 
    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("/motorJointRate",100))
    rospy.on_shutdown(stop)

    #Init joints
    init_joints()

    #Setup Publishers and Subscribers
    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
    rospy.Subscriber("/motor_output",Float32,motor_outputCallback)


    print("The Motor Joint Publisher is Running")

    try:
    #Run the node
        while not rospy.is_shutdown(): 

            #Setup the variables (run only one time)
            if first == True:
                start_time = rospy.get_time() 
                last_time = rospy.get_time()
                current_time = rospy.get_time()
                first = False

        #System simulation
            else:
            #Define sampling time
                current_time = rospy.get_time()
                dt = current_time - last_time
        
                #Get the angle of the motor shaft using the motor angular speed
                if dt >= sample_time:                   
                    #Integrate the speed to get the position
                    motorAngle += motorOutput.data* dt

                    #Fill the message with the position information
                    contJoints.header.stamp = rospy.Time.now()
                    contJoints.position[0] = wrap_to_Pi(motorAngle)
                    
                    #Update the time
                    last_time = rospy.get_time()

                    #Publish the joint angle message
                    joint_pub.publish(contJoints)

            #Wait and repeat the loop (sleep)
            loop_rate.sleep()
 
    except rospy.ROSInterruptException:
        pass