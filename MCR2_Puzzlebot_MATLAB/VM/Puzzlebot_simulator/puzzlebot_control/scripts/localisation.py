#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf_conversions
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class DeadReckoning:
    def __init__(self):

        #Set the parameters of the system
        self.sample_time = rospy.get_param("~sample_time",0.02)

        self.l = rospy.get_param("~wheelBase",0.19)
        self.r = rospy.get_param("~wheelRadius",0.05)

        self.noise = rospy.get_param("~cov_ellipsoid",0)
        self.k_r = rospy.get_param("~wheelCov_kr",0.01)
        self.k_l = rospy.get_param("~wheelCov_kl",0.01)

        self.X = rospy.get_param("~initialPose_x",0.0)
        self.Y = rospy.get_param("~initialPose_y",0.0)
        self.Th = self.wrap_to_Pi(rospy.get_param("~initialPose_th",0.0))

        # Setup Variables to be used
        self.first = True
        self.clock_msg = None

        
        self.start_time = None
        self.current_time = None
        self.last_time = None
        self.last_update = None
        self.proc_output = 0.0

        self.v_r = 0.0
        self.v_l = 0.0
        self.V = 0.0
        self.Omega = 0.0

        self.sigma_omega = np.eye(2)
        self.nabla_H_omegas = np.ones([3,2])
        self.Q_k =  np.zeros([3,3])
        self.H_k = np.eye(3)      
        self.Sigma = np.zeros([3,3])


        # Setup the Subscribers
        rospy.Subscriber("joint_states", JointState, self.callback)
 
        #Setup de publishers
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.wr_pub = rospy.Publisher("wr", Float32, queue_size=1)
        self.wl_pub = rospy.Publisher("wl", Float32, queue_size=1)

        #Setup Transform Broadcasters and buffers
        self.tf_broadcaster = TransformBroadcaster()

        # Declare the output Messages
    def init_output_msgs(self):
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.Th)

        self.puzzlebot_odom = Odometry()
        self.puzzlebot_odom.header.stamp = rospy.Time.now()
        self.puzzlebot_odom.header.frame_id = "odom"
        self.puzzlebot_odom.child_frame_id = rospy.get_namespace()+"base_link"
        self.puzzlebot_odom.pose.pose.position.x = self.X
        self.puzzlebot_odom.pose.pose.position.y = self.Y
        self.puzzlebot_odom.pose.pose.position.z = 0.0
        self.puzzlebot_odom.pose.pose.orientation.x = q[0]
        self.puzzlebot_odom.pose.pose.orientation.y = q[1]
        self.puzzlebot_odom.pose.pose.orientation.z = q[2]
        self.puzzlebot_odom.pose.pose.orientation.w = q[3]
        self.puzzlebot_odom.pose.covariance = [0]*36
        self.puzzlebot_odom.twist.twist.linear.x = 0.0
        self.puzzlebot_odom.twist.twist.linear.y = 0.0
        self.puzzlebot_odom.twist.twist.linear.z = 0.0
        self.puzzlebot_odom.twist.twist.angular.x = 0.0
        self.puzzlebot_odom.twist.twist.angular.y = 0.0
        self.puzzlebot_odom.twist.twist.angular.z = 0.0
        self.puzzlebot_odom.twist.covariance = [0]*36

        self.wr = Float32()
        self.wr.data = 0.0

        self.wl = Float32()
        self.wl.data = 0.0

        #Define Transformations
    def init_tf(self):
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.Th)
        self.robot_tf = TransformStamped()
        self.robot_tf.header.stamp = rospy.Time.now()
        self.robot_tf.header.frame_id = "odom"
        self.robot_tf.child_frame_id = rospy.get_namespace()+"base_footprint"
        self.robot_tf.transform.translation.x = self.X
        self.robot_tf.transform.translation.y = self.Y
        self.robot_tf.transform.translation.z = 0.0
        self.robot_tf.transform.rotation.x = q[0]
        self.robot_tf.transform.rotation.y = q[1]
        self.robot_tf.transform.rotation.z = q[2]
        self.robot_tf.transform.rotation.w = q[3]

    #Define the callback functions
    def callback(self, data):
        try: 
            wr_idx = data.name.index("wheel_right_joint")
            wl_idx = data.name.index("wheel_left_joint")

            #Publish messages
            self.wr.data = data.velocity[wr_idx]
            self.wl.data = data.velocity[wl_idx]
        except:
            pass

    #Define the main RUN function
    def run (self):
        if self.first == True:
            self.init_output_msgs()
            self.init_tf()
            self.start_time = rospy.get_time() 
            self.last_time = rospy.get_time()
            self.current_time = rospy.get_time()
            self.last_update = rospy.Time.now()
            self.first = False
        
        #System
        else:
            #Define sampling time
            self.current_time = rospy.get_time()
            
            if rospy.get_time() < self.last_time:
                rospy.logwarn("Detected time jump, resetting state!")
                self.init_output_msgs()
                self.init_tf()
                self.last_time = rospy.get_time()

            dt = self.current_time - self.last_time
   
            #Dynamical System Simulation
            if dt >= self.sample_time:

                #Wheel Tangential Velocities
                self.v_r = self.r * self.wr.data
                self.v_l = self.r * self.wl.data

                #Robot Velocities
                self.V = (1/2.0) * (self.v_r + self.v_l)
                self.Omega = (1.0/self.l) * (self.v_r - self.v_l)

                #Calculate wheel covariance
                self.sigma_omega[0,0] = self.noise * self.k_r * np.abs(self.wr.data)
                self.sigma_omega[1,1] = self.noise * self.k_l * np.abs(self.wl.data)

                self.nabla_H_omegas[0,:] = np.cos(self.Th)
                self.nabla_H_omegas[1,:] = np.sin(self.Th)
                self.nabla_H_omegas[2,0] = 2.0/self.l
                self.nabla_H_omegas[2,1] = -2.0/self.l
                self.nabla_H_omegas = (1.0/2.0) * self.r * dt * self.nabla_H_omegas

                self.Q_k =  np.dot(np.dot(self.nabla_H_omegas,self.sigma_omega),self.nabla_H_omegas.T)

                self.H_k[0,2] = -dt * self.V * np.sin(self.Th)
                self.H_k[1,2] = dt * self.V * np.cos(self.Th)
                
                self.Sigma = np.dot(np.dot(self.H_k,self.Sigma),self.H_k.T) + self.Q_k

                #Pose estimation
                self.X += self.V*np.cos(self.Th) * dt
                self.Y += self.V*np.sin(self.Th) * dt
                self.Th = self.wrap_to_Pi(self.Th+self.Omega * dt)
                #self.Th = self.wrap_to_Pi(self.Th)
                
                self.last_time = rospy.get_time()

                self.q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.Th)

                self.puzzlebot_odom.header.stamp = rospy.Time.now()
                self.puzzlebot_odom.pose.pose.position.x = self.X
                self.puzzlebot_odom.pose.pose.position.y = self.Y
                self.puzzlebot_odom.pose.pose.position.z = 0.0
                self.puzzlebot_odom.pose.pose.orientation.x = self.q[0]
                self.puzzlebot_odom.pose.pose.orientation.y = self.q[1]
                self.puzzlebot_odom.pose.pose.orientation.z = self.q[2]
                self.puzzlebot_odom.pose.pose.orientation.w = self.q[3]
                self.puzzlebot_odom.pose.covariance[0] = self.Sigma[0,0]
                self.puzzlebot_odom.pose.covariance[1] =self.Sigma[0,1]
                self.puzzlebot_odom.pose.covariance[5] = self.Sigma[0,2]
                self.puzzlebot_odom.pose.covariance[6] = self.Sigma[1,0]
                self.puzzlebot_odom.pose.covariance[7] = self.Sigma[1,1]
                self.puzzlebot_odom.pose.covariance[11] = self.Sigma[1,2]
                self.puzzlebot_odom.pose.covariance[30] = self.Sigma[2,0]
                self.puzzlebot_odom.pose.covariance[31] = self.Sigma[2,1]
                self.puzzlebot_odom.pose.covariance[35] = self.Sigma[2,2]
                self.puzzlebot_odom.twist.twist.linear.x = self.V
                self.puzzlebot_odom.twist.twist.angular.z = self.Omega

                #Define Transformations
                self.robot_tf.header.stamp = rospy.Time.now()
                self.robot_tf.transform.translation.x = self.puzzlebot_odom.pose.pose.position.x
                self.robot_tf.transform.translation.y = self.puzzlebot_odom.pose.pose.position.y
                self.robot_tf.transform.translation.z = 0.0
                self.robot_tf.transform.rotation.x = self.puzzlebot_odom.pose.pose.orientation.x
                self.robot_tf.transform.rotation.y = self.puzzlebot_odom.pose.pose.orientation.y
                self.robot_tf.transform.rotation.z = self.puzzlebot_odom.pose.pose.orientation.z
                self.robot_tf.transform.rotation.w = self.puzzlebot_odom.pose.pose.orientation.w


                #Publish odometry and base_link transform
                self.odom_pub.publish(self.puzzlebot_odom)
                self.wr_pub.publish(self.wr.data)
                self.wl_pub.publish(self.wl.data)
                self.tf_broadcaster.sendTransform(self.robot_tf)      


    #Stop Condition
    def stop(self):
    #Setup the stop message (can be the same as the control message)
        print("Stopping")

    #wrap to pi function
    def wrap_to_Pi(self,theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
            result += 2 * np.pi
        return result - np.pi


if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("Puzzlebot_Odometry_Estimator")
    Estimation = DeadReckoning()
 
    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))
    rospy.on_shutdown(Estimation.stop)

    print("The Estimator is Running")
    
    try:
        #Wait for the /clock to publish a value different 
        wait = True
        while wait:
            rospy.loginfo("Waiting for /clock to start publishing...")
            if (rospy.Time.now().to_sec() > 0.0):
                wait = False
                rospy.loginfo("/clock started publishing...")
            loop_rate.sleep()

    #Run the node
        while not rospy.is_shutdown(): 
            try:
                Estimation.run()
                loop_rate.sleep()
            except rospy.ROSTimeMovedBackwardsException:
                pass
    except rospy.ROSInterruptException:
        pass