#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf_conversions


chassis = Marker()
left_wheel = Marker()
right_wheel = Marker()
caster_ball = Marker()

base_link_tf = TransformStamped()
chassis_tf = TransformStamped()
left_wheel_tf = TransformStamped()
right_wheel_tf = TransformStamped()
caster_ball_tf = TransformStamped()

def init_base_link():
    base_link_tf.header.frame_id = "world"
    base_link_tf.header.stamp = rospy.Time.now()
    base_link_tf.child_frame_id = "base_link"
    base_link_tf.transform.translation.x = 0.0
    base_link_tf.transform.translation.y = 0.0
    base_link_tf.transform.translation.z = 0.0
    base_link_tf.transform.rotation.x = 0.0
    base_link_tf.transform.rotation.y = 0.0
    base_link_tf.transform.rotation.z = 0.0
    base_link_tf.transform.rotation.w = 1.0

def init_chassis():
    chassis.header.frame_id = "chassis"
    chassis.header.stamp = rospy.Time.now()
    chassis
    chassis.id = 0
    chassis.type = 1
    chassis.action = 0

    # Position and orientation
    chassis.pose.position.x = 0.0
    chassis.pose.position.y = 0.0
    chassis.pose.position.z = 0.0
    chassis.pose.orientation.x = 0.0
    chassis.pose.orientation.y = 0.0
    chassis.pose.orientation.z = 0.0
    chassis.pose.orientation.w = 1.0
    chassis.scale.x = 1.0
    chassis.scale.y = 0.5
    chassis.scale.z = 0.5
    chassis.color.r = 0.0
    chassis.color.g = 0.0
    chassis.color.b = 1.0
    chassis.color.a = 1.0
    chassis.lifetime = rospy.Duration(0)

    # Initialise the chassis transform
    chassis_tf.header.frame_id = "base_link"
    chassis_tf.child_frame_id = "chassis"
    chassis_tf.transform.translation.x = 0.0
    chassis_tf.transform.translation.y = 0.0
    chassis_tf.transform.translation.z = 0.0
    chassis_tf.transform.rotation.x = 0.0
    chassis_tf.transform.rotation.y = 0.0
    chassis_tf.transform.rotation.z = 0.0
    chassis_tf.transform.rotation.w = 1.0

def init_left_wheel():
    left_wheel.header.frame_id = "left_wheel"
    left_wheel.header.stamp = rospy.Time.now()
    left_wheel.id = 1
    left_wheel.type = 3
    left_wheel.action = 0

    # Position and orientation
    left_wheel.pose.position.x = 0.0
    left_wheel.pose.position.y = 0.0
    left_wheel.pose.position.z = 0.0
    left_wheel.pose.orientation.x = 0.0
    left_wheel.pose.orientation.y = 0.0
    left_wheel.pose.orientation.z = 0.0
    left_wheel.pose.orientation.w = 1.0
    left_wheel.scale.x = 0.1
    left_wheel.scale.y = 0.1
    left_wheel.scale.z = 0.1
    left_wheel.color.r = 1.0
    left_wheel.color.g = 0.0
    left_wheel.color.b = 0.0
    left_wheel.color.a = 1.0
    left_wheel.lifetime = rospy.Duration(0)

    # Initialise the left wheel transform
    left_wheel_tf.header.frame_id = "chassis"
    left_wheel_tf.header.stamp = rospy.Time.now()
    left_wheel_tf.child_frame_id = "left_wheel"
    left_wheel_tf.transform.translation.x = 0.5
    left_wheel_tf.transform.translation.y = 0.0
    left_wheel_tf.transform.translation.z = 0.0
    left_wheel_tf.transform.rotation.x = 0.0
    left_wheel_tf.transform.rotation.y = 0.0
    left_wheel_tf.transform.rotation.z = 0.0
    left_wheel_tf.transform.rotation.w = 1.0

def init_right_wheel():
    right_wheel.header.frame_id = "right_wheel"
    right_wheel.header.stamp = rospy.Time.now()
    right_wheel.id = 2
    right_wheel.type = 3
    right_wheel.action = 0

    # Position and orientation
    right_wheel.pose.position.x = 0.0
    right_wheel.pose.position.y = 0.0
    right_wheel.pose.position.z = 0.0
    right_wheel.pose.orientation.x = 0.0
    right_wheel.pose.orientation.y = 0.0
    right_wheel.pose.orientation.z = 0.0
    right_wheel.pose.orientation.w = 1.0
    right_wheel.scale.x = 0.1
    right_wheel.scale.y = 0.1
    right_wheel.scale.z = 0.1
    right_wheel.color.r = 1.0
    right_wheel.color.g = 0.0
    right_wheel.color.b = 0.0
    right_wheel.color.a = 1.0
    right_wheel.lifetime = rospy.Duration(0)

    # Initialise the right wheel transform
    right_wheel_tf.header.frame_id = "chassis"
    right_wheel_tf.header.stamp = rospy.Time.now()
    right_wheel_tf.child_frame_id = "right_wheel"
    right_wheel_tf.transform.translation.x = -0.5
    right_wheel_tf.transform.translation.y = 0.0
    right_wheel_tf.transform.translation.z = 0.0
    right_wheel_tf.transform.rotation.x = 0.0
    right_wheel_tf.transform.rotation.y = 0.0
    right_wheel_tf.transform.rotation.z = 0.0
    right_wheel_tf.transform.rotation.w = 1.0

def init_caster_ball():
    caster_ball.header.frame_id = "caster_ball"
    caster_ball.header.stamp = rospy.Time.now()
    caster_ball.id = 3
    caster_ball.type = 2
    caster_ball.action = 0

    # Position and orientation
    caster_ball.pose.position.x = 0.0
    caster_ball.pose.position.y = 0.0
    caster_ball.pose.position.z = 0.0
    caster_ball.pose.orientation.x = 0.0
    caster_ball.pose.orientation.y = 0.0
    caster_ball.pose.orientation.z = 0.0
    caster_ball.pose.orientation.w = 1.0
    caster_ball.scale.x = 0.1
    caster_ball.scale.y = 0.1
    caster_ball.scale.z = 0.1
    caster_ball.color.r = 1.0
    caster_ball.color.g = 0.0
    caster_ball.color.b = 0.0
    caster_ball.color.a = 1.0
    caster_ball.lifetime = rospy.Duration(0)

    # Initialise the caster ball transform
    caster_ball_tf.header.frame_id = "chassis"
    caster_ball_tf.header.stamp = rospy.Time.now()
    caster_ball_tf.child_frame_id = "caster_ball"
    caster_ball_tf.transform.translation.x = 0.0
    caster_ball_tf.transform.translation.y = 0.0
    caster_ball_tf.transform.translation.z = -0.5
    caster_ball_tf.transform.rotation.x = 0.0
    caster_ball_tf.transform.rotation.y = 0.0
    caster_ball_tf.transform.rotation.z = 0.0
    caster_ball_tf.transform.rotation.w = 1.0


if __name__=='__main__':
    rospy.init_node("simple_mobile_robot")
    loop_rate = rospy.Rate(10)
    print("The robot is ready")

    init_base_link()
    init_chassis()
    init_left_wheel()
    init_right_wheel()
    init_caster_ball()

    chassis_pub = rospy.Publisher("chassis", Marker, queue_size=10)
    left_wheel_pub = rospy.Publisher("left_wheel", Marker, queue_size=10)
    right_wheel_pub = rospy.Publisher("right_wheel", Marker, queue_size=10)
    caster_ball_pub = rospy.Publisher("caster_ball", Marker, queue_size=10)

    base_link_tf = StaticTransformBroadcaster()
    chassis_tf = StaticTransformBroadcaster()
    left_wheel_tf = StaticTransformBroadcaster()
    right_wheel_tf = StaticTransformBroadcaster()
    caster_ball_tf = StaticTransformBroadcaster()
            
    base_link_tf.sendTransform(base_link_tf)
    chassis_tf.sendTransform(chassis_tf)
    left_wheel_tf.sendTransform(left_wheel_tf)
    right_wheel_tf.sendTransform(right_wheel_tf)
    caster_ball_tf.sendTransform(caster_ball_tf)

    try:
        while not rospy.is_shutdown():
            # Publish the markers
            chassis_pub.publish(chassis)
            left_wheel_pub.publish(left_wheel)
            right_wheel_pub.publish(right_wheel)
            caster_ball_pub.publish(caster_ball)

            # Publish the transforms

            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass
