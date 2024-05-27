#!/usr/bin/env python3

## I'm using this script to develop types of noise for lidar-based experiments
## This uses Mizzou's first bag file, but will be adapted to Airsim whenever that gets going

import rospy, random, struct, time
import numpy as np
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
from std_msgs import *
from std_msgs.msg import Float32
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
from geometry_msgs.msg import Twist



global my_data
my_data = 0

global left_thruster_value, right_thruster_value
left_thruster_value = 0.0
right_thruster_value = 0.0

# ROS Topics
subscriberR_topic = '/wamv/thrusters/right_thrust_cmd'
subscriberL_topic = '/wamv/thrusters/left_thrust_cmd'
publisher_topic = '/cmd_vel'

# ROS Publisher
pub = rospy.Publisher(publisher_topic, Twist, queue_size=10)

def thrusters_to_cmd_vel(left_thruster, right_thruster):
    """
    Converts left and right thruster commands into a cmd_vel message and publishes it.
    """
    cmd_vel_msg = Twist()

    # Calculate linear and angular velocities
    linear_velocity = (left_thruster + right_thruster) / 1.95
    angular_velocity = (right_thruster - left_thruster) / 1.95

    # Define maximum velocities (adjust these as necessary)
    max_linear_velocity = 1  # Maximum linear velocity
    max_angular_velocity = 0.1  # Maximum angular velocity

    # Scale velocities if they exceed the maximum limits
    if abs(linear_velocity) > max_linear_velocity:
        scale = max_linear_velocity / abs(linear_velocity)
        linear_velocity *= scale
        angular_velocity *= scale
    
    if abs(angular_velocity) > max_angular_velocity:
        scale = max_angular_velocity / abs(angular_velocity)
        linear_velocity *= scale
        angular_velocity *= scale

    # Apply the scaled velocities to the cmd_vel message
    cmd_vel_msg.linear.x = linear_velocity
    cmd_vel_msg.angular.z = angular_velocity

    # Publish the cmd_vel message
    pub.publish(cmd_vel_msg)
    rospy.loginfo("Published cmd_vel: linear.x=%.2f, angular.z=%.2f", cmd_vel_msg.linear.x, cmd_vel_msg.angular.z)

def callbackR(msg):
    global right_thruster_value
    right_thruster_value = msg.data
    rospy.loginfo("Right Thruster Subscribed: %f", right_thruster_value)
    thrusters_to_cmd_vel(left_thruster_value, right_thruster_value)

def callbackL(msg):
    global left_thruster_value
    left_thruster_value = msg.data
    rospy.loginfo("Left Thruster Subscribed: %f", left_thruster_value)
    thrusters_to_cmd_vel(left_thruster_value, right_thruster_value)

def listener():
    rospy.init_node('interceptLidar', anonymous=True)
    rospy.Subscriber(subscriberL_topic, Float32, callbackL, queue_size=10)
    rospy.Subscriber(subscriberR_topic, Float32, callbackR, queue_size=10)

    print("Subscribed to ", subscriberR_topic)
    print("Subscribed to ", subscriberL_topic)
    print("Publishing to ", publisher_topic)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

