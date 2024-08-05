#!/usr/bin/env python3
import copy
import rospy
import tf
import tf2_ros
from nav_msgs.msg import Odometry
import geometry_msgs.msg

def handle_odom_pose(ros_odom_msg):

    br = tf2_ros.TransformBroadcaster()

    map_2_odom                 = geometry_msgs.msg.TransformStamped() # comment for gmapping
    map_2_odomR                 = geometry_msgs.msg.TransformStamped() 
    map_2_odomL                 = geometry_msgs.msg.TransformStamped() 
    #odom_2_base_link           = geometry_msgs.msg.TransformStamped()
    #wamv_base_link_2_base_link = geometry_msgs.msg.TransformStamped()
    
    map_2_odom.header.stamp = rospy.Time.now() # comment for gmapping
    map_2_odom.header.frame_id = "/map" # comment for gmapping
    map_2_odom.child_frame_id = "/odom" # comment for gmapping
    map_2_odom.transform.rotation.w = 1 # comment for gmapping

    map_2_odomR.header.stamp = rospy.Time.now() # comment for gmapping
    map_2_odomR.header.frame_id = "/map" # comment for gmapping
    map_2_odomR.child_frame_id = "/odom_right" # comment for gmapping
    map_2_odomR.transform.rotation.w = 1 # comment for gmapping

    map_2_odomL.header.stamp = rospy.Time.now() # comment for gmapping
    map_2_odomL.header.frame_id = "/map" # comment for gmapping
    map_2_odomL.child_frame_id = "/odom_left" # comment for gmapping
    map_2_odomL.transform.rotation.w = 1 # comment for gmapping

    #odom_2_base_link.header.stamp = rospy.Time.now()
    #odom_2_base_link.header.frame_id = "/odom"
    #odom_2_base_link.child_frame_id = "/base_link"
    #odom_2_base_link.transform.rotation.w = 1

    #wamv_base_link_2_base_link.header.stamp = rospy.Time.now()
    #wamv_base_link_2_base_link.header.frame_id = "/wamv/base_link"
    #wamv_base_link_2_base_link.child_frame_id = "/base_link"
    #wamv_base_link_2_base_link.transform.rotation.w = 1

    br.sendTransform(map_2_odom) # comment for gmapping
    br.sendTransform(map_2_odomR)
    br.sendTransform(map_2_odomL)
    #br.sendTransform(odom_2_base_link)
    #br.sendTransform(wamv_base_link_2_base_link)
    
def callback(data):
    handle_odom_pose(data)

if __name__ == "__main__":
    rospy.init_node('wamv_tf_expander')
    rospy.Subscriber("odom",Odometry,callback)
    rospy.Subscriber("odom_right",Odometry,callback)
    rospy.Subscriber("odom_left",Odometry,callback)
    rospy.spin()
