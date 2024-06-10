#!/usr/bin/env python3
# Ensure this shebang line matches your environment

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf

def publish_initial_pose():
    rospy.init_node('initial_pose_publisher', anonymous=True)
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    pose.pose.pose.position.x = 0.0
    pose.pose.pose.position.y = 0.0
    pose.pose.pose.position.z = 0.0
    quaternion = tf.transformations.quaternion_from_euler(0, 0, -3.14)
    pose.pose.pose.orientation.x = quaternion[0]
    pose.pose.pose.orientation.y = quaternion[1]
    pose.pose.pose.orientation.z = quaternion[2]
    pose.pose.pose.orientation.w = quaternion[3]
    pose.pose.covariance = [0.0] * 36

    while not rospy.is_shutdown():
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_initial_pose()
    except rospy.ROSInterruptException:
        pass

