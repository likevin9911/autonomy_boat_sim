#!/usr/bin/env python3

## I'm using this script to develop types of noise for lidar-based experiments
## This uses Mizzou's first bag file, but will be adapted to Airsim whenever that gets going

#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

def callback(odom_msg):
    # Modify the frame_id if necessary
    odom_msg.header.frame_id = "odom"
    pub.publish(odom_msg)

if __name__ == '__main__':
    rospy.init_node('odometry_republisher')

    # Subscriber to /odometry/filtered
    rospy.Subscriber('/wamv/robot_localization/odometry/filtered', Odometry, callback)

    # Publisher to /odom
    pub = rospy.Publisher('wamv/odom', Odometry, queue_size=10)

    rospy.spin()


