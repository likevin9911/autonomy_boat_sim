#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def publish_cmd_vel():
    rospy.init_node('cmd_vel_publisher', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = 0.5  # Forward velocity
        twist.angular.z = 0.2  # Angular velocity
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_cmd_vel()
    except rospy.ROSInterruptException:
        pass
