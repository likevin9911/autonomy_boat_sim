#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def publish_goal():
    rospy.init_node('simple_goal_publisher', anonymous=True)
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    
    # Wait for the publisher to connect to subscribers
    rospy.sleep(1)

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = 1.0
    goal.pose.position.y = 1.0
    goal.pose.orientation.w = 1.0

    pub.publish(goal)
    rospy.loginfo("Goal published!")

if __name__ == '__main__':
    try:
        publish_goal()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass