#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class SimpleController:
    def __init__(self):
        self.left_thrust_pub = rospy.Publisher('/thrusters/left_thrust_cmd', Float32, queue_size=10)
        self.right_thrust_pub = rospy.Publisher('/thrusters/right_thrust_cmd', Float32, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, msg):
        # Convert linear.x to thrust commands (simplified)
        left_thrust = msg.linear.x
        right_thrust = msg.linear.x

        # Publish thrust commands
        self.left_thrust_pub.publish(left_thrust)
        self.right_thrust_pub.publish(right_thrust)

if __name__ == '__main__':
    rospy.init_node('simple_controller')
    SimpleController()
    rospy.spin()

