#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class SimpleController:
    def __init__(self):
        self.left_thrust_pub = rospy.Publisher('/thrusters/left_thrust_cmd', Float32, queue_size=10)
        self.right_thrust_pub = rospy.Publisher('/thrusters/right_thrust_cmd', Float32, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Parameters (adjust these based on your robot's specifications)
        self.max_thrust = 1.0  # Maximum thrust value
        self.engine_separation = 0.5  # Distance between left and right thrusters

    def cmd_vel_callback(self, msg):
        try:
            # Convert linear and angular velocities to differential drive commands
            left_thrust = msg.linear.x - (msg.angular.z * self.engine_separation / 2)
            right_thrust = msg.linear.x + (msg.angular.z * self.engine_separation / 2)

            # Scale and clamp thrust values
            left_thrust = self.scale_thrust(left_thrust)
            right_thrust = self.scale_thrust(right_thrust)

            # Publish thrust commands
            self.left_thrust_pub.publish(Float32(left_thrust))
            self.right_thrust_pub.publish(Float32(right_thrust))

            rospy.logdebug(f"Left thrust: {left_thrust}, Right thrust: {right_thrust}")
        except Exception as e:
            rospy.logerr(f"Error in cmd_vel_callback: {e}")

    def scale_thrust(self, thrust):
        # Scale and clamp thrust to be within [-max_thrust, max_thrust]
        return max(min(thrust, self.max_thrust), -self.max_thrust)

if __name__ == '__main__':
    rospy.init_node('simple_controller', log_level=rospy.INFO)
    controller = SimpleController()
    rospy.loginfo("Simple controller node is running.")
    rospy.spin()