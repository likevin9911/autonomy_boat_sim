#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class ThrusterController:
    def __init__(self):
        rospy.init_node('thruster_controller', anonymous=True)
        
        # Subscribe to cmd_vel
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Publishers for left and right thrusters
        self.left_pub = rospy.Publisher('/left_thruster_cmd', Float64, queue_size=10)
        self.right_pub = rospy.Publisher('/right_thruster_cmd', Float64, queue_size=10)
        
        # You may need to adjust these values based on your system
        self.max_linear_velocity = 1.0
        self.max_angular_velocity = 1.0
        
    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Convert to thruster commands
        left_thrust = self.calculate_thrust(linear_x, angular_z, 'left')
        right_thrust = self.calculate_thrust(linear_x, angular_z, 'right')
        
        # Publish thruster commands
        self.left_pub.publish(Float64(left_thrust))
        self.right_pub.publish(Float64(right_thrust))
        
    def calculate_thrust(self, linear_x, angular_z, side):
        # This is a simple differential drive-like model
        # You may need to adjust this based on your specific thruster configuration
        if side == 'left':
            return (linear_x - angular_z) / self.max_linear_velocity
        elif side == 'right':
            return (linear_x + angular_z) / self.max_linear_velocity
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = ThrusterController()
        controller.run()
    except rospy.ROSInterruptException:
        pass