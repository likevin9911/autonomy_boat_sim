#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
import random
import time

class GPSNoiseInjector:
    def __init__(self):
        rospy.init_node('gps_noise_injector')
        
        # Noise control
        self.add_noise_to_left = True
        self.last_switch = rospy.get_time()
        
        # Publishers
        self.noisy_gps_pub_left = rospy.Publisher('/gps_left/gps/fix', NavSatFix, queue_size=10)
        self.noisy_gps_pub_right = rospy.Publisher('/gps_right/gps/fix', NavSatFix, queue_size=10)
        self.noisy_vel_pub_left = rospy.Publisher('/gps_left/gps/fix_velocity', Vector3Stamped, queue_size=10)
        self.noisy_vel_pub_right = rospy.Publisher('/gps_right/gps/fix_velocity', Vector3Stamped, queue_size=10)
        
        # Subscribers
        self.left_gps_sub = rospy.Subscriber('/gps_left/gps/fix', NavSatFix, self.left_gps_callback)
        self.right_gps_sub = rospy.Subscriber('/gps_right/gps/fix', NavSatFix, self.right_gps_callback)
        self.left_vel_sub = rospy.Subscriber('/gps_left/gps/fix_velocity', Vector3Stamped, self.left_velocity_callback)
        self.right_vel_sub = rospy.Subscriber('/gps_right/gps/fix_velocity', Vector3Stamped, self.right_velocity_callback)

        # Noise levels (standard deviations from MATLAB calculations)
        self.noise_level_lat = 1.7718e-06  # Replace with combined_std_lat from MATLAB
        self.noise_level_lon = 1.9079e-06  # Replace with combined_std_lon from MATLAB
        self.noise_level_alt = 1.9898     # Replace with combined_std_alt from MATLAB
        self.noise_level_vx = 1.9026     # Replace with combined_std_vx from MATLAB
        self.noise_level_vy = 2.0432     # Replace with combined_std_vy from MATLAB
        self.noise_level_vz = 2.0324     # Replace with combined_std_vz from MATLAB

    def add_position_noise(self, data):
        noisy_data = data
        noisy_data.latitude += random.gauss(0, self.noise_level_lat)
        noisy_data.longitude += random.gauss(0, self.noise_level_lon)
        #noisy_data.altitude += random.gauss(0, self.noise_level_alt)
        return noisy_data

    def add_velocity_noise(self, data):
        noisy_data = data.vector
        noisy_data.x += random.gauss(0, self.noise_level_vx)
        noisy_data.y += random.gauss(0, self.noise_level_vy)
        #noisy_data.z += random.gauss(0, self.noise_level_vz)
        return noisy_data

    def left_gps_callback(self, data):
        if self.add_noise_to_left:
            data = self.add_position_noise(data)
        self.noisy_gps_pub_left.publish(data)

    def right_gps_callback(self, data):
        if not self.add_noise_to_left:
            data = self.add_position_noise(data)
        self.noisy_gps_pub_right.publish(data)

    def left_velocity_callback(self, msg):
        if self.add_noise_to_left:
            msg.vector = self.add_velocity_noise(msg)
        self.noisy_vel_pub_left.publish(msg)

    def right_velocity_callback(self, msg):
        if not self.add_noise_to_left:
            msg.vector = self.add_velocity_noise(msg)
        self.noisy_vel_pub_right.publish(msg)

    def switch_noise_target(self):
        current_time = rospy.get_time()
        if current_time - self.last_switch > 10:  # switch every 10 seconds
            self.add_noise_to_left = not self.add_noise_to_left
            self.last_switch = current_time
            rospy.loginfo("Switching noise target to: {}".format("Left" if self.add_noise_to_left else "Right"))

if __name__ == '__main__':
    noise_injector = GPSNoiseInjector()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        noise_injector.switch_noise_target()
        rate.sleep()
