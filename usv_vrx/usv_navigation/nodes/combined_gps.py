#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix

class GPSCombiner:
    def __init__(self):
        self.gps1_sub = rospy.Subscriber('/gps_left/gps/fix', NavSatFix, self.gps1_callback)
        self.gps2_sub = rospy.Subscriber('/gps_right/gps/fix', NavSatFix, self.gps2_callback)
        self.combined_gps_pub = rospy.Publisher('/combined_gps/fix', NavSatFix, queue_size=10)
        
        self.gps1_data = None
        self.gps2_data = None

    def gps1_callback(self, data):
        self.gps1_data = data
        self.combine_and_publish()

    def gps2_callback(self, data):
        self.gps2_data = data
        self.combine_and_publish()

    def combine_and_publish(self):
        if self.gps1_data and self.gps2_data:
            combined_data = NavSatFix()
            combined_data.header.stamp = rospy.Time.now()
            combined_data.header.frame_id = "base_link"
            # Simple average of the positions
            combined_data.latitude = (self.gps1_data.latitude + self.gps2_data.latitude) / 2
            combined_data.longitude = (self.gps1_data.longitude + self.gps2_data.longitude) / 2
            combined_data.altitude = (self.gps1_data.altitude + self.gps2_data.altitude) / 2
            # You can also add more sophisticated combination logic here

            self.combined_gps_pub.publish(combined_data)

if __name__ == '__main__':
    rospy.init_node('gps_combiner', anonymous=True)
    gps_combiner = GPSCombiner()
    rospy.spin()

