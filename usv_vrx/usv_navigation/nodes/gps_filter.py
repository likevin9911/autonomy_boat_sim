#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
import math

subscriber_topic_gps = '/wamv/sensors/gps/gps/fix'

class GPSFilter:
    def __init__(self):
        self.previous_latitude = None
        self.previous_longitude = None
        self.previous_altitude = None
        self.distance_threshold = 3.0  # meters

        rospy.init_node('gps_filter_listener', anonymous=True)
        rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, self.gps_callback)
        rospy.spin()

    def gps_callback(self, data):
        if self.previous_latitude is None:
            self.previous_latitude = data.latitude
            self.previous_longitude = data.longitude
            self.previous_altitude = data.altitude
            rospy.loginfo("Initial GPS Data: Latitude: %f, Longitude: %f, Altitude: %f", 
                          data.latitude, data.longitude, data.altitude)
            return

        distance = self.calculate_distance(self.previous_latitude, self.previous_longitude, data.latitude, data.longitude)

        if distance < self.distance_threshold:
            rospy.loginfo("Filtered GPS Data: Latitude: %f, Longitude: %f, Altitude: %f", 
                          data.latitude, data.longitude, data.altitude)
            self.previous_latitude = data.latitude
            self.previous_longitude = data.longitude
            self.previous_altitude = data.altitude
        else:
            rospy.logwarn("Outlier detected and ignored: Latitude: %f, Longitude: %f, Altitude: %f", 
                          data.latitude, data.longitude, data.altitude)

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        # Haversine formula to calculate the distance between two points on the Earth
        R = 6371000  # Radius of the Earth in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi / 2.0) * math.sin(delta_phi / 2.0) + \
            math.cos(phi1) * math.cos(phi2) * \
            math.sin(delta_lambda / 2.0) * math.sin(delta_lambda / 2.0)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c
        return distance

if __name__ == '__main__':
    try:
        GPSFilter()
    except rospy.ROSInterruptException:
        pass



def gps_callback(data):
    rospy.loginfo("GPS Data: Latitude: %f, Longitude: %f, Altitude: %f", 
                  data.latitude, data.longitude, data.altitude)

def gps_listener():
    rospy.init_node('gps_listener', anonymous=True)
    rospy.Subscriber(subscriber_topic_gps, NavSatFix, gps_callback) #From Plugin (LAT, LONG, ALT)
    rospy.spin()

if __name__ == '__main__':
    try:
        gps_listener()
    except rospy.ROSInterruptException:
        pass
