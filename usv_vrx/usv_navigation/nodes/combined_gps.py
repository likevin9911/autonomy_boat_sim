#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

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

    # def transform_to_base_link(self, data, frame_id):
    #     try:
    #         transform = self.tf_buffer.lookup_transform('base_link', frame_id, rospy.Time(0), rospy.Duration(1.0))
    #         pose_stamped = PoseStamped()
    #         pose_stamped.header = data.header
    #         pose_stamped.pose.position.x = data.latitude
    #         pose_stamped.pose.position.y = data.longitude
    #         pose_stamped.pose.position.z = data.altitude
    #         transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
    #         transformed_data = NavSatFix()
    #         transformed_data.header = transformed_pose.header
    #         transformed_data.latitude = transformed_pose.pose.position.x
    #         transformed_data.longitude = transformed_pose.pose.position.y
    #         transformed_data.altitude = transformed_pose.pose.position.z
    #         return transformed_data
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    #         rospy.logerr(f"Transform error: {e}")
    #         return None

    def combine_and_publish(self):
        if self.gps1_data and self.gps2_data:
            combined_data = NavSatFix()
            combined_data.header.stamp = rospy.Time.now()
            combined_data.header.frame_id = "base_link"
            # Simple average of the positions
            combined_data.latitude = (self.gps1_data.latitude + self.gps2_data.latitude) / 2
            combined_data.longitude = (self.gps1_data.longitude + self.gps2_data.longitude) / 2
            combined_data.altitude = (self.gps1_data.altitude + self.gps2_data.altitude) / 2

            self.combined_gps_pub.publish(combined_data)

if __name__ == '__main__':
    rospy.init_node('gps_combiner', anonymous=True)
    gps_combiner = GPSCombiner()
    rospy.spin()
