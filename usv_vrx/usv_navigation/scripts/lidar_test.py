#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import math

def callback(pointcloud_msg):
    # Extract points from the PointCloud2 message
    points = list(pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True))

    # Calculate the number of points to keep (front 10%)
    num_points = len(points)
    front_10_percent = int(num_points * 0.2)

    # Sort points by x-coordinate to get the front points (assuming x is forward direction)
    points_sorted = sorted(points, key=lambda point: point[0], reverse=True)
    front_points = points_sorted[:front_10_percent]

    # # Find the furthest distance point that is less than 99 meters
    # max_distance = 0
    # for point in front_points:
    #     distance = math.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
    #     if distance < 99 and distance > max_distance:
    #         max_distance = distance

    # # Print the furthest distance point less than 99 meters
    # rospy.loginfo(f"Furthest distance point under 99 meters: {max_distance} meters")

    # Create a new PointCloud2 message for the front points
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = pointcloud_msg.header.frame_id
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]

    front_pointcloud_msg = pc2.create_cloud(header, fields, front_points)

    # Publish the new PointCloud2 message
    pub.publish(front_pointcloud_msg)

if __name__ == '__main__':
    rospy.init_node('pointcloud_front_10_percent', anonymous=True)

    rospy.Subscriber('/velodyne_points', PointCloud2, callback)

    pub = rospy.Publisher('/front_velodyne_points', PointCloud2, queue_size=10)

    rospy.spin()
