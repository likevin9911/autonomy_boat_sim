#!/usr/bin/env python
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from sklearn.cluster import DBSCAN
import numpy as np
import math
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

# Function to transform a point from LiDAR frame to global frame
def transform_lidar_to_global(point, target_frame, source_frame):
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    point_stamped = PointStamped()
    point_stamped.header.frame_id = source_frame
    point_stamped.point.x = point[0]
    point_stamped.point.y = point[1]
    point_stamped.point.z = point[2]

    try:
        transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
        transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        return transformed_point.point.x, transformed_point.point.y, transformed_point.point.z
    except tf2_ros.LookupException as e:
        rospy.logerr(f"Transform lookup failed: {e}")
        return None

# Function to calculate bounding box from clustered points
def calculate_bounding_box(points):
    min_x = min(point[0] for point in points)
    max_x = max(point[0] for point in points)
    min_y = min(point[1] for point in points)
    max_y = max(point[1] for point in points)
    min_z = min(point[2] for point in points)
    max_z = max(point[2] for point in points)
    return (min_x, min_y, min_z), (max_x, max_y, max_z)

# Function to calculate the centroid of a bounding box
def calculate_centroid(bbox_min, bbox_max):
    centroid_x = (bbox_min[0] + bbox_max[0]) / 2
    centroid_y = (bbox_min[1] + bbox_max[1]) / 2
    centroid_z = (bbox_min[2] + bbox_max[2]) / 2
    return centroid_x, centroid_y, centroid_z

# Function to publish bounding boxes as markers in RViz
def publish_bounding_box(marker_pub, bbox_min, bbox_max, frame_id, marker_id):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "bounding_box"
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = (bbox_min[0] + bbox_max[0]) / 2
    marker.pose.position.y = (bbox_min[1] + bbox_max[1]) / 2
    marker.pose.position.z = (bbox_min[2] + bbox_max[2]) / 2
    marker.scale.x = bbox_max[0] - bbox_min[0]
    marker.scale.y = bbox_max[1] - bbox_min[1]
    marker.scale.z = bbox_max[2] - bbox_min[2]
    marker.color.a = 0.5  # Transparency
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    marker_pub.publish(marker)

def callback(pointcloud_msg):
    # Extract points from the PointCloud2 message
    points = list(pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True))

    # Calculate the number of points to keep (front 20%)
    num_points = len(points)
    front_20_percent = int(num_points * 0.2)

    # Sort points by x-coordinate to get the front points (assuming x is forward direction)
    points_sorted = sorted(points, key=lambda point: point[0], reverse=True)
    front_points = points_sorted[:front_20_percent]

    # Cluster the front points using DBSCAN
    points_np = np.array(front_points)
    clustering = DBSCAN(eps=0.5, min_samples=10).fit(points_np)
    labels = clustering.labels_

    unique_labels = set(labels)
    marker_id = 0

    for label in unique_labels:
        if label == -1:
            continue  # Skip noise points

        cluster_points = [p for p, l in zip(front_points, labels) if l == label]
        bbox_min, bbox_max = calculate_bounding_box(cluster_points)
        centroid = calculate_centroid(bbox_min, bbox_max)

        # Transform centroid to GPS coordinates
        transformed_centroid = transform_lidar_to_global(centroid, "gps_frame", pointcloud_msg.header.frame_id)
        
        if transformed_centroid:
            rospy.loginfo(f"Cluster centroid GPS coordinates: {transformed_centroid}")

        # Publish the bounding box marker
        publish_bounding_box(marker_pub, bbox_min, bbox_max, pointcloud_msg.header.frame_id, marker_id)
        marker_id += 1

    # Publish the new PointCloud2 message for the front points
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = pointcloud_msg.header.frame_id
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]

    front_pointcloud_msg = pc2.create_cloud(header, fields, front_points)
    pub.publish(front_pointcloud_msg)

if __name__ == '__main__':
    rospy.init_node('pointcloud_front_20_percent', anonymous=True)

    rospy.Subscriber('/velodyne_points', PointCloud2, callback)
    pub = rospy.Publisher('/front_velodyne_points', PointCloud2, queue_size=10)
    marker_pub = rospy.Publisher('/bounding_box_markers', Marker, queue_size=10)

    rospy.spin()
