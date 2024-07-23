#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from usv_msg.msg import WaypointReached

class WaypointMonitor:
    def __init__(self):
        self.current_waypoints = []
        rospy.init_node('waypoint_reached_publisher', log_level=rospy.DEBUG)
        self.waypoint_sub = rospy.Subscriber('waypoints', Path, self.waypoint_callback)
        self.position_sub = rospy.Subscriber('ground_truth', Odometry, self.position_callback)
        self.waypoint_reached_pub = rospy.Publisher('waypoint_reached', WaypointReached, queue_size=10)
        self.tolerance = 1.0  # Tolerance in meters for reaching a waypoint
        rospy.loginfo("WaypointMonitor initialized")

    def waypoint_callback(self, msg):
        rospy.loginfo(f"Received {len(msg.poses)} waypoints")
        self.current_waypoints = msg.poses

    def position_callback(self, odom_msg):
        rospy.loginfo("Position updated")
        # Extract the position from the Odometry message's pose
        self.current_position = odom_msg.pose.pose.position
        self.check_waypoint_reached()

    def check_waypoint_reached(self):
        if not self.current_waypoints:
            rospy.loginfo("No waypoints to process")
            return
        #if not self.current_position:
        #    rospy.loginfo("Current position not available")
        #    return
        distance = self.distance(self.current_position, self.current_waypoints[0].pose.position)
        rospy.loginfo(f"Distance to next waypoint: {distance}")
        if distance < self.tolerance:
            rospy.loginfo(f"Waypoint {self.current_waypoints[0].header.seq} reached")
            waypoint_reached_msg = WaypointReached()
            waypoint_reached_msg.reached = True
            waypoint_reached_msg.waypoint_id = self.current_waypoints[0].header.seq
            self.waypoint_reached_pub.publish(waypoint_reached_msg)
            self.current_waypoints.pop(0)  # Move to the next waypoint

    @staticmethod
    def distance(pos1, pos2):
        return ((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2) ** 0.5

if __name__ == '__main__':
    try:
        monitor = WaypointMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
