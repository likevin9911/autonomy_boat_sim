#!/usr/bin/env python

import rospy
from usv_msg.msg import WaypointRoute, Waypoint
from geometry_msgs.msg import Pose

waypoints = [
    {'x': -36.554, 'y': -0.743, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.688, 'ow': 0.725, 'nav_type': 1, 'station_duration': -1.0, 'speed': 2.0},
    {'x': -39.328, 'y': 39.125, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': -0.005, 'ow': 1.0, 'nav_type': 1, 'station_duration': -1.0, 'speed': 2.0},
    {'x': -2.706, 'y': 42.156, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': -0.686, 'ow': 0.728, 'nav_type': 1, 'station_duration': -1.0, 'speed': 2.0},
    {'x': -0.452, 'y': 0.238, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': -1.000, 'ow': 0.003, 'nav_type': 1, 'station_duration': -1.0, 'speed': 2.0},
]

def publish_waypoint(pub, waypoint):
    wp = Waypoint()
    wp.nav_type = waypoint['nav_type']
    wp.pose.position.x = waypoint['x']
    wp.pose.position.y = waypoint['y']
    wp.pose.position.z = waypoint['z']
    wp.pose.orientation.x = waypoint['ox']
    wp.pose.orientation.y = waypoint['oy']
    wp.pose.orientation.z = waypoint['oz']
    wp.pose.orientation.w = waypoint['ow']
    wp.station_duration = waypoint['station_duration']
    
    waypoint_route = WaypointRoute()
    waypoint_route.waypoints.append(wp)
    waypoint_route.speed = waypoint['speed']
    
    pub.publish(waypoint_route)
    rospy.loginfo(f"Published waypoint: {waypoint}")

def main():
    rospy.init_node('waypoint_publisher')
    pub = rospy.Publisher('/waypoints_cmd', WaypointRoute, queue_size=10)

    rospy.loginfo("Press Enter to publish the next waypoint...")

    for waypoint in waypoints:
        input("Press Enter to publish the next waypoint...")
        rospy.loginfo(f"Publishing waypoint: {waypoint}")
        publish_waypoint(pub, waypoint)
        rospy.sleep(1)  # Give some time for the message to be processed

    rospy.loginfo("All waypoints have been published.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
