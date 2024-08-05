#!/usr/bin/env python

import rospy
from usv_msg.msg import WaypointRoute, Waypoint, WaypointReached
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
import subprocess


def load_topics(filename):
    """ Load topics from a file """
    with open(filename, 'r') as file:
        topics = file.read().strip().split('\n')
    return topics

def start_rosbag_record(bag_name, topics):
    """ Start recording topics to a rosbag """
    command = ['rosbag', 'record', '-O', bag_name] + topics
    return subprocess.Popen(command)

# waypoints = [
#     {'x': 87.5253695297870, 'y': 24.3617247259244, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.0, 'ow': 1.0, 'nav_type': 0, 'station_duration': 0.0, 'speed': 2.0},
#     {'x': 98.6307537809480, 'y': 13.8036193810403, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.0, 'ow': 1.0, 'nav_type': 0, 'station_duration': 0.0, 'speed': 2.0},
#     {'x': 99.0040196539485, 'y': -1.48951189778745, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.0, 'ow': 1.0, 'nav_type': 0, 'station_duration': 0.0, 'speed': 2.0},
#     {'x': 88.4121991053689, 'y': -12.5639547770843, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.0, 'ow': 1.0, 'nav_type': 0, 'station_duration': 0.0, 'speed': 2.0},
#     {'x': 73.0692536246497, 'y': -12.9278591219336, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.0, 'ow': 1.0, 'nav_type': 0, 'station_duration': 0.0, 'speed': 2.0},
#     {'x': 61.9562751346966, 'y': -2.36955653876066, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.0, 'ow': 1.0, 'nav_type': 0, 'station_duration': 0.0, 'speed': 2.0},
#     {'x': 61.5906896583037, 'y': 12.9234287859872, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.0, 'ow': 1.0, 'nav_type': 0, 'station_duration': 0.0, 'speed': 2.0},
#     {'x': 72.1825192326214, 'y': 23.9978242684156, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.0, 'ow': 1.0, 'nav_type': 0, 'station_duration': 0.0, 'speed': 2.0},
#     {'x': 87.5253695297870, 'y': 24.3617247259244, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.0, 'ow': 1.0, 'nav_type': 0, 'station_duration': 0.0, 'speed': 2.0},
#     {'x': 0, 'y': 0, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.0, 'ow': 1.0, 'nav_type': 1, 'station_duration': -1.0, 'speed': 2.0}
# ]

# 4wp
waypoints = [
   {'x': -36.554, 'y': -0.743, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.688, 'ow': 0.725, 'nav_type': 0, 'station_duration': 0.0, 'speed': 2.0},
   {'x': -39.328, 'y': 39.125, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': -0.005, 'ow': 1.0, 'nav_type': 0, 'station_duration': 0.0, 'speed': 2.0},
   {'x': -2.706, 'y': 42.156, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': -0.686, 'ow': 0.728, 'nav_type': 0, 'station_duration': 0.0, 'speed': 2.0},
   {'x': -0.452, 'y': 0.238, 'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': -1.000, 'ow': 0.003, 'nav_type': 1, 'station_duration': -1.0, 'speed': 2.0},
]

current_waypoint_index = 0  # Track the current waypoint index

def publish_waypoint(pub, waypoint):
    path = Path()
    path.header.frame_id = "map"  # Assuming the frame_id that your system uses
    path.header.stamp = rospy.Time.now()

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "map"
    pose_stamped.header.stamp = path.header.stamp
    pose_stamped.pose = Pose(Point(waypoint['x'], waypoint['y'], waypoint['z']),
                             Quaternion(waypoint['ox'], waypoint['oy'], waypoint['oz'], waypoint['ow']))

    path.poses.append(pose_stamped)
    pub.publish(path)
    rospy.loginfo(f"Published waypoint at {waypoint['x']}, {waypoint['y']}")

def waypoint_reached_callback(msg):
    global current_waypoint_index, waypoint_pub
    rospy.loginfo(f"Waypoint reached callback triggered with message: {msg}")
    if msg.reached:
        rospy.loginfo(f"Waypoint {current_waypoint_index} reached.")
        current_waypoint_index += 1  # Move to the next waypoint
        if current_waypoint_index < len(waypoints):  # Check if more waypoints are available
            rospy.loginfo(f"Publishing next waypoint: {waypoints[current_waypoint_index]}")
            publish_waypoint(waypoint_pub, waypoints[current_waypoint_index])
        else:
            rospy.loginfo("All waypoints have been reached.")
            if rosbag_process:
                rosbag_process.terminate()
                rosbag_process.wait()  # Ensure the process has terminated before continuing
            rospy.signal_shutdown("All waypoints reached.")

def main():
    global waypoint_pub, rosbag_process
    rospy.init_node('waypoint_publisher')
    waypoint_pub = rospy.Publisher('/waypoints', Path, queue_size=10)
    rospy.Subscriber('/waypoint_reached', WaypointReached, waypoint_reached_callback)

    topics = load_topics('topics.txt')  # Assume topics.txt is in the same directory
    bag_name = input("Enter the name for the rosbag: ")

    rospy.loginfo("Waiting for the first waypoint to be published...")
    input("Press Enter to continue...")
    rosbag_process = start_rosbag_record(bag_name, topics)

    if waypoints:
        publish_waypoint(waypoint_pub, waypoints[0])  # Publish the first waypoint

    rospy.spin()  # Keep the node alive to listen for callback events
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
