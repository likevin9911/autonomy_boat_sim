#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalStatusArray

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('waypoint_navigator', anonymous=True)
        self.goal_pub = rospy.Publisher('/wamv/move_base_simple/goal', PoseStamped, queue_size=10)
        self.status_sub = rospy.Subscriber('/wamv/move_base/status', GoalStatusArray, self.status_callback)
        self.waypoints = [
            (-37.868, 18.045)
            #(109.75, 54.5),
            #(44.684, 93.634),
            #(116.690, 19.649)  # Additional waypoints can be added here
        ]
        self.current_waypoint_index = 0
        self.goal_reached = False

    def status_callback(self, msg):
        if not msg.status_list:
            rospy.loginfo("No statuses received in the message.")
            return

        latest_status = msg.status_list[-1].status
        rospy.loginfo("Latest status: {}".format(latest_status))

        if latest_status == 3:  # SUCCEEDED
            self.goal_reached = True
            rospy.loginfo("Reached waypoint {}".format(self.current_waypoint_index + 1))
            self.advance_waypoint()

    def advance_waypoint(self):
        if self.goal_reached and self.current_waypoint_index < len(self.waypoints) - 1:
            self.current_waypoint_index += 1
            self.send_next_waypoint()
        elif self.goal_reached:
            rospy.loginfo("All waypoints reached.")
            rospy.signal_shutdown("Navigation complete.")

    def send_next_waypoint(self):
        self.goal_reached = False
        point = self.waypoints[self.current_waypoint_index]
        goal = PoseStamped()
        goal.header = Header(stamp=rospy.Time.now(), frame_id='map')
        goal.pose.position.x = point[0]
        goal.pose.position.y = point[1]
        goal.pose.orientation.w = 1.0  # No rotation assumed
        rospy.loginfo("Sending goal: x={}, y={}".format(point[0], point[1]))
        self.goal_pub.publish(goal)

    def run(self):
        if self.waypoints:
            rospy.sleep(2)  # Delay briefly to allow connections to be established
            self.send_next_waypoint()
        rospy.spin()

if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
        navigator.run()
    except rospy.ROSInterruptException as e:
        rospy.loginfo("Waypoint navigation interrupted. Error: {}".format(str(e)))
