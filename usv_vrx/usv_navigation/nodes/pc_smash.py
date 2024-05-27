#!/usr/bin/env python3

import subprocess
import os
import time
import rospy
from usv_msg.msg import Course
import threading

# Global variable to keep track of the last message time

last_msg_time = None

def launch_process(command):
    process = subprocess.Popen(command)
    #time.sleep(10)  # Adjust sleep time based on expected init duration
    return process

def course_cmd_callback(msg):
    global last_msg_time
    last_msg_time = time.time()
    #rospy.loginfo("Received from /course_cmd")

def stop_recording():
    rospy.loginfo("No messages on /course_cmd received. Stopping rosbag recording.")
    rospy.signal_shutdown("Course completed!")

def manage_roscore():
    """Start or restart roscore, ensuring clean startup and shutdown."""
    global roscore_process
    if roscore_process:
        roscore_process.terminate()
        roscore_process.wait()
    roscore_process = subprocess.Popen('roscore')
    time.sleep(1)  

def main():
    global last_msg_time, roscore_process
    last_msg_time = None
    roscore_process = None

    manage_roscore()

    msg_timeout = 10.0  # Timeout after 30 seconds of no messages

    rospy.init_node('rosbag_recording_setup', anonymous=True)
    rospy.Subscriber("/course_cmd", Course, course_cmd_callback)
    
    #planners = ['dijk', 'nav', 'grid']
    
    # Loop through each planner type
    #for planner in planners:
        #print(f"Starting runs for {planner} planner.")

    #for i in range(1,4):

    try:
        #ros = ['roscore']
        bu = ['roslaunch', 'usv_gazebo', 'usv_bringup.launch', 'gui:=false']
        rvi = ['roslaunch', 'usv_gazebo', 'rviz.launch']
        navst = ['roslaunch', 'usv_gazebo', 'usv_navstack.launch']
        ctrl = ['roslaunch', 'usv_control', 'control.launch']

        #launch_process(ros)
        #time.sleep(2)
        launch_process(bu)
        time.sleep(15)
        launch_process(rvi)
        time.sleep(5)
        launch_process(navst)
        time.sleep(5)
        launch_process(ctrl)
        time.sleep(10)

        script_path = "/home/nope/vrx_ws/src/usv_vrx/usv_navigation/nodes/"
        script_name = "follow_set_of_waypoints.py"
        full_script_path = os.path.join(script_path, script_name)

        recording_directory = os.getcwd()
        rospy.loginfo(f"Recording to directory: {recording_directory}") 

        
        if os.path.exists(full_script_path):
            wpc_command = ['python3', full_script_path] 
            wpc = subprocess.Popen(wpc_command, cwd=script_path)  
            rospy.loginfo("Waypoint follower script started.")
            time.sleep(0.5)  # Give some time to ensure the script is running
            
            # Initialize the last message time to current time before starting the recording
            last_msg_time = time.time()
            
            
            topics_to_record = [
                "/clicked_point",
                "/clock",
                "/course_cmd",
                "/diagnostics",
                "/gazebo/link_states",
                "/gazebo/model_states",
                "/gazebo/parameter_descriptions",
                "/gazebo/parameter_updates",
                "/gazebo/performance_metrics",
                "/gazebo/set_link_state",
                "/gazebo/set_model_state",
                "/initialpose",
                "/nav_marker",
                "/nav_marker_array",
                "/nav_marker_global_planner",
                "/request_waypoints",
                "/rosout",
                "/rosout_agg",
                "/tf",
                "/tf_static",
                "/vrx",
                "/vrx/debug/wind/direction",
                "/vrx/debug/wind/speed",
                "/vrx/dock_2022_placard1/shuffle",
                "/vrx/dock_2022_placard2/shuffle",
                "/vrx/dock_2022_placard3/shuffle",
                "/vrx/task/info",
                "/wamv/global_planner/costmap/costmap",
                "/wamv/global_planner/costmap/costmap_updates",
                "/wamv/global_planner/costmap/footprint",
                "/wamv/global_planner/costmap/inflation_layer/parameter_descriptions",
                "/wamv/global_planner/costmap/inflation_layer/parameter_updates",
                "/wamv/global_planner/costmap/obstacle_layer/parameter_descriptions",
                "/wamv/global_planner/costmap/obstacle_layer/parameter_updates",
                "/wamv/global_planner/costmap/parameter_descriptions",
                "/wamv/global_planner/costmap/parameter_updates",
                "/wamv/global_planner/costmap/static_layer/parameter_descriptions",
                "/wamv/global_planner/costmap/static_layer/parameter_updates",
                "/wamv/global_planner/goal",
                "/wamv/global_planner/planner/parameter_descriptions",
                "/wamv/global_planner/planner/parameter_updates",
                "/wamv/global_planner/planner/potential",
                "/wamv/ground_truth",
                "/wamv/joint_states",
                "/wamv/map",
                "/wamv/map_metadata",
                "/wamv/map_updates",
                "/wamv/odom",
                "/wamv/robot_localization/gps/filtered",
                "/wamv/robot_localization/odometry/filtered",
                "/wamv/robot_localization/odometry/gps",
                "/wamv/robot_localization/set_pose",
                "/wamv/sensors/gps/gps/fix",
                "/wamv/sensors/gps/gps/fix/position/parameter_descriptions",
                "/wamv/sensors/gps/gps/fix/position/parameter_updates",
                "/wamv/sensors/gps/gps/fix/status/parameter_descriptions",
                "/wamv/sensors/gps/gps/fix/status/parameter_updates",
                "/wamv/sensors/gps/gps/fix/velocity/parameter_descriptions",
                "/wamv/sensors/gps/gps/fix/velocity/parameter_updates",
                "/wamv/sensors/gps/gps/fix_velocity",
                "/wamv/sensors/imu/imu/data",
                "/wamv/sensors/imu/imu/data/accel/parameter_descriptions",
                "/wamv/sensors/imu/imu/data/accel/parameter_updates",
                "/wamv/sensors/imu/imu/data/bias",
                "/wamv/sensors/imu/imu/data/rate/parameter_descriptions",
                "/wamv/sensors/imu/imu/data/rate/parameter_updates",
                "/wamv/sensors/imu/imu/data/yaw/parameter_descriptions",
                "/wamv/sensors/imu/imu/data/yaw/parameter_updates",
                "/wamv/sensors/lidars/lidar/scan",
                "/wamv/sensors/lidars/lidar_wamv/points",
                "/wamv/thrusters/lateral_thrust_angle",
                "/wamv/thrusters/lateral_thrust_cmd",
                "/wamv/thrusters/left_thrust_angle",
                "/wamv/thrusters/left_thrust_cmd",
                "/wamv/thrusters/right_thrust_angle",
                "/wamv/thrusters/right_thrust_cmd",
                "/wamv/visualization_marker",
                "/wamv/visualization_marker_array",
                "/wamv/waypoints",
                "/waypoints_cmd",
            ]
            
            record_processes = []
            #for base_name in base_names:
            #bag_name = f'dijk_GPSnois0.5_zeroEnvParam_9wp_{i}.bag' #0.0 0.2 0.5 1.0 1.5 2.0
            #bag_name = f'grid_GPSnoise0.2_zeroEnvParam_9wp_1.bag' #0.0 0.2 0.5 1.0
            bag_name = f'nav_GPSnoise0.2_zeroEnvParam_9wp_1.bag'
            record_command = ['rosbag', 'record', '-O', bag_name] + topics_to_record
            record_processes.append(subprocess.Popen(record_command, cwd=recording_directory))
            
            # Wait indefinitely until a shutdown signal is received
            while not rospy.is_shutdown():
                if last_msg_time and (time.time() - last_msg_time > msg_timeout):
                    stop_recording()
                time.sleep(1)
            
            # Terminate all rosbag record processes
            for process in record_processes:
                process.terminate()
                process.wait()
                
        else:
            rospy.logerr("Error: Script not found at " + full_script_path)

    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
        pass

    finally:
        
        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            os.system("rosnode kill "+ node)
        #nodes_to_kill = ['/gazebo', '/path_to_waypoints', '/wamv/global_planner', '/wamv/map_server', '/wamv/mapping', '/wamv/odom_relay', '/wamv/pointcloud_to_laserscan', '/wamv/robot_localization/ekf_localization'
        #                '/wamv/robot_localization/navsat_transform_node', '/wamv/robot_state_publisher', '/wamv/wamv_course_controller', '/wamv/wamv_tf_expander', '/wamv/wamv_waypoint_follower']
        #for node in nodes_to_kill:
        #    rospy.loginfo("Shutting down selected ROS nodes.")
        #    os.system(f"rosnode kill {node}")
            #rospy.loginfo("Shutting down selected ROS nodes.")
            os.system('rosnode kill -a')
            os.system('killall gzserver gzclient rviz')
            os.system('killall -9 rosmaster')
            #subprocess.Popen('roscore')
            #manage_roscore()

if __name__ == "__main__":
    main()
