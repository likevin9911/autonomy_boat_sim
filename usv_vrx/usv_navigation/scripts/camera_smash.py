#!/usr/bin/env python3

import subprocess
from subprocess import Popen, PIPE
import os
import time
import rosnode
import rosgraph
import argparse
import rospy
from std_msgs.msg import *
import psutil



global k
timee = ['rosparam set use_sim_time true']
cp = ['rosclean', 'purge', '-y']
nodes_to_exclude = ['/noise_chatter', '/pointcloud', '/rosout', '/camera_noise']

def get_all_ros_nodes():
    try:
        output = subprocess.check_output(['rosnode', 'list'])
        active_nodes = output.decode('utf-8').split('\n')[:-1]  # Remove the last empty item
        return active_nodes
    except subprocess.CalledProcessError:
        return []



#noi = ['python3','./addLidar+DC+Noise2D.py']
noi_g = ['python3','./camera_noise/green_laser.py']
noi_r = ['python3','./camera_noise/red_laser.py']
noi_lf = ['python3','./camera_noise/light_flash.py']

#noi_rc = ['python3','./camera_noise/random_circles.py']
#noi_im = ['python3','./camera_noise/image_modifier.py']

noi_pub = ['python3','./node_list_dcl.py']


rvi = ['rviz', '-d', 'camera.rviz', '__anonymous:=false', '__name:=camera_noise']
#rvi = ['rviz', '-d', 'pointcloud.rviz', '__anonymous:=false', '__name:=pointcloud']

k_all_nodes = ['rosnode', 'kill', 'interpolated_node', '/word2map_tf','/gt/trajectory_server_loam', '/floam_odom_estimation_node', '/floam_laser_processing_node', '/floam_laser_mapping_node','/base_link/trajectory_server_loam']
#cam_rtab = 'roslaunch realsense2_camera opensource_tracking.launch  > /dev/null 2>&1'
cam_rtab = ['rosparam', 'set', 'use_sim_time', 'true','&&','roslaunch','realsense2_camera','opensource_tracking.launch']


bags = ['camera_lidar2.bag']
#bags = ['camera_lidar1.bag']


ks = ['rosnode', 'kill', '/map_saver']
tim = ['rosparam', 'set', 'use_sim_time', 'true']

mo = ['rosrun', 'map_server', 'map_saver']
state = ['./state.py']



try:
    k = 1
    m = 0
    time1 = subprocess.Popen(tim)
    rviz = subprocess.Popen(rvi)
    pub_noi1 = subprocess.Popen(noi_pub)
    noi1 = subprocess.Popen(noi_r)


    for j in range(1):
        ros_purge = subprocess.Popen(cp)
        time2 = subprocess.Popen(tim)
        #play = ['rosbag', 'play', '--clock', '-q', bags[j], '-u', '10', '--topics', '/velodyne_points', '/camera/color/image_raw']
        #play = ['rosbag', 'play', '--clock', '-q', bags[j], '--topics', '/velodyne_points', '/camera/color/image_raw']
        play = ['rosbag', 'play', '--clock', '-q', bags[j]]

        for i in range(60):
            if k == 4 and i == 0:
                k = 1
            if i == 15 or i == 30 or i == 45:
                k += 1
            m = i - 15 * (k - 1)


            # Run Depth Camera Mapping
            dc_map = subprocess.Popen(cam_rtab)
            #subprocess.Popen(pc_m, shell=True)
            time.sleep(2)
          


            #filename = 'mod_pc_local'+'_bag_'+str(j+1)+'_noi_'+f"{m*0.25+0.25:.3f}"+'_(xc=0,yc=0,R=35)_'+'PC_'+str(k)+'.pcd'
            #filename = 'mod_pc'+'_bag_'+str(j+1)+'_noi_'+f"{m*0.05+0.05:.3f}"+'_(x,y,z=Nosie:-x,-y,-z=Nosie)_'+'PC_'+str(k)+'.pcd'
            #filename = 'add_clouds_grid'+'_bag_'+str(j+1)+'_noi_'+str(int(m*20)+280)+'_(W=20, vx=Nosie)_'+'PC_'+str(k)+'.pcd'
            filename = 'cam_'+'bag_'+str(j+1)+'_noi_'+str(int(m*150)+100)+'_X(-1,1)Y(-1,1)_'+'PC_'+str(k)
            #filename = 'add_snow_local'+'_bag_'+str(j+1)+'_noi_'+str(int(m*100)+1000)+'Width_X(3,3)Y(3,3)_'+'Center_(-2,0)'+'PC_'+str(k)+'.pcd'

            #sav = f'''rosservice call /hdl_graph_slam/save_map "{{resolution: 0.05, destination: '/home/sinloops/lidar-dc/Noisy_Images/{filename}'}}"'''
 #          rosrun pcl_ros pointcloud_to_pcd input:=/map _prefix:=/home/sinloops/lidar-dc/dcl_

            #sav = ['rosrun', 'pcl_ros', 'pointcloud_to_pcd', 'input:=/map', '_prefix:='+filename]
            #sav = ['rosrun', 'pcl_ros', 'pointcloud_to_pcd', 'input:=/map', '_prefix:=/home/sinloops/lidar-dc/' + filename, '> /dev/null 2>&1']\
            print('\n\nDebug SAVE\n\n')
            sav = ['rosrun', 'pcl_ros', 'pointcloud_to_pcd', f'input:=/map', f'_prefix:={filename}']


            time.sleep(0.5)
            msav = subprocess.Popen(sav)
            subprocess.call(play)  # this needs to run until complete before moving onto the next line of code
            print('\n\nDebug Before Killed All Nodes\n\n')
            time.sleep(1)

            #kan = subprocess.Popen(k_all_nodes)
            k_all_nodes = get_all_ros_nodes()
            filtered_nodes = ["rosnode", "kill"] + [node for node in k_all_nodes if node not in nodes_to_exclude]
            kan = subprocess.Popen(filtered_nodes)


            print('\n\nDebug After Killed All Nodes\n\n')
            time.sleep(1)
            print('\n\nKilled All Nodes\n\n')
            print('\n\nBag Done Playing\n\n')

            time.sleep(0.5)
            subprocess.Popen(mo)
            time.sleep(1)
            subprocess.call(ks)
            time.sleep(0.5)

            subprocess.Popen(state)
            time.sleep(0.5)



            print('\n\nDone Making '+ str(i+1) + ' Maps\n\n')
            time.sleep(1)

except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
    pass

nodes = os.popen("rosnode list").readlines()
for i in range(len(nodes)):
    nodes[i] = nodes[i].replace("\n", "")

for node in nodes:
    if node != "/rosout":
        os.system("rosnode kill "+ node)
