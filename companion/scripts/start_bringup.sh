#!/bin/bash
IDX=$(cat /home/pi/catkin_ws/src/bluerov_bringup/param/bag_file_index.txt)
NEW_IDX=$((${IDX} + 1))
printf ${NEW_IDX} > /home/pi/catkin_ws/src/bluerov_bringup/param/bag_file_index.txt

source /opt/ros/noetic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
source /home/pi/.bashrc
export ROS_IP=192.168.2.2
sleep 15
python3 /home/pi/catkin_ws/src/ping_nodelet/pingmessage.py
roslaunch bluerov_bringup bluerov_bringup.launch bag_file_index:="${NEW_IDX}"
sleep 5.0
#source /home/pi/sam_code_ws/devel/setup.bash && roslaunch onboot_auto_sensor_logging republish.launch
#sleep 5.0
