#!/bin/bash
#sleep 30.0
IDX=$(cat /home/oem/bluerov_bringup/param/bag_file_index.txt)
source /opt/ros/noetic/setup.bash
source /home/oem/catkin_ws/devel/setup.bash
source /home/oem/.bashrc
export ROS_IP=192.168.2.185
cd /home/oem/Sonar-3D-15-api-example
echo "hello sonar3d"
python3 interface_sonar_api.py --ip 192.168.2.96 --acoustic True --speed 1500
sleep 15.0
python3 save_sonar_data.py --index "${IDX}"

