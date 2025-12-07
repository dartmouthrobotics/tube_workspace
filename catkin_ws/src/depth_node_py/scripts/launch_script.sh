#!/bin/bash
# Command to be typed.
# sudo su -c ./launch_script.sh root

SETUP_FILE="$HOME/catkin_ws/devel/setup.bash"
source /opt/ros/noetic/setup.bash
source $SETUP_FILE
rosrun depth_node_py depth.py

