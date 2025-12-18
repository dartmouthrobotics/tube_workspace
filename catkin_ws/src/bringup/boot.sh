#!/bin/bash


source /opt/ros/noetic/setup.bash
source /home/oem/catkin_ws/devel/setup.bash


roscore & 
sleep 15

# bash /home/oem/companion/scripts/start_sonar_3d.sh &

roslaunch bringup bringup.launch &
roslaunch bringup logger.launch &

wait

