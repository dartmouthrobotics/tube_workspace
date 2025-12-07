#!/bin/bash
#sleep 60
source /home/pi/.bashrc
source /opt/ros/noetic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
export ROS_IP=192.168.2.2
export LD_LIBRARY_PATH=/home/pi/catkin_ws/devel/lib:/opt/ros/noetic/lib:/home/pi/repositories/tritech_gemini-sdk-v2-0-36-2-for-arm-aarch64_2022-06-20_1643/bin/
/home/pi/repositories/tritech_gemini-sdk-v2-0-36-2-for-arm-aarch64_2022-06-20_1643/bin/GeminiSDKConsoleApp
