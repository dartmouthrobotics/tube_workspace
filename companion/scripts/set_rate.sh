#!/bin/bash

source /opt/ros/noetic/setup.bash
sleep 30
rosrun mavros mavsys rate --all 50
