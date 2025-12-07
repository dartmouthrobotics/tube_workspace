#!/bin/bash
cd /home/pi/repositories/ros-apple-silicon/
echo "hello"
docker compose up &
sleep 10
docker compose exec ros /bin/bash -c ". /root/catkin_ws/devel/setup.bash && roslaunch ping360_node example.launch"
