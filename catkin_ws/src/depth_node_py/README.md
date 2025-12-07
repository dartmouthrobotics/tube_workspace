# Depth node
ROS driver for the [BAR30](https://bluerobotics.com/store/sensors-cameras/sensors/bar-depth-pressure-sensor/) pressure sensor.

## Configuration
Currently assumes an FT232H for converting i2c to USB, such as [Adafruit FT232H breakout]( https://www.adafruit.com/product/2264?srsltid=AfmBOooP4qmrw4U4Shh0vieSEjy3pmEpwMlXaabe-pU7nMImCcFcvII9)

## To create udev rules for the ft232h.
rosrun depth_node_py create_udev_rules

# To run the node.
rosrun depth_node_py run_node.sh

## Disable password.
 http://askubuntu.com/questions/147241/execute-sudo-without-password
Add 
<user> ALL=(ALL) NOPASSWD: ALL
