#!/usr/bin/env python3

import yaml

file_name = "/home/pi/catkin_ws/src/bluerov_bringup/param/bag_file_index.yaml"
with open(file_name, 'r') as file:
    config = yaml.safe_load(file)

prev_index = config['bag_file_name_index']
new_index = int(prev_index) + 1

config['bag_file_name_index'] = str(new_index)

with open(file_name, 'w') as file:
    file.write(yaml.dump(config, default_flow_style=False))
