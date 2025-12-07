#!/usr/bin/python
"""
Push depth to Water Linked Underwater GPS
"""
import requests

import rospy

import message_filters

from mavros_msgs.msg import VFR_HUD
from sensor_msgs.msg import Temperature

url = None

def data_callback(depth_msg, temp_msg=None):
    global url
    if temp_msg is None:
        temp = 24.0 
    else:
        temp = temp_msg.temperature # Temperature in degrees Celsius.

    depth = -depth_msg.altitude # Depth is necessary in meter, positive.
    if depth < 0:
        depth = depth_msg.altitude # Depth is necessary in meter, positive.

    payload = dict(depth=depth, temp=temp)
    r = requests.put(url, json=payload, timeout=10)
    if r.status_code != 200:
        rospy.logerr("Error setting depth: {} {}".format(r.status_code, r.text))
        pass

def main():
    global url
    rospy.init_node("depth_for_water_linked")
    baseurl = rospy.get_param("url", "http://192.168.2.94")

    url = '{}/api/v1/external/depth'.format(baseurl)

    depth_sub = message_filters.Subscriber('mavros/vfr_hud', VFR_HUD)
    temp_sub = message_filters.Subscriber('mavros/temperature/data_raw', Temperature)
    ts = message_filters.ApproximateTimeSynchronizer([depth_sub, temp_sub], 1, 0.1)

    ts.registerCallback(data_callback)
 
    rospy.spin()

if __name__ == "__main__":
    main()
