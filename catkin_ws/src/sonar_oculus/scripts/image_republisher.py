#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from sonar_oculus.msg import OculusPing

def ping_callback(msg):
    """Extract compressed image from OculusPing and republish"""
    # The compressed image is in msg.ping
    compressed_img = msg.ping
    compressed_img.header = msg.header
    pub.publish(compressed_img)

if __name__ == '__main__':
    rospy.init_node('sonar_image_republisher')
    
    # Subscribe to sonar ping
    rospy.Subscriber('/sonar_oculus_node/M750d/ping', OculusPing, ping_callback)
    
    # Publish compressed image
    pub = rospy.Publisher('/sonar_image/compressed', CompressedImage, queue_size=10)
    
    rospy.loginfo("Sonar image republisher started")
    rospy.spin()
