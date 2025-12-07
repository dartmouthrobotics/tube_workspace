#!/usr/bin/env python

import serial
import time
import os
import rospy

from apriltag_ros.msg import AprilTagDetectionArray



class LightController():
    def __init__(self):

        self.MULTIBEAM_DIE = 28
	self.killed_multibeam = False

        self.ALL_LIGHTS = 30
        self.LIGHT_1 = 32
        self.LIGHT_2 = 34
        self.LIGHT_3 = 31
        self.LIGHT_4 = 33

        self.LIGHT_ARRAY = [self.LIGHT_1, self.LIGHT_2, self.LIGHT_3, self.LIGHT_4]

        self.current_id = 0
        self.LIGHTS_OFF = 29

        self.PAUSE = 5
	self.PAUSE_OFF = 2

        self.ser = serial.Serial(
            port='/dev/ttyUITube',
            baudrate=9600,
            timeout=.1,
        )

        self.sub_apriltag = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback)

        packet = bytearray([ord('>'), ord('l'), ord('c'), 255, 255, 255, 0, ord('\n')])
        self.ser.write(packet)

	time.sleep(1)

	packet = bytearray([ord('>'), ord('h'), 255, 255, 255, 255, 0, ord('\n')])
        self.ser.write(packet)
	time.sleep(1)
	self.turn_off_lights()
	time.sleep(3)

	packet = bytearray([ord('>'), ord('l'), ord('c'), 0, 0, 0, 0, ord('\n')])
        self.ser.write(packet)


	time.sleep(3)

        print("Initialized")

    def callback(self, apriltag_data):
        # print("Detected ID: " + str(apriltag_data.detections))
        # new_id = apriltag_data.detections.id

	new_id = 0
	if (apriltag_data.detections):
	    new_id = apriltag_data.detections[0].id[0]

            if (new_id == self.current_id):
		return

            if ((new_id == self.MULTIBEAM_DIE) and (not self.killed_multibeam)):
		self.current_id = new_id
		self.killed_multibeam = True
                print("Killing multibeam ROS node.")
                os.system("rosnode kill gemini")
		os.system("docker container stop multibeam-daemon")

                packet = bytearray([ord('>'), ord('l'), ord('s'), 255, 0, 255, 0, ord('\n')])
                self.ser.write(packet)

                time.sleep(3)

                packet = bytearray([ord('>'), ord('l'), ord('c'), 0, 0, 0, 0, ord('\n')])
                self.ser.write(packet)


	    elif (new_id == self.LIGHTS_OFF):
		self.current_id = new_id
		self.turn_off_lights()


            elif (new_id == self.ALL_LIGHTS):
		save_id = self.current_id
		if (save_id not in self.LIGHT_ARRAY):
		    save_id = self.LIGHT_1
		self.current_id = new_id
                print("All light cycle.")

                packet = bytearray([ord('>'), ord('l'), ord('c'), 0, 255, 255, 0, ord('\n')])
                self.ser.write(packet)

                time.sleep(3)

                packet = bytearray([ord('>'), ord('l'), ord('s'), 0, 0, 0, 0, ord('\n')])
                self.ser.write(packet)

                self.cycle_lights(save_id)


            elif ((new_id == self.LIGHT_1) or
                  (new_id == self.LIGHT_2) or
                  (new_id == self.LIGHT_3) or
                  (new_id == self.LIGHT_4)):

		self.current_id = new_id
                self.turn_single_light(new_id)

	    time.sleep(1)


    def cycle_lights(self, save_id):

        for id in self.LIGHT_ARRAY:
            self.turn_single_light(id)
            time.sleep(self.PAUSE)
            #self.turn_off_lights()
            #time.sleep(self.PAUSE_OFF)

        self.turn_single_light(save_id)


    def turn_single_light(self, id):
        light_id = id - 31
        power = [0, 0, 0, 0]

        power[light_id] = 125

        print("Turning on Light " + str(light_id + 1))

        packet = bytearray([ord('>'), ord('h'), power[0], power[1], power[2], power[3], 0, ord('\n')])
        self.ser.write(packet)


    def turn_off_lights(self):
	print("Turning off all lights.")

        packet = bytearray([ord('>'), ord('h'), 0, 0, 0, 0, 0, ord('\n')])
        self.ser.write(packet)


if __name__ == "__main__":
    """
    Initialize node for controlling the lights and multibeam
    """

    rospy.init_node("light_cycle", anonymous=False)

    light_controller = LightController()

    rospy.spin()
