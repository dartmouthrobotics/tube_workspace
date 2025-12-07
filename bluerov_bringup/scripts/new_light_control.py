#!/usr/bin/env python

import rospy
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import RCOut
import time
import os

from apriltag_ros.msg import AprilTagDetectionArray

MAV_CMD_DO_SET_SERVO = 183
MAV_CMD_DO_REPEAT_SERVO = 184

class LightController():
    def __init__(self):

        self.MULTIBEAM_DIE = 28
        self.killed_multibeam = False

        self.ALL_LIGHTS = 30
        self.LIGHT_1 = 31#32
        self.LIGHT_2 = 32#4
        self.LIGHT_3 = 33#31
        self.LIGHT_4 = 34#33

        self.LIGHT_ARRAY = [self.LIGHT_1, self.LIGHT_2, self.LIGHT_3, self.LIGHT_4]

        self.current_id = 0
        self.LIGHTS_OFF = 29

        self.PAUSE = 5
        self.PAUSE_OFF = 2

        self.sub_apriltag = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback)

        self.LIGHT_CHANNELS = [11, 10, 12, 9]
        self.ON = 1500      # 1300 1500 1700
        self.OFF = 1100
        self.SLEEP_TIME = 2
        self.NUM_CYCLES = 2

        self.pub_light_channels = [1100, 1100, 1100, 1100]

        self.change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.change_param = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        self.send_command = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

        self.pub_light = rospy.Publisher('light_activity', RCOut, queue_size=1)

        rospy.wait_for_service('/mavros/cmd/command')

        self.light_id = 0
        self.msg = RCOut()
        self.msg.header.frame_id = "lights"

        self.first_msg_received = False

        print("Initialized")

    def callback(self, apriltag_data):
        #print("Detected ID: " + str(apriltag_data.detections))
        # new_id = apriltag_data.detections.id

        new_id = 0
        if (apriltag_data.detections):
            new_id = apriltag_data.detections[0].id[0]
        else:
            if not self.first_msg_received:
                light_controller.turn_single_light(light_controller.LIGHT_1)
                light_controller.turn_single_light(light_controller.LIGHT_2)
                light_controller.turn_single_light(light_controller.LIGHT_3)
                light_controller.turn_single_light(light_controller.LIGHT_4)

                self.first_msg_received = True

        if (new_id == self.current_id):
            return

        if ((new_id == self.MULTIBEAM_DIE) and (not self.killed_multibeam)):
            self.current_id = new_id
            self.killed_multibeam = True
            print("Killing multibeam ROS node.")
            os.system("rosnode kill gemini")
            os.system("docker container stop multibeam-daemon")


        elif (new_id == self.LIGHTS_OFF):
            self.turn_off_lights(self.light_id+31)
            self.current_id = new_id


        elif (new_id == self.ALL_LIGHTS):
            save_id = self.current_id
            if (save_id not in self.LIGHT_ARRAY):
                save_id = self.LIGHT_1
            self.current_id = new_id
            print("All light cycle.")

            self.cycle_lights(save_id)


        elif ((new_id == self.LIGHT_1) or
                (new_id == self.LIGHT_2) or
                (new_id == self.LIGHT_3) or
                (new_id == self.LIGHT_4)):

            self.turn_off_lights(self.light_id+31)
            time.sleep(1)
            self.current_id = new_id
            self.turn_single_light(new_id)




    def cycle_lights(self, save_id):
        self.turn_off_lights(self.light_id+31)
        time.sleep(self.PAUSE_OFF)
        print("cycle_lights")

        for i in range(self.NUM_CYCLES):
            for id in self.LIGHT_ARRAY:
                self.turn_single_light(id)
                time.sleep(self.PAUSE)
                self.turn_off_lights(id)
                time.sleep(self.PAUSE_OFF)

        self.turn_single_light(save_id)


    def turn_single_light(self, id):
        self.light_id = id - 31
        print("Turning on Light " + str(self.light_id + 1))

        broadcast = False
        command = MAV_CMD_DO_SET_SERVO
        confirmation = 0
        param1 = self.LIGHT_CHANNELS[self.light_id]
        param2 = self.ON
        param3 = 1.0
        param4 = 2.5
        param5 = param6 = param7 = 0.0
        out = self.send_command(broadcast, command, confirmation, param1, param2, param3, param4, param5, param6, param7)
        #time.sleep(self.PAUSE)

        self.pub_light_channels[self.light_id] = self.ON
        self.msg.channels = self.pub_light_channels
        self.msg.header.stamp = rospy.Time.now()
        self.pub_light.publish(self.msg)


    def turn_off_lights(self, id):
        self.light_id = id - 31

        print("Turning off all lights.")

        broadcast = False
        command = MAV_CMD_DO_SET_SERVO
        confirmation = 0
        param1 = self.LIGHT_CHANNELS[self.light_id]
        param2 = self.OFF
        param3 = 1.0
        param4 = 2.5
        param5 = param6 = param7 = 0.0
        out = self.send_command(broadcast, command, confirmation, param1, param2, param3, param4, param5, param6, param7)

        self.pub_light_channels[self.light_id] = self.ON
        self.msg.channels = self.pub_light_channels
        self.msg.header.stamp = rospy.Time.now()
        self.pub_light.publish(self.msg)


if __name__ == "__main__":

    rospy.init_node("light_cycle", anonymous=False)

    light_controller = LightController()

    rospy.spin()
