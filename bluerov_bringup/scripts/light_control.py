#!/usr/bin/env python3
import rospy
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import RCOut


MAV_CMD_DO_SET_SERVO = 183
MAV_CMD_DO_REPEAT_SERVO = 184

class LightController():
    def __init__(self):
        # Light: top left, bottom left, bottom right, top right
        #self.LIGHT_CHANNELS = [9, 10, 11, 12]
        self.LIGHT_CHANNELS = [9, 12]
        #self.LIGHT_CHANNELS = [9]
        self.ON = 1700      # 1300 1500 1700 1900
        self.OFF = 1100
        self.ACTIVE_TIME = 5
        self.SLEEP_TIME = 2

        self.pub_light_channels = [1100, 1100, 1100, 1100]

        self.change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.change_param = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        self.send_command = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

        self.pub_light = rospy.Publisher('light_activity', RCOut, queue_size=1)

        print("Initialized")



    def spin(self):
        rospy.wait_for_service('/mavros/cmd/command')

        light_ind = 0
        msg = RCOut()
        msg.header.frame_id = "lights"
        light_blinking = False # TODO parameter

        while not rospy.is_shutdown():
            """
            Turning one light on.
            """
            try:
                print("turning on: " + str(self.LIGHT_CHANNELS[light_ind]))

                broadcast = False
                command = MAV_CMD_DO_SET_SERVO
                confirmation = 0
                param1 = self.LIGHT_CHANNELS[light_ind]
                param2 = self.OFF
                param3 = 1.0
                param4 = 2.5
                param5 = param6 = param7 = 0.0
                out = self.send_command(broadcast, command, confirmation, param1, param2, param3, param4, param5, param6, param7)

                self.pub_light_channels[light_ind] = self.OFF
                msg.channels = self.pub_light_channels
                msg.header.stamp = rospy.Time.now()
                self.pub_light.publish(msg)

                # Wait for 1 second with light ON
                rospy.sleep(self.ACTIVE_TIME)

            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed")

            if light_blinking:
                """
                Turning the light off.
                """
                try:
                    print("turning off: " + str(self.LIGHT_CHANNELS[light_ind]))

                    broadcast = False
                    command = MAV_CMD_DO_SET_SERVO
                    confirmation = 0
                    param1 = self.LIGHT_CHANNELS[light_ind]
                    param2 = self.OFF
                    param3 = 1.0
                    param4 = 2.5
                    param5 = param6 = param7 = 0.0

                    out = self.send_command(broadcast, command, confirmation, param1, param2, param3, param4, param5, param6, param7)

                    self.pub_light_channels[light_ind] = self.OFF
                    msg.channels = self.pub_light_channels
                    msg.header.stamp = rospy.Time.now()
                    self.pub_light.publish(msg)

                    # Wait for 1 second with light OFF
                    rospy.sleep(self.SLEEP_TIME)

                except rospy.ServiceException as e:
                    rospy.loginfo("Service call failed")

            """
            Update which light will be used in next iteration.
            """
            light_ind = light_ind + 1
            if (light_ind >= len(self.LIGHT_CHANNELS)):
                light_ind = light_ind % len(self.LIGHT_CHANNELS)



if __name__ == "__main__":
    """
    Initialize node for controlling the lights in a cycle
    """

    rospy.init_node("light_cycle", anonymous=False)

    light_controller = LightController()
    light_controller.spin()
