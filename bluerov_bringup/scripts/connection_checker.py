#!/usr/bin/python
import rospy
import subprocess
import shlex
from subprocess import Popen, PIPE
from threading import Timer

rospy.init_node("connection_checker")
rospy.loginfo("Beginning to check for connections to the robot.")


def run(cmd, timeout_sec):
    """ Source: https://stackoverflow.com/a/10012262 """
    proc = Popen(shlex.split(cmd), stdout=PIPE, stderr=PIPE)
    timer = Timer(timeout_sec, proc.kill)
    try:
        timer.start()
        stdout, _ = proc.communicate()
        return stdout
    finally:
        timer.cancel()

start_time = rospy.get_time()

rate = rospy.Rate(hz=1)
while not rospy.is_shutdown():

    reached_robot = False
    out = run("ping 192.168.2.2 -c 1", timeout_sec=0.5)
    if "1 received" in out:
        reached_robot = True
    
    secs_elapsed = rospy.get_time() - start_time
    if secs_elapsed > 5 and not reached_robot:
        rospy.logerr("Connection checker was unable to ping the "
            + "BlueROV's Raspberry Pi (192.168.2.2). "
            + "You may not be connected to the robot. "
            + "Is the USB cable plugged in?")
        exit()
    rate.sleep()
