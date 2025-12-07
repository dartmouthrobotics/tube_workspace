# bluerov_bringup

## Overview

In this package there are:

* launch file that can be run for starting all nodes and the bag recording, starting everything on the top computer.

### License

The source code is released under a [MIT license](bluerov_bringup/LICENSE).

**Author: Alberto Quattrini Li<br />
Affiliation: [Dartmouth College](https://www.dartmouth.edu/)<br />
Maintainer: Alberto Quattrini Li, alberto.quattrini.li@dartmouth.edu**

## Installation

*Note:*  Until https://github.com/ros-drivers/audio_common/pull/121 this pr is merged, you need to use Alberto's fork of audio\_common.

The package has been tested on Ubuntu 16.04. As such, the following instructions
relate to [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu).

### Installation from packages

General package dependencies:

    sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools

ROS package dependencies:

    sudo apt-get install ros-kinetic-mavros-*

Packages that need source compilation:

* [gscam](https://github.com/ros-drivers/gscam.git) NOTE: this is necessary at the time of writing, as in Ubuntu 16.04, the package is compiled with gstreamer-0.1.
* [bluerov](https://github.com/bluerobotics/bluerov-ros-pkg)
* [audio_capture](https://github.com/quattrinili/audio_common.git) NOTE: fork that allows the use of udpsrc.

### Configuration of the robot

Note that some modifications need to be done on the Raspberry Pi of the BlueROV, so that the information from the pixhawk, camera, and audio are published to multiple ports, allowing the use of ROS and QGroundControl at the same time:
* mavproxy
* gstreamer2.param
* gstreamer-audio.sh

## Usage

There is already an launch file that runs nodes for getting the topics related to the pixhawk, camera, and audio:

	roslaunch bluerov_bringup bluerov_bringup.launch
