# System Bringup and Status Check Guide

## Table of Contents
1. [System Overview](#system-overview)
2. [Hardware Configuration](#hardware-configuration)
3. [Network Setup](#network-setup)
4. [Launch Files](#launch-files)
5. [Checking Bringup Service](#checking-bringup-service)
6. [ROS Topics Verification](#ros-topics-verification)
7. [Storage and SSD Mount](#storage-and-ssd-mount)
8. [Troubleshooting](#troubleshooting)
9. [Useful Commands Summary](#useful-commands-summary)

---

## System Overview

The system runs a ROS Noetic-based data acquisition setup that includes:
- **Spinnaker Camera Driver** - High-speed camera acquisition
- **Oculus M750d Sonar** (169.254.44.32) - Multibeam imaging sonar
- **Mini AHRS IMU** - Inertial measurement unit
- **BAR30 Depth Sensor** - Pressure and depth measurement
- **Rosbag Recording** - Automatic data logging

The bringup service starts automatically on boot via systemd.

---

## Hardware Configuration

### Sensors

| Sensor | Model | Interface | IP Address | Frame Rate |
|--------|-------|-----------|------------|------------|
| Camera | FLIR/Point Grey | USB 3.0 | - | 30 Hz |
| Sonar | Blueprint Subsea Oculus M750d | Ethernet | 169.254.44.32 | ~10 Hz |
| IMU | Mini AHRS | USB Serial | /dev/ttyUSB0 | 100 Hz |
| Depth | Blue Robotics BAR30 | I2C/USB | - | 10 Hz |

### Network Interfaces

- **Ethernet (enp3s0)**: 
  - Primary: 192.168.1.100/24
  - Link-local: 169.254.1.100/16 (for Oculus sonar)

---

## Network Setup

### Required Network Configuration

The Oculus sonar requires both a standard IP and link-local address on the same interface.

#### Check Current Configuration
```bash
ip addr show enp3s0
```

**Expected output:**
```
inet 192.168.1.100/24 ...
inet 169.254.1.100/16 ...
```

#### If Link-Local Address is Missing

**Temporary (until reboot):**
```bash
sudo ip addr add 169.254.1.100/16 dev enp3s0
```

**Permanent (via netplan):**
```bash
sudo nano /etc/netplan/01-network-manager-all.yaml
```
```yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    enp3s0:
      dhcp4: no
      addresses:
        - 192.168.1.100/24
        - 169.254.1.100/16
      optional: true
```
```bash
sudo netplan apply
```

### Verify Sonar Connectivity
```bash
# Check sonar is broadcasting
sudo tcpdump -i enp3s0 -n udp port 52102 -c 5

# Expected: UDP broadcasts from 169.254.44.32 every second
```

---

## Launch Files

### Available Launch Files

#### 1. `bringup.launch` - Start All Sensors
Launches all sensor drivers without recording.
```bash
roslaunch bringup bringup.launch
```

**Includes:**
- Camera (Spinnaker)
- Oculus M750d Sonar
- IMU (Mini AHRS)
- Depth Sensor (BAR30)
- Sonar image republisher

#### 2. `logger.launch` - Recording Only
Records data from running sensors (requires bringup to be running).
```bash
# Terminal 1: Start sensors
roslaunch bringup bringup.launch

# Terminal 2: Start recording
roslaunch bringup logger.launch
```

**Recorded Topics:**
```
/sonar_oculus_node/M750d/ping
/camera_array/cam0/image_raw/compressed
/camera_array/cam0/camera_info
/mini_ahrs_ros/imu
/mini_ahrs_ros/magnetic_field
/mini_ahrs_ros/temperature
/bar30/depth
/bar30/pressure
/bar30/temperature
/tf
/tf_static
```

**Recording Location:** `/home/$(USER)/data/`

#### 3. `bringup_and_record.launch` - Complete System
Starts all sensors AND recording automatically with 5-second delay.
```bash
roslaunch bringup bringup_and_record.launch
```

**Optional Arguments:**
```bash
# Custom recording delay
roslaunch bringup bringup_and_record.launch record_delay:=10

# Custom experiment name
EXPERIMENT=pool_test roslaunch bringup bringup_and_record.launch
```

#### 4. `visualize.launch` - Sonar Visualization
Opens rqt_image_view to display sonar images (requires sonar to be running).
```bash
roslaunch bringup visualize.launch
```

---

## Oculus Sonar Driver Modifications

### Changes Made to `sonar.cpp`

The original sonar driver required several critical fixes to work with the Oculus M750d:

#### 1. **UDP Broadcast Reception**
**Problem:** Socket not configured to receive broadcast packets  
**Fix:** Added `SO_BROADCAST` socket option
```cpp
// Enable broadcast reception
if (setsockopt(sockUDP, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(int)) < 0)
  error("setsockopt(SO_BROADCAST) failed");
```

#### 2. **Auto-Discovery Implementation**
**Problem:** Used blocking `ioctl` that didn't receive broadcasts  
**Fix:** Replaced with `recvfrom()` with timeout
```cpp
// Use recvfrom to receive broadcast
int bytesRead = recvfrom(sockUDP, (char*)&osm, sizeof(osm), 0,
                         (struct sockaddr*)&from, &fromlen);
```

#### 3. **Fire Command Timing**
**Problem:** Fire command sent before TCP connection established  
**Fix:** Moved Fire command to after successful TCP connection
```cpp
ROS_INFO("Connected!");
// Now send the initial Fire command after connection is established
sonar.Fire(mode, ping_rate, range, gain, soundspeed, salinity);
```

#### 4. **Publishing Logic**
**Problem:** Only published when `id > latest_id`, missing initial pings  
**Fix:** Changed to `id != latest_id` to catch all new data
```cpp
if (nbeams > 0 && nbins > 0 && rawSize > 0 && id != latest_id) {
    latest_id = id;
    // ... publish ping
}
```

#### 5. **Error Message Filtering**
**Problem:** Spurious "Unrecognised message ID:128" warnings  
**Fix:** Added filter for known status messages
```cpp
else if (pOmh->msgId != messageDummy && pOmh->msgId != 0x80)
    std::cerr << "Unrecognised message ID:" + std::to_string(pOmh->msgId) + "\n";
```

### Modified Files
```
sonar_oculus/
├── src/
│   ├── sonar.cpp           # Main driver (modified)
│   └── OculusClient.cpp    # Client handler (modified)
└── scripts/
    └── image_republisher.py # Extracts compressed JPEG from ping message
```

---

## Checking Bringup Service

### Check Service Status
```bash
sudo systemctl status robot-sensors.service
```

**Expected output:** `Active: active (running)`

### Service Control Commands
```bash
# Start the service
sudo systemctl start robot-sensors.service

# Stop the service
sudo systemctl stop robot-sensors.service

# Restart the service
sudo systemctl restart robot-sensors.service

# Enable service on boot
sudo systemctl enable robot-sensors.service

# Disable service on boot
sudo systemctl disable robot-sensors.service
```

### View Service Logs
```bash
# View recent logs
journalctl -u robot-sensors.service -n 50

# View logs in real-time
journalctl -u robot-sensors.service -f

# View sonar-related logs
journalctl -u robot-sensors.service | grep -i sonar

# View initialization sequence
journalctl -u robot-sensors.service | grep -E "Initializing|Connected|Fire command"
```

---

## ROS Topics Verification

### Source ROS Environment
```bash
source /opt/ros/noetic/setup.bash
source ~/tube_workspace/catkin_ws/devel/setup.bash
```

### List All Topics
```bash
rostopic list
```

**Expected topics:**
```
/sonar_oculus_node/M750d/ping              # Raw sonar ping data
/sonar_image/compressed                     # Extracted JPEG image
/camera_array/cam0/image_raw/compressed
/camera_array/cam0/camera_info
/mini_ahrs_ros/imu
/mini_ahrs_ros/magnetic_field
/mini_ahrs_ros/temperature
/bar30/depth
/bar30/pressure
/bar30/temperature
/tf
/tf_static
```

### Check Sonar Topic Rate
```bash
rostopic hz /sonar_oculus_node/M750d/ping
```

**Expected:** ~10 Hz

### Check Sonar Data
```bash
# View ping metadata
rostopic echo /sonar_oculus_node/M750d/ping --noarr

# Check image format
rostopic echo /sonar_oculus_node/M750d/ping/ping/format
# Expected: "; jpeg compressed"
```

### Check Camera Topic Rate
```bash
rostopic hz /camera_array/cam0/image_raw/compressed
```

**Expected:** ~30 Hz

### Check IMU Topic Rate
```bash
rostopic hz /mini_ahrs_ros/imu
```

**Expected:** ~100 Hz

### Check All Topic Rates at Once
```bash
rostopic hz /sonar_oculus_node/M750d/ping &
rostopic hz /camera_array/cam0/image_raw/compressed &
rostopic hz /mini_ahrs_ros/imu &
rostopic hz /bar30/depth &
```

---

## Storage and SSD Mount

### Check SSD Mount Status
```bash
df -h ~/data
```

**Expected output (SSD mounted):**
```
Filesystem      Size  Used Avail Use% Mounted on
/dev/sda1       XXG   XXG   XXG  XX% /home/sam/data
```

### Check Disk Usage
```bash
# Overall disk usage
df -h

# Data folder size
du -sh ~/data

# List bag files with sizes
ls -lh ~/data/*.bag*
```

### Check Active Recording
```bash
ls -la ~/data/*.active
```

### View Rosbag Information
```bash
# For completed bags
rosbag info ~/data/tank_YYYY-MM-DD-HH-MM-SS.bag

# For active recordings
rosbag info ~/data/tank_YYYY-MM-DD-HH-MM-SS.bag.active

# Check topics in bag
rosbag info ~/data/tank_*.bag | grep -A 20 "topics:"
```

### If SSD is Not Mounted
```bash
# Check if device exists
lsblk

# Mount manually
sudo mount /dev/sda1 ~/data

# Verify mount
df -h ~/data

# Set ownership
sudo chown -R $USER:$USER ~/data
```

---

## Quick Check Manual

### Sonar Not Publishing Data

#### 1. Check Network Configuration
```bash
# Verify link-local IP is configured
ip addr show enp3s0 | grep 169.254

# Should show: inet 169.254.1.100/16
```

#### 2. Verify Sonar is Broadcasting
```bash
sudo tcpdump -i enp3s0 -n udp port 52102 -c 5
```

**Expected output:**
```
IP 169.254.44.32.52102 > 255.255.255.255.52102: UDP, length 142
```

#### 3. Check Sonar Node Logs
```bash
# If running manually
roslaunch bringup bringup.launch
# Look for:
# [INFO] Temperature OK; the sonar will ping normally
# [INFO] The IP address is 169.254.44.32
# [INFO] Connected!

# If running as service
journalctl -u robot-sensors.service | grep -A 5 "Oculus"
```

#### 4. Restart Services
```bash
# Restart 
sudo systemctl restart robot-sensors.service

# Or restart sonar node
rosnode kill /sonar_oculus_node
# Wait a few seconds, it will auto-restart
```

#### 5. Common Issues

**Issue:** "Waiting for UDP broadcast..." repeats forever
- **Cause:** SO_BROADCAST not set or link-local IP missing
- **Fix:** Verify network configuration and rebuild with modified sonar.cpp

**Issue:** "Connected!" but no ping data
- **Cause:** Fire command not sent or TCP connection failed
- **Fix:** Check `netstat -an | grep 52100` for ESTABLISHED connection

**Issue:** "Unrecognised message ID:128"
- **Cause:** Harmless status message from sonar
- **Fix:** Already filtered in updated code

### ROS Master Not Running
```bash
# Check if roscore is running
rosnode list

# If error, check bringup service
sudo systemctl status robot-sensors.service

# Restart if needed
sudo systemctl restart robot-sensors.service
```

### Camera Not Publishing
```bash
# Check camera node
rosnode info /acquisition_node

# Check USB connection
lsusb | grep -i flir

# Check camera permissions
ls -l /dev/bus/usb/
```

### IMU Not Publishing
```bash
# Check USB serial device
ls -l /dev/ttyUSB*

# Check permissions
sudo usermod -a -G dialout $USER
# (logout and login required)

# Test IMU communication
rosrun mini_ahrs_ros mini_ahrs_standalone_node _usb_port:=/dev/ttyUSB0
```

### Rosbag Not Recording

1. **Check rosbag process is running:**
```bash
ps aux | grep rosbag
```

2. **Check disk space:**
```bash
df -h ~/data
```

3. **Check for recording errors in logs:**
```bash
journalctl -u robot-sensors.service | grep -i record
```

4. **Verify topics exist:**
```bash
rostopic list | grep -E "sonar|camera|imu|bar30"
```

### Recover Incomplete Bag Files
```bash
rosbag reindex ~/data/tank_YYYY-MM-DD-HH-MM-SS.bag.active
```

---

## Useful Commands 

### System Status
| Task | Command |
|------|---------|
| Check service status | `sudo systemctl status robot-sensors.service` |
| View service logs | `journalctl -u robot-sensors.service -f` |
| Restart service | `sudo systemctl restart robot-sensors.service` |
| Stop service | `sudo systemctl stop robot-sensors.service` |

### Network
| Task | Command |
|------|---------|
| Check network config | `ip addr show enp3s0` |
| Add link-local IP | `sudo ip addr add 169.254.1.100/16 dev enp3s0` |
| Check sonar broadcast | `sudo tcpdump -i enp3s0 -n udp port 52102 -c 5` |
| Ping sonar | `ping 169.254.44.32` |

### ROS Topics
| Task | Command |
|------|---------|
| List topics | `rostopic list` |
| Check sonar rate | `rostopic hz /sonar_oculus_node/M750d/ping` |
| Check camera rate | `rostopic hz /camera_array/cam0/image_raw/compressed` |
| View sonar info | `rostopic echo /sonar_oculus_node/M750d/ping --noarr` |

### Storage
| Task | Command |
|------|---------|
| Check SSD mount | `df -h ~/data` |
| List bag files | `ls -lh ~/data/*.bag*` |
| View bag info | `rosbag info ~/data/tank_*.bag` |
| Check active recording | `ls -la ~/data/*.active` |

---

## Configuration Files

### Service Files
- `/etc/systemd/system/robot-sensors.service` - Main systemd service

### Launch Files
- `~/tube_workspace/catkin_ws/src/bringup/launch/bringup.launch` - Start all sensors
- `~/tube_workspace/catkin_ws/src/bringup/launch/logger.launch` - Recording only
- `~/tube_workspace/catkin_ws/src/bringup/launch/bringup_and_record.launch` - Full system

### Network Configuration
- `/etc/netplan/01-network-manager-all.yaml` - Netplan network config

### Modified Sonar Files
- `~/tube_workspace/catkin_ws/src/sonar_oculus/src/sonar.cpp` - Main driver
- `~/tube_workspace/catkin_ws/src/sonar_oculus/src/OculusClient.cpp` - Client handler
- `~/tube_workspace/catkin_ws/src/sonar_oculus/scripts/image_republisher.py` - Image extractor

---

## Quick Health Check

Run these commands to verify the system is working:
```bash
# 1. Check service
sudo systemctl status robot-sensors.service | grep Active

# 2. Check network
ip addr show enp3s0 | grep 169.254

# 3. Check SSD
df -h ~/data

# 4. Check sonar
rostopic hz /sonar_oculus_node/M750d/ping

# 5. Check camera
rostopic hz /camera_array/cam0/image_raw/compressed

# 6. Check recording
ls -la ~/data/*.active
```

All checks should pass for normal operation.

---

## Manual Testing

### Test Individual Sensors

**Sonar only:**
```bash
roslaunch sonar_oculus sonar_oculus.launch
```

**Camera only:**
```bash
roslaunch spinnaker_sdk_camera_driver node_acquisition.launch
```

**IMU only:**
```bash
roslaunch mini_ahrs_ros mini_ahrs_standalone.launch usb_port:=/dev/ttyUSB0
```

**Depth sensor only:**
```bash
rosrun depth_node_py run_node.sh
```

### Test Recording
```bash
# Terminal 1: Start sensors
roslaunch bringup bringup.launch

# Terminal 2: Record manually
rosbag record -o ~/data/test /sonar_oculus_node/M750d/ping /camera_array/cam0/image_raw/compressed

# Terminal 3: Monitor
rostopic hz /sonar_oculus_node/M750d/ping
```

---

## Development Notes

### Building from Source
```bash
cd ~/tube_workspace/catkin_ws
catkin_make
source devel/setup.bash
```

### Testing Changes
```bash
# Test modified code
roslaunch bringup bringup.launch

# If working, update service
sudo systemctl restart robot-sensors.service
```

### Adding New Sensors

1. Add sensor driver to `bringup.launch`
2. Add sensor topics to `logger.launch`
3. Test manually first
4. Update systemd service if needed

---

**Common Log Locations:**
- Service logs: `journalctl -u robot-sensors.service`
- ROS logs: `~/.ros/log/`
- Bag files: `~/data/`
