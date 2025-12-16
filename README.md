# System Bringup and Status Check Guide

## Table of Contents
1. [System Overview](#system-overview)
2. [Checking Bringup Service](#checking-bringup-service)
3. [Network Configuration](#network-configuration)
4. [ROS Topics Verification](#ros-topics-verification)
5. [Storage and SSD Mount](#storage-and-ssd-mount)
6. [Troubleshooting](#troubleshooting)
7. [Useful Commands Summary](#useful-commands-summary)

---

## System Overview

The system runs a ROS-based data acquisition setup that includes:
- Spinnaker camera driver
- 3D Sonar (192.168.2.96)
- Mini AHRS IMU
- BAR30 depth sensor
- Rosbag recording

The bringup service starts automatically on boot via systemd.

---

## Checking Bringup Service

### Check Service Status
```bash
sudo systemctl status bringup.service
```

**Expected output:** `Active: active (running)`

### Check Network Service Status
```bash
sudo systemctl status bringup-network.service
```

### Service Control Commands
```bash
# Start the service
sudo systemctl start bringup.service

# Stop the service
sudo systemctl stop bringup.service

# Restart the service
sudo systemctl restart bringup.service

# Enable service on boot
sudo systemctl enable bringup.service

# Disable service on boot
sudo systemctl disable bringup.service
```

### View Service Logs
```bash
# View recent logs
journalctl -u bringup.service -n 50

# View logs in real-time
journalctl -u bringup.service -f

# View sonar-related logs
journalctl -u bringup.service | grep -i sonar
```

---

## Network Configuration

### Check Multicast Route (Required for Sonar)
```bash
ip route show | grep 224
```

**Expected output:**
```
224.0.0.96 dev eno1 scope link
```

### If Multicast Route is Missing
```bash
sudo ip route add 224.0.0.96/32 dev eno1
```

### Check Network Interface
```bash
ip addr show eno1
```

### Ping Sonar Device
```bash
ping 192.168.2.96
```

---

## ROS Topics Verification

### Source ROS Environment
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

### List All Topics
```bash
rostopic list
```

**Expected topics:**
```
/camera_array/cam0/image_raw
/camera_array/cam0/image_raw/compressed
/camera_array/cam0/camera_info
/mini_ahrs_ros/imu
/mini_ahrs_ros/magnetic_field
/mini_ahrs_ros/temperature
/sonar_3d/raw_data
/sonar_3d/raw_data_multibyte
/bar30/depth
/bar30/pressure
/bar30/temperature
/tf
/tf_static
```

### Check Sonar Topic Rate
```bash
rostopic hz /sonar_3d/raw_data_multibyte
```

**Expected:** ~18-20 Hz

### Check Camera Topic Rate
```bash
rostopic hz /camera_array/cam0/image_raw/compressed
```

### Check IMU Topic Rate
```bash
rostopic hz /mini_ahrs_ros/imu
```

### Check All Topic Rates at Once
```bash
rostopic hz /sonar_3d/raw_data_multibyte &
rostopic hz /camera_array/cam0/image_raw/compressed &
rostopic hz /mini_ahrs_ros/imu &
```

---

## Storage and SSD Mount

### Check SSD Mount Status
```bash
df -h /home/oem/data
```

**Expected output (SSD mounted):**
```
Filesystem      Size  Used Avail Use% Mounted on
/dev/sda1       XXG   XXG   XXG  XX% /home/oem/data
```

### Check Disk Usage
```bash
# Overall disk usage
df -h

# Data folder size
du -sh /home/oem/data

# List bag files with sizes
ls -lh /home/oem/data/*.bag*
```

### Check Active Recording
```bash
ls -la /home/oem/data/*.active
```

### View Rosbag Information
```bash
# For completed bags
rosbag info /home/oem/data/tank_YYYY-MM-DD-HH-MM-SS.bag

# For active recordings
rosbag info /home/oem/data/tank_YYYY-MM-DD-HH-MM-SS.bag.active
```

### If SSD is Not Mounted
```bash
# Mount manually
sudo mount /dev/sda1 /home/oem/data

# Verify mount
df -h /home/oem/data

# Set ownership
sudo chown -R oem:oem /home/oem/data
```

---

## Troubleshooting

### Sonar Not Publishing Data

1. Check multicast route:
```bash
   ip route show | grep 224
```

2. If missing, add route:
```bash
   sudo ip route add 224.0.0.96/32 dev eno1
```

3. Restart bringup service:
```bash
   sudo systemctl restart bringup.service
```

4. Check sonar process is running:
```bash
   ps aux | grep save_sonar
```

### ROS Master Not Running
```bash
# Check if roscore is running
rosnode list

# If error, check bringup service
sudo systemctl status bringup.service

# Restart if needed
sudo systemctl restart bringup.service
```

### Camera Not Publishing
```bash
# Check camera node
rosnode info /acquisition_node

# Check USB connection
lsusb | grep -i flir
```

### Rosbag Not Recording

1. Check logger launch:
```bash
   ps aux | grep rosbag
```

2. Check disk space:
```bash
   df -h /home/oem/data
```

3. Check for recording errors in logs:
```bash
   journalctl -u bringup.service | grep -i record
```

### SSH Connection Lost After Reboot

The multicast route might be too broad. Fix:
```bash
# On local machine, remove broad route
sudo ip route del 224.0.0.0/4 dev eno1

# Add specific route
sudo ip route add 224.0.0.96/32 dev eno1
```

### Recover Incomplete Bag Files
```bash
rosbag reindex /home/oem/data/tank_YYYY-MM-DD-HH-MM-SS.bag.active
```

---

## Useful Commands Summary

| Task | Command |
|------|---------|
| Check bringup status | `sudo systemctl status bringup.service` |
| View service logs | `journalctl -u bringup.service -f` |
| Check multicast route | `ip route show \| grep 224` |
| Add multicast route | `sudo ip route add 224.0.0.96/32 dev eno1` |
| List ROS topics | `rostopic list` |
| Check sonar rate | `rostopic hz /sonar_3d/raw_data_multibyte` |
| Check SSD mount | `df -h /home/oem/data` |
| List bag files | `ls -lh /home/oem/data/*.bag*` |
| View bag info | `rosbag info <filename>` |
| Restart bringup | `sudo systemctl restart bringup.service` |
| Stop bringup | `sudo systemctl stop bringup.service` |

---

## Configuration Files

### Bringup Service File
`/etc/systemd/system/bringup.service`

### Network Service File
`/etc/systemd/system/bringup-network.service`

### Boot Script
`/home/oem/catkin_ws/src/bringup/boot.sh`

### Logger Launch File
`/home/oem/catkin_ws/src/bringup/launch/logger.launch`

### Sonar Script
`/home/oem/Sonar-3D-15-api-example/save_sonar_data.py`

---

## Quick Health Check

Run these commands to verify system is working:
```bash
# 1. Check service
sudo systemctl status bringup.service | grep Active

# 2. Check network route
ip route show | grep 224

# 3. Check SSD
df -h /home/oem/data

# 4. Check sonar
rostopic hz /sonar_3d/raw_data_multibyte

# 5. Check recording
ls -la /home/oem/data/*.active
```

All checks should pass for normal operation.
