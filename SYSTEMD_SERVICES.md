# Jetson Orin Nano - Systemd Auto-Start Services

## Overview
Four systemd services are configured to automatically start on boot, providing plug-and-play functionality for the rover.

**Date Created**: October 31, 2025
**Last Updated**: October 31, 2025 (Added USB Camera service)
**Jetson IP**: 192.168.254.100

---

## Service Architecture

```
systemd services (auto-start on boot)
├── jetson-rover-server.service  (Port 5000)
│   └── Cube Orange MAVLink control
├── jetson-rplidar.service       (ROS 2)
│   └── RPLidar A1 laser scanner @ ~7 Hz
├── jetson-camera.service        (ROS 2)
│   └── USB Camera @ 30 FPS (640x480 JPEG)
└── jetson-sensor-bridge.service (Port 5001)
    └── ROS 2 → HTTP bridge (depends on LiDAR + Camera)
```

---

## Service Files

### 1. jetson-rover-server.service

**Location**: `/etc/systemd/system/jetson-rover-server.service`

```ini
[Unit]
Description=Jetson Rover Control Server (MAVLink → Cube Orange)
After=network.target

[Service]
Type=simple
User=jay
WorkingDirectory=/home/jay
ExecStart=/usr/bin/python3 /home/jay/jetson_rover_server.py
Restart=always
RestartSec=5
StandardOutput=append:/var/log/jetson-rover-server.log
StandardError=append:/var/log/jetson-rover-server.log

[Install]
WantedBy=multi-user.target
```

**Purpose**:
- Provides HTTP REST API on port 5000
- Controls Cube Orange flight controller via MAVLink
- Handles ARM, DISARM, waypoint navigation

**Endpoints**:
- `GET  /api/health` - Health check
- `GET  /api/status` - Rover status (GPS, battery, armed state)
- `POST /api/arm` - ARM motors
- `POST /api/disarm` - DISARM motors
- `POST /api/target` - Send navigation target
- `POST /api/stop` - Emergency stop

**Log File**: `/var/log/jetson-rover-server.log`

---

### 2. jetson-rplidar.service

**Location**: `/etc/systemd/system/jetson-rplidar.service`

```ini
[Unit]
Description=RPLidar A1 ROS 2 Node
After=network.target

[Service]
Type=simple
User=jay
Environment="ROS_DOMAIN_ID=42"
ExecStartPre=/bin/bash -c 'chmod 666 /dev/ttyUSB0 2>/dev/null || true'
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && ros2 launch rplidar_ros rplidar_a1_launch.py'
Restart=always
RestartSec=5
StandardOutput=append:/var/log/jetson-rplidar.log
StandardError=append:/var/log/jetson-rplidar.log

[Install]
WantedBy=multi-user.target
```

**Purpose**:
- Launches RPLidar A1 ROS 2 node
- Publishes laser scan data to `/scan` topic
- 360° obstacle detection at ~7 Hz

**ROS 2 Topics**:
- `/scan` - LaserScan messages (360° polar data)

**Hardware**:
- Device: `/dev/ttyUSB0` (auto-configured permissions)
- Range: 0.15m to 12m
- Scan Rate: ~7 Hz
- Data Points: ~885 points/scan

**Log File**: `/var/log/jetson-rplidar.log`

---

### 3. jetson-camera.service

**Location**: `/etc/systemd/system/jetson-camera.service`

```ini
[Unit]
Description=USB Camera ROS 2 Node
After=network.target

[Service]
Type=simple
User=jay
Environment="ROS_DOMAIN_ID=42"
ExecStartPre=/bin/bash -c 'chmod 666 /dev/video0 2>/dev/null || true'
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p image_width:=640 -p image_height:=480 -p framerate:=30.0'
Restart=always
RestartSec=5
StandardOutput=append:/var/log/jetson-camera.log
StandardError=append:/var/log/jetson-camera.log

[Install]
WantedBy=multi-user.target
```

**Purpose**:
- Launches USB camera ROS 2 node
- Publishes compressed JPEG images to `/image_raw/compressed` topic
- 640x480 resolution at 30 FPS

**ROS 2 Topics**:
- `/image_raw/compressed` - CompressedImage messages (JPEG format)

**Hardware**:
- Device: `/dev/video0` (auto-configured permissions)
- Resolution: 640x480
- Frame Rate: 30 FPS
- Format: YUV422 → JPEG compressed

**Log File**: `/var/log/jetson-camera.log`

---

### 4. jetson-sensor-bridge.service

**Location**: `/etc/systemd/system/jetson-sensor-bridge.service`

```ini
[Unit]
Description=ROS 2 Sensor Bridge (HTTP API)
Requires=jetson-rplidar.service jetson-camera.service
After=network.target jetson-rplidar.service jetson-camera.service

[Service]
Type=simple
User=jay
WorkingDirectory=/home/jay
Environment="ROS_DOMAIN_ID=42"
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && python3 /home/jay/ros2_sensor_bridge.py'
Restart=always
RestartSec=5
StandardOutput=append:/var/log/jetson-sensor-bridge.log
StandardError=append:/var/log/jetson-sensor-bridge.log

[Install]
WantedBy=multi-user.target
```

**Purpose**:
- Bridges ROS 2 topics to HTTP REST API
- Provides sensor data on port 5001
- Subscribes to `/scan` (LiDAR) and `/image_raw/compressed` (camera)

**Endpoints**:
- `GET http://192.168.254.100:5001/api/lidar` - Latest LiDAR scan (JSON)
- `GET http://192.168.254.100:5001/api/camera` - Latest camera image (base64 JPEG)
- `GET http://192.168.254.100:5001/api/sensors/status` - Sensor status

**Dependencies**:
- Requires `jetson-rplidar.service` to be running first
- Starts after network and RPLidar service

**Log File**: `/var/log/jetson-sensor-bridge.log`

---

## Installation

These services are already installed on the Jetson. To reinstall or update:

```bash
# 1. Copy service files to systemd directory
sudo cp jetson-rover-server.service /etc/systemd/system/
sudo cp jetson-rplidar.service /etc/systemd/system/
sudo cp jetson-sensor-bridge.service /etc/systemd/system/

# 2. Reload systemd daemon
sudo systemctl daemon-reload

# 3. Enable services (auto-start on boot)
sudo systemctl enable jetson-rover-server
sudo systemctl enable jetson-rplidar
sudo systemctl enable jetson-sensor-bridge

# 4. Start services now
sudo systemctl start jetson-rover-server
sudo systemctl start jetson-rplidar
sudo systemctl start jetson-sensor-bridge
```

---

## Service Management

### Check Status
```bash
# Check all services
sudo systemctl status jetson-rover-server
sudo systemctl status jetson-rplidar
sudo systemctl status jetson-sensor-bridge

# Quick status check
sudo systemctl is-active jetson-rover-server jetson-rplidar jetson-sensor-bridge
```

### View Logs
```bash
# Real-time log monitoring
sudo tail -f /var/log/jetson-rover-server.log
sudo tail -f /var/log/jetson-rplidar.log
sudo tail -f /var/log/jetson-sensor-bridge.log

# View recent logs with journalctl
sudo journalctl -u jetson-rover-server -f
sudo journalctl -u jetson-rplidar -f
sudo journalctl -u jetson-sensor-bridge -f
```

### Restart Services
```bash
# Restart individual service
sudo systemctl restart jetson-rover-server
sudo systemctl restart jetson-rplidar
sudo systemctl restart jetson-sensor-bridge

# Restart all rover services
sudo systemctl restart jetson-rover-server jetson-rplidar jetson-sensor-bridge
```

### Stop Services
```bash
# Stop individual service
sudo systemctl stop jetson-rover-server

# Stop all rover services
sudo systemctl stop jetson-rover-server jetson-rplidar jetson-sensor-bridge
```

### Disable Auto-Start
```bash
# Disable individual service
sudo systemctl disable jetson-rover-server

# Disable all rover services
sudo systemctl disable jetson-rover-server jetson-rplidar jetson-sensor-bridge
```

---

## Testing Services

### Test Rover Server (Port 5000)
```bash
# Health check
curl http://192.168.254.100:5000/api/health

# Get rover status
curl http://192.168.254.100:5000/api/status | python3 -m json.tool
```

### Test Sensor Bridge (Port 5001)
```bash
# Check sensor status
curl http://192.168.254.100:5001/api/sensors/status | python3 -m json.tool

# Get LiDAR data
curl http://192.168.254.100:5001/api/lidar | python3 -m json.tool | head -50

# Get camera image (base64)
curl http://192.168.254.100:5001/api/camera | python3 -m json.tool
```

### Test ROS 2 Topics
```bash
# List ROS 2 topics
source /opt/ros/humble/setup.bash
ros2 topic list

# Monitor LiDAR scan rate
ros2 topic hz /scan

# View LiDAR data
ros2 topic echo /scan --once

# Monitor camera feed
ros2 topic hz /image_raw/compressed
```

---

## Troubleshooting

### Service Won't Start
```bash
# Check for errors in logs
sudo journalctl -u jetson-rplidar -n 50

# Check for port conflicts
sudo lsof -i :5000  # Rover server
sudo lsof -i :5001  # Sensor bridge

# Check for USB device permissions
ls -la /dev/ttyUSB0  # RPLidar
ls -la /dev/ttyACM0  # Cube Orange
ls -la /dev/video0   # Camera
```

### LiDAR Service Failing
```bash
# Check USB device exists
ls -la /dev/ttyUSB0

# Manually fix permissions
sudo chmod 666 /dev/ttyUSB0

# Restart service
sudo systemctl restart jetson-rplidar
```

### Port Conflicts
```bash
# If port 5001 is already in use
sudo lsof -i :5001
# Kill the conflicting process or reboot

# Reboot Jetson to cleanly start all services
sudo reboot
```

### ROS 2 Environment Issues
```bash
# Verify ROS 2 installation
source /opt/ros/humble/setup.bash
ros2 topic list

# Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID  # Should be 42
```

---

## Auto-Start Verification

After reboot, all three services should start automatically:

```bash
# Wait 30 seconds after reboot, then check
sudo systemctl is-active jetson-rover-server jetson-rplidar jetson-sensor-bridge

# Expected output:
# active
# active
# active

# Test HTTP endpoints
curl http://192.168.254.100:5000/api/health
curl http://192.168.254.100:5001/api/sensors/status
```

---

## Performance

### Resource Usage
- **jetson-rover-server**: ~20MB RAM, <5% CPU
- **jetson-rplidar**: ~50MB RAM, ~10% CPU
- **jetson-sensor-bridge**: ~80MB RAM, ~15% CPU
- **Total**: ~150MB RAM, ~30% CPU (acceptable)

### Update Rates
- Rover control API: On-demand (REST calls)
- LiDAR scan: ~7 Hz
- Camera feed: ~7 Hz (when camera connected)
- Sensor bridge: ~10 Hz polling

---

## Dependencies

### Python Packages
- `pymavlink` - MAVLink protocol
- `flask` - HTTP REST API server
- `rclpy` - ROS 2 Python client library
- `sensor_msgs` - ROS 2 sensor message types
- `cv_bridge` - ROS 2 image conversion

### ROS 2 Packages
- `ros-humble-ros-base` - ROS 2 core
- `ros-humble-rplidar-ros` - RPLidar driver
- `ros-humble-usb-cam` - USB camera driver
- `ros-humble-image-transport` - Image streaming
- `ros-humble-mavros` - MAVLink bridge

---

## Network Configuration

### Jetson Orin Nano
- **IP Address**: 192.168.254.100 (static)
- **Hostname**: jetson1
- **Rover Server**: Port 5000
- **Sensor Bridge**: Port 5001

### Mobile RTK Module
- **IP Address**: 192.168.254.127 (Raspberry Pi 5)
- **Dashboard**: Connects to both ports

### RTK Base Station
- **IP Address**: 192.168.254.165
- **NTRIP**: Provides RTK corrections

---

## Session History

**October 31, 2025 - Evening Session**:
- Created all three systemd service files
- Configured auto-start on boot
- Set up passwordless SSH for automation
- Verified all services working after reboot
- Resolved LiDAR stability issues (port conflicts)
- All services now operational and production-ready

---

## Future Improvements

1. **Create udev rules** for persistent USB device names:
   - `/dev/rplidar` instead of `/dev/ttyUSB0`
   - `/dev/cubeorange` instead of `/dev/ttyACM0`
   - `/dev/camera` instead of `/dev/video0`

2. **Consolidate servers**: Merge jetson_rover_server.py and ros2_sensor_bridge.py into single unified server on port 5000

3. **Add MAVROS2 service**: Create systemd service for Cube Orange MAVLink bridge

4. **Health monitoring**: Add watchdog to restart services on failure

5. **Resource monitoring**: Log CPU/RAM usage to detect performance issues

---

**Last Updated**: October 31, 2025
**Status**: Production-ready, all services operational
**Verified**: Tested after reboot, all auto-start services working
