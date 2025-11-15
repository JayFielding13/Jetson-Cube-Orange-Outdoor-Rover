# Jetson Cube Orange Outdoor Rover

**High-precision autonomous outdoor rover with RTK GPS and ROS2 integration**

---

## Overview

This is a professional-grade autonomous outdoor rover platform combining:
- **Jetson Orin Nano** - Main compute platform running ROS2 Humble
- **Cube Orange+** - Flight controller running ArduRover firmware
- **HERE 3+ GPS** - RTK-capable GPS with centimeter-level accuracy
- **RPLidar A1** - 360° 2D laser scanner for obstacle detection
- **USB Camera** - Vision system for future object detection

**Key Features:**
- RTK GPS positioning (<5cm accuracy)
- ROS2 Humble sensor integration
- MAVROS2 integration with Cube Orange
- Real-time telemetry via UDP/HTTP
- Web-based dashboard control and visualization
- Autonomous navigation capabilities

---

## System Architecture

### Hardware Components

| Component | Model | IP Address | Connection | Purpose |
|-----------|-------|------------|------------|---------|
| **Main Computer** | Jetson Orin Nano 8GB | 192.168.254.100 | WiFi | ROS2, sensors, autonomy |
| **Flight Controller** | Cube Orange+ | - | USB (MAVROS2) | Motor control, GPS |
| **Control Terminal** | Raspberry Pi 5 | 192.168.254.127 | WiFi | Dashboard, control UI |
| **RTK Base Station** | Raspberry Pi (RTKPi) | 192.168.254.165 | WiFi | RTCM corrections |
| **GPS Module** | HERE 3+ RTK GPS | - | CAN (via Cube) | Position, heading |
| **LiDAR** | RPLidar A1 | - | USB | Obstacle detection |
| **Camera** | USB Webcam | - | USB | Vision (future) |

### Network Architecture

```
┌──────────────────────────────────────────────────────┐
│                 192.168.254.0/24                      │
│                   WiFi Network                        │
└──────────────────────────────────────────────────────┘
           │              │              │
    ┌──────▼──────┐ ┌────▼─────┐ ┌─────▼──────┐
    │   Jetson    │ │   Pi 5   │ │   RTKPi    │
    │ .254.100    │ │ .254.127 │ │ .254.165   │
    │             │ │          │ │            │
    │ ROS2 Humble │ │Dashboard │ │RTCM Server │
    │ MAVROS2     │ │Control UI│ │MQTT Broker │
    └─────┬───────┘ └──────────┘ └────────────┘
          │
      ┌───▼────┐
      │  Cube  │
      │Orange+ │
      │        │
      │ArduRover
      └────┬───┘
           │
      ┌────▼────┐
      │ HERE 3+ │
      │RTK GPS  │
      └─────────┘
```

### Software Stack

**Jetson Orin Nano (192.168.254.100):**
```
ROS2 Humble
├── MAVROS2                    # Cube Orange communication
├── RPLidar ROS2 Driver        # Laser scanner
├── USB Camera Driver          # Vision input
├── ROS2 Unified Bridge        # HTTP/UDP telemetry (port 5001)
├── RTCM Forwarder             # RTK corrections to GPS
└── Jetson Rover Server        # Legacy control (port 5000)
```

---

## Quick Start

### 1. Power On Sequence

1. Power on RTK Base Station (192.168.254.165)
2. Wait for base station GPS lock (~60 seconds)
3. Power on Jetson Orin Nano (192.168.254.100)
4. Wait for all ROS2 services to start (~30 seconds)
5. Power on Raspberry Pi 5 Dashboard (192.168.254.127)
6. Launch dashboard: `python3 09_dashboard_enhanced.py`

### 2. Verify System Status

**Check Jetson Services:**
```bash
ssh jay@192.168.254.100
sudo systemctl status jetson-mavros2
sudo systemctl status jetson-unified-bridge
sudo systemctl status jetson-rplidar
sudo systemctl status jetson-rtcm-forwarder
```

**Check GPS Status:**
```bash
curl http://192.168.254.100:5001/api/gps | python3 -m json.tool
```

Expected output:
- `fix_type`: 6 (RTK Fixed) or 5 (RTK Float)
- `satellites_visible`: >15
- `hdop`: <0.5

### 3. Dashboard Control

On Raspberry Pi 5 (192.168.254.127):
```bash
cd ~/robot-control-terminal
DISPLAY=:0 python3 09_dashboard_enhanced.py
```

**Dashboard Features:**
- Real-time GPS visualization
- LiDAR obstacle view
- Camera feed
- Waypoint navigation
- Geofence management
- Follow-me mode
- Survey mode with fence posts

---

## System Services

### Jetson Systemd Services

| Service | Port | Description | Auto-start |
|---------|------|-------------|------------|
| `jetson-mavros2.service` | - | MAVROS2 ROS2 node for Cube Orange | ✓ |
| `jetson-unified-bridge.service` | 5001 | ROS2 HTTP/UDP telemetry bridge | ✓ |
| `jetson-rplidar.service` | - | RPLidar A1 ROS2 driver | ✓ |
| `jetson-rtcm-forwarder.service` | - | MQTT to MAVROS2 RTK bridge | ✓ |
| `jetson-rover-server.service` | 5000 | Legacy rover control server | ✓ |

### Service Management

**View logs:**
```bash
sudo journalctl -u jetson-mavros2 -f
sudo journalctl -u jetson-unified-bridge -f
sudo journalctl -u jetson-rtcm-forwarder -f
```

**Restart services:**
```bash
sudo systemctl restart jetson-mavros2
sudo systemctl restart jetson-unified-bridge
```

---

## RTK GPS System

### RTK Architecture

```
RTK Base Station (192.168.254.165)
          │
          │ MQTT (RTCM corrections)
          │
          ▼
   RTCM Forwarder (Jetson)
          │
          │ MAVLink GPS_RTCM_DATA
          │
          ▼
    Cube Orange+
          │
          ▼
     HERE 3+ GPS
```

### RTK Status Monitoring

**Check RTK corrections flow:**
```bash
ssh jay@192.168.254.100
sudo journalctl -u jetson-rtcm-forwarder -n 20
```

**Monitor GPS fix type:**
```bash
ssh jay@192.168.254.100
~/monitor_rtk_fix.sh
```

**GPS Fix Types:**
- 0 = No GPS
- 1 = No Fix
- 2 = 2D Fix
- 3 = 3D Fix (standard GPS, 2-5m accuracy)
- 4 = DGPS
- 5 = RTK Float (10-50cm accuracy)
- 6 = RTK Fixed (<5cm accuracy) ✓ TARGET

### RTK Convergence Time

| Start Type | Time to RTK Float | Time to RTK Fixed |
|------------|-------------------|-------------------|
| Cold Start | 2-10 minutes | 5-30 minutes |
| Warm Start | 1-5 minutes | 2-15 minutes |
| Hot Start | 30s-2 minutes | 1-5 minutes |

**Requirements for RTK Fixed:**
- Base station <20km away
- 15+ satellites visible
- HDOP <0.5
- Clear sky view
- RTCM corrections flowing (2-5 Hz)

---

## ROS2 Topics

### Key Topics

**GPS & Navigation:**
```bash
/mavros/global_position/global     # GPS position (NavSatFix)
/mavros/gps1/raw                   # Raw GPS data (GPSRAW)
/mavros/gps1/rtk                   # RTK status
/mavros/gps_rtk/send_rtcm          # RTCM corrections input
```

**Sensors:**
```bash
/scan                              # RPLidar laser scan
/camera/image_raw                  # Camera feed
/camera/image_raw/compressed       # Compressed camera
```

**Control:**
```bash
/mavros/rc/override                # RC override commands
/mavros/setpoint_velocity/cmd_vel  # Velocity commands
```

### Topic Inspection

```bash
# List all topics
ros2 topic list

# View topic data
ros2 topic echo /mavros/gps1/raw --once

# Check topic frequency
ros2 topic hz /scan

# View topic info
ros2 topic info /mavros/global_position/global
```

---

## HTTP API Reference

### Base URL
```
http://192.168.254.100:5001/api
```

### Endpoints

**GPS Data:**
```bash
GET /api/gps
```
Returns:
```json
{
  "latitude": 45.123456,
  "longitude": -122.123456,
  "altitude": 123.45,
  "fix_type": 6,
  "satellites_visible": 17,
  "hdop": 0.07,
  "timestamp": 1234567890.123
}
```

**LiDAR Scan:**
```bash
GET /api/lidar/scan
```

**Camera Feed:**
```bash
GET /api/camera/latest
```

**Rover Control:**
```bash
POST /api/control/arm
POST /api/control/disarm
POST /api/waypoint
```

---

## File Locations

### Jetson Orin Nano (192.168.254.100)

**ROS2 Launch Files:**
```
~/mavros_cube_orange.launch.py        # MAVROS2 launch configuration
```

**Service Scripts:**
```
~/ros2_unified_bridge.py              # HTTP/UDP telemetry bridge
~/rtcm_mqtt_forwarder.py              # MQTT to MAVROS2 forwarder
```

**Monitoring Tools:**
```
~/monitor_rtk_fix.sh                  # RTK status monitor
~/monitor_gps_acquisition.sh          # GPS acquisition monitor
```

**Service Configs:**
```
/etc/systemd/system/jetson-mavros2.service
/etc/systemd/system/jetson-unified-bridge.service
/etc/systemd/system/jetson-rplidar.service
/etc/systemd/system/jetson-rtcm-forwarder.service
```

**Logs:**
```
/var/log/jetson-rover-server.log
/var/log/jetson-rplidar.log
/var/log/jetson-sensor-bridge.log
```

### Development Machine

**Session Notes:**
```
SESSION_LOG_NOV01_2025.md             # MAVROS2 migration & RTK integration
SESSION_LOG_NOV02_2025.md             # Survey mode UI improvements
NEXT_SESSION_TODO_NOV02.md            # Next session tasks
```

**Configuration:**
```
config.py                             # System configuration
jetson_rover_server.py                # Legacy server
jetson_rover_server_mavlink.py        # MAVLink server
```

---

## Current Status

### ✅ Completed Features

**Core System:**
- [x] ROS2 Humble installed and configured
- [x] MAVROS2 integration with Cube Orange
- [x] RPLidar A1 integration
- [x] USB camera integration
- [x] Systemd auto-start services

**RTK GPS:**
- [x] RTK corrections forwarding (MQTT → MAVROS2)
- [x] GPS satellite count display
- [x] RTK status monitoring
- [x] GPSRAW message format migration

**Dashboard:**
- [x] Real-time GPS visualization
- [x] LiDAR obstacle view
- [x] Camera feed display
- [x] Waypoint navigation
- [x] Geofence management with fence posts
- [x] Survey mode with statistics
- [x] Follow-me mode

### 🔄 In Progress

**Immediate Priority:**
- [ ] Monitor RTK convergence to Fixed status (fix_type 6)
- [ ] Verify <5cm accuracy with RTK Fixed
- [ ] Document RTK convergence time

**High Priority:**
- [ ] Unified bridge graceful shutdown (signal handlers)
- [ ] Consolidate rover servers (ports 5000 + 5001)
- [ ] Battery monitoring from MAVROS2

### 📋 Future Enhancements

**Medium Priority:**
- [ ] Obstacle visualization on dashboard map
- [ ] SLAM (slam_toolbox)
- [ ] Nav2 autonomous navigation
- [ ] Object detection (YOLOv8)

**Nice to Have:**
- [ ] rosbag2 recording
- [ ] Udev rules for USB devices
- [ ] Performance optimization

---

## Configuration Parameters

### GPS Settings

**Cube Orange Parameters:**
```
GPS_TYPE = 1              # AUTO
GPS_AUTO_CONFIG = 1       # Enable auto-config
GPS_INJECT_TO = 1         # Inject to first GPS
GPS_TYPE2 = 0             # No second GPS
```

### Network Settings

```python
JETSON_IP = "192.168.254.100"
DASHBOARD_IP = "192.168.254.127"
RTK_BASE_IP = "192.168.254.165"
NETWORK = "192.168.254.0/24"
GATEWAY = "192.168.254.254"
```

### ROS2 Settings

```bash
ROS_DOMAIN_ID=42
ROS_LOCALHOST_ONLY=0
```

---

## Troubleshooting

### GPS Not Achieving RTK Fixed

**Check RTCM corrections flow:**
```bash
ssh jay@192.168.254.100
sudo journalctl -u jetson-rtcm-forwarder -f
```
Should see: "RTCM Stats: XXXXX bytes, XX messages"

**Check MAVROS RTK topic:**
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
ros2 topic hz /mavros/gps_rtk/send_rtcm
```
Should show 1-5 Hz

**Verify base station:**
```bash
ping 192.168.254.165
ssh jay@192.168.254.165
sudo systemctl status gps-publisher
```

### Service Won't Start

**Check logs:**
```bash
sudo journalctl -u jetson-mavros2 -n 50
```

**Restart all services:**
```bash
sudo systemctl restart jetson-mavros2
sudo systemctl restart jetson-unified-bridge
sudo systemctl restart jetson-rtcm-forwarder
```

### Dashboard Freeze

**Kill and restart:**
```bash
ssh jay@192.168.254.127
pkill -9 -f '09_dashboard_enhanced.py'
cd ~/robot-control-terminal
DISPLAY=:0 python3 09_dashboard_enhanced.py &
```

### High CPU/Temperature

**Check power mode:**
```bash
sudo /usr/sbin/nvpmodel -q
```

**Monitor with jtop:**
```bash
sudo jtop
```

---

## Safety

### Pre-Operation Checklist

- [ ] RTK base station powered and GPS locked
- [ ] Jetson all services running (systemctl status)
- [ ] GPS fix type ≥ 3 (preferably 5 or 6)
- [ ] Satellite count ≥ 10
- [ ] Dashboard connected and responsive
- [ ] Clear area for testing (30m+ radius)
- [ ] Emergency stop accessible

### Operating Limits

- **GPS Requirements:** 10+ satellites, HDOP <2.0
- **RTK Requirements:** 15+ satellites, base <20km, HDOP <0.5
- **Weather:** No rain (electronics protection needed)
- **Temperature:** -10°C to 50°C operating range
- **Testing Area:** Open field, clear sky view, no obstacles

---

## Development Team

**Project Designer:** Anatoly "Tolya" Makarov
**Hardware/Software Development:** Jay
**Documentation Date:** November 7, 2025

---

## Related Documentation

- `SYSTEMD_SERVICES.md` - Service configuration details
- `SESSION_NOTES_OCT31_ROS2.md` - ROS2 integration session
- `SESSION_LOG_NOV01_2025.md` - MAVROS2 & RTK integration
- `SESSION_LOG_NOV02_2025.md` - Survey mode improvements
- `NEXT_SESSION_TODO_NOV02.md` - Upcoming tasks
- `DEPLOYMENT_CHECKLIST.md` - Deployment procedures

---

## Quick Reference Commands

**SSH to Jetson:**
```bash
ssh jay@192.168.254.100
```

**Check all services:**
```bash
sudo systemctl status jetson-*
```

**Monitor RTK:**
```bash
~/monitor_rtk_fix.sh
```

**View GPS data:**
```bash
curl http://192.168.254.100:5001/api/gps | python3 -m json.tool
```

**Deploy dashboard:**
```bash
scp 09_dashboard_enhanced.py jay@192.168.254.127:~/robot-control-terminal/
ssh jay@192.168.254.127 "pkill -9 python3; cd ~/robot-control-terminal && DISPLAY=:0 python3 09_dashboard_enhanced.py &"
```

---

**Last Updated:** November 7, 2025
**System Status:** Operational - RTK converging
