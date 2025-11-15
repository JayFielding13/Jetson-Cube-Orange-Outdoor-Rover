# Jetson Cube Orange Outdoor Rover - Project Index

**Created:** November 7, 2025
**Jetson IP:** 192.168.254.100
**Dashboard IP:** 192.168.254.127
**RTK Base IP:** 192.168.254.165

---

## Project Files

### Core Server Applications

| File | Description | Location | Status |
|------|-------------|----------|--------|
| `jetson_rover_server.py` | Legacy rover control server (port 5000) | This folder + Jetson | ✓ Deployed |
| `jetson_rover_server_mavlink.py` | MAVLink-based server variant | This folder + Jetson | ✓ Deployed |
| `config.py` | System configuration parameters | This folder + Jetson | ✓ Active |

### Session Documentation

| File | Description | Date | Topics Covered |
|------|-------------|------|----------------|
| `SESSION_NOTES_OCT31_ROS2.md` | ROS2 integration session | Oct 31, 2025 | ROS2 Humble install, RPLidar, camera, MAVROS2 |
| `SESSION_LOG_NOV01_2025.md` | MAVROS2 & RTK GPS integration | Nov 1, 2025 | RTK setup, GPSRAW migration, RTCM forwarder |
| `SESSION_LOG_NOV02_2025.md` | Survey mode UI improvements | Nov 2, 2025 | Fence posts, layout reorganization |
| `NEXT_SESSION_TODO_NOV02.md` | Upcoming session tasks | Nov 2, 2025 | RTK convergence monitoring, testing |

### System Documentation

| File | Description | Key Information |
|------|-------------|-----------------|
| `SYSTEMD_SERVICES.md` | Systemd service configurations | Auto-start setup, service management |
| `DEPLOYMENT_CHECKLIST.md` | Deployment procedures | Step-by-step deployment guide |
| `README.md` | **Main project documentation** | Complete system overview and reference |

### Ultrasonic Sensor Module

| File | Description | Status |
|------|-------------|--------|
| `esp32_ultrasonic_raw.ino` | ESP32 firmware for 6 ultrasonic sensors | ✓ Ready |
| `ultrasonic_bridge.py` | ROS2 node for sensor data processing | ✓ Ready |
| `ultrasonic_monitor.py` | Standalone debugging tool (no ROS2) | ✓ Ready |
| `jetson-ultrasonic-bridge.service` | Systemd service for auto-start | ✓ Ready |
| `ESP32_PINOUT_DIAGRAM.md` | Physical pin layout and connections | ✓ Complete |
| `ESP32_ULTRASONIC_WIRING_GUIDE.md` | Complete wiring instructions | ✓ Complete |
| `ULTRASONIC_SETUP_GUIDE.md` | Setup and deployment guide | ✓ Complete |

---

## Files on Jetson (192.168.254.100)

### ROS2 Launch Files
```
~/mavros_cube_orange.launch.py        # MAVROS2 configuration for Cube Orange
```

### Service Scripts
```
~/ros2_unified_bridge.py              # ROS2 HTTP/UDP telemetry bridge (port 5001)
~/rtcm_mqtt_forwarder.py              # MQTT to MAVROS2 RTK forwarder
~/jetson_rover_server.py              # Legacy rover server (port 5000)
~/ultrasonic_bridge.py                # ROS2 ultrasonic sensor bridge
```

### Monitoring Tools
```
~/monitor_rtk_fix.sh                  # Real-time RTK status monitor
~/monitor_gps_acquisition.sh          # GPS acquisition monitor
~/ultrasonic_monitor.py               # Ultrasonic sensor debugging tool
```

### Systemd Service Definitions
```
/etc/systemd/system/jetson-mavros2.service
/etc/systemd/system/jetson-unified-bridge.service
/etc/systemd/system/jetson-rplidar.service
/etc/systemd/system/jetson-rtcm-forwarder.service
/etc/systemd/system/jetson-rover-server.service
/etc/systemd/system/jetson-ultrasonic-bridge.service
```

### Log Files
```
/var/log/jetson-rover-server.log
/var/log/jetson-rplidar.log
/var/log/jetson-sensor-bridge.log
```

---

## System Architecture Summary

### Hardware Stack
```
┌─────────────────────────────────────────────┐
│  Jetson Orin Nano (192.168.254.100)        │
│  - ROS2 Humble                              │
│  - MAVROS2                                  │
│  - RPLidar Driver                           │
│  - Camera Driver                            │
│  - Ultrasonic Sensor Bridge                │
└─────────┬──────────────────┬────────────────┘
          │ USB              │ USB
  ┌───────▼────────┐    ┌───▼──────────────┐
  │  Cube Orange+  │    │  ESP32 DevKit    │
  │  ArduRover FW  │    │  + 6x AJ-SR04M   │
  └───────┬────────┘    │  Ultrasonics     │
          │ CAN         └──────────────────┘
  ┌───────▼────────┐
  │  HERE 3+ GPS   │
  │  RTK Capable   │
  └────────────────┘
```

### Network Stack
```
WiFi Network: 192.168.254.0/24
├── Jetson Orin Nano (.100) - ROS2 compute
├── Raspberry Pi 5 (.127)   - Dashboard UI
└── RTK Base (RTKPi) (.165) - RTCM corrections
```

### Software Services
```
Jetson Services (auto-start on boot):
├── jetson-mavros2.service            → Cube Orange communication
├── jetson-unified-bridge.service     → HTTP/UDP telemetry (port 5001)
├── jetson-rplidar.service            → LiDAR sensor driver
├── jetson-rtcm-forwarder.service     → RTK corrections to GPS
├── jetson-ultrasonic-bridge.service  → Ultrasonic sensor array
└── jetson-rover-server.service       → Legacy control (port 5000)
```

---

## Key Capabilities

### ✅ Current Features

**Navigation & Positioning:**
- RTK GPS positioning (centimeter-level accuracy when RTK Fixed)
- Real-time GPS telemetry via HTTP API
- Waypoint navigation support
- Follow-me mode
- Geofence management

**Sensors:**
- 360° LiDAR scanning (RPLidar A1)
- 6x ultrasonic sensors (AJ-SR04M, 360° coverage, 6m range)
- USB camera feed
- GPS position, heading, altitude
- Satellite count, HDOP, fix type

**Control Interfaces:**
- Web dashboard (Raspberry Pi 5)
- HTTP API (port 5001)
- ROS2 topics
- MAVLink commands (via Cube Orange)

**Autonomy:**
- ROS2-based sensor fusion
- RTK correction forwarding
- Real-time obstacle detection (LiDAR)
- Survey mode with fence post visualization

---

## Development Workflow

### 1. Editing Code Locally

Edit files in this directory on your development machine:
```bash
cd "~/Desktop/Mini Rover Development/Jetson Cube Orange Outdoor Rover"
# Edit files as needed
```

### 2. Deploy to Jetson

```bash
# Copy individual file
scp jetson_rover_server.py jay@192.168.254.100:~/

# Copy multiple files
scp *.py jay@192.168.254.100:~/

# Copy and restart service
scp ros2_unified_bridge.py jay@192.168.254.100:~/
ssh jay@192.168.254.100 "sudo systemctl restart jetson-unified-bridge"
```

### 3. Test on Jetson

```bash
# SSH to Jetson
ssh jay@192.168.254.100

# Check service status
sudo systemctl status jetson-unified-bridge

# View logs
sudo journalctl -u jetson-unified-bridge -f

# Test manually
python3 ~/ros2_unified_bridge.py
```

### 4. Update Documentation

After making changes, update:
- Session logs in this directory
- `README.md` if architecture changes
- `NEXT_SESSION_TODO.md` for future tasks

---

## Quick Commands Reference

### Service Management
```bash
# Check all Jetson services
ssh jay@192.168.254.100 "sudo systemctl status jetson-*"

# Restart specific service
ssh jay@192.168.254.100 "sudo systemctl restart jetson-mavros2"

# View service logs
ssh jay@192.168.254.100 "sudo journalctl -u jetson-mavros2 -f"
```

### GPS Monitoring
```bash
# Check current GPS status
curl http://192.168.254.100:5001/api/gps | python3 -m json.tool

# Monitor RTK convergence
ssh jay@192.168.254.100 "~/monitor_rtk_fix.sh"

# Check RTCM corrections
ssh jay@192.168.254.100 "sudo journalctl -u jetson-rtcm-forwarder -n 20"
```

### ROS2 Commands
```bash
# List ROS2 topics
ssh jay@192.168.254.100
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
ros2 topic list

# Monitor GPS topic
ros2 topic echo /mavros/gps1/raw --once

# Check LiDAR
ros2 topic hz /scan

# Check Ultrasonic Sensors
ros2 topic list | grep ultrasonic
ros2 topic echo /ultrasonic/front --once
ros2 topic hz /ultrasonic/pointcloud
```

---

## Next Steps

Refer to `NEXT_SESSION_TODO_NOV02.md` for the prioritized task list:

**Immediate:**
1. Monitor RTK convergence to Fixed status (fix_type 6)
2. Test rover navigation with RTK precision
3. Document RTK performance results

**High Priority:**
1. Implement unified bridge graceful shutdown
2. Consolidate rover servers (merge ports 5000 + 5001)
3. Add battery monitoring from MAVROS2

**Future Enhancements:**
1. SLAM mapping (slam_toolbox)
2. Nav2 autonomous navigation
3. Object detection with camera (YOLOv8)

---

## Version History

| Date | Version | Changes | Author |
|------|---------|---------|--------|
| Nov 8, 2025 | 1.1 | Ultrasonic sensor array integration (ESP32 + 6x AJ-SR04M) | Jay |
| Nov 7, 2025 | 1.0 | Initial project organization and documentation | Jay |
| Nov 2, 2025 | 0.9 | Survey mode UI improvements, fence posts | Jay |
| Nov 1, 2025 | 0.8 | MAVROS2 migration, RTK GPS integration | Jay |
| Oct 31, 2025 | 0.7 | ROS2 Humble installation, sensor integration | Jay |

---

## Contact & Support

**Project Designer:** Anatoly "Tolya" Makarov
**Developer:** Jay
**Project Location:** `~/Desktop/Mini Rover Development/Jetson Cube Orange Outdoor Rover/`

For troubleshooting, refer to:
- `README.md` - Comprehensive system documentation
- Session logs - Detailed implementation notes
- `SYSTEMD_SERVICES.md` - Service configuration

---

**Last Updated:** November 8, 2025
