# Jetson Cube Orange Outdoor Rover

**High-precision autonomous outdoor rover with RTK GPS and ROS2 integration**

---

## Overview

This is a professional-grade autonomous outdoor rover platform combining:
- **Jetson Orin Nano** - Main compute platform running ROS2 Humble
- **Cube Orange+** - Flight controller running ArduRover firmware
- **HERE 3+ GPS** - RTK-capable GPS with centimeter-level accuracy
- **RPLidar A1** - 360 degree 2D laser scanner for obstacle detection
- **USB Camera** - Vision system for future object detection

**Key Features:**
- RTK GPS positioning (<5cm accuracy)
- ROS2 Humble sensor integration
- MAVROS2 integration with Cube Orange
- Real-time telemetry via UDP/HTTP
- Web-based dashboard control and visualization
- Reactive obstacle avoidance (VFH algorithm)
- AprilTag visual tracking and following

---

## Project Structure

```
Jetson Cube Orange Outdoor Rover/
├── simulation/              # Desktop Gazebo/RViz simulation
│   ├── ros2_ws/src/jetson_rover_sim/   # Simulation ROS2 package
│   ├── launch_local_sim.sh             # Launch simulation locally
│   └── run_xbox_controller.sh          # Xbox controller teleop
│
├── rover/                   # Real hardware (Jetson deployment)
│   ├── scripts/             # Python scripts for Jetson
│   │   ├── jetson_rover_server.py      # Main control server
│   │   ├── config.py                   # GPIO pins, thresholds
│   │   └── ultrasonic_bridge.py        # Sensor ROS2 node
│   ├── firmware/            # ESP32 ultrasonic sensor firmware
│   ├── config/              # Configuration files
│   └── systemd/             # Auto-start service files
│
├── shared/                  # Code for both simulation & real hardware
│   └── ros2_ws/src/jetson_rover_bridge/  # HTTP API, obstacle avoidance
│
├── docs/                    # All documentation
│   ├── guides/              # Setup and how-to guides
│   ├── session_logs/        # Development session notes
│   └── architecture/        # System design documents
│
├── tools/                   # Development utilities
│   └── bin/                 # CLI tools (arduino-cli)
│
└── archive/                 # Legacy code
```

### Directory Purposes

| Directory | Description |
|-----------|-------------|
| `simulation/` | Everything for desktop testing - Gazebo worlds, URDF models, RViz configs |
| `rover/` | Code deployed to the physical Jetson rover - scripts, firmware, services |
| `shared/` | ROS2 packages that work identically with simulation or real hardware |
| `docs/` | Consolidated documentation - guides, session logs, architecture docs |

---

## Quick Start

### Simulation (Desktop Development)

```bash
# Build simulation workspace
cd simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# Launch full simulation
ros2 launch jetson_rover_sim full_simulation.launch.py

# Or use the convenience script
./simulation/launch_local_sim.sh
```

### Real Hardware (Jetson Deployment)

See the [Deployment Checklist](docs/DEPLOYMENT_CHECKLIST.md) for full procedures.

```bash
# SSH to Jetson (via Tailscale VPN)
ssh jay@100.91.191.47

# Check services
sudo systemctl status jetson-*

# Monitor RTK GPS
~/monitor_rtk_fix.sh
```

---

## System Architecture

### Hardware Components

| Component | Model | Tailscale IP | Hostname | Purpose |
|-----------|-------|--------------|----------|---------|
| **Main Computer** | Jetson Orin Nano 8GB | 100.91.191.47 | jetson1 | ROS2, sensors, autonomy |
| **Flight Controller** | Cube Orange+ | - | - | Motor control, GPS |
| **Control Terminal** | Raspberry Pi 5 | 100.73.233.124 | beaconpi | Dashboard, control UI |
| **RTK Base Station** | Raspberry Pi (RTKPi) | 100.66.67.11 | rtkpi | RTCM corrections |
| **Ubuntu Desktop** | Development PC | 100.73.129.15 | jay-desktop | Simulation, development |
| **GPS Module** | HERE 3+ RTK GPS | - | - | Position, heading |
| **LiDAR** | RPLidar A1 | - | - | Obstacle detection |
| **Ultrasonic** | 6x AJ-SR04M | - | - | 360 degree proximity |
| **Camera** | USB Webcam | - | - | Vision, AprilTag detection |

### Network Architecture (Tailscale VPN)

All devices are connected via Tailscale mesh VPN for secure access from anywhere.

```
┌──────────────────────────────────────────────────────┐
│              Tailscale Mesh VPN (100.x.x.x)          │
└──────────────────────────────────────────────────────┘
     │              │              │              │
┌────▼─────┐ ┌─────▼─────┐ ┌─────▼─────┐ ┌──────▼──────┐
│  Jetson  │ │ beaconpi  │ │  rtkpi    │ │jay-desktop  │
│100.91.   │ │100.73.    │ │100.66.    │ │100.73.      │
│191.47    │ │233.124    │ │67.11      │ │129.15       │
│          │ │           │ │           │ │             │
│ROS2+MAV  │ │ Dashboard │ │RTCM Server│ │ Simulation  │
└────┬─────┘ └───────────┘ └───────────┘ └─────────────┘
     │
 ┌───▼────┐
 │  Cube  │
 │Orange+ │
 │ArduRover
 └────┬───┘
      │
 ┌────▼────┐
 │ HERE 3+ │
 │RTK GPS  │
 └─────────┘
```

### Software Stack

**Jetson Orin Nano (100.91.191.47 / jetson1):**
```
ROS2 Humble
├── MAVROS2                    # Cube Orange communication
├── RPLidar ROS2 Driver        # Laser scanner
├── USB Camera Driver          # Vision input
├── Ultrasonic Bridge          # ESP32 sensor data
├── ROS2 Unified Bridge        # HTTP/UDP telemetry (port 5001)
├── Reactive Obstacle Avoidance # VFH steering algorithm
├── RTCM Forwarder             # RTK corrections to GPS
└── Jetson Rover Server        # Legacy control (port 5000)
```

---

## ROS2 Packages

### jetson_rover_sim (Simulation)

Location: `simulation/ros2_ws/src/jetson_rover_sim/`

Provides Gazebo simulation with:
- Full URDF model with accurate dimensions
- Simulated sensors (LiDAR, camera, GPS, ultrasonics)
- Test world environments
- RViz visualization configs

**Launch files:**
- `full_simulation.launch.py` - Complete Gazebo + RViz setup
- `spawn_rover.launch.py` - Spawn in Gazebo only
- `view_rover.launch.py` - View model without physics

### jetson_rover_bridge (Shared)

Location: `shared/ros2_ws/src/jetson_rover_bridge/`

Provides unified interface for simulation and real hardware:
- HTTP REST API (port 5001)
- GPS format conversion
- AprilTag detection and following
- Reactive obstacle avoidance (VFH)
- Multi-sensor fusion (LiDAR + ultrasonic)

**Launch files:**
- `ground_control_bridge.launch.py` - HTTP + GPS bridge
- `apriltag_follow_test.launch.py` - Full obstacle avoidance test

---

## HTTP API Reference

### Base URL
```
Simulation: http://localhost:5001/api
Real Robot: http://100.91.191.47:5001/api  (via Tailscale)
```

### Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/health` | GET | Health check |
| `/api/status` | GET | Robot status + GPS + odometry |
| `/api/gps` | GET | GPS data with RTK status |
| `/api/arm` | POST | ARM motors |
| `/api/disarm` | POST | DISARM motors |
| `/api/target` | POST | Send GPS waypoint |
| `/api/stop` | POST | Emergency stop |

---

## RTK GPS System

### RTK Status Monitoring

```bash
ssh jay@100.91.191.47
~/monitor_rtk_fix.sh
```

**GPS Fix Types:**
- 0 = No GPS
- 1 = No Fix
- 2 = 2D Fix
- 3 = 3D Fix (standard GPS, 2-5m accuracy)
- 4 = DGPS
- 5 = RTK Float (10-50cm accuracy)
- 6 = RTK Fixed (<5cm accuracy) - TARGET

### RTK Convergence Time

| Start Type | Time to RTK Float | Time to RTK Fixed |
|------------|-------------------|-------------------|
| Cold Start | 2-10 minutes | 5-30 minutes |
| Warm Start | 1-5 minutes | 2-15 minutes |
| Hot Start | 30s-2 minutes | 1-5 minutes |

---

## System Services (Jetson)

| Service | Port | Description |
|---------|------|-------------|
| `jetson-mavros2.service` | - | MAVROS2 for Cube Orange |
| `jetson-unified-bridge.service` | 5001 | HTTP/UDP telemetry bridge |
| `jetson-rplidar.service` | - | RPLidar A1 driver |
| `jetson-rtcm-forwarder.service` | - | RTK corrections forwarder |
| `jetson-rover-server.service` | 5000 | Legacy control server |

**Service management:**
```bash
# View logs
sudo journalctl -u jetson-mavros2 -f

# Restart services
sudo systemctl restart jetson-mavros2
sudo systemctl restart jetson-unified-bridge
```

---

## Current Status

### Completed Features

- [x] ROS2 Humble integration
- [x] MAVROS2 with Cube Orange
- [x] RTK GPS corrections forwarding
- [x] RPLidar A1 integration
- [x] 6x Ultrasonic sensor array (ESP32)
- [x] USB camera integration
- [x] Gazebo simulation environment
- [x] Reactive obstacle avoidance (VFH algorithm)
- [x] AprilTag detection and following
- [x] Multi-sensor fusion (LiDAR + ultrasonic)
- [x] Web dashboard with real-time visualization
- [x] Systemd auto-start services

### In Progress

- [ ] RTK Fixed convergence optimization
- [ ] Battery monitoring from MAVROS2
- [ ] Nav2 autonomous navigation

### Future Enhancements

- [ ] SLAM mapping (slam_toolbox)
- [ ] YOLOv8 object detection
- [ ] rosbag2 recording

---

## Documentation

All documentation is in the `docs/` directory:

| Document | Description |
|----------|-------------|
| [Deployment Checklist](docs/DEPLOYMENT_CHECKLIST.md) | Pre-operation checklist |
| [Desktop Quickstart](docs/guides/DESKTOP_QUICKSTART.md) | Desktop dev environment setup |
| [ROS2 Gazebo Quickstart](docs/guides/ROS2_GAZEBO_QUICKSTART.md) | ROS2 + Gazebo installation |
| [Reactive Avoidance Testing](docs/architecture/REACTIVE_AVOIDANCE_TESTING.md) | Obstacle avoidance testing |
| [AprilTag Follow Guide](docs/architecture/APRILTAG_FOLLOW_TESTING_GUIDE.md) | Visual tracking testing |

Hardware-specific docs are in `rover/`:
- ESP32 wiring diagrams
- Ultrasonic sensor setup
- Systemd service configuration

---

## Troubleshooting

### GPS Not Achieving RTK Fixed

```bash
# Check RTCM corrections flow
ssh jay@100.91.191.47
sudo journalctl -u jetson-rtcm-forwarder -f
# Should see: "RTCM Stats: XXXXX bytes, XX messages"

# Check MAVROS RTK topic
ros2 topic hz /mavros/gps_rtk/send_rtcm
# Should show 1-5 Hz
```

### Simulation Not Starting

```bash
# Rebuild workspaces
cd simulation/ros2_ws && colcon build
cd shared/ros2_ws && colcon build

# Source both
source /opt/ros/humble/setup.bash
source simulation/ros2_ws/install/setup.bash
source shared/ros2_ws/install/setup.bash
```

### Service Won't Start

```bash
# Check logs
sudo journalctl -u jetson-mavros2 -n 50

# Restart all services
sudo systemctl restart jetson-*
```

---

## Safety

### Pre-Operation Checklist

- [ ] RTK base station powered and GPS locked
- [ ] Jetson all services running
- [ ] GPS fix type >= 3 (preferably 5 or 6)
- [ ] Satellite count >= 10
- [ ] Dashboard connected and responsive
- [ ] Clear area for testing (30m+ radius)
- [ ] Emergency stop accessible

---

## Development Team

**Project Designer:** Anatoly "Tolya" Makarov
**Hardware/Software Development:** Jay
**Last Updated:** November 21, 2025
