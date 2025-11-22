# Jetson Rover Software

This is the master repository for software running on the Jetson Orin Nano rover.

## Directory Structure

```
rover/
├── stable/              # Currently deployed, production software
│   ├── scripts/         # Python scripts
│   ├── launch/          # ROS2 launch files
│   └── services/        # Systemd service definitions
├── experimental/        # Development sandbox for testing changes
│   └── [feature branches go here]
└── archive/             # Old versions kept for reference
    └── [dated backups]
```

## Workflow

### Making Changes

1. Copy files from `stable/` to `experimental/`
2. Make and test changes in `experimental/`
3. Once verified, copy back to `stable/` and update services
4. Move old version to `archive/` with date prefix

### Deploying Changes

To deploy a script change:
```bash
# Stop the service
sudo systemctl stop jetson-unified-bridge

# Copy new version
cp ~/rover/stable/scripts/ros2_unified_bridge.py ~/ros2_unified_bridge.py

# Start the service
sudo systemctl start jetson-unified-bridge
```

## Current Services

| Service | Script | Port | Description |
|---------|--------|------|-------------|
| jetson-mavros2 | mavros_cube_orange.launch.py | - | MAVLink bridge to Cube Orange |
| jetson-unified-bridge | ros2_unified_bridge.py | 5001 | HTTP API for sensors/GPS |
| jetson-rplidar | (ROS2 package) | - | RPLidar A1 driver |
| jetson-camera | (ROS2 package) | - | USB camera driver |
| jetson-rtcm-forwarder | rtcm_mqtt_forwarder.py | - | RTK corrections to GPS |

## Last Updated

November 21, 2025 - Initial organization
