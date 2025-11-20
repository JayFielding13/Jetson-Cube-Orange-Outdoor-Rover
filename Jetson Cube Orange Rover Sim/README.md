# Jetson Cube Orange Rover Simulation

ROS2 Humble simulation environment for the Jetson Cube Orange outdoor rover.

## Overview

This workspace contains Gazebo simulation packages for testing rover behaviors before deployment to the real hardware.

## Features

- **Full rover model** with accurate dimensions and mass properties
- **6 ultrasonic sensors** (AJ-SR04M waterproof)
  - Front, rear, side left, side right, corner left, corner right
  - 20cm - 6m range, 20 Hz update rate
- **RP-LIDAR A1** 360° laser scanner
- **Logitech C920X** camera
- **HERE 3+ RTK GPS**
- **Xbox 360 controller** teleoperation
- **Skid-steer drive** physics using planar move plugin

## Packages

- `jetson_rover_sim` - Main simulation package with URDF, worlds, and launch files
- `jetson_rover_bridge` - Bridge nodes for hardware integration

## Quick Start

### Build the workspace

```bash
cd "Jetson Cube Orange Rover Sim"
colcon build --symlink-install
source install/setup.bash
```

### Launch the simulation

```bash
ros2 launch jetson_rover_sim full_simulation.launch.py
```

This will start:
- Gazebo with the rover model
- RViz for visualization
- Xbox controller teleoperation (if controller is connected)

## Xbox Controller Mapping

- **Left stick vertical**: Forward/backward (scale: 0.7, turbo: 1.5)
- **Left stick horizontal**: Rotation (scale: 2.5, turbo: 3.5)
- **Right bumper (RB)**: Turbo mode
- **No deadman switch** - controller always active when connected

## Topics

### Sensors
- `/scan` - LIDAR data (sensor_msgs/LaserScan)
- `/camera/image_raw` - Camera feed (sensor_msgs/Image)
- `/camera/camera_info` - Camera calibration
- `/gps/fix` - GPS position (sensor_msgs/NavSatFix)
- `/ultrasonic/front` - Front ultrasonic (sensor_msgs/Range)
- `/ultrasonic/rear` - Rear ultrasonic
- `/ultrasonic/side_left` - Left side ultrasonic
- `/ultrasonic/side_right` - Right side ultrasonic
- `/ultrasonic/corner_left` - Front-left corner ultrasonic
- `/ultrasonic/corner_right` - Front-right corner ultrasonic

### Control
- `/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/odom` - Odometry (nav_msgs/Odometry)

## Known Issues

### Rotation Speed Tuning
The simulation uses a planar move plugin which doesn't perfectly replicate the 4-wheel skid-steer dynamics of the real robot. To compensate, the Xbox controller angular scaling has been increased (2.5x for normal, 3.5x for turbo) to match the real robot's rotation speed.

### Ultrasonic Sensors
The ultrasonic sensor Gazebo plugins were previously disabled due to a ROS2 Humble bug with multiple Range subscriptions. This has been re-enabled as of the latest update.

## Hardware Specifications

### Rover
- **Chassis**: 27" L × 23.75" W × 12" H
- **Mass**: 200 lbs (90.7 kg)
- **Wheelbase**: 17 inches
- **Track width**: 23.75 inches
- **Ground clearance**: 3 inches
- **Wheel diameter**: 10 inches

### Sensors
- **LIDAR**: RP-LIDAR A1 (12m range, 360°, 10 Hz)
- **Camera**: Logitech C920X (1920×1080, 62° FOV, 30 Hz)
- **GPS**: HERE 3+ RTK (5mm horizontal, 10mm vertical accuracy)
- **Ultrasonic**: AJ-SR04M (20cm-6m range, 70° cone, 20 Hz)

## Simulation vs Hardware

### Simulation-Only Features
Features that currently work **only in simulation**:

1. **AprilTag Following** (New Development)
   - `apriltag_mover.py` - Moves AprilTag in simulation for testing
   - `follow_apriltag_behavior.py` - Behavior tree for following AprilTags
   - `person_waypoint_driver.py` - Waypoint navigation for person tracking
   - Launch files: `follow_apriltag.launch.py`, `follow_moving_apriltag.launch.py`
   - Model: `person_with_apriltag/` - Person model with attached AprilTag

2. **Gazebo Sensor Simulation**
   - Lidar, GPS, Camera use Gazebo plugins
   - Ultrasonic sensors simulated as ray sensors (vs ESP32 hardware)

### Hardware-Only Features
Features that only work on the physical robot:

1. **ESP32 Ultrasonic Sensors** - Physical ESP32 microcontrollers with real ultrasonics
2. **MAVLink Communication** - Direct Cube Orange autopilot connection
3. **Physical PWM Control** - Direct motor control hardware

### Hybrid Features
Features that work in both but may behave differently:

1. **Obstacle Avoidance** - Works in both, but sensor characteristics differ
2. **Xbox Controller** - Works in both, but may need different scaling

## Related Documentation

All detailed simulation guides are in the **parent directory**:

### Setup & Quickstart
- `DESKTOP_SETUP_NOV15_2025.md` - Complete desktop simulation setup
- `DESKTOP_QUICKSTART.md` - Quick start guide
- `install_ros2_gazebo.sh` - Automated installation script

### Simulation Guides
- `LOCAL_SIMULATION_GUIDE.md` - Local simulation instructions
- `DISTRIBUTED_SIMULATION_GUIDE.md` - Multi-machine setup
- `ROS2_GAZEBO_QUICKSTART.md` - ROS2 Gazebo basics

### Testing & Development
- `APRILTAG_FOLLOW_TESTING_GUIDE.md` - AprilTag following tests
- `MOTION_TUNING_GUIDE.md` - Motion parameter tuning
- `DIFF_DRIVE_FIX.md` - Differential drive troubleshooting

### Launch Scripts (in parent directory)
- `launch_local_sim.sh` - Local simulation
- `launch_desktop_sim.sh` - Desktop with GUI
- `launch_desktop_sim_headless.sh` - Headless mode
- `ros2_distributed_setup.sh` - Distributed setup

## Development Notes

This simulation is part of the Jetson Cube Orange Outdoor Rover project. The goal is to develop and test autonomous navigation algorithms in simulation before deploying to the real hardware.

**Best Practice**: Always test new behaviors in simulation first, then verify on hardware.

Recent changes:
- 2025-11-19: Added AprilTag following behaviors (simulation only)
- 2025-11-19: Created simulation vs hardware feature documentation
- 2025-11-15: Re-enabled ultrasonic sensor Gazebo plugins
- 2025-11-15: Tuned Xbox controller rotation scaling to match real robot
