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

## Development Notes

This simulation is part of the Jetson Cube Orange Outdoor Rover project. The goal is to develop and test autonomous navigation algorithms in simulation before deploying to the real hardware.

Recent changes:
- 2025-11-15: Re-enabled ultrasonic sensor Gazebo plugins
- 2025-11-15: Tuned Xbox controller rotation scaling to match real robot
