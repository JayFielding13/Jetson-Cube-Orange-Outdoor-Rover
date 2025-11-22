# Simulation Environment

This directory contains everything needed for Gazebo/RViz simulation of the rover on a desktop workstation.

## Directory Structure

```
simulation/
├── ros2_ws/src/jetson_rover_sim/   # ROS2 simulation package
│   ├── urdf/                        # Robot URDF models
│   ├── launch/                      # Launch files
│   ├── worlds/                      # Gazebo world files
│   ├── models/                      # Custom Gazebo models
│   └── config/                      # RViz configs
├── launch_local_sim.sh              # Launch simulation locally
├── launch_desktop_sim.sh            # Launch via SSH to desktop
├── launch_desktop_sim_headless.sh   # Headless simulation
├── run_xbox_controller.sh           # Xbox controller teleop
└── rover_sensors.rviz               # RViz visualization config
```

## Quick Start

### Build the workspace
```bash
cd simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Launch Options

**Full simulation with Gazebo + RViz:**
```bash
ros2 launch jetson_rover_sim full_simulation.launch.py
```

**View robot model only (no physics):**
```bash
ros2 launch jetson_rover_sim view_rover.launch.py
```

**Spawn in Gazebo:**
```bash
ros2 launch jetson_rover_sim spawn_rover.launch.py
```

## Simulated Sensors

- RPLidar A1 (360 degree 2D laser scan)
- USB Camera
- HERE 3+ RTK GPS
- 6x Ultrasonic sensors
- IMU

## Requirements

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic (ros-humble-gazebo-ros-pkgs)
- RViz2
