#!/bin/bash
# Run the obstacle avoidance controller LOCALLY
# No network setup needed - everything runs on this machine!

echo "========================================="
echo "Starting Local Obstacle Avoidance"
echo "========================================="
echo ""
echo "Make sure simulation is running first!"
echo "  Run: ./launch_local_sim.sh"
echo ""
echo "The rover will:"
echo "  - Move forward when path is clear"
echo "  - Slow down when obstacles nearby (< 2.5m)"
echo "  - Stop and turn when too close (< 1.5m)"
echo ""
echo "Press Ctrl+C to stop"
echo "========================================="
echo ""

# Source ROS2 (no network config needed!)
source /opt/ros/humble/setup.bash
source ~/Desktop/Mini\ Rover\ Development/ros2_ws/install/setup.bash

# Run the controller
python3 simple_obstacle_avoidance.py
