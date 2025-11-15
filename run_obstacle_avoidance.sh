#!/bin/bash
# Run the obstacle avoidance controller
# This connects to the simulation running on the desktop

echo "========================================="
echo "Starting Obstacle Avoidance Controller"
echo "========================================="
echo ""
echo "Make sure the simulation is running on the desktop!"
echo ""
echo "The rover will:"
echo "  - Move forward when path is clear"
echo "  - Slow down when obstacles nearby (< 2.5m)"
echo "  - Stop and turn when too close (< 1.5m)"
echo ""
echo "Press Ctrl+C to stop"
echo "========================================="
echo ""

# Configure distributed ROS2
source ros2_distributed_setup.sh

# Run the controller
python3 simple_obstacle_avoidance.py
