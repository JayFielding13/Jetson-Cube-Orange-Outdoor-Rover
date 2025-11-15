#!/bin/bash
# ROS2 Distributed Setup Script for Jetson (Local Machine)
#
# This script configures the Jetson to communicate with the desktop simulation
# Run this on the Jetson Nano/Orin before viewing topics from the desktop

# Network Configuration
export JETSON_IP="192.168.254.194"
export DESKTOP_IP="192.168.254.209"

# Use the same ROS_DOMAIN_ID on both machines (0-101)
export ROS_DOMAIN_ID=42

# Configure DDS for network discovery
export ROS_LOCALHOST_ONLY=0

# Optional: Set ROS_MASTER_URI for better discovery
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Configure FastDDS for wide discovery
export FASTRTPS_DEFAULT_PROFILES_FILE=""

echo "========================================="
echo "ROS2 Distributed Setup - JETSON"
echo "========================================="
echo "Jetson IP:      $JETSON_IP"
echo "Desktop IP:     $DESKTOP_IP"
echo "ROS_DOMAIN_ID:  $ROS_DOMAIN_ID"
echo "Localhost Only: $ROS_LOCALHOST_ONLY"
echo "========================================="
echo ""
echo "To view topics from desktop simulation:"
echo "  ros2 topic list"
echo "  ros2 topic echo /scan"
echo "  ros2 topic hz /camera/image_raw"
echo ""
echo "To send commands to the rover:"
echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ..."
echo "========================================="
