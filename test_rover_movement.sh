#!/bin/bash
# Test script to verify rover movement in simulation
# This sends various movement commands to test the differential drive controller

echo "========================================="
echo "Rover Movement Test Script"
echo "========================================="
echo ""
echo "Make sure the simulation is running on the desktop!"
echo "This script will send movement commands to test the rover."
echo ""
echo "Commands will be sent in sequence:"
echo "  1. Move forward (3 seconds)"
echo "  2. Stop (1 second)"
echo "  3. Turn left (2 seconds)"
echo "  4. Stop (1 second)"
echo "  5. Turn right (2 seconds)"
echo "  6. Stop (1 second)"
echo "  7. Move backward (3 seconds)"
echo "  8. Stop"
echo ""
read -p "Press Enter to start the test (Ctrl+C to cancel)..."
echo ""

# Configure distributed ROS2
source ros2_distributed_setup.sh

echo "Test 1: Moving forward at 0.5 m/s for 3 seconds..."
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5}" &
PUB_PID=$!
sleep 3
kill $PUB_PID 2>/dev/null
echo "✓ Forward complete"
echo ""

echo "Test 2: Stopping for 1 second..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once
sleep 1
echo "✓ Stop complete"
echo ""

echo "Test 3: Turning left at 0.5 rad/s for 2 seconds..."
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "angular: {z: 0.5}" &
PUB_PID=$!
sleep 2
kill $PUB_PID 2>/dev/null
echo "✓ Left turn complete"
echo ""

echo "Test 4: Stopping for 1 second..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once
sleep 1
echo "✓ Stop complete"
echo ""

echo "Test 5: Turning right at -0.5 rad/s for 2 seconds..."
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "angular: {z: -0.5}" &
PUB_PID=$!
sleep 2
kill $PUB_PID 2>/dev/null
echo "✓ Right turn complete"
echo ""

echo "Test 6: Stopping for 1 second..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once
sleep 1
echo "✓ Stop complete"
echo ""

echo "Test 7: Moving backward at -0.3 m/s for 3 seconds..."
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "linear: {x: -0.3}" &
PUB_PID=$!
sleep 3
kill $PUB_PID 2>/dev/null
echo "✓ Backward complete"
echo ""

echo "Test 8: Final stop..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once
echo "✓ Final stop complete"
echo ""

echo "========================================="
echo "Movement test complete!"
echo "========================================="
echo ""
echo "Did the rover move in the simulation?"
echo "  - Forward/backward motion"
echo "  - Left/right turning"
echo "  - Smooth stops"
echo ""
echo "If yes: ✓ Differential drive controller is working!"
echo "If no: Check Gazebo console for errors"
echo "========================================="
