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
# Get the directory where this script lives
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Source shared workspace
source "$PROJECT_ROOT/shared/ros2_ws/install/setup.bash" 2>/dev/null || {
    echo "Workspace not built yet. Building..."
    cd "$PROJECT_ROOT/shared/ros2_ws"
    colcon build
    source install/setup.bash
}

# Run the controller
python3 "$SCRIPT_DIR/scripts/simple_obstacle_avoidance.py"
