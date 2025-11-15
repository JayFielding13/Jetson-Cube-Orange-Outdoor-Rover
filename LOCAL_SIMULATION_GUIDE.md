# Local Simulation Guide - Simplified Workflow
## Run Everything on This AMD Ryzen Laptop!

**Date:** November 11, 2025
**Discovery:** You're already running native Ubuntu 22.04 with ROS2 Humble!

---

## 🎉 The Good News

You don't need WSL2, dual boot, or distributed ROS2. **Everything you need is already installed!**

**Your Setup:**
- ✅ AMD Ryzen 5 PRO 5650U (6 cores, 12 threads)
- ✅ 28GB RAM
- ✅ AMD Radeon Vega Graphics (integrated)
- ✅ Ubuntu 22.04 LTS (native, not VM!)
- ✅ ROS2 Humble
- ✅ Gazebo 11.10.2
- ✅ RViz2
- ✅ All required packages

**This is an EXCELLENT robotics development machine!**

---

## 🚀 Quick Start (Super Simple!)

### Step 1: Launch Simulation

```bash
cd ~/Desktop/Mini\ Rover\ Development/Jetson\ Cube\ Orange\ Outdoor\ Rover/
./launch_local_sim.sh
```

**Gazebo will open showing your rover!**

### Step 2: Run Obstacle Avoidance (In Another Terminal)

```bash
cd ~/Desktop/Mini\ Rover\ Development/Jetson\ Cube\ Orange\ Outdoor\ Rover/
./run_local_obstacle_avoidance.sh
```

**Watch the rover navigate autonomously!**

---

## 📂 Available Scripts

All in: `~/Desktop/Mini Rover Development/Jetson Cube Orange Outdoor Rover/`

### Simulation Launch:
- **`launch_local_sim.sh`** - Start Gazebo simulation locally
- **`launch_local_sim_with_rviz.sh`** - Gazebo + RViz visualization (coming soon)

### Control & Testing:
- **`run_xbox_controller.sh`** - Xbox 360 controller manual control (NEW!)
- **`run_local_obstacle_avoidance.sh`** - Autonomous obstacle avoidance
- **`test_rover_movement.sh`** - Test basic movements
- **`simple_obstacle_avoidance.py`** - The obstacle avoidance controller

### Development:
- **`ultrasonic_gui.py`** - Visualize ultrasonic sensors
- **`ultrasonic_visualizer.py`** - Alternative visualization

---

## 🎮 Manual Control

### Option 1: Xbox 360 Controller (Recommended!)

```bash
# First, install the joystick packages (one-time setup):
sudo apt install -y ros-humble-joy ros-humble-teleop-twist-joy

# Plug in your Xbox 360 controller, then run:
./run_xbox_controller.sh
```

**Xbox Controller Mapping:**
- **Left Stick Up/Down** - Forward/Backward (0.7 m/s)
- **Left Stick Left/Right** - Turn Left/Right (0.8 rad/s)
- **Right Bumper (RB)** - Turbo speed boost (1.5 m/s, 1.2 rad/s)
- **NO DEADMAN SWITCH** - Stick controls rover directly!

### Option 2: Keyboard Control

```bash
# Install if needed:
sudo apt install ros-humble-teleop-twist-keyboard

# Run teleop:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Keyboard Controls:**
- `i` - Forward
- `k` - Stop
- `,` - Backward
- `j` - Turn left
- `l` - Turn right

---

## 🔧 Common Tasks

### Launch with Test World (More Obstacles)

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/Mini\ Rover\ Development/ros2_ws/install/setup.bash
ros2 launch jetson_rover_sim visualize_rover.launch.py world:=test_yard
```

### Check Sensor Topics

```bash
# List all topics
ros2 topic list

# Monitor LiDAR
ros2 topic hz /scan

# View GPS
ros2 topic echo /gps/fix --once

# View camera
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

### Rebuild Simulation (After Changes)

```bash
cd ~/Desktop/Mini\ Rover\ Development/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jetson_rover_sim
source install/setup.bash
```

---

## 🎯 Workflow Comparison

### OLD (Complicated):
```
AMD Laptop → SSH → VirtualBox Desktop → Poor Graphics (10-15 fps)
            ↓
    Complex networking
    Distributed ROS2 setup
    Multiple machines to manage
```

### NEW (Simple):
```
AMD Laptop (local) → Gazebo (40-60 fps) → Smooth!
                   ↓
    Everything in one place
    No networking needed
    Easy debugging
```

---

## 💻 Hardware Performance

**Your AMD Ryzen 5 PRO 5650U:**

| Task | Performance | Rating |
|------|-------------|--------|
| Gazebo Simulation | 40-60 fps | ⭐⭐⭐⭐⭐ |
| ROS2 Compilation | Fast | ⭐⭐⭐⭐ |
| Multiple Nodes | Excellent | ⭐⭐⭐⭐⭐ |
| ML Training | Good (CPU) | ⭐⭐⭐ |

**This machine is MORE than capable for your rover project!**

---

## 🔄 When to Use the Jetson Nano

**Development (This AMD Laptop):**
- ✅ Simulation development
- ✅ Algorithm testing
- ✅ Code writing
- ✅ Debugging

**Deployment (Jetson Nano on Real Rover):**
- ✅ Final testing with real sensors
- ✅ Field operations
- ✅ Actual rover control

**Keep them separate and simple!**

---

## 🐛 Troubleshooting

### Gazebo won't start
```bash
# Kill any existing instances
pkill gzserver
pkill gzclient

# Try again
./launch_local_sim.sh
```

### Topics not visible
```bash
# Make sure you sourced ROS2
source /opt/ros/humble/setup.bash
source ~/Desktop/Mini\ Rover\ Development/ros2_ws/install/setup.bash

# Check topics
ros2 topic list
```

### Poor performance
```bash
# Check system resources
htop

# Close other applications
# AMD Vega graphics should handle Gazebo well
```

---

## 📊 Next Steps

Now that simulation is running locally:

1. **Test obstacle avoidance** (today)
   - Run `./launch_local_sim.sh`
   - Run `./run_local_obstacle_avoidance.sh`
   - Watch rover navigate!

2. **Add GPS waypoint navigation** (this week)
   - Navigate to specific coordinates
   - Combine with obstacle avoidance

3. **Integrate ultrasonics** (next week)
   - Add close-range safety
   - 6-sensor coverage

4. **Test in test_yard world**
   - More complex environment
   - Multiple obstacles
   - GPS waypoints

5. **Deploy to Jetson Nano** (when ready)
   - Transfer tested code
   - Connect real sensors
   - First autonomous run!

---

## 🎉 Summary

**You're already set up perfectly!**

- ✅ Native Ubuntu 22.04 on powerful hardware
- ✅ ROS2 Humble + Gazebo installed
- ✅ Rover simulation ready
- ✅ Obstacle avoidance code ready
- ✅ No complexity, just develop!

**The distributed setup we were building?** You don't need it! Your laptop can handle everything locally with better performance.

**Run this now:**
```bash
./launch_local_sim.sh
```

Then in another terminal:
```bash
./run_local_obstacle_avoidance.sh
```

**Watch your rover navigate autonomously in beautiful 40-60 fps!** 🚀

---

**Questions? Check:**
- [OBSTACLE_AVOIDANCE_GUIDE.md](OBSTACLE_AVOIDANCE_GUIDE.md) - Detailed guide
- [QUICK_TEST_OBSTACLE_AVOIDANCE.md](QUICK_TEST_OBSTACLE_AVOIDANCE.md) - Quick start
- [SESSION_LOG_NOV11_2025_SIMULATION.md](SESSION_LOG_NOV11_2025_SIMULATION.md) - What we built today

**Happy simulating!** 🤖
