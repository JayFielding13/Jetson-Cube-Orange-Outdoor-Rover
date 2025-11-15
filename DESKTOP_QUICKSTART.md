# Desktop Quick Start Guide
**Ubuntu Desktop (192.168.254.66) - Simulation & LLM Development**

---

## Quick Launch Commands

### Run Full Simulation (Gazebo + RViz + Xbox Controller)

**Option 1: Using unified launch file (recommended)**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch jetson_rover_sim full_simulation.launch.py
```

This launches:
- Gazebo with rover in test_yard world
- RViz2 with all sensor visualizations
- Xbox controller support (no deadman switch)

**Option 2: Using script**
```bash
cd ~
./launch_local_sim.sh
```

---

## Individual Components

### Launch Gazebo Only
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch jetson_rover_sim spawn_rover.launch.py
```

### Launch RViz2 Only
```bash
source /opt/ros/humble/setup.bash
rviz2 -d ~/rover_sensors.rviz
```

### Launch Xbox Controller Only
```bash
cd ~
./run_xbox_controller.sh
```

**Controls:**
- Left Stick Up/Down: Forward/Backward (0.7 m/s)
- Left Stick Left/Right: Rotate (0.8 rad/s)
- Right Bumper: Turbo mode (1.5 m/s, 1.2 rad/s)
- No button press required!

---

## LLM Commands

### Interactive Chat
```bash
ollama run llama3.1:8b
```

### Single Query
```bash
ollama run llama3.1:8b "Your robotics question here"
```

### Examples
```bash
# Get help with ROS 2
ollama run llama3.1:8b "Explain ROS 2 topics and nodes in simple terms"

# PID tuning advice
ollama run llama3.1:8b "How do I tune PID gains for a robot?"

# Code generation
ollama run llama3.1:8b "Write a Python ROS 2 publisher node that publishes velocity commands"
```

---

## Distributed ROS 2 (Connect to Jetson)

To see live data from the Jetson (192.168.254.100):

```bash
# Make sure ROS_DOMAIN_ID matches
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# List all topics (will show Jetson's topics if it's running)
ros2 topic list

# View Jetson's GPS data
ros2 topic echo /mavros/global_position/global

# View Jetson's LiDAR
ros2 topic echo /scan

# Visualize Jetson's sensors in RViz
rviz2 -d ~/rover_sensors.rviz
```

**Note:** Jetson must be powered on and running MAVROS2/ROS2 services.

---

## Common Tasks

### Rebuild Workspace After Changes
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Check GPU Usage
```bash
watch -n 1 nvidia-smi
```

### List ROS 2 Topics
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic list
```

### View Node Graph
```bash
source /opt/ros/humble/setup.bash
rqt_graph
```

---

## Troubleshooting

### Simulation won't start
```bash
# Check if Gazebo is already running
pkill -9 gzserver
pkill -9 gzclient

# Try again
cd ~/ros2_ws
source install/setup.bash
ros2 launch jetson_rover_sim full_simulation.launch.py
```

### Xbox controller not working
```bash
# Check controller is connected
ls /dev/input/js*
# Should show: /dev/input/js0

# Test controller
jstest /dev/input/js0
```

### Can't see Jetson's ROS 2 topics
```bash
# Verify ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID  # Should be 42

# Ping Jetson
ping 192.168.254.100

# Check firewall
sudo ufw status
sudo ufw allow from 192.168.254.0/24
```

### Ollama not responding
```bash
# Check service
systemctl status ollama

# Restart if needed
sudo systemctl restart ollama
```

---

## File Locations

**ROS 2 Workspace:**
```
~/ros2_ws/
  src/
    jetson_rover_sim/     # Gazebo simulation
    jetson_rover_bridge/  # Hardware bridge
  build/
  install/
```

**Configuration Files:**
```
~/rover_sensors.rviz          # RViz configuration
~/xbox_rover.config.yaml      # Controller config
~/launch_local_sim.sh         # Simulation launcher
~/run_xbox_controller.sh      # Controller launcher
```

**Ollama Models:**
```
/usr/share/ollama/.ollama/models/
```

---

## Development Workflow

### 1. Edit Code Locally
Edit files on this machine or via VS Code SSH

### 2. Rebuild
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

### 3. Test in Simulation
```bash
source install/setup.bash
ros2 launch jetson_rover_sim full_simulation.launch.py
```

### 4. Deploy to Jetson (when ready)
```bash
scp -r ~/ros2_ws/src jay@192.168.254.100:~/ros2_ws/
ssh jay@192.168.254.100 "cd ~/ros2_ws && colcon build"
```

---

## System Resources

**Typical Usage:**
- Idle: ~2GB RAM, 10W GPU
- Gazebo + RViz: ~4GB RAM, 80-150W GPU
- Ollama LLM: +2GB RAM, +50-100W GPU
- Total simultaneous: ~6GB RAM, 150-250W GPU (well within 12GB VRAM)

**Monitor Resources:**
```bash
# GPU
nvidia-smi

# CPU/RAM
htop

# Disk
df -h
```

---

## Next Steps

1. **Test the simulation**
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   ros2 launch jetson_rover_sim full_simulation.launch.py
   ```

2. **Experiment with LLM**
   ```bash
   ollama run llama3.1:8b
   ```

3. **Connect to live Jetson** (if powered on)
   ```bash
   source /opt/ros/humble/setup.bash
   export ROS_DOMAIN_ID=42
   ros2 topic list
   ```

4. **Read the full setup documentation**
   - [DESKTOP_SETUP_NOV15_2025.md](DESKTOP_SETUP_NOV15_2025.md)

---

**Status:** ✅ Desktop fully configured and ready
**Last Updated:** November 15, 2025
