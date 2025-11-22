# Desktop Setup - November 15, 2025
**System:** Ubuntu 22.04 Desktop for Simulation & LLM Development
**IP Address:** 100.73.129.15
**Updated By:** Jay with Claude Code assistance

---

## System Overview

This desktop is configured as a development and simulation workstation for the Jetson Cube Orange Outdoor Rover project, with the following capabilities:

- **GPU-accelerated simulation** (Gazebo + RViz2)
- **Local LLM inference** (Ollama with Llama 3.1 8B)
- **ROS 2 Humble** development environment
- **Distributed ROS 2** networking with Jetson (100.91.191.47)

---

## Hardware Specifications

**Graphics Card:**
```
NVIDIA GeForce RTX 5070
- 12GB VRAM
- CUDA 13.0 support
- Driver: 580.105.08
```

**Operating System:**
```
Ubuntu 22.04.5 LTS (Jammy Jellyfish)
Python 3.10.12
```

---

## Installed Software

### 1. ROS 2 Humble

**Status:** ✅ Installed and configured

**Key Packages:**
- `ros-humble-desktop` - Full ROS 2 desktop installation
- `ros-humble-rviz2` - 3D visualization tool
- `ros-humble-rviz-default-plugins` - Standard RViz plugins
- `ros-humble-rqt` - Qt-based GUI tools
- `ros-humble-rqt-common-plugins` - Common RQT plugins

**Environment Configuration:**
```bash
# Added to ~/.bashrc
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

**Verification:**
```bash
# Check ROS installation
source /opt/ros/humble/setup.bash
ros2 --version
rviz2 --version

# List available ROS 2 nodes
ros2 node list

# Check topics
ros2 topic list
```

---

### 2. Gazebo Simulator

**Status:** ✅ Installed

**Version:** Gazebo 11.10.2 (Classic Gazebo)

**Packages:**
- `gazebo` - Main simulator binary
- `gazebo-common` - Shared files
- `gazebo-plugin-base` - Base plugins
- `libgazebo-dev` - Development headers

**Launch Gazebo:**
```bash
gazebo
```

---

### 3. Ollama (LLM Runtime)

**Status:** ✅ Installed and running

**Installation Date:** November 15, 2025

**Service Status:**
```bash
systemctl status ollama
# Should show: active (running)
```

**Installed Models:**
- `llama3.1:8b` (4.9 GB) - Meta's Llama 3.1 8B parameter model

**Usage Examples:**

**Interactive chat:**
```bash
ollama run llama3.1:8b
```

**Single query:**
```bash
ollama run llama3.1:8b "Your question here"
```

**API access:**
```bash
curl http://localhost:11434/api/generate -d '{
  "model": "llama3.1:8b",
  "prompt": "Explain ROS2 nodes",
  "stream": false
}'
```

**Python API access:**
```python
import requests
import json

response = requests.post('http://localhost:11434/api/generate',
    json={
        'model': 'llama3.1:8b',
        'prompt': 'What is a kalman filter?',
        'stream': False
    })

result = json.loads(response.text)
print(result['response'])
```

**Test Query:**
```bash
ollama run llama3.1:8b "Explain what a PID controller is in 2 sentences."
```

Expected output: Accurate explanation of PID controllers

---

### 4. NVIDIA GPU Configuration

**Driver:** 580.105.08
**CUDA:** Version 13.0

**Check GPU status:**
```bash
nvidia-smi
```

**GPU Info:**
```
GPU: NVIDIA GeForce RTX 5070
Memory: 12227 MiB total
Temperature: ~32°C idle
Power: 10W idle / 250W max
```

**Ollama GPU Detection:**
Ollama automatically detected and is using the NVIDIA GPU for inference acceleration.

---

## Network Configuration

**Desktop IP:** 100.73.129.15
**Network:** 192.168.254.0/24

**Other Systems:**
- Jetson Orin Nano: 100.91.191.47
- Raspberry Pi 5 Dashboard: 100.73.233.124
- RTK Base Station: 100.66.67.11

**ROS 2 Domain ID:** 42 (matches Jetson configuration)

---

## Next Steps

### Immediate Tasks

1. **Sync Rover Project to Desktop**
   ```bash
   # From this machine, copy workspace to desktop
   scp -r ros2_ws jay@100.73.129.15:~/
   ```

2. **Test Distributed ROS 2**
   ```bash
   # On desktop, check for Jetson's ROS 2 topics
   source /opt/ros/humble/setup.bash
   export ROS_DOMAIN_ID=42
   ros2 topic list
   # Should see topics from Jetson if it's running
   ```

3. **Build Rover Simulation**
   ```bash
   # On desktop
   cd ~/ros2_ws
   colcon build --packages-select jetson_rover_sim
   source install/setup.bash
   ros2 launch jetson_rover_sim full_simulation.launch.py
   ```

### Future Enhancements

**NVIDIA Isaac Sim (Optional):**
- Advanced GPU-accelerated physics simulation
- Photorealistic rendering
- Requires NVIDIA Omniverse installation

**LLM Integration Ideas:**
1. **Natural language robot control**
   - Voice commands → LLM → ROS 2 commands
   - Example: "Move forward 2 meters" → `/cmd_vel` topic

2. **Autonomous decision making**
   - Sensor data → LLM analysis → navigation decisions
   - Example: LiDAR scan → "obstacle detected, turn right"

3. **Code generation**
   - Generate ROS 2 nodes from natural language
   - Auto-tune PID parameters based on performance logs

4. **Documentation assistant**
   - Generate technical documentation from code
   - Answer questions about codebase

---

## LLM Integration Examples

### Example 1: Simple ROS 2 Query Interface

Create a Python script that uses Ollama to answer ROS 2 questions:

```python
#!/usr/bin/env python3
import requests
import json

def ask_llm(question):
    response = requests.post('http://localhost:11434/api/generate',
        json={
            'model': 'llama3.1:8b',
            'prompt': f"You are a ROS 2 expert. {question}",
            'stream': False
        })
    return json.loads(response.text)['response']

# Example usage
question = "How do I create a publisher in ROS 2 Python?"
answer = ask_llm(question)
print(answer)
```

### Example 2: Sensor Data Analysis

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import requests
import json

class LidarLLMAnalyzer(Node):
    def __init__(self):
        super().__init__('lidar_llm_analyzer')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)

    def lidar_callback(self, msg):
        # Get minimum distance from LiDAR
        min_distance = min(msg.ranges)

        # Ask LLM for decision
        prompt = f"A robot's LiDAR detected an obstacle at {min_distance:.2f} meters. Should it stop, slow down, or continue? Respond with one word: STOP, SLOW, or CONTINUE."

        response = requests.post('http://localhost:11434/api/generate',
            json={'model': 'llama3.1:8b', 'prompt': prompt, 'stream': False})

        decision = json.loads(response.text)['response'].strip()
        self.get_logger().info(f'LLM decision: {decision}')

def main():
    rclpy.init()
    node = LidarLLMAnalyzer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Troubleshooting

### ROS 2 Not Finding Topics from Jetson

**Problem:** Desktop can't see Jetson's ROS 2 topics

**Solution:**
```bash
# Check firewall
sudo ufw status
sudo ufw allow from 192.168.254.0/24

# Verify ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID  # Should be 42

# Check network connectivity
ping 100.91.191.47
```

### Ollama Not Using GPU

**Problem:** Ollama running on CPU instead of GPU

**Check:**
```bash
# Verify NVIDIA driver
nvidia-smi

# Check Ollama logs
journalctl -u ollama -n 50

# Should see: "NVIDIA GPU installed"
```

### Gazebo Performance Issues

**Problem:** Gazebo running slowly

**Solutions:**
```bash
# Check GPU is being used
nvidia-smi

# Reduce physics update rate in world file
# Edit: ~/.gazebo/worlds/your_world.world
# Set: <max_step_size>0.01</max_step_size>
```

---

## Useful Commands

### ROS 2 Commands

```bash
# Source ROS environment
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42

# List all topics
ros2 topic list

# Echo topic data
ros2 topic echo /scan

# View node graph
rqt_graph

# Launch RViz2
rviz2
```

### Ollama Commands

```bash
# List models
ollama list

# Pull new model
ollama pull <model-name>

# Remove model
ollama rm <model-name>

# Check service
systemctl status ollama
```

### System Monitoring

```bash
# GPU usage
watch -n 1 nvidia-smi

# CPU/Memory
htop

# Disk usage
df -h
```

---

## Performance Notes

**GPU Performance:**
- Idle: 10W, 32°C
- Ollama inference: ~50-100W
- Gazebo simulation: ~80-150W

**LLM Inference Speed:**
- Llama 3.1 8B: ~50-100 tokens/second on RTX 5070
- Response time: 1-3 seconds for short queries

**Simulation Performance:**
- Gazebo can run multiple robots simultaneously
- RViz2 real-time sensor visualization
- No significant lag with current hardware

---

## Setup Checklist

- [x] Ubuntu 22.04 installed
- [x] NVIDIA drivers installed (580.105.08)
- [x] ROS 2 Humble installed
- [x] RViz2 and visualization tools installed
- [x] Gazebo 11.10.2 installed
- [x] Ollama installed and running
- [x] Llama 3.1 8B model downloaded (4.9 GB)
- [x] ROS 2 environment configured (ROS_DOMAIN_ID=42)
- [x] Network configured (100.73.129.15)
- [ ] Rover workspace synced from laptop/Jetson
- [ ] Distributed ROS 2 tested with Jetson
- [ ] Isaac Sim installed (optional)

---

## System Info Summary

```
Hostname: jay-desktop
IP: 100.73.129.15
OS: Ubuntu 22.04.5 LTS
Kernel: 6.8.0-87-generic
Python: 3.10.12

GPU: NVIDIA GeForce RTX 5070 (12GB)
Driver: 580.105.08
CUDA: 13.0

ROS: ROS 2 Humble
Gazebo: 11.10.2
Ollama: 0.12.11

LLM: Llama 3.1 8B (4.9 GB)
```

---

## Related Documentation

- [README.md](README.md) - Main project documentation
- [PROJECT_INDEX.md](PROJECT_INDEX.md) - File locations and architecture
- [SESSION_LOG_NOV12_2025.md](SESSION_LOG_NOV12_2025.md) - Latest simulation work
- [LOCAL_SIMULATION_GUIDE.md](LOCAL_SIMULATION_GUIDE.md) - How to run simulations
- [LLM_INTEGRATION_PLAN.md](LLM_INTEGRATION_PLAN.md) - LLM integration roadmap (to be created)

---

**Status:** Desktop ready for simulation and LLM development
**Next Session:** Sync rover project and test distributed ROS 2
**Last Updated:** November 15, 2025
