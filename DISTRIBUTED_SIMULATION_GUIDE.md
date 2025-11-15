# Distributed ROS2 Simulation Guide
## Jetson Cube Orange Rover - Desktop Simulation Setup

This guide explains how to run the Gazebo simulation on your powerful desktop while controlling and monitoring it from the Jetson.

---

## Overview

**Architecture:**
- **Desktop (192.168.254.209)**: Runs Gazebo simulation, RViz (GPU-accelerated)
- **Jetson (192.168.254.194)**: Develops control code, monitors topics, sends commands
- **Network**: Both machines communicate via ROS2 DDS over local network

**Benefits:**
- Offload heavy visualization to desktop GPU
- Develop and test control algorithms on Jetson
- Same ROS2 topics work for both simulation and real robot
- Validate software before deploying to physical rover

---

## Quick Start

### Option 1: Desktop Visualization (Recommended)

**On Desktop:**
1. Open terminal on desktop (or SSH from Jetson with X11 forwarding)
2. Run:
   ```bash
   source ~/ros2_distributed_setup.sh
   cd ~/ros2_ws
   source install/setup.bash
   ros2 launch jetson_rover_sim visualize_rover.launch.py world:=test_yard
   ```

**On Jetson:**
1. Open terminal
2. Configure network:
   ```bash
   source ~/Desktop/Mini\ Rover\ Development/Jetson\ Cube\ Orange\ Outdoor\ Rover/ros2_distributed_setup.sh
   ```
3. Monitor topics:
   ```bash
   ros2 topic list
   ros2 topic hz /scan
   ros2 topic echo /camera/image_raw --once
   ```

### Option 2: Headless Simulation (Desktop runs Gazebo, Jetson runs RViz)

**On Jetson:**
```bash
# Launch headless simulation on desktop
./launch_desktop_sim_headless.sh

# In another terminal, view locally
source ros2_distributed_setup.sh
rviz2
```

### Option 3: Easy Launch from Jetson

```bash
# Launch with GUI on desktop (requires X11 forwarding or physical desktop access)
./launch_desktop_sim.sh

# Or launch headless
./launch_desktop_sim_headless.sh
```

---

## Network Configuration

Both machines must have:
- **Same ROS_DOMAIN_ID**: 42 (configurable in setup scripts)
- **ROS_LOCALHOST_ONLY**: 0 (disabled)
- **Same network**: 192.168.254.x subnet
- **Firewall**: Allow UDP multicast (ROS2 DDS discovery)

The `ros2_distributed_setup.sh` script configures all of this automatically.

---

## Available Topics

Once simulation is running, these topics are available network-wide:

### Sensors
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | 10 Hz | RP-LIDAR 360° scan |
| `/camera/image_raw` | sensor_msgs/Image | 30 Hz | Logitech C920X camera |
| `/camera/camera_info` | sensor_msgs/CameraInfo | 30 Hz | Camera calibration |
| `/gps/fix` | sensor_msgs/NavSatFix | 10 Hz | HERE 3+ GPS position |
| `/ultrasonic/front` | sensor_msgs/Range | 10 Hz | Front ultrasonic |
| `/ultrasonic/corner_left` | sensor_msgs/Range | 10 Hz | Corner left ultrasonic |
| `/ultrasonic/corner_right` | sensor_msgs/Range | 10 Hz | Corner right ultrasonic |
| `/ultrasonic/side_left` | sensor_msgs/Range | 10 Hz | Side left ultrasonic |
| `/ultrasonic/side_right` | sensor_msgs/Range | 10 Hz | Side right ultrasonic |
| `/ultrasonic/rear` | sensor_msgs/Range | 10 Hz | Rear ultrasonic |
| `/odom` | nav_msgs/Odometry | 50 Hz | Wheel odometry |

### Control
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands (linear.x, angular.z) |

---

## Testing the Distributed Setup

### 1. Verify Network Discovery

**On Jetson:**
```bash
source ros2_distributed_setup.sh
ros2 topic list
```

You should see all the sensor topics from the desktop simulation.

### 2. Monitor Sensor Data

**LiDAR:**
```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

**Camera:**
```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

**GPS:**
```bash
ros2 topic echo /gps/fix --once
```

**Ultrasonics:**
```bash
ros2 topic echo /ultrasonic/front --once
```

### 3. Send Control Commands

**From Jetson to Desktop Simulation:**
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5}" --once

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "angular: {z: 0.5}" --once

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once
```

Or use keyboard teleop:
```bash
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Developing Control Code

### Python Example (Run on Jetson)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class RoverController(Node):
    def __init__(self):
        super().__init__('rover_controller')

        # Subscribe to LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Rover controller started')

    def scan_callback(self, msg):
        # Get distance straight ahead (index 0)
        front_distance = msg.ranges[0]

        cmd = Twist()

        if front_distance > 2.0:
            # Clear ahead - move forward
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
        else:
            # Obstacle detected - turn right
            cmd.linear.x = 0.0
            cmd.angular.z = -0.5

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    controller = RoverController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Save as `simple_controller.py` and run:**
```bash
source ros2_distributed_setup.sh
python3 simple_controller.py
```

The code runs on Jetson but controls the simulated rover on the desktop!

---

## Troubleshooting

### Topics Not Visible

**Check ROS_DOMAIN_ID matches on both machines:**
```bash
# On both machines
echo $ROS_DOMAIN_ID  # Should be 42
```

**Check ROS_LOCALHOST_ONLY is disabled:**
```bash
# On both machines
echo $ROS_LOCALHOST_ONLY  # Should be 0
```

**Check firewall (Ubuntu):**
```bash
# Allow ROS2 DDS multicast
sudo ufw allow from 192.168.254.0/24
```

**Restart network discovery:**
```bash
# On Jetson
source ros2_distributed_setup.sh
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

### Simulation Won't Launch on Desktop

**Check if Gazebo is already running:**
```bash
ssh jay@192.168.254.209 "pkill gzserver; pkill gzclient"
```

**Check for X11 forwarding (if using SSH with GUI):**
```bash
ssh -X -i ~/.ssh/jetson_to_desktop jay@192.168.254.209
echo $DISPLAY  # Should show something like localhost:10.0
```

**Use headless mode instead:**
```bash
./launch_desktop_sim_headless.sh
```

### Slow Network Performance

**Check network bandwidth:**
```bash
# On Jetson
ping -c 5 192.168.254.209
```

**Reduce camera resolution in URDF** (if needed):
Edit `~/ros2_ws/src/jetson_rover_sim/urdf/sensors/lidar_camera_gps.xacro`:
```xml
<!-- Change from 1920x1080 to 640x480 -->
<width>640</width>
<height>480</height>
```

**Use compressed image transport:**
```bash
sudo apt install ros-humble-compressed-image-transport
ros2 run image_transport republish raw compressed --ros-args -r in:=/camera/image_raw -r out/compressed:=/camera/image_raw/compressed
```

---

## Simulation Worlds

### Empty World
```bash
ros2 launch jetson_rover_sim spawn_rover.launch.py
```

### Test Yard (Recommended)
```bash
ros2 launch jetson_rover_sim visualize_rover.launch.py world:=test_yard
```

**Test Yard Features:**
- Grass terrain (0.8 friction)
- Pavement section (1.0 friction)
- Obstacles: tree, post, curb, wall, rocks
- 4 GPS waypoint markers
- 30m × 20m geofence boundary

---

## Transition to Real Robot

When you're ready to deploy to the physical rover:

**Your control code doesn't change!** Same topics, same messages.

**On Real Jetson with Physical Hardware:**
1. Change launch file parameter:
   ```bash
   ros2 launch your_controller.launch.py use_sim_time:=false
   ```

2. Real sensors publish to same topics:
   - MAVROS → `/mavros/imu/data`, `/mavros/global_position/global`
   - LiDAR driver → `/scan`
   - Camera driver → `/camera/image_raw`
   - ESP32 ultrasonic → `/ultrasonic/*`

3. Cube Orange receives `/cmd_vel` and controls motors

**That's it!** Your tested simulation code now controls the real robot.

---

## File Reference

- **[ros2_distributed_setup.sh](ros2_distributed_setup.sh)** - Configure Jetson for distributed ROS2
- **[launch_desktop_sim.sh](launch_desktop_sim.sh)** - Launch simulation with GUI on desktop
- **[launch_desktop_sim_headless.sh](launch_desktop_sim_headless.sh)** - Launch headless simulation
- **~/ros2_ws/src/jetson_rover_sim/** - Simulation package on desktop
- **~/.ssh/jetson_to_desktop** - SSH key for passwordless access

---

## Next Steps

1. **Test the distributed setup:**
   ```bash
   ./launch_desktop_sim.sh
   # In another terminal:
   source ros2_distributed_setup.sh
   ros2 topic list
   ```

2. **Run a simple controller** (example above)

3. **Validate sensor data** from all sensors

4. **Implement navigation logic** using Nav2 or custom controller

5. **Test thoroughly** before deploying to real robot

---

## Summary

✅ Desktop runs heavy Gazebo simulation with GPU
✅ Jetson develops and tests control algorithms
✅ Same ROS2 topics work in simulation and real robot
✅ Network-transparent - code location doesn't matter
✅ Safe testing before deploying to 200 lb physical rover

**Ready to simulate!** Run `./launch_desktop_sim.sh` or follow Quick Start above.

---

**For more information:**
- [QUICK_START.md](../ros2_ws/src/jetson_rover_sim/QUICK_START.md) - Simulation package guide
- [TESTING_GUIDE.md](../ros2_ws/src/jetson_rover_sim/TESTING_GUIDE.md) - Comprehensive testing procedures
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/) - Official ROS2 documentation
