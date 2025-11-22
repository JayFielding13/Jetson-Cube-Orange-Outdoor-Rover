# ROS 2 Humble + Gazebo Quick Start Guide

## Installation Summary

Successfully installed on Ubuntu 22.04:
- **ROS 2 Humble Desktop** (includes RViz2)
- **Gazebo 11.10.2** (Classic Gazebo)
- **Ignition Gazebo Garden** (via ros-humble-ros-gz)
- **ROS 2 development tools** (colcon, rosdep)
- **Additional packages**: Gazebo-ROS integration, rqt, tf2-tools

**Disk space used**: ~5 GB
**Remaining space**: 77 GB

---

## Basic Commands

### Starting ROS 2 Tools

```bash
# Always source ROS 2 first (or open new terminal)
source /opt/ros/humble/setup.bash

# Launch RViz2 (visualization tool)
rviz2

# Launch Gazebo Classic
gazebo

# Launch Ignition Gazebo (newer version)
ign gazebo
# or
gz sim

# Launch rqt (ROS 2 GUI tools)
rqt
```

### Useful ROS 2 Commands

```bash
# List all ROS 2 packages
ros2 pkg list

# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Echo topic data
ros2 topic echo /topic_name

# Show node info
ros2 node info /node_name

# Run a ROS 2 package
ros2 run <package_name> <executable_name>

# Launch a launch file
ros2 launch <package_name> <launch_file>
```

---

## Testing Your Setup

### 1. Test RViz2
```bash
source /opt/ros/humble/setup.bash
rviz2
```
You should see the RViz2 GUI window open.

### 2. Test Gazebo Classic
```bash
source /opt/ros/humble/setup.bash
gazebo
```
You should see an empty Gazebo world.

### 3. Test ROS 2 + Gazebo Integration
```bash
source /opt/ros/humble/setup.bash

# Terminal 1: Launch Gazebo with ROS 2
ros2 launch gazebo_ros gazebo.launch.py

# Terminal 2: List ROS 2 topics (should see Gazebo topics)
ros2 topic list
```

### 4. Test TurtleBot3 Simulation (Optional Demo)
```bash
# Install TurtleBot3 packages
sudo apt install -y ros-humble-turtlebot3*

# Set robot model
export TURTLEBOT3_MODEL=waffle

# Launch TurtleBot3 in Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# In another terminal, control the robot
ros2 run turtlebot3_teleop teleop_keyboard
```

---

## Creating Your Jetson Cube Orange Rover Simulation

### Step 1: Create a workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Step 2: Create your rover package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_rover_sim
```

### Step 3: Create URDF model (robot description)
Create a file: `~/ros2_ws/src/my_rover_sim/urdf/rover.urdf.xacro`

Basic rover URDF structure:
```xml
<?xml version="1.0"?>
<robot name="jetson_rover" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Add wheels, sensors, etc. here -->
</robot>
```

### Step 4: Create Gazebo launch file
Create: `~/ros2_ws/src/my_rover_sim/launch/rover_sim.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_rover', '-file', 'path/to/urdf'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
    ])
```

---

## Integrating with Your Jetson Hardware

### Simulation Workflow:
1. **Develop** algorithms in simulation on this PC
2. **Test** navigation, sensor fusion, etc. in Gazebo
3. **Validate** with MAVROS2/PX4 SITL (Software In The Loop)
4. **Deploy** tested code to Jetson Cube Orange

### Key Packages for Your Project:
- `ros-humble-mavros` - For PX4/ArduPilot integration
- `ros-humble-robot-localization` - For sensor fusion (GPS, IMU)
- `ros-humble-navigation2` - For autonomous navigation
- `ros-humble-rviz2` - For visualization

### Install MAVROS for simulation:
```bash
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras
```

---

## Troubleshooting

### Issue: RViz2 crashes or runs slowly
- Your Intel GPU should handle it fine
- If issues persist, reduce display quality in RViz2 settings

### Issue: Gazebo runs slowly
- Close unnecessary applications
- Reduce physics update rate in Gazebo world file
- Use simpler 3D models

### Issue: "Cannot find package"
```bash
# Rebuild package database
rosdep update
source /opt/ros/humble/setup.bash
```

### Issue: Command not found
```bash
# Make sure ROS 2 is sourced
source /opt/ros/humble/setup.bash
# Or open a new terminal (auto-sources from ~/.bashrc)
```

---

## Next Steps

1. **Learn ROS 2 basics**: https://docs.ros.org/en/humble/Tutorials.html
2. **Create your rover URDF model** based on Jetson Cube Orange specs
3. **Integrate MAVROS2** for PX4 simulation
4. **Add sensors** (camera, GPS, IMU) to your Gazebo model
5. **Test navigation algorithms** before deploying to hardware

---

## Useful Resources

- ROS 2 Humble Docs: https://docs.ros.org/en/humble/
- Gazebo Tutorials: https://classic.gazebosim.org/tutorials
- RViz2 User Guide: https://github.com/ros2/rviz
- URDF Tutorial: https://wiki.ros.org/urdf/Tutorials
- Navigation2: https://navigation.ros.org/

---

## Disk Space Management

Current usage: 145 GB / 234 GB (66%)
ROS 2 + Gazebo: ~5 GB

**Tips to save space:**
- Clean APT cache: `sudo apt clean`
- Remove old packages: `sudo apt autoremove`
- Clean ROS 2 build artifacts: `rm -rf ~/ros2_ws/build ~/ros2_ws/install`
