# Obstacle Avoidance Development Guide
## Testing in Gazebo Simulation

**Date:** November 11, 2025
**Purpose:** Develop and test obstacle avoidance before deploying to real rover

---

## Overview

We've created a simple obstacle avoidance controller that uses LiDAR data to navigate around obstacles. The rover will:

- **Move forward** when the path is clear (> 2.5m)
- **Slow down** when obstacles are nearby (1.5m - 2.5m)
- **Stop and turn** when obstacles are too close (< 1.5m)

---

## Quick Start

### 1. Make sure simulation is running on desktop

The simulation should already be running from earlier. If not:

```bash
# On desktop (or SSH from Jetson)
source ~/ros2_distributed_setup.sh
cd ~/ros2_ws
source install/setup.bash
ros2 launch jetson_rover_sim spawn_rover.launch.py
```

### 2. Run the obstacle avoidance controller

**On Jetson:**
```bash
cd ~/Desktop/Mini\ Rover\ Development/Jetson\ Cube\ Orange\ Outdoor\ Rover
./run_obstacle_avoidance.sh
```

Or manually:
```bash
source ros2_distributed_setup.sh
python3 simple_obstacle_avoidance.py
```

### 3. Watch the rover navigate!

On your desktop, you should see the rover:
- Driving forward
- Turning when it gets close to obstacles
- Navigating around the environment

---

## How It Works

### LiDAR Sector Analysis

The controller divides the 360° LiDAR scan into sectors:

```
           FRONT (0°)
              |
   FRONT-LEFT | FRONT-RIGHT
      (+45°)  |  (-45°)
              |
    LEFT -----+----- RIGHT
    (+90°)           (-90°)
              |
              |
           REAR (180°)
```

It monitors these key sectors:
- **Front**: -30° to +30°
- **Front-Left**: 30° to 90°
- **Front-Right**: -90° to -30°
- **Left**: 90° to 150°
- **Right**: -150° to -90°

### Decision Logic

**Clear Path (> 2.5m):**
```
linear.x = 0.5 m/s (full speed)
angular.z = 0.0 (straight)
```

**Nearby Obstacle (1.5m - 2.5m):**
```
linear.x = 0.15 m/s (30% speed)
angular.z = ±0.15 rad/s (gentle turn away)
```

**Danger Zone (< 1.5m):**
```
linear.x = 0.0 m/s (STOP!)
angular.z = ±0.3 rad/s (sharp turn away)
```

**Turn Direction:**
- If obstacle on left → Turn right (negative angular.z)
- If obstacle on right → Turn left (positive angular.z)

---

## Testing Scenarios

### Scenario 1: Simple Wall Approach

**Setup:** Rover facing a wall in empty world
**Expected:** Rover approaches, stops at 1.5m, turns away

**Test:**
1. Launch simulation
2. Start obstacle avoidance
3. Watch rover approach obstacle
4. Should stop and turn before collision

### Scenario 2: Narrow Corridor

**Setup:** Place boxes to create corridor
**Expected:** Rover navigates through, slowing when walls are close

**How to add obstacles in Gazebo:**
1. In Gazebo window, go to "Insert" tab (left panel)
2. Click "Box" or "Cylinder"
3. Click in world to place object
4. Repeat to create corridor

### Scenario 3: Corner Navigation

**Setup:** Two walls forming 90° corner
**Expected:** Rover approaches, detects both walls, turns appropriately

### Scenario 4: Open Field with Scattered Obstacles

**Setup:** Multiple objects randomly placed
**Expected:** Rover weaves between obstacles, never stopping

**Use test_yard world for this:**
```bash
# Restart simulation with test_yard
ros2 launch jetson_rover_sim visualize_rover.launch.py world:=test_yard
```

---

## Adjusting Parameters

You can tune the behavior by editing [simple_obstacle_avoidance.py](simple_obstacle_avoidance.py):

### Speed Settings (Lines 24-27)

```python
self.declare_parameter('max_speed', 0.5)      # Maximum forward speed
self.declare_parameter('min_distance', 1.5)   # STOP distance
self.declare_parameter('safe_distance', 2.5)  # Slow down distance
self.declare_parameter('turn_speed', 0.3)     # Turn rate
```

**To make rover more aggressive:**
```python
max_speed = 0.8          # Faster
min_distance = 1.0       # Stops closer
safe_distance = 2.0      # Worries less
turn_speed = 0.5         # Turns sharper
```

**To make rover more cautious:**
```python
max_speed = 0.3          # Slower
min_distance = 2.0       # Stops farther
safe_distance = 3.5      # Worries more
turn_speed = 0.2         # Turns gentler
```

### Sector Angles (Lines 62-96)

You can modify which angles are monitored:

```python
sectors = {
    'front': min_in_range(-30, 30),        # Widen to -45, 45 for more cautious
    'front_left': min_in_range(30, 90),
    'left': min_in_range(90, 150),
    'front_right': min_in_range(-90, -30),
    'right': min_in_range(-150, -90),
}
```

---

## Adding Ultrasonic Integration

Currently only uses LiDAR. Let's add ultrasonics for close-range safety!

### Enhanced Version with Ultrasonics

Edit [simple_obstacle_avoidance.py](simple_obstacle_avoidance.py) to add:

```python
from sensor_msgs.msg import Range

class ObstacleAvoidance(Node):
    def __init__(self):
        # ... existing code ...

        # Add ultrasonic subscribers
        self.ultrasonic_ranges = {}

        for sensor in ['front', 'corner_left', 'corner_right',
                      'side_left', 'side_right', 'rear']:
            self.create_subscription(
                Range,
                f'/ultrasonic/{sensor}',
                lambda msg, s=sensor: self.ultrasonic_callback(msg, s),
                10
            )

    def ultrasonic_callback(self, msg, sensor_name):
        """Store ultrasonic reading"""
        if msg.range < msg.max_range:  # Valid reading
            self.ultrasonic_ranges[sensor_name] = msg.range

    def control_loop(self):
        # ... existing LiDAR logic ...

        # Emergency stop if ultrasonic detects very close obstacle
        if self.ultrasonic_ranges:
            min_ultrasonic = min(self.ultrasonic_ranges.values())

            if min_ultrasonic < 0.5:  # 50cm emergency stop
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed  # Spin in place
                self.get_logger().warn(f'🚨 EMERGENCY STOP - Ultrasonic: {min_ultrasonic:.2f}m')
                self.cmd_pub.publish(cmd)
                return

        # ... rest of control logic ...
```

This adds a safety layer using the 6 ultrasonic sensors for last-moment collision avoidance!

---

## Troubleshooting

### Rover doesn't move

**Check:**
```bash
source ros2_distributed_setup.sh
ros2 topic list | grep scan
```

Should see `/scan` topic. If not, simulation isn't running or network misconfigured.

### Rover moves but doesn't avoid obstacles

**Check LiDAR data:**
```bash
source ros2_distributed_setup.sh
ros2 topic echo /scan --once
```

Look at `ranges` array - should show numbers, not all `inf`.

### Rover is too cautious/aggressive

Edit parameters in [simple_obstacle_avoidance.py](simple_obstacle_avoidance.py) lines 24-27.

### Controller crashes

Check Python errors:
```bash
python3 simple_obstacle_avoidance.py
```

Most common: Missing `numpy`:
```bash
pip3 install numpy
```

---

## Next Steps

### 1. Add Goal-Directed Navigation

Instead of random wandering, navigate to a GPS waypoint:

```python
class WaypointNavigator(Node):
    def __init__(self):
        # Subscribe to GPS
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)

        # Set target waypoint
        self.target_lat = 37.7749  # Example
        self.target_lon = -122.4194

    def gps_callback(self, msg):
        # Calculate bearing to target
        bearing = self.calculate_bearing(
            msg.latitude, msg.longitude,
            self.target_lat, self.target_lon
        )

        # Turn toward target (if no obstacles)
        # ... combine with obstacle avoidance ...
```

### 2. Add State Machine

Implement states: NAVIGATING, AVOIDING, STUCK, GOAL_REACHED

```python
from enum import Enum

class RoverState(Enum):
    NAVIGATING = 1
    AVOIDING = 2
    STUCK = 3
    GOAL_REACHED = 4

class SmartNavigator(Node):
    def __init__(self):
        self.state = RoverState.NAVIGATING
        self.stuck_counter = 0

    def control_loop(self):
        if self.state == RoverState.NAVIGATING:
            # Head toward goal
            pass
        elif self.state == RoverState.AVOIDING:
            # Avoid obstacle, then return to navigating
            pass
        elif self.state == RoverState.STUCK:
            # Backup and try different direction
            pass
```

### 3. Add Recovery Behaviors

If rover gets stuck (no progress for 10 seconds):
- Back up 1 meter
- Rotate 90°
- Try again

### 4. Implement Geofence

Check GPS position, stop if approaching boundary:

```python
def check_geofence(self, lat, lon):
    # Define boundary
    boundary = {
        'min_lat': 37.7700,
        'max_lat': 37.7800,
        'min_lon': -122.4200,
        'max_lon': -122.4100
    }

    if not (boundary['min_lat'] <= lat <= boundary['max_lat'] and
            boundary['min_lon'] <= lon <= boundary['max_lon']):
        self.get_logger().warn('⚠️  Approaching geofence boundary!')
        return False
    return True
```

---

## Files Created

- **[simple_obstacle_avoidance.py](simple_obstacle_avoidance.py)** - Main controller
- **[run_obstacle_avoidance.sh](run_obstacle_avoidance.sh)** - Launch script
- **[OBSTACLE_AVOIDANCE_GUIDE.md](OBSTACLE_AVOIDANCE_GUIDE.md)** - This guide

---

## Real Robot Deployment

When ready to test on real hardware:

1. **Same code works!** - No changes needed
2. **Only difference:**
   ```bash
   # On real robot
   source /opt/ros/humble/setup.bash  # Don't use ros2_distributed_setup.sh
   python3 simple_obstacle_avoidance.py
   ```

3. **Real sensors provide data:**
   - RP-LIDAR → `/scan`
   - ESP32 ultrasonics → `/ultrasonic/*`
   - MAVROS receives `/cmd_vel` → controls motors

4. **Start conservatively:**
   - Reduce `max_speed` to 0.2 m/s for first tests
   - Increase `min_distance` to 2.5m
   - Test in open area with RC override ready

---

## Summary

You now have a working obstacle avoidance system running in simulation! The rover uses LiDAR data to navigate around obstacles autonomously.

**Current Status:**
- ✅ LiDAR-based obstacle detection
- ✅ Variable speed control (fast/slow/stop)
- ✅ Turn-away behavior
- ✅ Sector-based analysis
- ✅ Real-time decision making

**Next to Add:**
- ⏳ Ultrasonic integration for close-range safety
- ⏳ GPS waypoint navigation
- ⏳ State machine for complex behaviors
- ⏳ Recovery from stuck situations
- ⏳ Geofence boundary checking

**When Validated:** Deploy to real rover with confidence!

---

**Have fun testing! Try driving the rover around the simulation and watch it avoid obstacles autonomously.**
