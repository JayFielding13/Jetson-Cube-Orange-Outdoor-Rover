# Reactive Obstacle Avoidance Testing Guide
**Vector Field Histogram approach - NEVER STOPS, ALWAYS PROBLEM-SOLVES**

---

## What Changed

### Philosophy Shift: From Stop to Steer

**OLD APPROACH (Removed):**
- Robot stops when obstacle detected
- Problem: Leads to "paralysis" from false positives or legitimate obstacles
- Result: Robot stuck, unable to progress

**NEW APPROACH (Reactive):**
- Robot **steers away** from obstacles while maintaining motion
- Uses Vector Field Histogram (VFH) to find safest direction
- Slows down near obstacles but maintains minimum forward speed (0.1 m/s)
- "Bendy ruler" logic - always tries to find a way around
- Problem-solving behavior instead of defensive stopping

---

## New Nodes

### 1. reactive_obstacle_avoidance
**Purpose:** Builds polar histogram and finds safe steering direction

**Inputs:**
- `/obstacles/ultrasonic/closest` - Closest ultrasonic reading
- `/obstacles/lidar/closest` - Closest LiDAR obstacle
- `/cmd_vel` or desired direction (future: from goal planner)

**Outputs:**
- `/avoidance/steering_vector` (Vector3) - Safe direction to steer (x, y components)
- `/avoidance/safe_direction` (Float32) - Safe direction angle (radians)
- `/avoidance/speed_scale` (Vector3) - Speed scaling factor (0.1 to 1.0)
- `/avoidance/critical_danger` (Bool) - True if obstacle < 0.4m (critical zone)

**Key Algorithm:**
1. Build 360° polar histogram (72 sectors, 5° each)
2. Add obstacle density from both LiDAR and ultrasonics
3. Weight obstacles closer = higher density
4. Find sector with lowest density closest to desired direction
5. Scale speed: 1.0 (clear) → 0.1 (critical, but still moving!)

**Parameters:**
```python
critical_distance: 0.4      # Absolute minimum - speed_scale = 0.1
influence_distance: 2.5     # Beyond this, no avoidance needed
obstacle_weight: 2.0        # Histogram density weighting
sector_angle: 5.0           # Degrees per sector
num_sectors: 72             # 360 / 5 = 72 sectors
```

### 2. apriltag_follower_reactive
**Purpose:** Follows AprilTag while blending with reactive avoidance

**Key Features:**
- **State Machine:**
  - IDLE - Waiting for enable
  - FOLLOWING - Clear path, normal tag tracking
  - AVOIDING_AND_SEEKING - Obstacles present, blend avoidance + following
  - TAG_LOST - Just lost tag, slow coast
  - SEARCHING - Actively searching for lost tag

- **Blending Logic:**
  - Normal: 70% avoidance, 30% tag-following direction
  - Critical (speed_scale < 0.5): 90% avoidance, 10% tag-following
  - Ensures minimum forward speed (0.1 m/s) unless critical danger

- **Smart Re-acquisition:**
  - Remembers which side tag was last seen
  - Searches in safe direction (avoids obstacles while rotating)
  - Maintains slow forward motion during search
  - Timeout after 15 seconds

**Parameters:**
```python
target_distance: 2.0        # Follow distance (meters)
distance_tolerance: 0.3     # Acceptable error
max_linear_speed: 0.6       # m/s
max_angular_speed: 1.0      # rad/s
linear_kp: 0.5              # Distance P gain
angular_kp: 1.2             # Angle P gain
avoidance_blend: 0.7        # Weight toward avoidance (0-1)
min_forward_speed: 0.1      # Never stop! (m/s)
search_angular_speed: 0.4   # Search rotation speed
search_timeout: 15.0        # Give up after (seconds)
lost_tag_timeout: 1.5       # Wait before searching
```

---

## Testing Plan

### Prerequisites

**Install Dependencies:**
```bash
# AprilTag library
pip3 install pupil-apriltags
# OR
pip3 install dt-apriltags
```

**Build Workspace:**
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jetson_rover_bridge jetson_rover_sim
source install/setup.bash
```

---

### Test 1: Basic Reactive Avoidance (No AprilTag)

**Goal:** Verify reactive avoidance steers away from obstacles without stopping

**Setup:**
```bash
# Terminal 1 - Launch simulation
cd ~/ros2_ws && source install/setup.bash
ros2 launch jetson_rover_sim full_simulation.launch.py

# Terminal 2 - Launch reactive avoidance only
cd ~/ros2_ws && source install/setup.bash
ros2 run jetson_rover_bridge reactive_obstacle_avoidance
```

**Monitoring:**
```bash
# Terminal 3 - Watch avoidance outputs
ros2 topic echo /avoidance/safe_direction
ros2 topic echo /avoidance/speed_scale
ros2 topic echo /avoidance/critical_danger
```

**Testing:**
1. Place obstacle 1.5m in front of rover (using Gazebo Insert)
2. Send test velocity command:
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
   ```
3. Observe: speed_scale should decrease (but not to 0)
4. Observe: safe_direction should point away from obstacle

**Expected:**
- `speed_scale` drops to ~0.3-0.5 at 1.5m distance
- `safe_direction` shows steering away from obstacle
- `critical_danger` = false (only true if < 0.4m)

---

### Test 2: AprilTag Following with Clear Path

**Goal:** Verify basic tag following works

**Setup:**
```bash
# Terminal 1 - Main simulation
cd ~/ros2_ws && source install/setup.bash
ros2 launch jetson_rover_sim full_simulation.launch.py

# Terminal 2 - Reactive AprilTag system
cd ~/ros2_ws && source install/setup.bash
ros2 launch jetson_rover_bridge apriltag_follow_test.launch.py
```

**Insert AprilTag in Gazebo:**
1. In Gazebo GUI, click "Insert" tab
2. Navigate to `ros2_ws/src/jetson_rover_sim/models/`
3. Select `apriltag_stand`
4. Place 3 meters in front of rover

**Enable Following:**
```bash
# Terminal 3
cd ~/ros2_ws && source install/setup.bash
ros2 topic pub --once /follow_me/enable std_msgs/Bool "data: true"
```

**Monitoring:**
```bash
# Terminal 4 - Watch follower
ros2 topic echo /apriltag/detected
ros2 topic echo /follow_me/active
```

**Expected:**
- Rover approaches tag
- Stops at 2.0m ± 0.3m
- Smooth motion, no oscillation
- Log shows: "STATE: FOLLOWING"

---

### Test 3: Reactive Avoidance While Following

**Goal:** Verify rover steers around obstacle WITHOUT STOPPING

**Setup:**
1. Start with Test 2 setup (tag following enabled)
2. Rover should be following tag at 2.0m distance

**Testing:**
1. In Gazebo, insert a box obstacle between rover and tag
2. Place obstacle ~1.0m from rover, slightly off-center
3. Observe rover behavior

**Expected Behavior:**
- Rover **does NOT stop**
- Speed decreases (you'll see it slow down)
- Rover steers away from obstacle (left or right)
- Log shows: "STATE: AVOIDING & SEEKING"
- Log shows speed_scale value (e.g., 0.3)
- Rover maintains motion at reduced speed

**Critical Test - Very Close Obstacle:**
1. Place obstacle 0.5m from rover
2. Rover should slow to 10% speed (min_forward_speed)
3. Rover should NOT come to complete stop
4. Rover should steer strongly away

**What NOT to see:**
- ❌ Complete stop (speed = 0)
- ❌ Collision with obstacle
- ❌ "Paralysis" behavior

---

### Test 4: Tag Re-acquisition After Avoidance

**Goal:** Verify rover finds tag after losing sight during avoidance

**Setup:**
1. Enable tag following
2. Rover following tag at 2.0m

**Testing:**
1. Place large obstacle directly between rover and tag
2. Rover steers away to avoid
3. Tag goes out of camera FOV
4. Wait 1.5 seconds (lost_tag_timeout)
5. Rover should enter SEARCHING state

**Expected:**
- Log: "STATE: TAG LOST" (first 1.5 seconds)
- Log: "STATE: SEARCHING" (after 1.5 seconds)
- Rover rotates while moving slowly forward
- Rotation direction: toward where tag was last seen
- Search uses safe direction (avoids obstacles)
- When tag visible again: "✓ Tag re-acquired!"
- Log: "STATE: FOLLOWING" resumes

---

### Test 5: Complex Scenario - Moving Tag + Obstacles

**Goal:** Full system test with dynamic environment

**Setup:**
1. Launch full system
2. Enable following
3. Place multiple obstacles in environment

**Testing:**
1. Use Gazebo to move AprilTag around
2. Move tag behind obstacles
3. Move tag left and right
4. Observe rover behavior

**Expected:**
- Rover continuously adapts
- Never stops completely (always ≥ 0.1 m/s)
- Smoothly blends following + avoidance
- Re-acquires tag when visible
- Logs show state transitions:
  - FOLLOWING → AVOIDING_AND_SEEKING → TAG_LOST → SEARCHING → FOLLOWING

---

## Tuning Parameters

### If Rover Too Cautious (Slows Too Early)

**Adjust reactive_obstacle_avoidance:**
```python
critical_distance: 0.3      # Decrease (was 0.4)
influence_distance: 2.0     # Decrease (was 2.5)
```

### If Rover Too Aggressive (Gets Too Close)

**Adjust reactive_obstacle_avoidance:**
```python
critical_distance: 0.6      # Increase (was 0.4)
influence_distance: 3.0     # Increase (was 2.5)
obstacle_weight: 3.0        # Increase (was 2.0) - makes obstacles "stronger"
```

### If Avoidance Doesn't Steer Enough

**Adjust apriltag_follower_reactive:**
```python
avoidance_blend: 0.8        # Increase (was 0.7) - more weight to avoidance
```

### If Following Too Slow/Weak

**Adjust apriltag_follower_reactive:**
```python
max_linear_speed: 0.8       # Increase (was 0.6)
linear_kp: 0.7              # Increase (was 0.5)
avoidance_blend: 0.5        # Decrease (was 0.7) - more weight to following
```

### If Search Takes Too Long

**Adjust apriltag_follower_reactive:**
```python
search_angular_speed: 0.6   # Increase (was 0.4)
lost_tag_timeout: 1.0       # Decrease (was 1.5) - start searching sooner
```

### If Rover Oscillates/Unstable

**Adjust apriltag_follower_reactive:**
```python
angular_kp: 0.8             # Decrease (was 1.2)
max_angular_speed: 0.7      # Decrease (was 1.0)
```

---

## Monitoring Commands

### Real-time Topic Monitoring

```bash
# Obstacle detection
ros2 topic echo /obstacles/lidar/closest
ros2 topic echo /obstacles/ultrasonic/closest

# Reactive avoidance
ros2 topic echo /avoidance/safe_direction
ros2 topic echo /avoidance/speed_scale
ros2 topic echo /avoidance/critical_danger

# AprilTag detection
ros2 topic echo /apriltag/detected
ros2 topic echo /apriltag/target_pose

# Follower state
ros2 topic echo /follow_me/active

# Velocity commands
ros2 topic echo /cmd_vel
```

### Topic Rates (Performance)

```bash
ros2 topic hz /apriltag/detected     # Should be ~30 Hz
ros2 topic hz /cmd_vel               # Should be ~10 Hz
ros2 topic hz /avoidance/safe_direction  # Should be ~10 Hz
```

### Debug Images

```bash
# View AprilTag detection visualization
ros2 run rqt_image_view rqt_image_view /apriltag/debug_image
```

---

## Troubleshooting

### Rover Still Stops at Obstacles

**Possible Causes:**
1. Old obstacle_fusion node still running
2. min_forward_speed set too low

**Fix:**
```bash
# Kill all old nodes
ros2 node list  # Check for obstacle_fusion
pkill -f obstacle_fusion

# Verify reactive node is running
ros2 node list | grep reactive

# Increase minimum speed
# Edit launch file: min_forward_speed: 0.15
```

### Rover Ignores Obstacles / Collides

**Possible Causes:**
1. critical_distance too small
2. Sensors not publishing data
3. avoidance_blend too low

**Debug:**
```bash
# Check sensor data
ros2 topic hz /scan  # LiDAR
ros2 topic hz /ultrasonic/front  # Ultrasonics

# Check avoidance is running
ros2 node info /reactive_obstacle_avoidance

# Increase safety margins
# Edit launch: critical_distance: 0.6, influence_distance: 3.0
```

### Tag Not Detected

**Check:**
```bash
# Camera working?
ros2 topic hz /camera/image_raw

# View camera feed
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# View debug image with tag overlay
ros2 run rqt_image_view rqt_image_view /apriltag/debug_image
```

**Fix texture if tag shows gray:**
```bash
cd ~/ros2_ws/src/jetson_rover_sim/models/apriltag_stand
wget https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11/tag36_11_00000.png -O materials/textures/apriltag_0.png
```

### Rover Spins in Place During Search

**Possible Causes:**
1. min_forward_speed = 0 during search
2. Search logic not adding forward velocity

**Check Code:**
```python
# In apriltag_follower_reactive.py, SEARCHING state should have:
cmd.linear.x = self.min_speed  # Should be 0.1, not 0
```

### Avoidance Too Sensitive / False Positives

**Possible Causes:**
1. LiDAR seeing ground/wheels
2. Ultrasonic noise

**Fix:**
```bash
# Increase minimum valid ranges
# In launch file for lidar_detector:
# min_valid_range: 0.15  (was 0.1)

# For ultrasonic_detector:
# min_valid_range: 0.3  (was 0.2)
```

---

## Performance Targets

**Target Metrics:**
- AprilTag detection: 30 Hz
- Control loop: 10 Hz
- Re-acquisition time: < 3 seconds
- Following accuracy: ± 30cm
- **ZERO complete stops** during normal operation
- **ZERO collisions** during testing
- Speed scale range: 0.1 (critical) to 1.0 (clear)

---

## Next Steps After Testing

### 1. Outdoor Deployment

**Print AprilTag:**
- Download: https://github.com/AprilRobotics/apriltag-imgs/tree/master/tag36h11
- Use: tag36_11_00000.png
- Size: 6.5 inches (165mm) square
- Mount on rigid backing

**Deploy to Jetson:**
```bash
# From desktop
cd ~/ros2_ws
scp -r src/jetson_rover_bridge jay@100.91.191.47:~/ros2_ws/src/

# On Jetson
ssh jay@100.91.191.47
cd ~/ros2_ws
colcon build --packages-select jetson_rover_bridge
source install/setup.bash
```

### 2. Camera Calibration

```bash
sudo apt install ros-humble-camera-calibration
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.0254 image:=/camera/image_raw
```

### 3. Real-World Testing Sequence

1. **Static tag, clear path** - Verify basic following
2. **Static tag, single obstacle** - Verify avoidance
3. **Static tag, multiple obstacles** - Verify complex avoidance
4. **Walking with tag, clear path** - Verify dynamic following
5. **Walking with tag, obstacles** - Full system test

---

## Files Modified

```
ros2_ws/src/jetson_rover_bridge/
├── setup.py                                    [UPDATED]
├── jetson_rover_bridge/
│   ├── reactive_obstacle_avoidance.py         [NEW]
│   └── apriltag_follower_reactive.py          [NEW]
└── launch/
    └── apriltag_follow_test.launch.py         [UPDATED]
```

---

**Status:** ✅ Built and ready for testing!

**Philosophy:** "Never stop, always find a way!" - Problem-solving rover 🤖
