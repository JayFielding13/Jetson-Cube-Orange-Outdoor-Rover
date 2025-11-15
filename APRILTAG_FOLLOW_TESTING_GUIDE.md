# AprilTag Follow-Me Testing Guide
**Complete obstacle avoidance and AprilTag following system**

---

## What We Built

### **5 New ROS 2 Nodes:**
1. ✅ `obstacle_detector_ultrasonic` - Monitors 6 ultrasonic sensors
2. ✅ `obstacle_detector_lidar` - Processes LiDAR scans
3. ✅ `obstacle_fusion` - Combines both sensors for safety
4. ✅ `apriltag_detector` - Finds AprilTags in camera feed
5. ✅ `apriltag_follower` - Smart following with tag re-acquisition

### **Key Features:**
- **Multi-sensor obstacle avoidance** (LiDAR + Ultrasonics)
- **AprilTag visual tracking**
- **Smart tag re-acquisition** after losing sight
- **State machine** for robust behavior
- **Emergency stop** on obstacle detection
- **Search pattern** when tag is lost

---

## Testing on Desktop (Gazebo Simulation)

### Step 1: Build the Workspace

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jetson_rover_bridge jetson_rover_sim
source install/setup.bash
```

### Step 2: Launch Full Simulation with AprilTag

**Terminal 1 - Main Simulation:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch jetson_rover_sim full_simulation.launch.py
```

This launches:
- Gazebo with rover
- RViz with sensors
- Xbox controller support

### Step 3: Add AprilTag to Gazebo

**Option A: Add via Gazebo GUI**
1. In Gazebo window, click "Insert" tab
2. Navigate to: `ros2_ws/src/jetson_rover_sim/models/`
3. Select "apriltag_stand"
4. Click in world to place it (3-5 meters in front of rover)

**Option B: Add via Command (if model path is set)**
```bash
# First ensure Gazebo can find the model
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/jetson_rover_sim/models

# Then insert via command (in a new terminal)
gz model --spawn-model=apriltag_stand --model-name=tag_target --pose="3 0 0 0 0 0"
```

### Step 4: Launch Obstacle Avoidance & AprilTag Following

**Terminal 2 - AprilTag System:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch jetson_rover_bridge apriltag_follow_test.launch.py
```

This starts:
- Ultrasonic obstacle detector
- LiDAR obstacle detector
- Sensor fusion
- AprilTag detector
- AprilTag follower (disabled by default)

### Step 5: Monitor System Status

**Terminal 3 - Topic Monitoring:**
```bash
cd ~/ros2_ws
source install/setup.bash

# Watch AprilTag detection
ros2 topic echo /apriltag/detected

# Watch tag position
ros2 topic echo /apriltag/target_pose

# Watch obstacle status
ros2 topic echo /obstacles/danger

# View debug image (if installed image viewer)
ros2 run rqt_image_view rqt_image_view /apriltag/debug_image
```

### Step 6: Enable Follow-Me Mode

**Terminal 4 - Enable Follower:**
```bash
cd ~/ros2_ws
source install/setup.bash

# Enable following
ros2 topic pub --once /follow_me/enable std_msgs/Bool "data: true"

# Disable following
ros2 topic pub --once /follow_me/enable std_msgs/Bool "data: false"
```

---

## Testing Scenarios

### Test 1: Basic Tag Detection

**Goal:** Verify AprilTag is being detected

**Steps:**
1. Launch simulation + AprilTag system
2. Place AprilTag 3m in front of rover
3. Monitor `/apriltag/detected` topic

**Expected:**
- Topic shows `data: true`
- `/apriltag/target_pose` shows distance ~3.0m
- Debug image shows green box around tag

### Test 2: Simple Following

**Goal:** Rover follows stationary tag

**Steps:**
1. Place tag 3m in front
2. Enable follow-me mode
3. Observe rover movement

**Expected:**
- Rover drives toward tag
- Stops at 2m distance (target_distance parameter)
- Maintains position

### Test 3: Moving Tag

**Goal:** Rover tracks moving tag

**Steps:**
1. Enable follow-me
2. In Gazebo, select AprilTag model
3. Use mouse to drag tag around
4. Move it left, right, forward, backward

**Expected:**
- Rover adjusts to keep tag at 2m
- Smooth tracking
- No oscillation

### Test 4: Obstacle Avoidance

**Goal:** Rover stops when obstacle detected

**Steps:**
1. Enable follow-me
2. Place box between rover and tag (using Gazebo Insert)
3. Rover should approach obstacle

**Expected:**
- Rover stops at ~0.8m from obstacle (danger_distance)
- Doesn't hit obstacle
- Obstacle warnings in terminal
- Follow-me pauses until obstacle removed

### Test 5: Tag Re-Acquisition

**Goal:** Rover finds lost tag

**Steps:**
1. Enable follow-me with tag visible
2. Rotate tag 180° (rover loses sight)
3. Wait 1 second
4. Rover should start searching

**Expected:**
- After 1s, rover starts rotating
- Searches in direction tag was last seen
- Re-acquires tag when it comes into view
- Resumes following

### Test 6: Obstacle Avoidance + Re-Acquisition

**Goal:** Combined scenario

**Steps:**
1. Enable follow-me
2. Place obstacle to rover's right
3. Move tag behind obstacle (rover must avoid)
4. Rover loses sight of tag during avoidance
5. Clear path to tag

**Expected:**
- Rover stops at obstacle
- Tag lost during stop
- Rover searches for tag
- Re-acquires and resumes following

---

## RViz Visualization

In RViz2, you should see:
- **Robot Model:** Orange rover with sensors
- **LaserScan:** Red/green points showing LiDAR
- **Ultrasonic Ranges:** Cone shapes for each sensor
- **Camera Image:** (if enabled) Shows AprilTag detection

**To view AprilTag debug image in RViz:**
1. Click "Add" button
2. Select "Image"
3. Topic: `/apriltag/debug_image`

---

## Parameter Tuning

### Obstacle Detection

**In `apriltag_follow_test.launch.py`:**

```python
# LiDAR detector parameters
'danger_distance': 0.8,    # Stop distance (meters)
'warning_distance': 2.0,   # Warning distance
'field_of_view': 120.0,    # Front arc (degrees)

# Ultrasonic detector parameters
'danger_distance': 0.5,
'warning_distance': 1.0,
```

**Adjust if:**
- Rover stops too early → Decrease danger_distance
- Rover gets too close → Increase danger_distance
- False positives → Narrow field_of_view

### Following Behavior

```python
# AprilTag follower parameters
'target_distance': 2.0,      # Follow distance (meters)
'distance_tolerance': 0.3,   # Acceptable error
'max_linear_speed': 0.5,     # Max forward speed (m/s)
'max_angular_speed': 0.8,    # Max rotation speed (rad/s)
'linear_kp': 0.5,            # Distance controller gain
'angular_kp': 1.0,           # Rotation controller gain
```

**Adjust if:**
- Following too slow → Increase max speeds
- Too aggressive → Decrease max speeds or gains
- Oscillates → Decrease gains (especially angular_kp)
- Too sluggish → Increase gains

### Re-Acquisition

```python
'search_angular_speed': 0.3,  # Search rotation speed (rad/s)
'search_timeout': 10.0,       # Give up after (seconds)
'lost_tag_timeout': 1.0,      # Wait before searching (seconds)
```

**Adjust if:**
- Searches too soon → Increase lost_tag_timeout
- Searches too slowly → Increase search_angular_speed
- Gives up too early → Increase search_timeout

---

## Troubleshooting

### "AprilTag library not installed"

**Solution:**
```bash
pip3 install pupil-apriltags
# Or
pip3 install dt-apriltags
```

### Tag not detected in simulation

**Check:**
1. Is camera publishing? `ros2 topic hz /camera/image_raw`
2. Is tag in camera view? Check `/apriltag/debug_image`
3. Is model loaded? Look for tag in Gazebo world
4. Is texture applied? Tag should show pattern, not gray

**Fix texture issue:**
```bash
# Ensure texture file exists
ls ~/ros2_ws/src/jetson_rover_sim/models/apriltag_stand/materials/textures/apriltag_0.png

# If missing, regenerate:
cd ~/ros2_ws/src/jetson_rover_sim/models/apriltag_stand
wget https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11/tag36_11_00000.png -O materials/textures/apriltag_0.png
```

### Rover doesn't follow tag

**Check:**
1. Is follower enabled? `ros2 topic echo /follow_me/enable`
2. Is tag detected? `ros2 topic echo /apriltag/detected`
3. Are obstacles blocking? `ros2 topic echo /obstacles/danger`
4. Is cmd_vel being published? `ros2 topic hz /cmd_vel`

**Enable follower:**
```bash
ros2 topic pub --once /follow_me/enable std_msgs/Bool "data: true"
```

### Rover stops immediately / won't move

**Likely cause:** Obstacle danger triggered

**Check:**
```bash
ros2 topic echo /obstacles/danger
ros2 topic echo /obstacles/lidar/danger
ros2 topic echo /obstacles/ultrasonic/danger
```

**Solutions:**
- Increase danger_distance thresholds
- Check for spurious sensor readings
- Verify LiDAR not seeing ground/wheels as obstacles

### Tag re-acquisition doesn't work

**Debug:**
```bash
# Watch follower state
ros2 topic echo /follow_me/active

# Watch detected status
ros2 topic echo /apriltag/detected
```

**Check:**
- `lost_tag_timeout` may be too long
- Tag may be completely out of camera FOV
- Search pattern may be wrong direction

---

## Command Reference

### Quick Start
```bash
# Terminal 1: Main simulation
cd ~/ros2_ws && source install/setup.bash
ros2 launch jetson_rover_sim full_simulation.launch.py

# Terminal 2: AprilTag system
cd ~/ros2_ws && source install/setup.bash
ros2 launch jetson_rover_bridge apriltag_follow_test.launch.py

# Terminal 3: Enable following
ros2 topic pub --once /follow_me/enable std_msgs/Bool "data: true"
```

### Monitor Topics
```bash
# AprilTag detection
ros2 topic echo /apriltag/detected
ros2 topic echo /apriltag/target_pose

# Obstacles
ros2 topic echo /obstacles/danger
ros2 topic echo /obstacles/warning

# Follower state
ros2 topic echo /follow_me/active

# Debug image
ros2 run rqt_image_view rqt_image_view /apriltag/debug_image
```

### Control Commands
```bash
# Enable/disable following
ros2 topic pub --once /follow_me/enable std_msgs/Bool "data: true"
ros2 topic pub --once /follow_me/enable std_msgs/Bool "data: false"

# Emergency stop (send zero velocity)
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

---

## Next Steps

### After Simulation Testing

1. **Deploy to Real Jetson:**
   ```bash
   # Copy package to Jetson
   scp -r ~/ros2_ws/src/jetson_rover_bridge jay@192.168.254.100:~/ros2_ws/src/

   # Build on Jetson
   ssh jay@192.168.254.100
   cd ~/ros2_ws
   colcon build --packages-select jetson_rover_bridge
   source install/setup.bash
   ```

2. **Print Real AprilTag:**
   - Download tag from: https://github.com/AprilRobotics/apriltag-imgs/tree/master/tag36h11
   - Use tag36_11_00000.png
   - Print at 6.5 inches (165mm) square
   - Mount on rigid backing

3. **Calibrate Camera:**
   ```bash
   # Install calibration tools
   sudo apt install ros-humble-camera-calibration

   # Run calibration
   ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.0254 image:=/camera/image_raw
   ```

4. **Outdoor Testing:**
   - Start with tag stationary
   - Progress to walking with tag
   - Add obstacles
   - Test re-acquisition

---

## Performance Metrics

**Target Performance:**
- AprilTag detection rate: 30 Hz
- Tag re-acquisition time: <2 seconds
- Obstacle detection latency: <100ms
- Following accuracy: ±30cm
- No collisions during testing

**Monitor with:**
```bash
ros2 topic hz /apriltag/detected
ros2 topic hz /obstacles/danger
ros2 topic hz /cmd_vel
```

---

## Files Created

```
ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/
├── obstacle_detector_ultrasonic.py
├── obstacle_detector_lidar.py
├── obstacle_fusion.py
├── apriltag_detector.py
└── apriltag_follower.py

ros2_ws/src/jetson_rover_bridge/launch/
└── apriltag_follow_test.launch.py

ros2_ws/src/jetson_rover_sim/models/apriltag_stand/
├── model.config
├── model.sdf
├── materials/
│   ├── scripts/apriltag.material
│   └── textures/apriltag_0.png
└── apriltag_0.png
```

---

**Status:** ✅ Ready for simulation testing!
**Next:** Run the tests above and tune parameters
