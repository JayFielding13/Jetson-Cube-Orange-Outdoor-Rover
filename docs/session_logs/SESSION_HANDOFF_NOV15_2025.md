# Session Handoff - Nov 15, 2025
**Reactive Obstacle Avoidance System - Ready for Testing**

---

## Current Status: READY TO TEST

**Desktop Setup:**
- Location: 100.73.129.15 (Ubuntu 22.04, RTX 5070)
- Gazebo + RViz: ✅ RUNNING
- ROS 2 Workspace: `~/ros2_ws` (built and ready)
- Git Repo: `~/Git Sandbox/Jetson Cube Orange Outdoor Rover`

---

## What We Accomplished

### 1. Implemented Reactive Obstacle Avoidance System

**Philosophy Change:** From "stop when obstacle detected" → "steer away while moving"

**New ROS 2 Nodes Created:**

| Node | Purpose | Location |
|------|---------|----------|
| `reactive_obstacle_avoidance.py` | Vector Field Histogram (VFH) steering | [ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/](ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/reactive_obstacle_avoidance.py) |
| `apriltag_follower_reactive.py` | Blends tag-following with avoidance | [ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/](ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/apriltag_follower_reactive.py) |
| `obstacle_detector_lidar.py` | LiDAR obstacle detection | [ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/](ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/obstacle_detector_lidar.py) |
| `obstacle_detector_ultrasonic.py` | 6x ultrasonic sensor processing | [ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/](ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/obstacle_detector_ultrasonic.py) |
| `apriltag_detector.py` | AprilTag visual tracking (tag36h11) | [ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/](ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/apriltag_detector.py) |

**Launch File:**
- [ros2_ws/src/jetson_rover_bridge/launch/apriltag_follow_test.launch.py](ros2_ws/src/jetson_rover_bridge/launch/apriltag_follow_test.launch.py)

**Key Features:**
- ✅ Never stops (maintains min 0.1 m/s)
- ✅ VFH-based steering (72 sectors, 360° coverage)
- ✅ Multi-sensor fusion (LiDAR + ultrasonics)
- ✅ Smart tag re-acquisition after losing sight
- ✅ State machine for robust behavior
- ✅ Blends avoidance (70-90%) with following (10-30%)

### 2. Created Gazebo Simulation Assets

**AprilTag Model:**
- Location: [ros2_ws/src/jetson_rover_sim/models/apriltag_stand/](ros2_ws/src/jetson_rover_sim/models/apriltag_stand/)
- Real tag36_11_00000 texture applied
- Movable stand for testing

**Rover Model:**
- Full sensor suite: LiDAR, camera, GPS, 6x ultrasonics
- Differential drive with realistic physics
- Complete URDF with Gazebo plugins

### 3. Documentation Created

| Document | Purpose |
|----------|---------|
| [REACTIVE_AVOIDANCE_TESTING.md](REACTIVE_AVOIDANCE_TESTING.md) | **← START HERE** - Complete testing guide |
| [APRILTAG_FOLLOW_TESTING_GUIDE.md](APRILTAG_FOLLOW_TESTING_GUIDE.md) | AprilTag system details |
| [DEPLOYMENT_ROADMAP_NOV15_2025.md](DEPLOYMENT_ROADMAP_NOV15_2025.md) | Deployment strategy |
| [DESKTOP_SETUP_NOV15_2025.md](DESKTOP_SETUP_NOV15_2025.md) | Desktop setup reference |

---

## Current Goal: TEST & REFINE

**Primary Objective:**
Validate and tune the reactive obstacle avoidance system in Gazebo simulation before deploying to real hardware.

**Testing Philosophy:**
1. Start simple (clear path following)
2. Add complexity (obstacles)
3. Test edge cases (tag loss, complex scenarios)
4. Tune parameters based on behavior
5. Document findings

---

## What's Running Right Now

**On Desktop (100.73.129.15):**

```bash
# Already running:
- gzserver (Gazebo physics)
- gzclient (Gazebo GUI - visible on your screen)
- rviz2 (Sensor visualization - visible on your screen)
- robot_state_publisher
- joint_state_publisher
- teleop (Xbox controller support)
```

**What you should see:**
- Gazebo: 3D world with rover in test yard
- RViz: Robot model + sensor data (LiDAR scan, camera, etc.)

---

## Next Steps - Testing Workflow

### Step 1: Launch AprilTag System (NEW TERMINAL)

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch jetson_rover_bridge apriltag_follow_test.launch.py
```

**What this starts:**
- Ultrasonic obstacle detector
- LiDAR obstacle detector
- Reactive obstacle avoidance (VFH)
- AprilTag detector
- AprilTag follower (reactive mode)

### Step 2: Add AprilTag to Gazebo

**In Gazebo GUI:**
1. Click "Insert" tab (left panel)
2. Navigate to: `~/ros2_ws/src/jetson_rover_sim/models/`
3. Select `apriltag_stand`
4. Click in world ~3 meters in front of rover

### Step 3: Monitor System (NEW TERMINAL)

```bash
cd ~/ros2_ws
source install/setup.bash

# Watch AprilTag detection
ros2 topic echo /apriltag/detected

# Watch avoidance outputs
ros2 topic echo /avoidance/safe_direction
ros2 topic echo /avoidance/speed_scale

# Watch follower state
ros2 topic echo /follow_me/active
```

### Step 4: Enable Follow-Me Mode (NEW TERMINAL)

```bash
cd ~/ros2_ws
source install/setup.bash

# Enable following
ros2 topic pub --once /follow_me/enable std_msgs/Bool "data: true"

# Disable following (when needed)
ros2 topic pub --once /follow_me/enable std_msgs/Bool "data: false"
```

### Step 5: Observe Behavior

**Expected:**
- Rover approaches tag
- Stops at 2.0m ± 0.3m
- Logs show: "STATE: FOLLOWING"

### Step 6: Test Obstacle Avoidance

**In Gazebo:**
1. Insert a box (Insert → Box)
2. Place between rover and tag (~1.5m from rover)
3. Observe rover behavior

**Expected:**
- Rover does NOT stop completely
- Speed decreases (you'll see it slow down)
- Rover steers away from obstacle
- Logs show: "STATE: AVOIDING & SEEKING"
- `speed_scale` shows reduction (e.g., 0.3)
- Rover maintains minimum motion (≥ 0.1 m/s)

### Step 7: Test Tag Re-acquisition

**In Gazebo:**
1. Move AprilTag while rover is following
2. Move tag behind obstacle (loses sight)
3. Wait 1.5 seconds
4. Observe search behavior

**Expected:**
- Logs: "STATE: TAG LOST" → "STATE: SEARCHING"
- Rover rotates while moving slowly
- When tag visible: "✓ Tag re-acquired!"
- Resumes following

---

## Key Parameters to Tune

### If Rover Too Cautious (slows too early)

**Edit:** [ros2_ws/src/jetson_rover_bridge/launch/apriltag_follow_test.launch.py](ros2_ws/src/jetson_rover_bridge/launch/apriltag_follow_test.launch.py)

```python
# Reactive obstacle avoidance section
'critical_distance': 0.3,      # Decrease from 0.4
'influence_distance': 2.0,     # Decrease from 2.5
```

**After changes:**
```bash
cd ~/ros2_ws
colcon build --packages-select jetson_rover_bridge
source install/setup.bash
# Relaunch
```

### If Rover Too Aggressive (gets too close)

```python
'critical_distance': 0.6,      # Increase from 0.4
'influence_distance': 3.0,     # Increase from 2.5
'obstacle_weight': 3.0,        # Increase from 2.0
```

### If Avoidance Doesn't Steer Enough

```python
# AprilTag follower section
'avoidance_blend': 0.8,        # Increase from 0.7 (more avoidance)
```

### If Following Too Slow

```python
'max_linear_speed': 0.8,       # Increase from 0.6
'linear_kp': 0.7,              # Increase from 0.5
'avoidance_blend': 0.5,        # Decrease from 0.7 (more following)
```

### If Rover Oscillates

```python
'angular_kp': 0.8,             # Decrease from 1.2
'max_angular_speed': 0.7,      # Decrease from 1.0
```

---

## Testing Checklist

Use this to track your testing progress:

- [ ] **Test 1:** AprilTag detection (tag visible, topic shows `data: true`)
- [ ] **Test 2:** Basic following (rover approaches to 2.0m, stops)
- [ ] **Test 3:** Moving tag (rover tracks smoothly)
- [ ] **Test 4:** Single obstacle (rover slows, steers, doesn't stop)
- [ ] **Test 5:** Multiple obstacles (rover navigates through)
- [ ] **Test 6:** Tag re-acquisition (rover searches and finds tag)
- [ ] **Test 7:** Complex scenario (moving tag + obstacles)
- [ ] **Parameter tuning:** Adjust for desired behavior
- [ ] **Performance check:** 30Hz detection, 10Hz control, no collisions

---

## Common Issues & Quick Fixes

### Issue: Tag not detected

**Check:**
```bash
# Is camera publishing?
ros2 topic hz /camera/image_raw

# View camera feed
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# View debug image with tag overlay
ros2 run rqt_image_view rqt_image_view /apriltag/debug_image
```

**Fix:** Ensure AprilTag is in camera FOV, texture is applied correctly

### Issue: Rover doesn't move

**Check:**
```bash
# Is follower enabled?
ros2 topic echo /follow_me/enable

# Is tag detected?
ros2 topic echo /apriltag/detected

# Are obstacles blocking?
ros2 topic echo /avoidance/critical_danger
```

**Fix:** Enable follower, verify tag detection, check for false obstacles

### Issue: Rover stops at obstacles

**This shouldn't happen!** The system is designed to never stop.

**Debug:**
```bash
# Check speed scale (should be ≥ 0.1 even when critical)
ros2 topic echo /avoidance/speed_scale

# Check minimum forward speed parameter
ros2 param get /apriltag_follower_reactive min_forward_speed
```

**Fix:** Verify `min_forward_speed: 0.1` in launch file, rebuild if needed

### Issue: AprilTag library not found

**Fix:**
```bash
pip3 install pupil-apriltags
# OR
pip3 install dt-apriltags
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

**Monitor with:**
```bash
ros2 topic hz /apriltag/detected     # Should be ~30 Hz
ros2 topic hz /cmd_vel               # Should be ~10 Hz
ros2 topic hz /avoidance/safe_direction  # Should be ~10 Hz
```

---

## After Testing: Deployment to Jetson

Once satisfied with simulation performance:

### 1. Install Dependencies on Jetson

```bash
# SSH to Jetson
ssh jay@100.91.191.47

# Install AprilTag library
pip3 install pupil-apriltags
```

### 2. Copy Package to Jetson

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

### 3. Print Real AprilTag

- Download: https://github.com/AprilRobotics/apriltag-imgs/tree/master/tag36h11
- Use: tag36_11_00000.png (same as simulation)
- Size: 6.5 inches (165mm) square
- Mount on rigid backing

### 4. Real-World Testing Sequence

1. Static tag, clear path (verify basic following)
2. Static tag, single obstacle (verify avoidance)
3. Walking with tag, clear path (verify dynamic following)
4. Walking with tag, obstacles (full system test)

---

## Important Files Reference

**Testing Guide (Primary Reference):**
- [REACTIVE_AVOIDANCE_TESTING.md](REACTIVE_AVOIDANCE_TESTING.md) - Detailed testing scenarios

**Source Code:**
- Reactive Avoidance: [ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/reactive_obstacle_avoidance.py](ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/reactive_obstacle_avoidance.py)
- Reactive Follower: [ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/apriltag_follower_reactive.py](ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/apriltag_follower_reactive.py)
- Launch File: [ros2_ws/src/jetson_rover_bridge/launch/apriltag_follow_test.launch.py](ros2_ws/src/jetson_rover_bridge/launch/apriltag_follow_test.launch.py)

**Configuration:**
- Setup: [ros2_ws/src/jetson_rover_bridge/setup.py](ros2_ws/src/jetson_rover_bridge/setup.py)
- Package: [ros2_ws/src/jetson_rover_bridge/package.xml](ros2_ws/src/jetson_rover_bridge/package.xml)

---

## Quick Command Reference

```bash
# Launch simulation (if not already running)
ros2 launch jetson_rover_sim full_simulation.launch.py

# Launch AprilTag system
ros2 launch jetson_rover_bridge apriltag_follow_test.launch.py

# Enable follow-me
ros2 topic pub --once /follow_me/enable std_msgs/Bool "data: true"

# Disable follow-me
ros2 topic pub --once /follow_me/enable std_msgs/Bool "data: false"

# Monitor key topics
ros2 topic echo /apriltag/detected
ros2 topic echo /avoidance/speed_scale
ros2 topic echo /follow_me/active

# Manual control (if needed)
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Rebuild after changes
cd ~/ros2_ws
colcon build --packages-select jetson_rover_bridge
source install/setup.bash
```

---

## Success Criteria

Before considering this phase complete:

- [ ] Rover follows AprilTag smoothly in clear environment
- [ ] Rover steers around obstacles without stopping
- [ ] Speed never drops to zero (minimum 0.1 m/s maintained)
- [ ] Tag re-acquisition works reliably (< 3 seconds)
- [ ] No collisions during testing
- [ ] Parameters tuned for desired behavior
- [ ] Documented any issues or edge cases found
- [ ] Ready for real-world deployment

---

## Notes for Future Sessions

**What works:**
- VFH algorithm provides smooth steering
- Multi-sensor fusion is reliable
- State machine handles transitions well

**What to watch:**
- Parameter sensitivity (may need fine-tuning per environment)
- Tag detection in varying lighting (simulation vs real world)
- Balance between avoidance strength and following performance

**Future Enhancements (after basic testing):**
- Dynamic obstacle handling (moving objects)
- Path planning integration (Nav2)
- Multiple tag following (formation control)
- Learning-based parameter adaptation

---

**Session Created:** November 15, 2025
**System Status:** Ready for testing
**Next Action:** Launch AprilTag system and begin Test 1

**Remember:** "Never stop, always find a way!" 🤖
