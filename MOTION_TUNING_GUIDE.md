# Motion Tuning with Xbox Controller

**Date:** November 15, 2025
**Goal:** Tune simulation motion to match real robot "feel" using Xbox controller

---

## Overview

We've added a **Motion Smoother** that applies realistic acceleration/deceleration to the robot. This prevents instant speed changes and mimics real-world motor physics.

```
Xbox Controller → /cmd_vel → Motion Smoother → /cmd_vel_smooth → Gazebo Robot
```

The Motion Smoother applies:
- **Acceleration limits** - How fast the robot speeds up
- **Deceleration limits** - How fast the robot slows down
- **Smooth ramping** - Gradual velocity changes instead of instant jumps

---

## Quick Start

### Terminal 1: Launch Simulation
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch jetson_rover_sim full_simulation.launch.py
```

### Terminal 2: Launch Motion Smoother
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run jetson_rover_bridge motion_smoother
```

### Terminal 3: Launch Xbox Controller
```bash
cd "~/Git Sandbox/Jetson Cube Orange Outdoor Rover"
./run_xbox_controller.sh
```

---

## How to Tune

###  1. Drive with Xbox Controller

**Controls:**
- **Left Stick Up/Down** - Forward/Backward (max 0.7 m/s, turbo: 1.5 m/s)
- **Left Stick Left/Right** - Turn (max 0.8 rad/s, turbo: 1.2 rad/s)
- **Right Bumper (hold)** - Turbo mode

### 2. Observe Motion Behavior

**Test these scenarios:**

1. **Forward acceleration** - Push stick forward, release
   - Does it speed up too fast or too slow?
   - Does it feel jerky or smooth?

2. **Braking** - Push stick forward, then release
   - Does it stop too quickly or coast too long?
   - Real robot should slow down fairly quickly

3. **Turning** - Move stick left/right
   - Does rotation ramp up smoothly?
   - Does it feel responsive?

4. **Direction changes** - Forward → Backward quickly
   - Should slow down first, then accelerate opposite direction
   - Should NOT instantly reverse!

### 3. Adjust Parameters

Edit [motion_smoother.py](ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/motion_smoother.py) and adjust these values:

```python
# Linear acceleration (m/s²)
self.declare_parameter('max_linear_accel', 1.0)  # ← Increase for faster speedup
self.declare_parameter('max_linear_decel', 2.0)  # ← Increase for faster braking

# Angular acceleration (rad/s²)
self.declare_parameter('max_angular_accel', 2.0)  # ← Increase for snappier turning
self.declare_parameter('max_angular_decel', 3.0)  # ← Increase for faster turn stop
```

**After editing:**
```bash
# Rebuild
cd ~/ros2_ws
colcon build --packages-select jetson_rover_bridge

# Restart motion smoother (Terminal 2)
# Press Ctrl+C, then:
source install/setup.bash
ros2 run jetson_rover_bridge motion_smoother
```

**Test again with Xbox controller!**

---

## Tuning Guidelines

### For Real Robot Match

Based on your real robot (200 lbs, 4WD skid-steer, wheelchair motors):

**Conservative (Safe):**
```python
max_linear_accel = 0.5   # Gentle acceleration
max_linear_decel = 1.0   # Moderate braking
max_angular_accel = 1.0  # Smooth turning
max_angular_decel = 2.0  # Controlled stop
```

**Moderate (Recommended Starting Point):**
```python
max_linear_accel = 1.0   # ← CURRENT
max_linear_decel = 2.0   # ← CURRENT
max_angular_accel = 2.0  # ← CURRENT
max_angular_decel = 3.0  # ← CURRENT
```

**Aggressive (Quick response):**
```python
max_linear_accel = 2.0   # Fast speedup
max_linear_decel = 4.0   # Quick braking
max_angular_accel = 3.0  # Snappy turning
max_angular_decel = 4.0  # Fast turn stop
```

### What "Feels Right"?

**Good indicators:**
- ✅ Smooth, predictable motion
- ✅ No sudden jerks or jumps
- ✅ Responds quickly but not instantaneously
- ✅ Slows down naturally when you release stick
- ✅ Can make fine adjustments easily

**Bad indicators:**
- ❌ Robot jumps forward instantly
- ❌ Stops dead immediately (unrealistic)
- ❌ Sluggish, takes forever to respond
- ❌ Overshoots when trying to position precisely

---

## Advanced: Real-Time Parameter Tuning

You can change parameters WITHOUT rebuilding using ROS2 params:

```bash
# Increase linear acceleration to 1.5 m/s²
ros2 param set /motion_smoother max_linear_accel 1.5

# Increase braking to 3.0 m/s²
ros2 param set /motion_smoother max_linear_decel 3.0

# Snappier rotation
ros2 param set /motion_smoother max_angular_accel 2.5
```

**Test immediately with Xbox controller!**

View current parameters:
```bash
ros2 param list /motion_smoother
ros2 param get /motion_smoother max_linear_accel
```

---

## Monitoring

### Watch Velocity Commands

**Terminal 4 (optional):**
```bash
cd ~/ros2_ws
source install/setup.bash

# See raw controller commands
ros2 topic echo /cmd_vel

# See smoothed output to robot
ros2 topic echo /cmd_vel_smooth
```

**Compare them side-by-side:**
- `/cmd_vel` - Instant jumps (from controller)
- `/cmd_vel_smooth` - Gradual ramping (to robot)

### Plot Velocities (Advanced)

```bash
ros2 run rqt_plot rqt_plot /cmd_vel/linear/x /cmd_vel_smooth/linear/x
```

You'll see the smoothing effect visually!

---

## Once You Find the Right Feel

### 1. Record Your Values

Write down the acceleration values that feel right:

```
Linear Accel:  _______  m/s²
Linear Decel:  _______  m/s²
Angular Accel: _______  m/s²
Angular Decel: _______  m/s²
```

### 2. Make Them Permanent

Edit [motion_smoother.py:21-28](ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/motion_smoother.py#L21-L28) with your values.

### 3. Apply to Autonomous Behaviors

The same motion smoother works for:
- ✅ Xbox controller (manual driving)
- ✅ Wander behavior
- ✅ Exploration behavior
- ✅ Any code publishing to `/cmd_vel`

Everything will have the same smooth, realistic motion!

---

## Integration with Autonomous Behaviors

When running autonomous behaviors, the motion smoother automatically applies:

```bash
# Terminal 1: Simulation
ros2 launch jetson_rover_sim full_simulation.launch.py

# Terminal 2: Motion Smoother
ros2 run jetson_rover_bridge motion_smoother

# Terminal 3: Autonomous behavior (picks one)
ros2 run jetson_rover_bridge wander_behavior
# OR
ros2 run jetson_rover_bridge exploration_behavior
```

The autonomous behavior sends commands to `/cmd_vel`, motion smoother applies acceleration limits, and robot moves smoothly!

---

## Troubleshooting

### Robot doesn't move
- Check motion smoother is running
- Verify `/cmd_vel_smooth` topic exists: `ros2 topic list | grep cmd_vel`
- Check Gazebo is subscribed: `ros2 topic info /cmd_vel_smooth`

### Robot moves but no smoothing
- Motion smoother might not be running
- Check which topic Gazebo listens to: should be `/cmd_vel_smooth`

### Too sluggish
- Increase `max_linear_accel` and `max_angular_accel`
- Try doubling the values

### Too jerky
- Decrease `max_linear_accel` and `max_angular_accel`
- Try halving the values

### Robot overshoots/coasts
- Increase `max_linear_decel` and `max_angular_decel`
- Higher decel = faster stopping

---

## Next Steps

Once motion feels right:

1. **Save your parameters** - Update motion_smoother.py with final values
2. **Test with autonomous behaviors** - Does exploration feel smooth?
3. **Deploy to real robot** - Same motion smoother code works on Jetson!

---

## Real Robot Deployment

When ready for real hardware, the motion smoother transfers directly:

```bash
# On Jetson Orin Nano
cd ~/ros2_ws
source install/setup.bash
ros2 run jetson_rover_bridge motion_smoother

# In another terminal - autonomous behavior
ros2 run jetson_rover_bridge exploration_behavior
```

The acceleration/deceleration values you tuned in simulation will apply to the real robot!

---

**Status:** Ready for tuning!
**Your mission:** Find the acceleration values that match your real robot's feel! 🎮🤖
