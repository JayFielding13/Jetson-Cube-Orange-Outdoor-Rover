# Quick Test: Obstacle Avoidance

## Ready to Test! 🚀

Your obstacle avoidance controller is ready. Here's how to test it:

---

## Step 1: Verify Simulation is Running

On your desktop, you should see the Gazebo window with the rover.

If not visible, restart it:
```bash
ssh jay@192.168.254.209
source ~/ros2_distributed_setup.sh
cd ~/ros2_ws
source install/setup.bash
ros2 launch jetson_rover_sim spawn_rover.launch.py
```

---

## Step 2: Add Some Obstacles (Optional)

In the Gazebo window on your desktop:

1. Click the **"Insert"** tab (left sidebar)
2. Click **"Box"** under "Simple Shapes"
3. Click in the world to place a box in front of the rover
4. Repeat to add a few more obstacles

Or just use the walls/objects already in the world!

---

## Step 3: Run the Obstacle Avoidance

**On your Jetson, run:**
```bash
cd ~/Desktop/Mini\ Rover\ Development/Jetson\ Cube\ Orange\ Outdoor\ Rover/
./run_obstacle_avoidance.sh
```

---

## Step 4: Watch the Magic! ✨

**You should see:**

**In the terminal (Jetson):**
```
Obstacle Avoidance Controller Started!
  Max Speed: 0.5 m/s
  Stop Distance: 1.5 m
  Safe Distance: 2.5 m
  Turn Speed: 0.3 rad/s

✓ CLEAR - Moving forward. Closest obstacle: 8.45m
✓ CLEAR - Moving forward. Closest obstacle: 7.32m
⚡ SLOW & TURN LEFT - Obstacle at 2.1m
⚠️  STOP & TURN LEFT - Obstacle at 1.2m
⚠️  STOP & TURN LEFT - Obstacle at 0.9m
✓ CLEAR - Moving forward. Closest obstacle: 5.67m
```

**On desktop (Gazebo):**
- Rover moves forward
- Slows down as it approaches obstacles
- Stops and turns when too close
- Continues navigating autonomously

---

## Controls

**Stop the rover:**
- Press `Ctrl+C` in the terminal
- Rover will send a stop command and shut down cleanly

**Restart:**
- Just run `./run_obstacle_avoidance.sh` again

**Manual control (in another terminal):**
```bash
source ros2_distributed_setup.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Testing Ideas

### Test 1: Simple Wall
Place the rover facing a wall. It should approach, stop at 1.5m, and turn away.

### Test 2: Corridor
Place boxes on left and right to create a corridor. Rover should navigate through.

### Test 3: Maze
Create a simple maze with boxes. Watch the rover find its way through!

### Test 4: Moving Obstacles
In Gazebo, you can drag objects around. Try moving an obstacle into the rover's path!

---

## Adjusting Behavior

Want the rover to be more/less aggressive? Edit [simple_obstacle_avoidance.py](simple_obstacle_avoidance.py):

**More Cautious:**
```python
self.declare_parameter('max_speed', 0.3)      # Slower
self.declare_parameter('min_distance', 2.0)   # Stops farther
self.declare_parameter('safe_distance', 3.5)  # Worries more
```

**More Aggressive:**
```python
self.declare_parameter('max_speed', 0.8)      # Faster
self.declare_parameter('min_distance', 1.0)   # Stops closer
self.declare_parameter('safe_distance', 2.0)  # Worries less
```

Then restart the controller.

---

## Troubleshooting

### "No scan data yet"
- Simulation not running on desktop
- Network not configured: Run `source ros2_distributed_setup.sh` first

### Rover doesn't move
- Check Gazebo is running: Look at desktop screen
- Check topics: `ros2 topic list | grep scan`

### Python error
Check the error message in terminal. Most common:
- Import error → Install missing package: `pip3 install <package>`
- Permission error → Script not executable: `chmod +x run_obstacle_avoidance.sh`

---

## What's Next?

Once obstacle avoidance works well:

1. **Add GPS waypoint navigation** - Make rover go to specific locations
2. **Integrate ultrasonics** - Use all 6 ultrasonic sensors for close-range safety
3. **Add recovery behaviors** - Back up and try another path if stuck
4. **Implement geofence** - Stop at boundary
5. **Deploy to real robot!** - Same code works on physical hardware

---

## Files Reference

- **[simple_obstacle_avoidance.py](simple_obstacle_avoidance.py)** - The controller code
- **[run_obstacle_avoidance.sh](run_obstacle_avoidance.sh)** - Quick launch script
- **[OBSTACLE_AVOIDANCE_GUIDE.md](OBSTACLE_AVOIDANCE_GUIDE.md)** - Detailed guide

---

**Ready? Run `./run_obstacle_avoidance.sh` and watch your rover navigate autonomously!** 🤖
