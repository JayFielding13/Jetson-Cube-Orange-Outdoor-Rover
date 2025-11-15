# Session Log - November 8, 2025
## Rover Simulation Ground Control Integration & Physics Tuning

### Session Overview
Integrated Mobile RTK Control Module with Gazebo simulation to enable ground control testing without requiring physical hardware. Created HTTP/ROS2 bridge system, interactive waypoint map, and tuned physics parameters for realistic behavior.

---

## 1. Dual Monitor Setup

### Issue
User wanted to extend desktop to second monitor for efficient workflow (Gazebo on external monitor, VS Code on laptop screen).

### Resolution
- Checked monitor configuration with `xrandr`
- Initially HDMI showed disconnected - user resolved with cable replacement
- Successfully extended desktop to dual monitor setup

---

## 2. Ground Control Integration with Simulation

### Objective
Enable Mobile RTK Control Module to communicate with simulated rover using same HTTP REST API as real Jetson Orin Nano hardware.

### Architecture Created

#### New ROS2 Package: `jetson_rover_bridge`
Location: `/home/jay/Desktop/Mini Rover Development/ros2_ws/src/jetson_rover_bridge/`

**Components:**

1. **HTTP Bridge** (`http_bridge.py`)
   - Flask HTTP server on port 5001
   - Translates REST API calls to ROS2 topics
   - Endpoints:
     - `GET /api/health` - Health check
     - `GET /api/status` - Robot status (GPS, armed state, position)
     - `POST /api/arm` - ARM motors
     - `POST /api/disarm` - DISARM motors
     - `POST /api/target` - Send GPS waypoint
     - `POST /api/stop` - Emergency stop
     - `POST /api/pause` - Pause movement
     - `POST /api/cancel` - Cancel mission
   - Publishers:
     - `/cmd_vel` (Twist) - Velocity commands
     - `/waypoint/goto` (PoseStamped) - Waypoint goals
     - `/rover/armed` (Bool) - Arm state
   - Subscribers:
     - `/gps/fix` (NavSatFix) - GPS position
     - `/odom` (Odometry) - Odometry data

2. **GPS Bridge** (`gps_bridge.py`)
   - Converts Gazebo PoseStamped format to NavSatFix format
   - Coordinate conversion: Local XY → GPS lat/lon
   - Reference origin: San Francisco (37.7749°N, 122.4194°W)
   - Publishes to: `/gps/fix`
   - Subscribes to: `/gps/pose`

3. **Interactive Map** (`interactive_map.py`)
   - Tkinter-based top-down map interface
   - Visual waypoint selection (click-to-send)
   - Real-time rover position display
   - Features:
     - 800x600 pixel canvas
     - 20 pixels per meter scale
     - Grid with 5m spacing
     - Rover shown as orange triangle with heading
     - Waypoints marked as red dots
     - Automatic coordinate conversion
   - Sends waypoints via HTTP API to localhost:5001

4. **Launch File** (`ground_control_bridge.launch.py`)
   - Launches GPS bridge and HTTP bridge together
   - Single command deployment

### Files Created/Modified

**Created:**
- `jetson_rover_bridge/jetson_rover_bridge/http_bridge.py`
- `jetson_rover_bridge/jetson_rover_bridge/gps_bridge.py`
- `jetson_rover_bridge/jetson_rover_bridge/interactive_map.py`
- `jetson_rover_bridge/launch/ground_control_bridge.launch.py`
- `jetson_rover_bridge/README.md`
- `jetson_rover_bridge/package.xml`
- `jetson_rover_bridge/resource/jetson_rover_bridge`

**Modified:**
- `jetson_rover_bridge/setup.py` - Added three console script entry points

### Launch Commands

```bash
# Terminal 1: Simulation
cd ~/Desktop/Mini\ Rover\ Development/ros2_ws
source install/setup.bash
ros2 launch jetson_rover_sim visualize_rover.launch.py world:=test_yard

# Terminal 2: Ground Control Bridge
cd ~/Desktop/Mini\ Rover\ Development/ros2_ws
source install/setup.bash
ros2 launch jetson_rover_bridge ground_control_bridge.launch.py

# Terminal 3: Interactive Map (optional)
cd ~/Desktop/Mini\ Rover\ Development/ros2_ws
source install/setup.bash
ros2 run jetson_rover_bridge interactive_map

# Terminal 4: Ground Control Dashboard (optional)
cd ~/Desktop/Mini\ Rover\ Development/Mobile\ RTK\ Control\ Module
python3 dashboard_sim.py
```

---

## 3. Mobile RTK Control Module Simulation Branch

### Issue
Original dashboard crashed with segmentation fault due to GPS hardware dependencies.

### Resolution
- Created `simulation-testing` branch in Mobile RTK Control Module repository
- Created `dashboard_sim.py` - simplified dashboard without hardware GPS
- Gets GPS data from robot HTTP API instead of hardware
- Dashboard connects to localhost:5001

**Branch Created:**
```bash
cd "Mobile RTK Control Module"
git checkout -b simulation-testing
```

**New File:**
- `Mobile RTK Control Module/dashboard_sim.py`

---

## 4. Robot Model Updates

### A. Wheel Position Correction

**Issue:** Rear wheels had center point at back edge of chassis, but needed rear edge of tire tangent to back edge.

**Solution:** Moved wheels forward by one wheel radius (0.127m)

**File Modified:** `ros2_ws/src/jetson_rover_sim/urdf/jetson_rover.urdf.xacro`

**Changes (Lines 35-36):**
```xml
<!-- OLD: Wheel center at back edge -->
<xacro:property name="rear_wheel_x_offset" value="${-chassis_length/2}"/>

<!-- NEW: Rear edge of tire at back edge -->
<xacro:property name="rear_wheel_x_offset" value="${-chassis_length/2 + wheel_radius}"/>
<xacro:property name="front_wheel_x_offset" value="${-chassis_length/2 + wheel_radius + wheelbase}"/>
```

**Result:**
- Rear wheel center: -0.2159m (was -0.3429m)
- Front wheel center: 0.2159m (maintaining 17" wheelbase)
- Rear tire edge now flush with chassis back edge

### B. Friction and Physics Improvements

**Issue:** Rover sliding forward without commands due to insufficient friction.

**Solution:** Improved wheel friction and damping to match real car tire behavior.

**File Modified:** `ros2_ws/src/jetson_rover_sim/urdf/jetson_rover.urdf.xacro`

**Changes:**

1. **Joint Dynamics (Line 94):**
```xml
<dynamics damping="5.0" friction="1.0"/>
```
- Prevents free-wheeling when no torque applied
- Simulates bearing resistance and brake drag

2. **Gazebo Wheel Friction (Lines 251-285, all 4 wheels):**
```xml
<gazebo reference="front_left_wheel">
  <material>Gazebo/Black</material>
  <mu1>1.5</mu1>              <!-- Increased from 1.0 -->
  <mu2>1.5</mu2>              <!-- Increased from 1.0 -->
  <kp>10000000.0</kp>         <!-- Increased from 1000000.0 -->
  <kd>1.0</kd>                <!-- Decreased from 100.0 -->
  <min_depth>0.001</min_depth> <!-- Added -->
</gazebo>
```

**Parameter Explanations:**
- `mu1` / `mu2`: Friction coefficients (1.5 = rubber on concrete)
- `kp`: Contact stiffness (higher = less wheel sinking)
- `kd`: Contact damping (optimized for stability)
- `min_depth`: Minimum penetration before contact forces applied

**Result:**
- Rover now stationary when no commands sent
- Realistic tire grip matching car tires
- No unwanted sliding or drift

**Rebuild Command:**
```bash
cd ~/Desktop/Mini\ Rover\ Development/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jetson_rover_sim --symlink-install
```

---

## 5. Integration Testing

### Test Script Created
File: `/home/jay/Desktop/Mini Rover Development/test_rover_integration.sh`

### Test Results
```
✓ HTTP Bridge health check: PASSED
✓ Robot status retrieval: PASSED
✓ GPS data flow: PASSED (lat: 37.7749°, lon: -122.4194°)
✓ ARM/DISARM commands: PASSED
✓ Waypoint commands: PASSED
✓ All ROS2 topics active: PASSED
```

**Active Topics Verified:**
- `/cmd_vel` - Velocity commands
- `/gps/fix` - GPS position
- `/odom` - Odometry

---

## System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                   Ground Control Layer                       │
├─────────────────────────────────────────────────────────────┤
│  Mobile RTK Control Module (dashboard_sim.py)               │
│  - GUI: GPS display, ARM/DISARM, waypoint entry             │
│  - Connects via HTTP to localhost:5001                      │
│                                                              │
│  Interactive Map (interactive_map.py)                        │
│  - Top-down view with click-to-send waypoints               │
│  - Sends HTTP POST to localhost:5001/api/target             │
└─────────────────────────────────────────────────────────────┘
                            ↕ HTTP REST API
┌─────────────────────────────────────────────────────────────┐
│                   Bridge Layer (ROS2)                        │
├─────────────────────────────────────────────────────────────┤
│  HTTP Bridge (http_bridge.py)                               │
│  - Flask server on port 5001                                │
│  - Translates HTTP → ROS2 topics                            │
│                                                              │
│  GPS Bridge (gps_bridge.py)                                 │
│  - Converts PoseStamped → NavSatFix                         │
│  - XY → GPS coordinate conversion                           │
└─────────────────────────────────────────────────────────────┘
                            ↕ ROS2 Topics
┌─────────────────────────────────────────────────────────────┐
│              Simulation Layer (Gazebo + RViz)                │
├─────────────────────────────────────────────────────────────┤
│  Gazebo Classic                                             │
│  - Physics simulation (improved friction)                   │
│  - Sensor simulation (GPS, LiDAR, Camera, Sonar)            │
│  - 4WD differential drive control                           │
│                                                              │
│  RViz2                                                       │
│  - 3D visualization                                          │
│  - Sensor data display                                       │
│  - TF tree visualization                                     │
└─────────────────────────────────────────────────────────────┘
```

---

## Key Technical Details

### GPS Coordinate Conversion
- **Origin:** 37.7749°N, 122.4194°W (San Francisco)
- **Meters per degree latitude:** 111,320 m
- **Meters per degree longitude:** 111,320 × cos(latitude) m
- **Formula:**
  ```python
  latitude = origin_lat + (y_meters / meters_per_degree_lat)
  longitude = origin_lon + (x_meters / meters_per_degree_lon)
  ```

### Robot Physical Properties
- **Chassis:** 27" × 23.75" × 12" (0.686m × 0.603m × 0.305m)
- **Mass:** 200 lbs (90.7 kg)
- **Wheelbase:** 17" (0.432m)
- **Track width:** 23.75" (0.603m)
- **Wheel diameter:** 10" (0.254m)
- **Ground clearance:** 3" (0.076m)
- **Wheel mass:** 10 lbs each (4.5 kg)

### Friction Parameters
- **Joint damping:** 5.0 N⋅m⋅s/rad
- **Joint friction:** 1.0 N⋅m
- **Surface friction μ1/μ2:** 1.5 (rubber on concrete)
- **Contact stiffness:** 10,000,000 N/m
- **Contact damping:** 1.0 N⋅s/m

---

## Files Modified This Session

1. **Created Package:**
   - `ros2_ws/src/jetson_rover_bridge/` (entire package)

2. **Modified URDF:**
   - `ros2_ws/src/jetson_rover_sim/urdf/jetson_rover.urdf.xacro`
     - Lines 35-36: Wheel positions
     - Line 94: Joint dynamics
     - Lines 251-285: Gazebo friction parameters

3. **Created Dashboard:**
   - `Mobile RTK Control Module/dashboard_sim.py`

4. **Created Test Script:**
   - `test_rover_integration.sh`

---

## Commands Reference

### Build Commands
```bash
# Build specific package
cd ~/Desktop/Mini\ Rover\ Development/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jetson_rover_sim --symlink-install
colcon build --packages-select jetson_rover_bridge --symlink-install

# Build all packages
colcon build --symlink-install
```

### Launch Commands
```bash
# Simulation only
ros2 launch jetson_rover_sim visualize_rover.launch.py world:=test_yard

# Ground control bridge
ros2 launch jetson_rover_bridge ground_control_bridge.launch.py

# Interactive map
ros2 run jetson_rover_bridge interactive_map

# Full system (requires 3-4 terminals)
# See "Launch Commands" section above
```

### Testing Commands
```bash
# Health check
curl http://localhost:5001/api/health

# Get status
curl http://localhost:5001/api/status

# ARM motors
curl -X POST http://localhost:5001/api/arm

# Send waypoint
curl -X POST http://localhost:5001/api/target \
  -H "Content-Type: application/json" \
  -d '{"latitude": 37.775, "longitude": -122.419}'

# DISARM motors
curl -X POST http://localhost:5001/api/disarm

# Run integration test
bash test_rover_integration.sh
```

### ROS2 Debug Commands
```bash
# List topics
ros2 topic list

# Monitor GPS data
ros2 topic echo /gps/fix

# Monitor odometry
ros2 topic echo /odom

# Send manual velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

---

## Outstanding Issues
None - all requested features implemented and tested successfully.

---

## Next Steps / Future Enhancements

### Potential Improvements
1. **Autonomous Navigation**
   - Integrate Nav2 for path planning
   - Obstacle avoidance using LiDAR data
   - Waypoint queue management

2. **Advanced Control**
   - PID tuning for precise movement
   - GPS waypoint following controller
   - Heading hold mode

3. **Simulation Enhancements**
   - Add terrain variations (slopes, rough ground)
   - Weather effects (rain, fog)
   - Multiple robot scenarios

4. **Ground Control Features**
   - Mission planning interface
   - Real-time telemetry graphs
   - Video feed integration
   - Battery monitoring simulation

5. **Testing Automation**
   - Automated test suite for waypoint navigation
   - Performance benchmarking
   - Regression testing scripts

---

## Session Success Metrics

✅ **Completed:**
- Ground control HTTP bridge fully functional
- GPS coordinate conversion working
- Interactive waypoint map operational
- Simulation branch created for Mobile RTK Control Module
- Physics tuning (friction/damping) successful
- Rover stationary when idle (no unwanted sliding)
- All integration tests passing
- Comprehensive documentation created

---

## Git Status at End of Session

**Modified files:**
- `ros2_ws/src/jetson_rover_sim/urdf/jetson_rover.urdf.xacro`
- `ros2_ws/src/jetson_rover_bridge/setup.py`

**New packages:**
- `ros2_ws/src/jetson_rover_bridge/` (complete package)

**New branch:**
- Mobile RTK Control Module: `simulation-testing`

**Recommended commit:**
```bash
cd ~/Desktop/Mini\ Rover\ Development
git add .
git commit -m "feat: Add ground control integration and improve physics

- Create jetson_rover_bridge package for HTTP/ROS2 communication
- Add GPS bridge for coordinate conversion
- Add interactive map for visual waypoint selection
- Fix wheel positioning (rear tire flush with chassis)
- Improve wheel friction and damping (mu=1.5, kp=10M, damping=5.0)
- Create simulation-friendly ground control dashboard
- Add comprehensive integration test script

Rover now integrates with Mobile RTK Control Module via HTTP API,
enabling full ground control testing in simulation."
```

---

## Notes
- All systems tested and working as expected
- Rover physics behavior matches real-world expectations
- Ground control integration seamless - identical API to real hardware
- Documentation comprehensive and ready for future reference

**Session Duration:** ~2 hours
**Date:** November 8, 2025
**Status:** ✅ Complete and Successful
