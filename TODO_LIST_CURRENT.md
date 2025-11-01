# Current TODO List - Mobile RTK Rover Project
**Last Updated**: October 31, 2025 - End of Day Session

---

## ✅ COMPLETED TODAY (October 31, 2025)

### Morning Session
- [x] Install ROS 2 Humble on Jetson Orin Nano
- [x] Install and configure RPLidar A1 ROS 2 driver
- [x] Install and configure USB camera ROS 2 driver
- [x] Install MAVROS2 packages
- [x] Create ROS 2 HTTP sensor bridge
- [x] Add LiDAR View and Camera View tabs to Mobile RTK Module

### Afternoon/Evening Session
- [x] Complete sensor visualization integration
- [x] Enhance LiDAR display with full-screen layout and zoom controls
- [x] Redesign Camera Feed tab to match LiDAR styling
- [x] Add connection progress dialog with threading
- [x] Configure Jetson systemd auto-start services (3 services)
- [x] Set up passwordless SSH to Jetson
- [x] Resolve LiDAR stability issues (systemd service conflicts)
- [x] Verify all services working after reboot

---

## IMMEDIATE PRIORITY (Next Session)

### 1. Complete Rover GPS Visualization (from Oct 30)
**Status**: Backend complete, UI pending
**Estimated Time**: 40 minutes

**Tasks**:
- [ ] Add rover GPS section to GPS Status tab (2-row layout)
- [ ] Draw blue rover triangle on Dashboard map
- [ ] Draw rover icon on Follow-Me map
- [ ] Draw rover icon on Survey Mode map
- [ ] Test rover position updates (every 5 seconds)

**Files to Modify**:
- `Mobile RTK Control Module/09_dashboard_enhanced.py`

**Reference**:
- See `Mobile RTK Control Module/NEXT_SESSION_TODO.md` from Oct 30 for detailed implementation steps

---

## HIGH PRIORITY

### 2. Camera Re-Integration
**Status**: Camera unplugged, ready to reconnect
**Estimated Time**: 15 minutes

**Tasks**:
- [ ] Plug camera back into Jetson
- [ ] Verify `/dev/video0` appears
- [ ] Restart jetson-rplidar service to detect camera
- [ ] Test Camera View tab displays video feed
- [ ] Verify ~7 FPS update rate

**Commands**:
```bash
ssh jay@192.168.254.100
ls -la /dev/video*  # Verify camera detected
sudo systemctl restart jetson-rplidar  # Will launch camera node
```

---

### 3. Configure MAVROS2 for Cube Orange
**Status**: MAVROS2 installed, not configured
**Estimated Time**: 2-3 hours

**Tasks**:
- [ ] Create MAVROS2 launch file for Cube Orange
- [ ] Configure connection parameters (USB: /dev/ttyACM0, baud: 57600)
- [ ] Launch MAVROS2 and verify connection
- [ ] Subscribe to MAVLink topics:
  - `/mavros/global_position/global` - GPS position
  - `/mavros/battery` - Battery status
  - `/mavros/state` - System state (armed, mode, etc.)
  - `/mavros/imu/data` - IMU data
- [ ] Test MAVLink commands (ARM, DISARM, set mode)
- [ ] Verify two-way communication

**Commands**:
```bash
# On Jetson
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:57600
ros2 topic list | grep mavros
ros2 topic echo /mavros/state
```

---

### 4. Integrate Rover GPS from MAVROS2
**Status**: Currently using HTTP polling from old server
**Estimated Time**: 2 hours

**Tasks**:
- [ ] Update `ros2_sensor_bridge.py` to subscribe to `/mavros/global_position/global`
- [ ] Add `/api/rover_gps` endpoint to HTTP bridge
- [ ] Modify Mobile RTK Module to poll new endpoint
- [ ] Remove dependency on old jetson_rover_server.py for GPS
- [ ] Update all three map tabs to use new GPS source
- [ ] Test rover position display accuracy

**Benefits**:
- Direct access to flight controller GPS
- Lower latency
- More reliable data source

---

## MEDIUM PRIORITY

### 5. Obstacle Visualization on Dashboard Map
**Status**: Not started
**Estimated Time**: 3-4 hours

**Tasks**:
- [ ] Parse LiDAR data to detect obstacles
- [ ] Convert polar coordinates to GPS coordinates
- [ ] Draw obstacles on Dashboard map (red dots/circles)
- [ ] Add danger zones (red overlay for obstacles <1m)
- [ ] Display nearest obstacle distance on map
- [ ] Add toggle to show/hide obstacle overlay

**Visual Design**:
- Red circles for obstacles <1m (danger)
- Orange circles for 1-2m (warning)
- Yellow circles for 2-5m (caution)

---

### 6. Unified Server Consolidation
**Status**: Currently running two servers
**Estimated Time**: 2-3 hours

**Tasks**:
- [ ] Merge jetson_rover_server.py (port 5000) and ros2_sensor_bridge.py (port 5001)
- [ ] Create single unified server on port 5000
- [ ] Add all sensor endpoints to unified server
- [ ] Update systemd service files
- [ ] Test backward compatibility with Mobile RTK Module
- [ ] Update documentation

**Benefits**:
- Simpler architecture
- Single point of failure (easier debugging)
- Reduced resource usage

---

### 7. Battery Monitoring from MAVROS2
**Status**: Not started
**Estimated Time**: 1-2 hours

**Tasks**:
- [ ] Subscribe to `/mavros/battery` topic
- [ ] Add battery voltage to HTTP bridge
- [ ] Display battery level on Mobile RTK Module
- [ ] Add low battery warning (visual + audible)
- [ ] Log battery data for analysis

**Display Location**:
- Control bar (bottom of screen)
- GPS Status tab (detailed battery info)

---

## NICE TO HAVE

### 8. SLAM (Simultaneous Localization and Mapping)
**Status**: Not started
**Estimated Time**: 1-2 days

**Tasks**:
- [ ] Install slam_toolbox: `sudo apt install ros-humble-slam-toolbox`
- [ ] Create SLAM launch file
- [ ] Configure parameters for RPLidar A1
- [ ] Run SLAM while driving rover
- [ ] Save generated maps
- [ ] Load and display maps on Mobile RTK Module

**Commands**:
```bash
ros2 launch slam_toolbox online_async_launch.py
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

---

### 9. Nav2 Autonomous Navigation
**Status**: Not started
**Estimated Time**: 2-3 days

**Tasks**:
- [ ] Install Nav2: `sudo apt install ros-humble-navigation2`
- [ ] Configure costmaps (global + local)
- [ ] Set up behavior tree for navigation
- [ ] Configure planners (DWB, Smac, etc.)
- [ ] Test autonomous waypoint navigation
- [ ] Integrate with Mobile RTK Module waypoint system

**Features**:
- Dynamic obstacle avoidance using LiDAR
- Path planning with A* or Dijkstra
- Recovery behaviors (stuck detection, rotation, backup)

---

### 10. Object Detection with Camera
**Status**: Not started
**Estimated Time**: 3-5 days

**Tasks**:
- [ ] Install TensorFlow or PyTorch on Jetson
- [ ] Download pre-trained YOLO model
- [ ] Create object detection ROS 2 node
- [ ] Publish detected objects to topic
- [ ] Display bounding boxes on camera feed
- [ ] Add object tracking

**Use Cases**:
- Person detection (follow-me mode)
- Obstacle classification (tree, rock, etc.)
- Sign detection (stop signs, markers)

---

### 11. Data Recording with rosbag2
**Status**: rosbag2 installed, not configured
**Estimated Time**: 1 hour

**Tasks**:
- [ ] Create recording launch file
- [ ] Record all topics: /scan, /image_raw/compressed, GPS, etc.
- [ ] Add record/stop buttons to Mobile RTK Module
- [ ] Implement automatic recording on mission start
- [ ] Add playback capability for debugging

**Commands**:
```bash
# Record all topics
ros2 bag record -a -o my_mission

# Record specific topics
ros2 bag record /scan /image_raw/compressed /mavros/global_position/global

# Playback
ros2 bag play my_mission
```

---

## MAINTENANCE & IMPROVEMENTS

### 12. Create Udev Rules for USB Devices
**Status**: Not started
**Estimated Time**: 30 minutes

**Tasks**:
- [ ] Create udev rule for RPLidar (persistent /dev/rplidar)
- [ ] Create udev rule for camera (persistent /dev/camera)
- [ ] Create udev rule for Cube Orange (persistent /dev/cubeorange)
- [ ] Test device persistence across reboots

**Benefits**:
- Stable device paths
- No more searching for /dev/ttyUSB* or /dev/video*

---

### 13. Performance Optimization
**Status**: Ongoing
**Estimated Time**: Variable

**Tasks**:
- [ ] Profile CPU/RAM usage on Jetson
- [ ] Optimize LiDAR scan rate if needed
- [ ] Reduce camera resolution if needed
- [ ] Implement adaptive update rates
- [ ] Monitor network bandwidth

---

### 14. Documentation
**Status**: In progress (session notes created)
**Estimated Time**: Ongoing

**Tasks**:
- [x] Session notes (Oct 31 - ROS 2 integration)
- [x] Session notes (Oct 31 - Continuation)
- [ ] User manual for Mobile RTK Module
- [ ] Hardware setup guide
- [ ] Software installation guide
- [ ] Troubleshooting guide
- [ ] API documentation for HTTP bridge

---

## SYSTEM CONFIGURATION

### Current IP Addresses
```
Mobile RTK Module:    192.168.254.127 (Pi 5)
Jetson Orin Nano:     192.168.254.100
RTK Base Station:     192.168.254.165
```

### Jetson Services (Auto-start on boot)
```
jetson-rover-server.service    - Port 5000 (Cube Orange control)
jetson-rplidar.service         - ROS 2 RPLidar node
jetson-sensor-bridge.service   - Port 5001 (HTTP bridge)
```

### Service Logs
```
/var/log/jetson-rover-server.log
/var/log/jetson-rplidar.log
/var/log/jetson-sensor-bridge.log
```

---

## QUICK WINS (Can do in <30 minutes)

1. Add distance/bearing calculation from Module to rover on Dashboard
2. Add compass heading display for rover orientation
3. Add timestamp to sensor data displays
4. Create keyboard shortcuts for common actions (ESC already mapped to exit)
5. Add sound effects for warnings (low battery, obstacles, etc.)
6. Plug camera back in and verify Camera View tab

---

## SESSION HANDOFF NOTES

**For Next Session**:
1. Start with quick wins: Plug camera back in, verify working
2. Implement rover GPS visualization (40 min task - high impact)
3. Then move to MAVROS2 configuration (longer task)
4. Test everything thoroughly before moving to advanced features

**Critical Files**:
- Dashboard: `/home/jay/Desktop/Mini Rover Development/Mobile RTK Control Module/09_dashboard_enhanced.py`
- Session notes: `/home/jay/Desktop/Mini Rover Development/Mobile RTK Control Module/SESSION_NOTES_OCT31_CONTINUATION.md`
- Jetson services: `/etc/systemd/system/jetson-*.service` (on Jetson)

**Key Commands**:
```bash
# Jetson service management
ssh jay@192.168.254.100
sudo systemctl status jetson-rover-server jetson-rplidar jetson-sensor-bridge
sudo systemctl restart <service-name>

# Dashboard deployment
scp 09_dashboard_enhanced.py jay@192.168.254.127:~/robot-control-terminal/
ssh jay@192.168.254.127 "pkill -9 python3; cd ~/robot-control-terminal && DISPLAY=:0 python3 09_dashboard_enhanced.py &"

# Test sensor bridge
curl http://192.168.254.100:5001/api/sensors/status | python3 -m json.tool
```

---

## BLOCKERS / DEPENDENCIES

**None currently** - All required hardware and software is installed and functional.

**Camera**: Temporarily unplugged but ready to reconnect.

---

## NOTES

- User prefers no constant progress summaries unless requested
- All foundation work complete - ready for advanced features
- Hardware is fully operational and tested
- Network architecture stable and performant
- Systemd services working perfectly on boot

---

**End of TODO List - Updated October 31, 2025**
