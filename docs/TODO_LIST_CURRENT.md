# Current TODO List - Mobile RTK Rover Project
**Last Updated**: November 2, 2025 - End of Day Session

---

## ✅ COMPLETED TODAY (November 2, 2025)

### Survey Mode UI/UX Improvements
- [x] Fix geofence fence post rendering (all posts now visible)
- [x] Add magenta fence posts for saved geofences
- [x] Add "Fence Posts" layer toggle checkbox
- [x] Reorganize layout: waypoints left column, geofences right column
- [x] Add centered two-row control bar (zoom controls + layer checkboxes)
- [x] Remove "SURVEY MAP" title header (freed 35px for map space)
- [x] Move Mode indicator to bottom statistics bar
- [x] Fix waypoint menu overflow issue
- [x] Add point-in-polygon helper function (for future geofence enforcement)
- [x] Add statistics display to bottom control bar (Waypoints, Geofences, GPS Fix, Mode)
- [x] Deploy and test all Survey Mode improvements

---

## ✅ COMPLETED PREVIOUSLY (November 1, 2025)

### MAVROS2 Migration & RTK GPS Integration
- [x] Fix GPS satellite count display (was showing 0)
- [x] Migrate from NavSatFix to GPSRAW message type
- [x] Rewrite unified bridge GPS callback for GPSRAW format (lat/lon int32 * 1e7, alt in mm)
- [x] Enable MAVROS2 RTK plugins (gps_status, gps_rtk)
- [x] Configure RTK topics for HERE 3+ GPS antenna
- [x] Discover and connect to RTKPi base station (100.66.67.11)
- [x] Create RTCM MQTT-to-MAVROS2 forwarder service
- [x] Deploy jetson-rtcm-forwarder.service (systemd auto-start)
- [x] Verify RTCM corrections forwarding (2.85 Hz, ~24,600 bytes/10s)
- [x] Fix rover ARM/disarm control issues (Jetson reboot resolved)
- [x] Fix dashboard freeze (killed and restarted process)
- [x] Create RTK troubleshooting guide and monitoring scripts
- [x] Document complete RTK architecture in session log

### Previous Rover GPS Visualization & RTK Integration (Oct 31)
- [x] Add rover GPS section to GPS Status tab (side-by-side layout)
- [x] Draw rover position on Dashboard map
- [x] Draw rover icon on Follow-Me map
- [x] Draw rover icon on Survey Mode map
- [x] Enhance rover server GPS telemetry (altitude, satellites, fix type, HDOP)
- [x] Implement MQTT-based RTK correction forwarding system
- [x] Create RTKForwarder class with RTCM fragmentation
- [x] Connect to base station via MQTT (100.66.67.11:1883)
- [x] Forward corrections to Cube Orange via MAVLink GPS_RTCM_DATA
- [x] Add RTK status to rover API endpoint
- [x] Add RTK CORRECTIONS box to GPS Status tab
- [x] Display real-time RTK connection status and correction counter

---

## ✅ COMPLETED PREVIOUSLY (October 31, 2025)

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

### 1. Monitor RTK Convergence & Verify RTK Fixed Status
**Status**: RTCM corrections flowing successfully, monitoring convergence
**Estimated Time**: 30 minutes - 2 hours

**Tasks**:
- [ ] Check current GPS fix type (should progress from 3 → 5 → 6)
- [ ] Run monitoring script: `ssh jay@100.91.191.47 "~/monitor_rtk_fix.sh"`
- [ ] Verify RTK Fixed (fix_type 6) achieved
- [ ] Test position accuracy with RTK (should be <5cm)
- [ ] Document RTK convergence time and final accuracy
- [ ] Verify dashboard shows "RTK Fixed" on GPS Status tab

**Current Status** (End of Nov 1 Session):
- RTK corrections: ✅ Forwarding (2.85 Hz, ~24,600 bytes/10s)
- GPS fix type: 3 (3D GPS - awaiting convergence)
- Satellites: 15-17
- HDOP: 0.07 (excellent)
- Expected convergence: 2-30 minutes
- RTCM forwarder: Running as systemd service

---

## HIGH PRIORITY

### 2. Camera Re-Integration ✅ COMPLETED
**Status**: Camera connected and working
**Completed**: November 1, 2025

**Tasks**:
- [x] Plug camera back into Jetson
- [x] Verify `/dev/video0` appears
- [x] Restart jetson-rplidar service to detect camera
- [x] Test Camera View tab displays video feed
- [x] Verify ~7 FPS update rate

**Result**: Camera feed visible on Dashboard Camera tab, working correctly

---

### 3. Unified Bridge Graceful Shutdown
**Status**: Minor issue, not critical
**Estimated Time**: 1 hour

**Tasks**:
- [ ] Add SIGTERM/SIGINT signal handlers to ros2_unified_bridge.py
- [ ] Implement graceful shutdown for all ROS 2 subscriptions
- [ ] Test service restart without 90+ second timeout
- [ ] Update systemd service with shorter timeout

**Current Issue**:
- Service takes 90+ seconds to restart due to stuck processes
- No impact on runtime operation, only affects restart/shutdown

**Benefits**:
- Faster service restarts
- Cleaner shutdown process
- Better resource cleanup

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
Mobile RTK Module:    100.73.233.124 (Pi 5)
Jetson Orin Nano:     100.91.191.47
RTK Base Station:     100.66.67.11
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
ssh jay@100.91.191.47
sudo systemctl status jetson-rover-server jetson-rplidar jetson-sensor-bridge
sudo systemctl restart <service-name>

# Dashboard deployment
scp 09_dashboard_enhanced.py jay@100.73.233.124:~/robot-control-terminal/
ssh jay@100.73.233.124 "pkill -9 python3; cd ~/robot-control-terminal && DISPLAY=:0 python3 09_dashboard_enhanced.py &"

# Test sensor bridge
curl http://100.91.191.47:5001/api/sensors/status | python3 -m json.tool
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
