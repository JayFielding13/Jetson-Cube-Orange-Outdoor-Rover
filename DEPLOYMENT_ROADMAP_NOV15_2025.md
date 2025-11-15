# Rover Deployment Roadmap - November 15, 2025
**From Simulation to Real-World Testing**

---

## Current Status Assessment

### ✅ What's Already Working

Based on your documentation, you have a **very mature system**:

**Hardware:**
- ✅ Jetson Orin Nano with ROS 2 Humble
- ✅ Cube Orange+ flight controller (ArduRover)
- ✅ HERE 3+ RTK GPS
- ✅ RPLidar A1
- ✅ USB Camera
- ✅ 6x Ultrasonic sensors (ESP32-based)
- ✅ RTK Base Station operational

**Software:**
- ✅ MAVROS2 integration
- ✅ All sensors integrated with ROS 2
- ✅ RTK corrections forwarding (MQTT → MAVROS2)
- ✅ Web dashboard on Pi 5
- ✅ Systemd auto-start services
- ✅ Telemetry API (HTTP + UDP)

**Last Status (Nov 2, 2025):**
- GPS Fix Type: 3 (3D GPS)
- Satellites: 15-17
- HDOP: 0.07 (excellent)
- RTCM corrections forwarding at 2.85 Hz
- Waiting for RTK convergence to Fixed (fix_type 6)

### ❓ What We Don't Know

1. Is the rover powered on and functional right now?
2. Did RTK ever converge to Fixed status?
3. Have you done any real-world navigation testing?
4. Are there any hardware issues preventing deployment?

---

## Deployment Roadmap - 4 Phases

### **Phase 1: System Verification (1-2 hours)**
**Goal:** Confirm all hardware and software is operational

### **Phase 2: Baseline Testing (2-4 hours)**
**Goal:** Test basic movement, sensors, and manual control

### **Phase 3: RTK & Precision Navigation (2-4 hours)**
**Goal:** Achieve centimeter-level accuracy and test autonomous navigation

### **Phase 4: Advanced Capabilities (Ongoing)**
**Goal:** Obstacle avoidance, path planning, mission execution

---

## Phase 1: System Verification ⚡ START HERE

### Why This Matters
Before any outdoor testing, we need to verify the rover is in a known-good state. Your last session was Nov 2 - we need to check if everything still works.

### 1.1 Power On and Connect (15 min)

**Power On Sequence:**
```bash
1. RTK Base Station (192.168.254.165)
   - Wait 60 seconds for GPS lock

2. Jetson Orin Nano (192.168.254.100)
   - Wait 30 seconds for boot
   - Wait 30 more for ROS 2 services

3. Raspberry Pi 5 Dashboard (192.168.254.127)
   - Connect to rover from dashboard
```

**Verify Network:**
```bash
# From your laptop
ping 192.168.254.100  # Jetson
ping 192.168.254.127  # Dashboard
ping 192.168.254.165  # RTK Base
```

**Success Criteria:**
- [ ] All three systems ping successfully
- [ ] Jetson SSH accessible
- [ ] Dashboard loads

### 1.2 Check System Services (10 min)

**SSH to Jetson and verify all services:**
```bash
ssh jay@192.168.254.100
sudo systemctl status jetson-mavros2
sudo systemctl status jetson-unified-bridge
sudo systemctl status jetson-rtcm-forwarder
sudo systemctl status jetson-rplidar
sudo systemctl status jetson-ultrasonic-bridge
```

**All should show:** `active (running)`

**If any failed:**
```bash
sudo journalctl -u <service-name> -n 50  # Check logs
sudo systemctl restart <service-name>
```

**Success Criteria:**
- [ ] All 5 services running
- [ ] No error messages in logs
- [ ] Services auto-started on boot

### 1.3 Verify Sensor Data (15 min)

**Check GPS:**
```bash
curl http://192.168.254.100:5001/api/gps | python3 -m json.tool
```

**Look for:**
- `fix_type`: 3 or higher (5-6 would be amazing!)
- `satellites_visible`: 10+
- `hdop`: <2.0
- `latitude`, `longitude`: Reasonable values

**Check LiDAR:**
```bash
ssh jay@192.168.254.100
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
ros2 topic hz /scan
```

**Should show:** ~5-10 Hz (RPLidar update rate)

**Check Camera:**
```bash
ros2 topic hz /camera/image_raw
```

**Should show:** ~30 Hz

**Check Ultrasonics:**
```bash
ros2 topic list | grep ultrasonic
ros2 topic echo /ultrasonic/front --once
```

**Should show:** Distance reading in meters

**Success Criteria:**
- [ ] GPS has fix (fix_type >= 3)
- [ ] LiDAR publishing at 5-10 Hz
- [ ] Camera publishing at ~30 Hz
- [ ] All 6 ultrasonic sensors publishing
- [ ] RTK corrections flowing (check logs)

### 1.4 Test Dashboard Control (20 min)

**On Raspberry Pi 5 Dashboard:**
```bash
cd ~/robot-control-terminal
DISPLAY=:0 python3 09_dashboard_enhanced.py
```

**Test Basic Functions:**
1. **ARM/DISARM Test**
   - [ ] Press ARM button
   - [ ] Rover arms successfully
   - [ ] Press DISARM
   - [ ] Rover disarms cleanly

2. **Sensor Display**
   - [ ] GPS position shows on map
   - [ ] LiDAR view shows obstacles
   - [ ] Camera feed displays
   - [ ] Satellite count accurate (15-17)

3. **Telemetry**
   - [ ] Battery voltage displayed
   - [ ] GPS HDOP shown
   - [ ] RTK corrections status

**Success Criteria:**
- [ ] Dashboard loads without errors
- [ ] All sensor data visible
- [ ] ARM/DISARM works
- [ ] No freezing or crashes

### 1.5 Safety Check (10 min)

**Physical Inspection:**
- [ ] Battery charged and secure
- [ ] All wires connected properly
- [ ] Motors free to rotate
- [ ] Emergency stop accessible
- [ ] GPS antenna has clear sky view
- [ ] LiDAR spinning freely
- [ ] Camera lens clean

**Software Safety:**
- [ ] Geofence configured (if needed)
- [ ] Maximum speed limits set
- [ ] Failsafe behavior configured

---

## Phase 2: Baseline Testing 🚗

### Goal
Verify basic rover movement and control in a controlled environment.

### 2.1 Pre-Flight Checks (10 min)

**Test Location Requirements:**
- [ ] Open area, 30+ meter clear radius
- [ ] Flat, smooth surface (parking lot ideal)
- [ ] Clear GPS sky view (no trees/buildings)
- [ ] Safe for testing (no people, vehicles, obstacles)
- [ ] Within WiFi range of router/access point

**Equipment Needed:**
- [ ] Charged rover battery
- [ ] Laptop for monitoring
- [ ] Measuring tape (for accuracy tests)
- [ ] Phone for photos/video
- [ ] Emergency stop within reach

### 2.2 Manual Control Test (20 min)

**Test 1: Static ARM Test**
```bash
1. Place rover on ground, wheels unobstructed
2. Dashboard: Press ARM
3. Observe: Motors should arm (slight sound/vibration)
4. Wait 30 seconds (no movement expected)
5. Dashboard: Press DISARM
```

**Success Criteria:**
- [ ] Rover arms immediately
- [ ] No unexpected movement
- [ ] Disarms cleanly

**Test 2: RC Override Test** (if you have RC transmitter)
```bash
1. ARM rover
2. Use RC stick to drive forward 1 meter
3. Stop
4. Reverse 1 meter
5. Test left/right turns
6. DISARM
```

**Success Criteria:**
- [ ] Rover responds to RC inputs
- [ ] Movement is smooth
- [ ] Stops promptly when stick centered
- [ ] All 4 wheels driving properly

### 2.3 Sensor Validation During Movement (30 min)

**Test 3: LiDAR Obstacle Detection**
```bash
1. ARM rover
2. Place object 2 meters in front
3. Dashboard: Check LiDAR view
4. Move object closer
5. Verify LiDAR detects it
6. DISARM
```

**Success Criteria:**
- [ ] LiDAR shows obstacle clearly
- [ ] Distance reading accurate
- [ ] Real-time updates visible

**Test 4: Ultrasonic Validation**
```bash
1. Place obstacles at different positions
2. Check ultrasonic readings on dashboard
3. Verify front, rear, side sensors all detecting
```

**Success Criteria:**
- [ ] All 6 sensors reporting
- [ ] Readings match physical distances
- [ ] No false positives

**Test 5: Camera View Validation**
```bash
1. Point camera at landmark
2. Verify image on dashboard
3. Check frame rate is smooth
```

**Success Criteria:**
- [ ] Camera feed clear
- [ ] No lag or freezing
- [ ] Adequate lighting

### 2.4 GPS Position Accuracy Test (20 min)

**Test 6: Static GPS Test**
```bash
1. Place rover in open area
2. Let GPS stabilize for 5 minutes
3. Record coordinates every 30 seconds (10 samples)
4. Calculate position variation
```

**Expected Results:**
- **With standard GPS (fix_type 3):** 1-3 meter variation
- **With RTK Float (fix_type 5):** 10-50cm variation
- **With RTK Fixed (fix_type 6):** <5cm variation! 🎯

**Commands to Monitor:**
```bash
# Watch GPS in real-time
ssh jay@192.168.254.100
~/monitor_rtk_fix.sh
```

**Success Criteria:**
- [ ] GPS maintains fix throughout test
- [ ] Position recorded successfully
- [ ] Variation calculated
- [ ] RTK status documented

---

## Phase 3: RTK & Precision Navigation 🎯

### Goal
Achieve RTK Fixed status and test centimeter-level navigation accuracy.

### 3.1 RTK Convergence Monitoring (10-30 min)

**This is critical for precision work!**

**Check RTK Status:**
```bash
curl http://192.168.254.100:5001/api/gps | python3 -m json.tool
```

**Look for `fix_type`:**
- `3` = 3D GPS (2-5m accuracy) - Standard GPS
- `5` = RTK Float (10-50cm) - RTK converging
- `6` = RTK Fixed (<5cm) - **THIS IS THE GOAL!** 🎯

**Monitor Convergence:**
```bash
ssh jay@192.168.254.100
~/monitor_rtk_fix.sh
```

**Verify RTK Corrections:**
```bash
ssh jay@192.168.254.100
sudo journalctl -u jetson-rtcm-forwarder -n 20
```

**Should see:** "RTCM Stats: XXXXX bytes, XX messages"

**Convergence Times:**
- **Cold start:** 5-30 minutes to RTK Fixed
- **Warm start:** 2-15 minutes to RTK Fixed
- **Hot start:** 1-5 minutes to RTK Fixed

**Success Criteria:**
- [ ] RTCM corrections flowing (2-5 Hz)
- [ ] GPS achieves RTK Float (fix_type 5)
- [ ] Eventually achieves RTK Fixed (fix_type 6)
- [ ] HDOP < 0.5
- [ ] 15+ satellites visible

### 3.2 RTK Accuracy Validation (15 min)

**Test 7: Precision Static Test**

**Once RTK Fixed is achieved:**
```bash
1. Place marker at rover position
2. Record GPS coordinates
3. Move rover away 10 meters
4. Navigate back to recorded coordinates
5. Measure distance from original marker
```

**Expected Result:**
- RTK Fixed should get you within 5cm of original position!

**Success Criteria:**
- [ ] Rover returns within 10cm of marker
- [ ] GPS maintained RTK Fixed throughout
- [ ] Position error documented

### 3.3 Autonomous Waypoint Navigation (30 min)

**Test 8: Short Distance Waypoint**

**Safety First:**
- Start with 5 meter waypoint
- Clear path, no obstacles
- Emergency stop ready

**Procedure:**
```bash
1. ARM rover
2. Dashboard: Drop waypoint 5m away
3. Send waypoint to rover
4. Monitor approach
5. Measure final position error
6. DISARM
```

**Test 9: Multi-Waypoint Mission**

**If Test 8 succeeds:**
```bash
1. Create square pattern (5m sides)
2. Drop 4 waypoints
3. Send mission to rover
4. Watch it execute
5. Verify it returns to start position
```

**Success Criteria:**
- [ ] Rover navigates to each waypoint
- [ ] Final position error <10cm (with RTK Fixed)
- [ ] Path is smooth and direct
- [ ] Returns to start successfully

### 3.4 Compare RTK vs Standard GPS (30 min)

**Test 10: Accuracy Comparison**

**Part A - Without RTK:**
```bash
1. Temporarily disable RTCM forwarder:
   sudo systemctl stop jetson-rtcm-forwarder
2. Wait for GPS to drop to fix_type 3
3. Send rover to waypoint 10m away
4. Measure final position error (expect 1-3m)
```

**Part B - With RTK:**
```bash
1. Re-enable RTCM forwarder:
   sudo systemctl start jetson-rtcm-forwarder
2. Wait for RTK Fixed (fix_type 6)
3. Send rover to SAME waypoint
4. Measure final position error (expect <10cm)
```

**Document the dramatic improvement!** 📸

**Success Criteria:**
- [ ] Standard GPS: 1-3m error
- [ ] RTK GPS: <10cm error
- [ ] Results documented with photos
- [ ] Demonstrate 100x accuracy improvement!

---

## Phase 4: Advanced Capabilities 🚀

### 4.1 Obstacle Avoidance (FUTURE)

**Prerequisites:**
- All Phase 1-3 tests passed
- Sensors validated
- Basic navigation working

**Implementation Options:**
1. **Simple reactive avoidance**
   - Use ultrasonic sensors
   - Stop if obstacle <1m
   - Simple state machine

2. **LiDAR-based avoidance**
   - Scan for gaps in obstacles
   - Steer around detected objects
   - Continue toward goal

3. **Full Nav2 integration**
   - SLAM mapping
   - Global path planning
   - Dynamic re-planning

**Recommendation:** Start with #1 (reactive), then #2, then #3

### 4.2 Path Planning & SLAM (FUTURE)

**Tools to integrate:**
- `slam_toolbox` - Real-time SLAM
- `nav2` - ROS 2 navigation stack
- `costmap_2d` - Obstacle cost mapping

**Steps:**
1. Create map of test area with SLAM
2. Configure Nav2 parameters
3. Test navigation with global planner
4. Add dynamic obstacle handling

### 4.3 Vision & Object Detection (FUTURE)

**Potential Applications:**
- Lane following
- Sign detection
- Person/vehicle detection
- Object tracking

**Recommendation:** Use your new desktop with RTX 5070 for:
- YOLOv8 training
- Model optimization
- Testing before deployment to Jetson

---

## Immediate Next Steps - Priority Order

### **TODAY - If Rover is Available:**

**1. Power On & Basic Health Check (30 min)**
```bash
# Complete Phase 1.1-1.3
- Power on all systems
- Verify services running
- Check sensor data
```

**2. Quick ARM/DISARM Test (10 min)**
```bash
# Phase 2.1-2.2
- Verify basic control works
- No movement testing yet
```

**3. RTK Status Check (15 min)**
```bash
# Phase 3.1
- Check current GPS fix type
- Verify RTCM corrections flowing
- Monitor for RTK convergence
```

**STOP HERE** if any issues found. Report back what's not working.

### **NEXT SESSION - Outdoor Testing:**

**4. Baseline Movement Tests (1 hour)**
```bash
# Phase 2.2-2.4
- Manual control validation
- Sensor checks during movement
- GPS accuracy baseline
```

**5. RTK Navigation Testing (1-2 hours)**
```bash
# Phase 3.2-3.4
- Wait for RTK Fixed
- Test precision waypoint navigation
- Compare RTK vs standard GPS
- Document accuracy improvements
```

---

## Key Documentation to Create

As you test, document:

1. **RTK Convergence Log**
   - Time to achieve RTK Float/Fixed
   - Environmental conditions
   - Satellite count, HDOP

2. **Navigation Accuracy Results**
   - Position errors with/without RTK
   - Photos/videos of testing
   - Waypoint mission results

3. **Sensor Performance**
   - LiDAR detection range
   - Ultrasonic accuracy
   - Camera quality

4. **Issues Encountered**
   - Hardware problems
   - Software bugs
   - Calibration needs

---

## Success Metrics

### Phase 1 Complete When:
- [ ] All systems powered on and running
- [ ] All 5 Jetson services active
- [ ] All sensors publishing data
- [ ] Dashboard control verified

### Phase 2 Complete When:
- [ ] Rover arms/disarms reliably
- [ ] Manual control working
- [ ] All sensors validated during movement
- [ ] GPS position baseline established

### Phase 3 Complete When:
- [ ] RTK Fixed status achieved (fix_type 6)
- [ ] <10cm navigation accuracy demonstrated
- [ ] RTK vs standard GPS comparison documented
- [ ] Multi-waypoint mission successful

### Phase 4 Complete When:
- [ ] Obstacle avoidance implemented
- [ ] SLAM mapping functional
- [ ] Nav2 autonomous navigation working
- [ ] Vision system integrated

---

## Critical Questions to Answer

**Before we proceed, I need to know:**

1. **Is your rover currently accessible and ready to power on?**
   - Do you have physical access to it?
   - Is it assembled and ready to test?

2. **What's your test environment?**
   - Do you have outdoor space for testing?
   - Is it flat, open, with good GPS view?

3. **When did you last test it?**
   - November 2? More recent?
   - Did RTK ever converge to Fixed?

4. **Are there any known hardware issues?**
   - Broken sensors?
   - Motor problems?
   - GPS antenna issues?

5. **What's your immediate goal?**
   - Just verify it works?
   - Test RTK accuracy?
   - Run autonomous missions?
   - Prepare for specific application?

---

## Recommended Approach

**Based on your simulation experience, I suggest:**

### **Option A: Conservative (Recommended)**
1. Phase 1 health check (TODAY)
2. Document current state
3. Fix any issues found
4. Plan outdoor testing session
5. Execute Phases 2-3 systematically

### **Option B: Aggressive (If Everything Works)**
1. Quick Phase 1 check (30 min)
2. Immediate outdoor testing (Phase 2)
3. RTK navigation testing (Phase 3)
4. Document results
5. Plan Phase 4 implementation

**I recommend Option A** - methodical validation before outdoor testing.

---

## What I Can Help With Right Now

1. **SSH into Jetson** and check system status
2. **Verify service health** and fix issues
3. **Review sensor data** to ensure everything is publishing
4. **Check RTK configuration** and troubleshoot if needed
5. **Create test scripts** for automated validation
6. **Update documentation** based on current findings

**Let's start with Phase 1 if your rover is accessible!**

---

**What do you want to tackle first?**
