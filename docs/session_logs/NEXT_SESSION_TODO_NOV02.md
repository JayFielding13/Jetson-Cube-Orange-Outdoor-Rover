# Next Session TODO - November 2, 2025
## Mobile RTK Control Module - RTK Convergence & Testing

**Previous Session**: November 1, 2025 - MAVROS2 Migration & RTK GPS Integration
**Status**: RTK corrections flowing, awaiting GPS convergence to RTK Fixed
**Network**: All systems stable and operational

---

## SESSION OVERVIEW

Today's session successfully:
- Fixed GPS satellite count display (was showing 0)
- Migrated from NavSatFix to GPSRAW message format
- Enabled MAVROS2 RTK plugins for HERE 3+ GPS
- Created RTCM MQTT-to-MAVROS2 forwarder (systemd service)
- Verified RTK corrections forwarding at 2.85 Hz (~24,600 bytes/10s)
- Fixed rover ARM/disarm control issues
- Fixed dashboard freeze

**Current GPS Status**:
- Fix Type: 3 (3D GPS)
- Satellites: 15-17
- HDOP: 0.07 (excellent)
- RTCM Corrections: Forwarding successfully
- Expected Convergence: 2-30 minutes to RTK Float/Fixed

---

## QUICK START CHECKLIST

Before starting the next session:

- [ ] Power on Jetson Orin Nano (IP: 100.91.191.47)
- [ ] Verify all systemd services running:
  ```bash
  ssh jay@100.91.191.47
  sudo systemctl status jetson-mavros2
  sudo systemctl status jetson-unified-bridge
  sudo systemctl status jetson-rtcm-forwarder
  ```
- [ ] Check RTK corrections still flowing:
  ```bash
  ssh jay@100.91.191.47
  sudo journalctl -u jetson-rtcm-forwarder -n 20
  ```
- [ ] Power on Mobile RTK Module (Raspberry Pi 5, IP: 100.73.233.124)
- [ ] Press CONNECT button on touchscreen
- [ ] Open GPS Status tab to monitor RTK convergence

---

## PRIORITY 1: VERIFY RTK CONVERGENCE (HIGH)

### Task 1.1: Check Current GPS Fix Type
**Objective**: Determine if GPS has converged to RTK Float or Fixed

**Steps**:
1. Check GPS via API:
   ```bash
   curl http://100.91.191.47:5001/api/gps | python3 -m json.tool
   ```
2. Look for `fix_type` field:
   - 3 = 3D GPS (standard, not RTK yet)
   - 5 = RTK Float (receiving corrections, converging)
   - 6 = RTK Fixed (full RTK accuracy, <5cm!)

**Expected Outcomes**:
- **If fix_type = 6 (RTK Fixed)**: ✅ Convergence successful! Proceed to Task 1.2
- **If fix_type = 5 (RTK Float)**: 🔄 Still converging, wait 5-15 minutes
- **If fix_type = 3 (3D GPS)**: ⚠️ Investigate - may need troubleshooting

### Task 1.2: Run RTK Monitoring Script
**Objective**: Watch GPS fix type progression in real-time

**Command**:
```bash
ssh jay@100.91.191.47 "~/monitor_rtk_fix.sh"
```

**What to Look For**:
- Fix type changing from 3 → 5 → 6
- Satellite count staying above 10
- HDOP remaining low (<1.0)
- RTK baseline data appearing (distance to base station)

**Success Criteria**:
- [ ] Script shows fix_type = 6 (RTK Fixed)
- [ ] RTK baseline shows reasonable distance (~known distance to base)
- [ ] Accuracy shows <50mm (5cm)

### Task 1.3: Verify Dashboard RTK Status
**Objective**: Confirm dashboard correctly displays RTK status

**Steps**:
1. Open GPS Status tab on dashboard
2. Check "RTK CORRECTIONS" box shows:
   - Status: Connected (green)
   - Corrections count updating
3. Check "ROVER GPS" shows:
   - Fix Type: "RTK Fixed" or "RTK Float"
   - HDOP: <1.0

**Success Criteria**:
- [ ] Dashboard shows "RTK Fixed" status
- [ ] RTK CORRECTIONS shows "Connected" in green
- [ ] Satellite count matches GPS data (15-17)

### Task 1.4: Test Position Accuracy
**Objective**: Verify centimeter-level accuracy achieved

**Method 1 - Static Test**:
1. Place rover in open area with clear sky view
2. Let GPS converge for 5-10 minutes
3. Record GPS coordinates every 30 seconds (10 samples)
4. Calculate standard deviation of positions
5. Expected: <5cm variation

**Method 2 - Dashboard Observation**:
1. Watch rover position on map
2. With RTK Fixed, icon should be rock-solid stable
3. Without RTK, icon will "drift" slightly (meter-level)

**Success Criteria**:
- [ ] Position variation <5cm over 5 minutes
- [ ] Rover icon stable on map (no drift)
- [ ] HDOP remains <0.2 consistently

---

## PRIORITY 2: RTK TROUBLESHOOTING (IF NEEDED)

### If GPS Stuck at Fix Type 3 (3D GPS)

**Possible Causes**:
1. RTCM corrections not reaching GPS
2. Base station too far away
3. GPS configuration issue
4. Poor satellite visibility

**Troubleshooting Steps**:

**Step 1 - Verify RTCM Corrections**:
```bash
# Check forwarder service logs
ssh jay@100.91.191.47
sudo journalctl -u jetson-rtcm-forwarder -f

# Should see: "RTCM Stats: XXXXX bytes, XX messages"
# If no messages: RTCM forwarder issue
```

**Step 2 - Check MAVROS RTK Topic**:
```bash
# On Jetson
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
ros2 topic hz /mavros/gps_rtk/send_rtcm

# Should show ~1-5 Hz
# If 0 Hz: Forwarder not publishing
```

**Step 3 - Verify Base Station**:
```bash
# Check base station reachable
ping 100.66.67.11

# Check base station MQTT
ssh jay@100.66.67.11
sudo systemctl status gps-publisher
```

**Step 4 - Check ArduPilot GPS Parameters**:
```bash
# Connect Mission Planner or QGroundControl
# Verify parameters:
GPS_TYPE = 1 (AUTO)
GPS_AUTO_CONFIG = 1
GPS_INJECT_TO = 1 (First GPS)
GPS_TYPE2 = 0 (None)
```

**Step 5 - Restart RTK Stack**:
```bash
ssh jay@100.91.191.47
sudo systemctl restart jetson-rtcm-forwarder
sudo systemctl restart jetson-mavros2
# Wait 2-5 minutes for convergence
```

**Reference Documentation**:
- RTK Setup Guide: `RTK_SETUP_GUIDE.md` (in `/tmp/` on Jetson)
- Troubleshooting: `RTK_TROUBLESHOOTING.md` (in `/tmp/` on Jetson)
- Session Log: `SESSION_LOG_NOV01_2025.md` (in project directory)

---

## PRIORITY 3: ROVER MOVEMENT TESTING (AFTER RTK VERIFIED)

Once RTK Fixed status is confirmed, proceed with rover testing:

### Task 3.1: Test Static ARM/DISARM
**Objective**: Verify basic rover control

**Steps**:
1. Press ARM button on dashboard
2. Verify rover arms successfully
3. Let sit for 30 seconds (no movement)
4. Press DISARM button
5. Verify rover disarms

**Success Criteria**:
- [ ] Rover arms immediately
- [ ] No unexpected movement
- [ ] Button states update correctly
- [ ] Rover disarms cleanly

### Task 3.2: Test Short Distance Navigation
**Objective**: Verify rover navigation with RTK accuracy

**Steps**:
1. Place rover in open area
2. ARM rover
3. Drop waypoint 5 meters away (on Dashboard map)
4. Send waypoint to rover
5. Watch rover navigate to target
6. Measure final position error

**Expected Performance**:
- **Without RTK**: 1-3 meter accuracy
- **With RTK Fixed**: <10cm accuracy! 🎯

**Success Criteria**:
- [ ] Rover navigates to waypoint
- [ ] Final position within 10cm of target
- [ ] Path is smooth and direct
- [ ] Rover stops precisely at target

### Task 3.3: Test Follow-Me Mode with RTK
**Objective**: Verify improved tracking accuracy

**Steps**:
1. Go to Follow-Me tab
2. Set follow distance to 5 meters
3. Press "START FOLLOW-ME"
4. Walk slowly in a circle
5. Observe rover tracking

**Expected Improvement**:
- With RTK, rover should maintain precise 5m distance
- Following should be smoother and more accurate
- Less overshoot and oscillation

**Success Criteria**:
- [ ] Rover maintains 5m ± 10cm distance
- [ ] Smooth tracking (no jerky movements)
- [ ] Follows curved paths accurately

---

## PRIORITY 4: DOCUMENTATION & TESTING

### Task 4.1: Document RTK Convergence Results

**Data to Record**:
- Time to achieve RTK Float (fix_type 5)
- Time to achieve RTK Fixed (fix_type 6)
- Final HDOP value
- Final accuracy (mm)
- Number of satellites
- Distance to base station (RTK baseline)

**Create Log Entry**:
Add results to session notes or create new file:
`RTK_CONVERGENCE_RESULTS_NOV02.md`

### Task 4.2: Performance Comparison Test

**Test Plan**:
1. **Test A - Without RTK**:
   - Disable RTCM forwarder temporarily
   - Send rover to waypoint
   - Measure final position error (expect 1-3m)

2. **Test B - With RTK**:
   - Re-enable RTCM forwarder
   - Wait for RTK Fixed
   - Send rover to same waypoint
   - Measure final position error (expect <10cm)

**Document Results**:
- Position error comparison
- Navigation smoothness
- Time to target
- Photos/videos of testing

---

## PRIORITY 5: SYSTEM IMPROVEMENTS (OPTIONAL)

### Task 5.1: Add Unified Bridge Signal Handlers

**Files to Modify**:
- `~/ros2_unified_bridge.py` on Jetson

**Implementation**:
```python
import signal
import sys

def signal_handler(sig, frame):
    print('Graceful shutdown requested...')
    # Cleanup ROS 2 subscriptions
    rclpy.shutdown()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)
```

**Benefits**:
- Faster service restarts (currently takes 90+ seconds)
- Cleaner shutdown process
- Better resource cleanup

### Task 5.2: Create RTK Status Widget

**Add to Dashboard**:
- Dedicated RTK status indicator
- Shows: Disconnected / Float / Fixed
- Color-coded: Red / Yellow / Green
- Displays baseline distance to base station
- Shows time since last correction

---

## KNOWN ISSUES

### 1. Unified Bridge Shutdown Delay
**Status**: Known, not critical
**Impact**: Service restarts take 90+ seconds
**Workaround**: Reboot Jetson if urgent restart needed
**Fix**: Add signal handlers (Priority 5, Task 5.1)

### 2. Dashboard Occasional Freeze
**Status**: Fixed this session, monitor for recurrence
**Impact**: Dashboard becomes unresponsive
**Fix**: Kill and restart dashboard process
**Root Cause**: Unknown, possibly old process state

---

## TESTING CHECKLIST

Before ending next session, verify:

- [ ] GPS achieved RTK Fixed status (fix_type 6)
- [ ] Documented convergence time and accuracy
- [ ] Tested rover navigation with RTK precision
- [ ] Dashboard correctly displays RTK status
- [ ] All systemd services running properly
- [ ] Session notes updated with test results

---

## QUICK REFERENCE

### System Status Commands

**Check GPS Status**:
```bash
curl http://100.91.191.47:5001/api/gps | python3 -m json.tool
```

**Monitor RTK Corrections**:
```bash
ssh jay@100.91.191.47
sudo journalctl -u jetson-rtcm-forwarder -f
```

**Check MAVROS2 RTK Topic**:
```bash
ssh jay@100.91.191.47
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
ros2 topic echo /mavros/mavros/gps1/rtk --once
```

**Restart Services**:
```bash
ssh jay@100.91.191.47
sudo systemctl restart jetson-rtcm-forwarder
sudo systemctl restart jetson-mavros2
sudo systemctl restart jetson-unified-bridge
```

### Network Configuration

- **Mobile RTK Module (Pi 5)**: 100.73.233.124
- **Jetson Orin Nano**: 100.91.191.47
- **RTK Base Station (RTKPi)**: 100.66.67.11
- **Gateway**: 192.168.254.254
- **Network**: 192.168.254.0/24

### File Locations

**Jetson**:
- MAVROS2 Launch: `~/mavros_cube_orange.launch.py`
- Unified Bridge: `~/ros2_unified_bridge.py`
- RTCM Forwarder: `~/rtcm_mqtt_forwarder.py`
- RTK Monitor: `~/monitor_rtk_fix.sh`
- GPS Acquisition: `~/monitor_gps_acquisition.sh`

**Development Machine**:
- Session Log: `~/Desktop/Mini Rover Development/SESSION_LOG_NOV01_2025.md`
- RTK Setup Guide: (deployed to Jetson `/tmp/`)
- RTK Troubleshooting: (deployed to Jetson `/tmp/`)

### Systemd Services

```bash
# All services on Jetson (100.91.191.47)
jetson-mavros2.service           # MAVROS2 for Cube Orange
jetson-unified-bridge.service    # HTTP API bridge (port 5001)
jetson-rtcm-forwarder.service    # MQTT to MAVROS2 RTK bridge
jetson-rplidar.service           # RPLidar A1 sensor
jetson-rover-server.service      # Legacy rover control (port 5000)
```

---

## SESSION GOALS

**Minimum Success**:
- Verify RTK convergence status (Float or Fixed)
- Document current GPS accuracy
- Confirm RTCM corrections still flowing

**Ideal Success**:
- Achieve RTK Fixed status (fix_type 6)
- Test rover navigation with <10cm accuracy
- Document convergence time and performance
- Create comparison between standard GPS and RTK

**Stretch Goals**:
- Implement unified bridge signal handlers
- Create dedicated RTK status widget
- Performance testing with multiple waypoint missions
- Video documentation of RTK accuracy

---

## IMPORTANT NOTES

### RTK Convergence Expectations

**Cold Start** (GPS off for >4 hours):
- Time to 3D fix: 30-90 seconds
- Time to RTK Float: 2-10 minutes
- Time to RTK Fixed: 5-30 minutes

**Warm Start** (GPS off for <4 hours):
- Time to 3D fix: 10-30 seconds
- Time to RTK Float: 1-5 minutes
- Time to RTK Fixed: 2-15 minutes

**Hot Start** (GPS just restarted):
- Time to 3D fix: 5-15 seconds
- Time to RTK Float: 30 seconds - 2 minutes
- Time to RTK Fixed: 1-5 minutes

### Factors Affecting Convergence

**Good Conditions** (faster convergence):
- Clear sky view (no obstructions)
- 15+ satellites visible
- Base station <10km away
- Strong RTCM correction signal
- No multipath (reflections)

**Poor Conditions** (slower convergence):
- Trees, buildings nearby
- <12 satellites visible
- Base station >20km away
- Weak/intermittent corrections
- Urban canyon, multipath

### GPS Fix Type Reference

| Fix Type | Name | Accuracy | Description |
|----------|------|----------|-------------|
| 0 | NO GPS | N/A | GPS not detected |
| 1 | NO FIX | N/A | GPS searching for satellites |
| 2 | 2D FIX | 10-20m | Lat/lon only, no altitude |
| 3 | 3D FIX | 2-5m | Standard GPS accuracy |
| 4 | DGPS | 1-3m | Differential GPS (basic corrections) |
| 5 | RTK FLOAT | 10-50cm | RTK corrections, converging |
| 6 | RTK FIXED | 2-5cm | Full RTK accuracy! 🎯 |

---

## SAFETY REMINDERS

1. **Test in Open Area**
   - Clear 30+ meter radius for RTK testing
   - No obstacles or hazards
   - Excellent GPS visibility required

2. **Hardware E-STOP Ready**
   - Keep within reach at all times
   - Test before rover movement
   - Use immediately if unexpected behavior

3. **RTK Testing Protocol**
   - Wait for RTK Fixed before navigation tests
   - Start with short distances (5m)
   - Gradually increase distance as confidence builds
   - Monitor GPS status continuously

4. **GPS Conditions**
   - Require 15+ satellites for RTK testing
   - HDOP must be <0.5 for RTK Fixed
   - Avoid testing near buildings or trees
   - Wait for stable RTK Fixed (not just momentary)

---

## SUCCESS METRICS

This session will be successful if:

1. **RTK Convergence Verified**
   - GPS achieves RTK Float (5) or Fixed (6) status
   - Documented convergence time
   - Confirmed <10cm accuracy

2. **System Stability**
   - All services running without crashes
   - RTCM corrections flowing continuously
   - Dashboard responsive and updating

3. **Documentation Complete**
   - Session results documented
   - Performance data recorded
   - Any issues noted for future sessions

4. **Optional - Rover Testing**
   - If RTK Fixed achieved quickly
   - Test navigation accuracy
   - Compare RTK vs non-RTK performance

---

**Estimated Session Time**: 1-3 hours
**Difficulty**: Low (mostly monitoring and testing)
**Prerequisites**: All systems from Nov 1 session running

**Ready to verify RTK convergence and achieve centimeter-level accuracy! 🛰️🎯**

---

*Created: November 1, 2025*
*For Session: November 2, 2025*
