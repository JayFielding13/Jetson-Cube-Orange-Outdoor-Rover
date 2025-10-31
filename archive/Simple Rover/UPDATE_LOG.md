# Simple Rover - Update Log

## Session: October 16, 2025 - Side Sensors & Navigation Improvements

### Overview
Added side-mounted ultrasonic sensors for wall detection and improved autonomous navigation to handle edge cases where the rover would get stuck near walls.

---

## Hardware Changes

### Side Sensors Added
- **Purpose**: Detect walls parallel to the rover to prevent scraping and improve navigation
- **Quantity**: 2× HC-SR04 ultrasonic sensors
- **Mounting**: Perpendicular to rover body (90° from forward direction)
  - Left side sensor: Detects walls on left side
  - Right side sensor: Detects walls on right side

### GPIO Pin Assignments
| Sensor | TRIG Pin | ECHO Pin | Physical Pins |
|--------|----------|----------|---------------|
| Left Side | GPIO 5 | GPIO 6 | 29 (TRIG), 31 (ECHO) |
| Right Side | GPIO 22 | GPIO 25 | 15 (TRIG), 22 (ECHO) |

### Voltage Dividers
- **Configuration**: 2× 2kΩ resistors in parallel (top) + 1× 2kΩ (bottom)
- **Ratio**: 1kΩ:2kΩ = 3.33V output from 5V input ✓
- **Applied to**: GPIO 6 (left side ECHO) and GPIO 25 (right side ECHO)

---

## Software Changes

### 1. Side Sensor Integration (`rover_controller_with_lora.py`)

#### New Parameters Added
```python
# GPIO Pin Assignments - Side Sensors
LEFT_SIDE_TRIG = 5
LEFT_SIDE_ECHO = 6
RIGHT_SIDE_TRIG = 22
RIGHT_SIDE_ECHO = 25

# Side Collision Avoidance Parameters
SIDE_WARNING_THRESHOLD = 10  # Steer away if side object within this distance (cm)
SIDE_INFLUENCE_GAIN = 0.5    # How much side sensors influence steering (0.0-1.0)
```

#### New Methods Added
1. **`calculate_side_steering_influence(left_side_dist, right_side_dist)`**
   - Calculates steering bias based on side sensor proximity to walls
   - Returns value from -1.0 (steer left) to +1.0 (steer right)
   - Active when side obstacles detected within 10cm

2. **Updated `read_sensors()`**
   - Now returns 4 values: `left_dist, right_dist, left_side_dist, right_side_dist`
   - All sensors read in each navigation cycle

#### Navigation Logic Updates
- Side sensor influence integrated into main autonomous loop
- Gently steers away from walls when detected within 10cm
- Does NOT cause emergency stops - only influences steering direction
- Side sensors override forward command to turn when wall proximity > 30% threshold

### 2. Navigation Threshold Tuning

#### Critical Bug Fix - Emergency Stop Oscillation
**Problem**: Rover would get stuck when parallel to wall at ~15cm distance
- Sensor readings fluctuating between 14.8cm and 15.2cm
- Emergency stop threshold = 15cm
- Caused oscillation between STOP and TURN commands

**Solution**: Reduced CRITICAL_THRESHOLD from 15cm → 10cm
```python
CRITICAL_THRESHOLD = 10   # Emergency stop threshold (cm)  [was 15]
```

#### Sensor Fusion Improvement
**Problem**: Rover incorrectly detecting "center obstacles" when approaching walls at 70-80° angles
- Both front sensors see wall at similar but not identical distances (e.g., 18cm and 22cm)
- Sensor fusion would treat this as center obstacle and trigger corner trap logic

**Solution**: Tightened SENSOR_FUSION_DIFF from 15cm → 8cm
```python
SENSOR_FUSION_DIFF = 8    # If sensors within this range, assume center blocked (cm)  [was 15]
```

**Result**:
- Sensor fusion now only triggers when readings are within 8cm (truly perpendicular approach)
- Angled approaches (70-80°) now handled by bendy ruler algorithm instead of corner escape
- Rover can smoothly turn away instead of stopping and rotating

---

## Testing & Verification

### Tests Performed
1. **All Sensors Test** (`test_all_sensors.py`)
   - Verified all 4 sensors reading distances correctly
   - Initial issue: Right side sensor timing out (999cm) - fixed loose wire
   - Final result: All 4 sensors working correctly

2. **LoRa Communication Test**
   - Verified transmitter-receiver link working
   - Confirmed AUTONOMOUS/STOP commands being transmitted
   - **Critical USB Port Issue**: Heltec data transmission unreliable on certain USB ports
   - **Solution**: Changed Heltec to different USB port on Raspberry Pi

3. **Autonomous Navigation Tests**
   - Tested wall following behavior
   - Tested corner escape logic
   - Tested side sensor influence on steering
   - Verified no emergency stops when parallel to walls at 10-20cm distance

### Known Issues & Solutions

#### Issue 1: USB Port Data Transmission
**Symptom**: Heltec OLED shows mode changes, but Pi receives no serial data
**Root Cause**: Some USB ports provide power but have faulty data lines
**Solution**: Switch Heltec to different USB port on Raspberry Pi
**Prevention**: Mark reliable USB ports for future use

#### Issue 2: Rover "Stuck" When Parallel to Walls
**Symptom**: Rover stops moving when at ~15cm from wall, front sensor and side sensor both detecting wall
**Root Cause**: Emergency stop threshold (15cm) too close to typical wall-following distance
**Solution**: Reduced CRITICAL_THRESHOLD to 10cm, giving 5cm buffer for bendy ruler to operate
**Result**: Rover now smoothly turns away from walls instead of stopping

#### Issue 3: False Corner Detection at Angles
**Symptom**: Rover incorrectly thinks it's trapped when approaching wall at 70-80° angle
**Root Cause**: Sensor fusion threshold (15cm) too wide, fusing readings that should remain separate
**Solution**: Reduced SENSOR_FUSION_DIFF to 8cm for more precise center obstacle detection
**Result**: Rover recognizes room to turn and navigates away smoothly

---

## Current System Parameters

### Detection Thresholds
```python
OBSTACLE_THRESHOLD = 50        # Start steering away at this distance (cm)
CRITICAL_THRESHOLD = 10        # Emergency stop threshold (cm) ✓ Updated
CORNER_TRAP_THRESHOLD = 20     # Both sensors < this = trapped in corner (cm)
CLEAR_PATH_THRESHOLD = 50      # Both sensors > this = clear path found (cm)
SIDE_WARNING_THRESHOLD = 10    # Steer away from side obstacles (cm) ✓ New
```

### Sensor Fusion & Control
```python
SENSOR_FUSION_DIFF = 8         # Sensor agreement threshold (cm) ✓ Updated
SIDE_INFLUENCE_GAIN = 0.5      # Side sensor steering influence ✓ New
TURN_GAIN = 0.8                # Steering aggressiveness
SPEED_GAIN = 0.6               # Speed reduction near obstacles
```

### Speed Settings
```python
BASE_SPEED = 100               # Base forward speed (0-255)
MAX_SPEED = 150                # Maximum forward speed
MIN_SPEED = 60                 # Minimum speed when avoiding obstacles
ESCAPE_TURN_SPEED = 100        # Speed for corner escape rotation
```

---

## Files Modified

### Main Code Files
1. **`/home/jay/Simple Rover/rover_controller_with_lora.py`**
   - Added side sensor GPIO definitions
   - Added side sensor objects initialization
   - Updated `read_sensors()` method to return 4 values
   - Added `calculate_side_steering_influence()` method
   - Integrated side sensor logic into main autonomous loop
   - Updated threshold constants (CRITICAL_THRESHOLD, SENSOR_FUSION_DIFF)

### Documentation Files
2. **`/home/jay/Desktop/Mini Rover Development/Simple Rover/WIRING_REFERENCE.md`**
   - Added side sensor wiring section
   - Documented GPIO pin assignments
   - Updated voltage divider information
   - Added wire checklist for side sensors

3. **`/home/jay/Desktop/Mini Rover Development/Simple Rover/SIDE_SENSOR_WIRING.md`**
   - Created detailed side sensor installation guide
   - Voltage divider circuit diagrams
   - Troubleshooting section

### Test Scripts
4. **`/home/jay/Simple Rover/test_all_sensors.py`**
   - Created comprehensive 4-sensor test script
   - Displays real-time readings from all sensors
   - Used to verify hardware installation

---

## System Service

### Systemd Service Status
- **Service Name**: `simple-rover.service`
- **Status**: Active and enabled
- **Auto-start**: Yes (on boot)
- **Last Restart**: October 13, 2025 23:08:56 PDT
- **Current Code**: Includes all updates (side sensors + threshold fixes)

### Service Management
```bash
# Restart service after code changes
sudo systemctl restart simple-rover.service

# Check service status
sudo systemctl status simple-rover.service

# View live logs
journalctl -u simple-rover.service -f
```

---

## Navigation Behavior Summary

### Normal Operation (AUTONOMOUS mode)
1. **Forward Movement**: Both front sensors > 50cm → Full speed forward
2. **Bendy Ruler Steering**: Obstacle 20-50cm → Slow down and steer away
3. **Side Wall Avoidance**: Wall < 10cm on side → Gentle steering correction
4. **Emergency Stop**: Obstacle < 10cm → Full stop
5. **Corner Escape**: Both sensors < 20cm → Rotate until clear path found

### Sensor Fusion Logic
- **Perpendicular Wall**: Left=25cm, Right=27cm (diff=2cm) → Fusion activated → Both read 25cm
- **Angled Approach**: Left=18cm, Right=26cm (diff=8cm) → No fusion → Steer toward open side
- **Clear One Side**: Left=15cm, Right=100cm (diff=85cm) → No fusion → Turn right

### Edge Case Handling
1. **Parallel to Wall (10-20cm)**: Side + front sensors detect wall → Smooth turn away (no stop)
2. **70-80° Angle to Wall**: No sensor fusion → Bendy ruler steers appropriately
3. **Tight Corner**: Both front sensors < 20cm → Corner escape rotation
4. **Very Close Obstacle**: Any sensor < 10cm → Emergency stop

---

## Future Improvements

### Potential Enhancements
1. **USB Port Reliability**
   - Consider udev rules to create stable device names based on USB IDs
   - Create `/dev/ttyArduino` and `/dev/ttyHeltec` symlinks
   - Prevents port assignment issues after reboots

2. **Hysteresis on Emergency Stop**
   - Add hysteresis to prevent oscillation at threshold boundary
   - Require distance to increase to 12cm before resuming from 10cm emergency stop

3. **Adaptive Speed Control**
   - Slow down more when both front AND side sensors detect obstacles
   - Increase confidence in tight navigation

4. **Corner Escape Improvement**
   - Remember last successful escape direction
   - Try opposite direction if first attempt fails

---

## Troubleshooting Guide

### Rover Won't Move in Autonomous Mode
1. Check LoRa transmitter battery
2. Verify both Heltec OLEDs show "AUTONOMOUS"
3. Check Heltec USB port - try different port if needed
4. Verify rover service is running: `systemctl status simple-rover.service`

### Rover Gets Stuck Near Walls
1. Verify CRITICAL_THRESHOLD = 10 (not 15)
2. Verify SENSOR_FUSION_DIFF = 8 (not 15)
3. Check sensor readings with `test_all_sensors.py`
4. Ensure side sensors mounted perpendicular to rover body

### Side Sensors Not Working
1. Verify voltage dividers built correctly (1kΩ:2kΩ ratio)
2. Test with `test_all_sensors.py`
3. Check for loose TRIG/ECHO connections
4. Verify 5V power and ground connections

### USB Port Issues
1. Try different USB port on Raspberry Pi
2. Check `ls -la /dev/ttyUSB*` to see detected devices
3. Use `udevadm info /dev/ttyUSB0` to identify device type
4. Arduino = CH340 chip, Heltec = CP2102 chip

---

## Summary

This update successfully added side-mounted ultrasonic sensors for improved wall detection and navigation. The rover now:
- ✅ Detects walls on both sides and steers away gently
- ✅ Handles parallel wall approaches without getting stuck
- ✅ Navigates angled approaches (70-80°) smoothly
- ✅ Correctly distinguishes corner traps from angled obstacles
- ✅ All 4 sensors integrated and working correctly

The navigation threshold tuning resolved the "stuck" behavior by:
- Reducing emergency stop threshold (15cm → 10cm) for better flow
- Tightening sensor fusion (15cm → 8cm) for accurate obstacle detection
- Providing smooth wall-following capability with side sensor influence

System is stable, auto-starts on boot, and performs autonomous navigation reliably.
