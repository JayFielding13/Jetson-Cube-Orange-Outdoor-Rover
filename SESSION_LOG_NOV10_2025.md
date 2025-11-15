# Session Log - November 10, 2025
## ESP32 Ultrasonic Firmware Debugging & Arduino Migration

**Date:** November 10, 2025
**Project:** Jetson Cube Orange Outdoor Rover
**Focus:** ESP32 GPIO Troubleshooting and Firmware Switch from CircuitPython to Arduino

---

## Session Overview

Successfully resolved critical GPIO pin conflicts and migrated from CircuitPython to Arduino firmware for improved GPIO compatibility. All 6 ultrasonic sensor LEDs now functioning, indicating proper trigger signal generation. Completed full system power-up test from Jetson Nano.

---

## Problems Identified

### 1. GPIO 9/10 Flash Pin Conflict (from Nov 8 session)
**Issue:** ESP32 entered continuous boot loop when GPIO 9/10 were used
**Root Cause:** GPIO 9/10 are connected to internal flash memory (SD_DATA2, SD_DATA3)
**Error:** `rst:0x8 (TG1WDT_SYS_RESET), boot:0x13 (SPI_FAST_FLASH_BOOT)`
**Impact:** ESP32 watchdog timer detected flash interference and reset system

### 2. Side Right Sensor Not Responding (Primary Issue)
**Symptoms:**
- All 5 sensor boards showed blue LED activity (receiving trigger signals)
- Side Right sensor board had power but NO blue LED
- Board verified functional by testing with adjacent working connector
- 5V power confirmed present at Side Right connector

**Initial Hypothesis:** Hardware wiring issue or pin conflicts

---

## Troubleshooting Journey

### Phase 1: CircuitPython GPIO Migration Attempts

After discovering GPIO 9/10 conflict, attempted multiple GPIO reassignments:

| Attempt | TRIG Pin | ECHO Pin | Result | Notes |
|---------|----------|----------|--------|-------|
| 1 | GPIO 9 | GPIO 10 | ❌ Boot loop | Flash pins - don't use |
| 2 | GPIO 16 | GPIO 17 | ❌ No LED | No trigger signals reaching sensor |
| 3 | GPIO 18 | GPIO 19 | ❌ No LED | Likely SPI flash conflict |
| 4 | GPIO 32 | GPIO 33 | ❌ No LED | Pins didn't work with CircuitPython |
| 5 | GPIO 33 | GPIO 32 | ❌ No LED | Swapped TRIG/ECHO to test wiring |

**Key Observations:**
- Bench tested with single physical sensor moved between positions
- Side Left sensor (GPIO 14/4) worked perfectly, proving approach was sound
- Multiple GPIO attempts all failed despite different pin locations
- CircuitPython upload process via REPL was unreliable
- Test code didn't execute consistently; old code kept running

### Phase 2: Root Cause Analysis

**Realization:** Problem was software-based, not hardware:
- Sensor boards receive trigger signals → blue LED blinks
- Side Right board had power but never showed LED
- This indicated GPIO initialization or CircuitPython GPIO availability issue
- User noted: "This seems strange that none of the GPIO pins would work"

### Phase 3: Solution - Switch to Arduino Firmware

**Decision:** Migrate to Arduino firmware for better GPIO support

**Why Arduino:**
- More mature GPIO library with better ESP32 pin support
- Reliable upload via Arduino IDE (vs unreliable CircuitPython REPL)
- Same JSON output format maintained for compatibility
- GPIO 32/33 are fully functional ADC pins on ESP32

---

## Final Pin Configuration

### Complete Pin Assignments (All Sensors)

| Sensor | Angle | TRIG Pin | ECHO Pin (via Level Shifter) | Physical Location |
|--------|-------|----------|-------------------------------|-------------------|
| Front | 0° | GPIO 15 | GPIO 2 (LV1) | L16, L15 |
| Corner Left | -45° | GPIO 13 | GPIO 12 (LV6) | R15, R13 |
| Corner Right | +45° | GPIO 5 | GPIO 27 (LV3) | L10, R11 |
| Side Left | -90° | GPIO 14 | GPIO 4 (LV4) | R12, L13 |
| **Side Right** | **+90°** | **GPIO 32** | **GPIO 33 (LV5)** | **R6, R7** ✅ **FIXED** |
| Rear | 180° | GPIO 25 | GPIO 26 (LV2) | R9, R10 |

**Critical Change:** Side Right moved to GPIO 32/33 with Arduino firmware

### GPIO Pins to Avoid on ESP32

⚠️ **Never Use These Pins:**
- **GPIO 6, 7, 8, 9, 10, 11:** Flash memory interface (SD_CLK, SD_DATA0-3, SD_CMD)
- **GPIO 0:** Boot mode selection (pull low for programming)
- **GPIO 2:** Must be floating or high during boot
- **GPIO 12:** MTDI - Boot voltage selector
- **GPIO 15:** MTDO - Boot debug output

✅ **Safe General-Purpose GPIO:**
- GPIO 4, 5, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33

---

## Implementation

### Firmware Files Updated

All three firmware variants updated with GPIO 32/33:

1. **`esp32_ultrasonic_raw/esp32_ultrasonic_raw.ino`** (Arduino - PRIMARY)
   - Lines 64-65: `SIDE_RIGHT_TRIG_PIN 32`, `SIDE_RIGHT_ECHO_PIN 33`
   - Successfully uploaded via Arduino IDE
   - 10 Hz update rate (100ms interval)
   - JSON output format for Jetson consumption

2. **`esp32_ultrasonic/src/main.cpp`** (PlatformIO Arduino)
   - Same pin configuration as .ino variant
   - Alternative build system option

3. **`circuitpython_ultrasonic/code.py`** (CircuitPython - DEPRECATED)
   - Updated for reference but CircuitPython proven unreliable
   - GPIO 32/33 didn't work with CircuitPython on this ESP32 variant
   - Kept for documentation purposes

### Documentation Updated

✅ All documentation reflects GPIO 32/33 configuration:
- [PIN_CHANGES_SUMMARY.md](PIN_CHANGES_SUMMARY.md)
- [SIDE_RIGHT_REWIRE_NOTE.md](SIDE_RIGHT_REWIRE_NOTE.md)
- [ESP32_ULTRASONIC_WIRING_GUIDE.md](ESP32_ULTRASONIC_WIRING_GUIDE.md)
- [ESP32_PINOUT_DIAGRAM.md](ESP32_PINOUT_DIAGRAM.md)

### Arduino IDE Upload Process

**Steps followed:**
1. Opened Arduino IDE
2. File → Open → `esp32_ultrasonic_raw/esp32_ultrasonic_raw.ino`
3. Tools → Board → ESP32 Dev Module
4. Tools → Port → /dev/ttyUSB0
5. Tools → Upload Speed → 115200
6. Clicked Upload (Ctrl+U)
7. Arduino IDE automatically erased CircuitPython and installed Arduino bootloader

**Upload Output:**
```
Connecting........_____.....
Writing at 0x00010000... (100%)
Wrote XXXXX bytes
Hard resetting via RTS pin...
```

---

## Testing & Verification

### Hardware Validation

✅ **All 6 Sensor Board LEDs Blinking:**
- Front: Blue LED active ✓
- Corner Left: Blue LED active ✓
- Corner Right: Blue LED active ✓
- Side Left: Blue LED active ✓
- **Side Right: Blue LED active ✓** ← **FIXED!**
- Rear: Blue LED active ✓

**LED Activity Indicates:**
- ESP32 GPIO pins properly initialized
- Trigger signals being generated at 10 Hz
- All sensor boards receiving signals via TRIG pins
- Power distribution working correctly

### Serial Output Verification

**JSON Data Format (at 115200 baud):**
```json
{"timestamp":12345,"sensors":[
  {"name":"front","angle":0,"distance":1.234,"valid":true},
  {"name":"corner_left","angle":-45,"distance":6.000,"valid":false},
  {"name":"corner_right","angle":45,"distance":6.000,"valid":false},
  {"name":"side_left","angle":-90,"distance":6.000,"valid":false},
  {"name":"side_right","angle":90,"distance":0.872,"valid":true},
  {"name":"rear","angle":180,"distance":6.000,"valid":false}
]}
```

**Key Findings:**
- Timestamp in milliseconds
- Side Right sensor showing `"valid":true` with actual distance readings
- Bench test with single sensor confirmed working
- Sensors without physical hardware show `"valid":false` with 6.000m (timeout)

### Full System Power-Up Test

✅ **Jetson Nano System Integration:**
- Powered up complete rover system from Jetson Nano
- ESP32 received 5V via USB from Jetson
- All sensors powered from external 5V rail
- Serial communication established at /dev/ttyUSB0
- JSON data streaming successfully
- No power-related issues observed

---

## Root Cause Summary

**Problem:** CircuitPython GPIO limitations on this ESP32 DevKit variant

**Evidence:**
1. Multiple different GPIO pins failed (16/17, 18/19, 32/33)
2. Side Left (GPIO 14/4) worked perfectly with same approach
3. Unreliable CircuitPython upload via REPL
4. Arduino firmware immediately worked with GPIO 32/33

**Conclusion:**
- CircuitPython for ESP32 has limited/unstable GPIO support on certain pins
- Arduino framework has mature, reliable GPIO implementation
- Some GPIO pins may not be properly initialized by CircuitPython on this board

---

## Files Created/Modified This Session

### New Documentation
- `SIDE_RIGHT_STATUS.md` - Status tracking document for Side Right sensor
- `SESSION_LOG_NOV10_2025.md` - This session log

### Modified Firmware
- `esp32_ultrasonic_raw/esp32_ultrasonic_raw.ino` - Updated GPIO 32/33, uploaded successfully
- `circuitpython_ultrasonic/code.py` - Updated but deprecated

### Updated Documentation
- `PIN_CHANGES_SUMMARY.md` - Documented GPIO 32/33 final configuration
- `SIDE_RIGHT_REWIRE_NOTE.md` - Updated wiring instructions
- Comments in all three firmware variants

---

## Current System Status

### ✅ Working
- ESP32 Arduino firmware running at 10 Hz
- All 6 sensor boards receiving trigger signals (LEDs blinking)
- JSON data streaming via USB serial at 115200 baud
- Side Right sensor (GPIO 32/33) operational with valid readings
- Full system powers up from Jetson Nano successfully
- Level shifter protecting all ECHO pins (3.3V ← 5V)

### ⏳ Next Steps (Testing Phase)

1. **Complete Sensor Array Testing:**
   - Connect all 6 physical ultrasonic sensors
   - Run GUI visualization tool: `python3 ultrasonic_gui.py`
   - Verify each sensor provides valid distance readings
   - Test 360° coverage with obstacles at different angles
   - Validate 6m maximum range and 2cm minimum range

2. **ROS2 Integration:**
   - Test `ultrasonic_ros2_publisher.py` with all sensors
   - Verify sensor_msgs/Range topics published correctly
   - Check TF frames for each sensor position
   - Integrate with navigation stack

3. **Physical Rover Deployment:**
   - Mount all sensors on rover chassis
   - Route wiring cleanly through frame
   - Secure ESP32 and level shifter
   - Test with rover motion (vibration testing)

---

## Lessons Learned

### Technical Insights

1. **ESP32 Flash Pins Are Off-Limits:**
   - GPIO 6-11 connected to internal flash - never use as GPIO
   - Violating this causes immediate boot loops
   - Always consult pinout diagrams before assignment

2. **CircuitPython Limitations:**
   - Not all GPIO pins work reliably with CircuitPython on ESP32
   - REPL upload process can be unreliable
   - Arduino framework more mature for ESP32 GPIO
   - Consider Arduino first for GPIO-heavy projects

3. **Blue LEDs Are Excellent Diagnostic:**
   - AJ-SR04M boards have onboard blue LED when receiving triggers
   - Immediate visual feedback of GPIO signal generation
   - Helped identify software vs hardware issues
   - No oscilloscope needed for basic debugging

4. **Systematic Troubleshooting Approach:**
   - Isolated hardware (sensor board tested with working connector)
   - Verified power delivery (5V confirmed present)
   - Tested multiple GPIO pins to rule out pin-specific issues
   - Recognized pattern (all new pins failing) suggested software root cause
   - Switch platforms when underlying issue identified

### Best Practices

1. **Always check GPIO restrictions before pin assignment**
2. **Use visual indicators (LEDs) for GPIO debugging**
3. **Document pin changes immediately with reasons**
4. **Test hardware isolation before software debugging**
5. **Consider firmware platform maturity for production systems**
6. **Keep alternative firmware versions for backup**

---

## Next Session Priorities

1. **Hardware:** Connect all 6 physical ultrasonic sensors to their boards
2. **Testing:** Run `ultrasonic_gui.py` to visualize 360° coverage
3. **Validation:** Verify distance accuracy across full 0.02m - 6m range
4. **Integration:** Test ROS2 publisher node with full sensor array
5. **Documentation:** Update DEPLOYMENT_CHECKLIST.md with Arduino upload steps

---

## Hardware Configuration Summary

### ESP32 Firmware
- **Type:** Arduino (.ino sketch)
- **IDE:** Arduino IDE 2.x
- **Board:** ESP32 Dev Module
- **Upload:** USB serial at 115200 baud
- **Update Rate:** 10 Hz (100ms interval)
- **Output:** JSON via Serial

### Sensor Configuration
- **Model:** AJ-SR04M (waterproof, IP67)
- **Quantity:** 6 sensors for 360° coverage
- **Range:** 0.02m - 6m (adjustable via timeout)
- **Power:** 5V from external rail
- **Signal:** 5V TRIG, 5V ECHO (level-shifted to 3.3V)

### Power Distribution
- **ESP32:** 5V via USB from Jetson Nano
- **Sensors:** 5V from external regulated power supply
- **Level Shifter:** 5V and 3.3V rails (HV/LV)
- **Common Ground:** All components share common ground

---

## Success Metrics

✅ **Session Goals Achieved:**
- [x] Identified root cause of Side Right sensor failure
- [x] Resolved GPIO pin conflicts (GPIO 9/10 → GPIO 32/33)
- [x] Successfully migrated from CircuitPython to Arduino firmware
- [x] Verified all 6 sensor boards receiving trigger signals
- [x] Confirmed Side Right sensor providing valid distance readings
- [x] Completed full system power-up test from Jetson Nano
- [x] Updated all documentation with final pin configuration

---

## Appendix: GPIO Testing Log

**Side Right Sensor - GPIO Attempt History:**

| Date | GPIO Pins | Firmware | LED Status | Result |
|------|-----------|----------|------------|--------|
| Nov 8 | 9, 10 | CircuitPython | N/A | Boot loop (flash pins) |
| Nov 10 | 16, 17 | CircuitPython | No LED | No trigger signals |
| Nov 10 | 18, 19 | CircuitPython | No LED | No trigger signals |
| Nov 10 | 32, 33 | CircuitPython | No LED | No trigger signals |
| Nov 10 | 33, 32 | CircuitPython | No LED | Swapped TRIG/ECHO test |
| Nov 10 | **32, 33** | **Arduino** | **✅ LED Blinking** | **✅ WORKING** |

**Conclusion:** Arduino firmware immediately resolved all GPIO issues.

---

**End of Session Log - November 10, 2025**

**Status:** ESP32 ultrasonic sensor array firmware operational and ready for full 6-sensor deployment testing.

**Next Session:** Full sensor array GUI testing and ROS2 integration validation.
