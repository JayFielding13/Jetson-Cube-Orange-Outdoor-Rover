# Quick Start: Ultrasonic Sensor Testing

**Your ESP32 is ready to go!** Arduino firmware is installed and running.

---

## Launch the Graphical Debugger

```bash
cd "/home/jay/Desktop/Mini Rover Development/Jetson Cube Orange Outdoor Rover"
python3 ultrasonic_gui.py
```

**What you'll see:**
- **1400x800 window** with professional radar display
- **Left side:** Real-time 360° radar showing all 6 sensors
- **Right side:** Sensor status table, distance graphs, statistics
- **Bottom:** Control instructions

**Current status (no sensors wired):**
- All sensors show "NO DATA" or "NO OBJECT"
- Distances: 6.000m (max range)
- Valid: false (no echo received)

---

## GUI Controls

| Key | Action |
|-----|--------|
| `SPACE` | Pause/Resume updates |
| `L` | Start/Stop data logging to CSV |
| `R` | Reset statistics |
| `S` | Save screenshot |
| `ESC` | Exit |

---

## Testing Workflow

### Without Sensors (Current State)

The GUI works right now even without sensors connected:

1. **Launch GUI** - You'll see the radar with rover in center
2. **Check "Sensor Status"** - All show "NO DATA" (expected)
3. **Watch "Distance History"** - Flat lines at 6m (expected)
4. **Monitor Statistics** - Should see ~10 Hz update rate

This confirms:
- ✓ ESP32 communicating via USB
- ✓ JSON data streaming correctly
- ✓ Firmware running at 10 Hz
- ✓ GUI receiving and parsing data

### With Sensors Wired

As you wire each sensor (follow `ESP32_ULTRASONIC_WIRING_GUIDE.md`):

1. **Connect sensor power and signal wires**
2. **Watch the GUI** - That sensor's status should change:
   - "NO DATA" → "NO OBJECT" (if nothing in front)
   - "NO DATA" → "CLEAR" (if object > 1.5m away)
   - "NO DATA" → "NEAR" (if object 0.5-1.5m)
   - "NO DATA" → "⚠ VERY CLOSE" (if object < 0.5m)

3. **Wave hand in front of sensor:**
   - Radar ray should shrink
   - Distance number should drop
   - Status should show "⚠ VERY CLOSE"
   - Distance graph should spike

4. **Verify correct sensor responds:**
   - Wave at Front → GREEN ray (0° / top)
   - Wave at Corner Right → BLUE ray (45° / top-right)
   - Wave at Side Right → YELLOW ray (90° / right)
   - Wave at Rear → RED ray (180° / bottom)
   - Wave at Side Left → ORANGE ray (-90° / left)
   - Wave at Corner Left → PURPLE ray (-45° / top-left)

---

## Data Logging

To record sensor data for analysis:

1. **Press `L`** in GUI
2. Console shows: `Logging started: ultrasonic_log_YYYYMMDD_HHMMSS.csv`
3. **Test sensors** (wave hands, move obstacles, etc.)
4. **Press `L`** again to stop
5. **Open CSV** in spreadsheet or analyze with Python

**CSV Format:**
```csv
timestamp,sensor,distance,valid
1699565123.456,front,2.345,True
1699565123.556,corner_left,1.234,True
```

**Use cases:**
- Validate sensor accuracy
- Check for noise/jitter
- Verify update rates
- Bug reports
- Calibration

---

## Troubleshooting

### "Could not connect to /dev/ttyUSB0"

```bash
# Check if ESP32 is connected
ls -la /dev/ttyUSB*

# Should see: /dev/ttyUSB0

# Check permissions
sudo usermod -a -G dialout $USER
# Then logout and login again
```

### GUI Shows "NO DATA" Forever

```bash
# Verify ESP32 is sending data
python3 -c "
import serial
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
for _ in range(5):
    print(ser.readline().decode('utf-8', errors='ignore'))
ser.close()
"

# Should see JSON like:
# {"timestamp":12607,"sensors":[...]}
```

If no JSON appears:
1. Arduino firmware may not be uploaded
2. Wrong serial port (try /dev/ttyUSB1)
3. ESP32 in boot loop (check with `screen /dev/ttyUSB0 115200`)

### Sensor Shows Wrong Angle

Check your wiring matches the pin assignments:

| Sensor | Expected Angle | TRIG Pin | ECHO Pin |
|--------|---------------|----------|----------|
| Front | 0° (top) | GPIO 15 | GPIO 2 |
| Corner Right | 45° | GPIO 5 | GPIO 27 |
| Side Right | 90° (right) | GPIO 16 | GPIO 17 |
| Rear | 180° (bottom) | GPIO 25 | GPIO 26 |
| Side Left | -90° (left) | GPIO 14 | GPIO 4 |
| Corner Left | -45° | GPIO 13 | GPIO 12 |

### Distance Readings Erratic

1. **Check 5V power supply** - Should be stable 4.8-5.2V
2. **Verify level shifter** - ECHO pins must go through level shifter
3. **Reduce electrical noise** - Keep sensor wires away from motors
4. **Check for cross-talk** - Sensors too close together (>30cm apart)

---

## Advanced: Debug Mode

For more detailed output, enable DEBUG mode:

1. Edit firmware: `esp32_ultrasonic/src/main.cpp`
2. Change line 29: `#define DEBUG_MODE true`
3. Re-upload: `platformio run --target upload`
4. Monitor serial: `screen /dev/ttyUSB0 115200`

You'll see human-readable output:
```
----------------------------------------
Time: 12.6 s

  front           (  +0°): 2.34 m
  corner_left     ( -45°): TIMEOUT / OUT OF RANGE
  corner_right    ( +45°): 1.56 m
  ...
```

**Don't forget to set `DEBUG_MODE false` when done testing!**

---

## What's Next

### Current Status:
- ✅ ESP32 firmware installed (Arduino)
- ✅ GUI debugger ready
- ✅ CircuitPython firmware available (experimental)
- ⏳ Sensors need to be wired

### Next Steps:

1. **Wire sensors** following `ESP32_ULTRASONIC_WIRING_GUIDE.md`
   - Start with Front sensor (easiest to test)
   - Add sensors one at a time
   - Verify each before moving to next

2. **Test with GUI**
   - Launch `python3 ultrasonic_gui.py`
   - Verify all sensors respond
   - Check angles are correct
   - Log data for validation

3. **Deploy to Jetson** (when all sensors working)
   - Copy `ultrasonic_bridge.py` to Jetson
   - Set up systemd service
   - Integrate with ROS2
   - See `ULTRASONIC_SETUP_GUIDE.md` Phase 3

---

## Files Overview

| File | Purpose | Status |
|------|---------|--------|
| **ultrasonic_gui.py** | Graphical debugging tool (PyGame) | ✅ Ready |
| ultrasonic_visualizer.py | Terminal ASCII radar | ✅ Ready |
| ultrasonic_monitor.py | Raw JSON monitor | ✅ Ready |
| **esp32_ultrasonic/** | Arduino firmware (PlatformIO) | ✅ Uploaded |
| circuitpython_ultrasonic/ | CircuitPython firmware | ✅ Available |
| ESP32_ULTRASONIC_WIRING_GUIDE.md | How to wire sensors | 📖 Reference |
| CIRCUITPYTHON_GUIDE.md | CircuitPython details | 📖 Reference |

**Recommended:** Use `ultrasonic_gui.py` for all sensor testing!

---

## Summary

**You're all set!** Your ESP32 is running Arduino firmware and streaming sensor data at 10 Hz. The graphical debugger is ready to use.

**To start testing:**
```bash
cd "/home/jay/Desktop/Mini Rover Development/Jetson Cube Orange Outdoor Rover"
python3 ultrasonic_gui.py
```

**Need help?** Check these guides:
- Wiring: `ESP32_ULTRASONIC_WIRING_GUIDE.md`
- CircuitPython: `CIRCUITPYTHON_GUIDE.md`
- Deployment: `ULTRASONIC_SETUP_GUIDE.md`

---

**Created:** November 9, 2025
**ESP32 Firmware:** Arduino (production mode)
**Update Rate:** 10 Hz
**Status:** Ready for sensor wiring and testing
