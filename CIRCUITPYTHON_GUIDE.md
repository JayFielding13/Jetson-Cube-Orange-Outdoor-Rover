# CircuitPython Ultrasonic Sensor Guide

**Date:** November 9, 2025
**Purpose:** Use CircuitPython firmware for ultrasonic sensors as alternative to Arduino

---

## What You Have

### 1. Graphical Debugging Tool (RECOMMENDED)

**File:** `ultrasonic_gui.py`
**Description:** Professional PyGame-based GUI for sensor testing

**Features:**
- 360° radar visualization
- Real-time distance graphs
- Individual sensor status
- Performance metrics (Hz, message count)
- Data logging to CSV
- Screenshot capability
- Pause/resume controls

**Usage:**
```bash
cd "Jetson Cube Orange Outdoor Rover"
python3 ultrasonic_gui.py

# Or specify custom serial port
python3 ultrasonic_gui.py /dev/ttyUSB1
```

**Controls:**
- `ESC` - Exit application
- `SPACE` - Pause/Resume data updates
- `L` - Toggle data logging (saves to CSV)
- `R` - Reset statistics
- `S` - Save screenshot

**Works with:** Both Arduino and CircuitPython firmware

---

### 2. CircuitPython Firmware

**File:** `circuitpython_ultrasonic/code.py`
**Description:** Python-based firmware for ESP32 (alternative to Arduino)

**Advantages:**
- Edit code live (no compile/upload)
- Pure Python (easier to modify)
- Interactive REPL for debugging
- Same JSON output as Arduino version

**Disadvantages:**
- Slower than Arduino (~5-8 Hz vs 10 Hz)
- No USB mass storage on regular ESP32 (need ESP32-S3)
- Must upload via serial REPL

---

## How to Use CircuitPython Firmware

### Current ESP32 (Standard, Not S3)

CircuitPython is already installed! But since standard ESP32 doesn't have native USB, you need to upload code via REPL:

#### Option 1: Manual REPL Upload (Quick Test)

```bash
# Connect to REPL
screen /dev/ttyUSB0 115200

# Press Ctrl+C to stop current code
# Paste the entire code.py contents
# Press Ctrl+D to soft reboot and run

# Exit screen: Ctrl+A, then K, then Y
```

#### Option 2: Python Upload Script (Recommended)

```bash
python3 -c "
import serial
import time

# Read CircuitPython code
with open('circuitpython_ultrasonic/code.py', 'r') as f:
    code = f.read()

# Connect to ESP32
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(0.5)

# Stop running code
ser.write(b'\x03')  # Ctrl+C
time.sleep(0.3)

# Enter paste mode
ser.write(b'\x05')  # Ctrl+E
time.sleep(0.2)

# Send code
ser.write(code.encode())
time.sleep(0.1)

# Execute
ser.write(b'\x04')  # Ctrl+D

ser.close()
print('Code uploaded to CircuitPython!')
"
```

### Future ESP32-S3 (When It Arrives)

The ESP32-S3 you ordered will be much easier:

1. **Plug in ESP32-S3 with CircuitPython**
2. **CIRCUITPY drive appears** (like a USB stick)
3. **Drag and drop `code.py`** to the drive
4. **Code runs automatically!**

No REPL needed, just copy files like any USB drive.

---

## Comparing Arduino vs CircuitPython

### Arduino Firmware (Current Production)

**File:** `esp32_ultrasonic_raw/esp32_ultrasonic_raw.ino`

**Pros:**
- ✓ Fast (10 Hz update rate)
- ✓ Efficient memory usage
- ✓ Reliable GPIO timing
- ✓ Best for production use

**Cons:**
- ✗ Must compile/upload for changes
- ✗ Requires Arduino IDE or PlatformIO
- ✗ C++ syntax

**When to use:**
- Final rover deployment
- Maximum performance needed
- Long-term reliability

### CircuitPython Firmware (Experimental)

**File:** `circuitpython_ultrasonic/code.py`

**Pros:**
- ✓ Edit code without compile
- ✓ Python syntax (familiar)
- ✓ Interactive REPL debugging
- ✓ Easy to modify on the fly

**Cons:**
- ✗ Slower (5-8 Hz typical)
- ✗ More memory usage
- ✗ Awkward upload on ESP32 (better on S3)

**When to use:**
- Quick prototyping
- Learning/experimentation
- Frequent code changes
- ESP32-S3 (with native USB)

---

## Testing Your Sensors

### Step 1: Upload Firmware

**Option A: Use Current Arduino Firmware**
```bash
# Arduino firmware is already on your ESP32
# It's running right now!
```

**Option B: Try CircuitPython Firmware**
```bash
# CircuitPython is also installed from our experiment
# Use the Python upload script above
```

### Step 2: Launch the GUI

```bash
cd "Jetson Cube Orange Outdoor Rover"
python3 ultrasonic_gui.py
```

You should see:
- Real-time radar display
- All 6 sensors listed
- "NO DATA" initially (no sensors wired yet)

### Step 3: Wire Sensors and Test

Follow `ESP32_ULTRASONIC_WIRING_GUIDE.md` to connect sensors.

As you wire each sensor, you'll see:
- Sensor status changes from "NO DATA" to "NO OBJECT" or "CLEAR"
- Radar ray appears for that sensor
- Distance graph starts updating

### Step 4: Verify Wiring

Test each sensor by waving your hand:
1. Wave hand in front of Front sensor
   - Should see green ray shrink on radar
   - Distance should drop to < 0.5m
   - Status: "⚠ VERY CLOSE"

2. Repeat for all 6 sensors
   - Verify correct sensor responds
   - Check angle matches (front = 0°, right = 90°, etc.)
   - Ensure smooth distance readings

---

## Debugging Tools

### 1. Graphical GUI (Best for Testing)
```bash
python3 ultrasonic_gui.py
```

**Use for:**
- Visual confirmation of sensor placement
- Real-time debugging
- Performance monitoring
- Distance calibration

### 2. Terminal Visualizer (ASCII Art)
```bash
python3 ultrasonic_visualizer.py
```

**Use for:**
- SSH sessions (no X11 needed)
- Low bandwidth
- Simple text output

### 3. Raw Monitor (Minimal)
```bash
python3 ultrasonic_monitor.py
```

**Use for:**
- JSON validation
- Data format debugging
- Quick sensor checks

---

## Data Logging

The GUI can log all sensor data to CSV:

1. **Press `L`** in the GUI to start logging
2. File created: `ultrasonic_log_YYYYMMDD_HHMMSS.csv`
3. **Press `L`** again to stop logging

**CSV Format:**
```csv
timestamp,sensor,distance,valid
1699565123.456,front,2.345,True
1699565123.556,corner_left,1.234,True
1699565123.656,side_right,6.000,False
```

**Uses:**
- Analyze sensor performance
- Calibration data
- Bug reports
- Performance testing

---

## Switching Between Arduino and CircuitPython

### To Arduino (Production Mode)

```bash
cd "esp32_ultrasonic_raw"
# Open in Arduino IDE and upload
# OR use PlatformIO:
cd ../esp32_ultrasonic
platformio run --target upload
```

### To CircuitPython (Experimental Mode)

CircuitPython is already installed from our earlier test!

```bash
# Just upload new code.py via REPL
# See "Option 2: Python Upload Script" above
```

### Back to CircuitPython Base

```bash
# Re-flash CircuitPython firmware
python3 ~/.platformio/packages/tool-esptoolpy/esptool.py \
    --chip esp32 --port /dev/ttyUSB0 --baud 460800 \
    write_flash -z 0x0 /tmp/circuitpython-esp32.bin
```

---

## Recommended Workflow

### For Initial Sensor Testing:

1. ✓ Keep Arduino firmware (already working)
2. ✓ Use GUI tool: `python3 ultrasonic_gui.py`
3. ✓ Wire sensors one at a time
4. ✓ Verify each sensor with hand wave test
5. ✓ Save screenshot when all working

### For Debugging Issues:

1. Enable DEBUG_MODE in firmware (Arduino or CircuitPython)
2. Monitor serial output: `screen /dev/ttyUSB0 115200`
3. Check for timeouts or invalid readings
4. Use GUI logging to capture problem
5. Check wiring with multimeter if needed

### For Rover Deployment:

1. Use Arduino firmware (best performance)
2. Set DEBUG_MODE = false
3. Deploy `ultrasonic_bridge.py` to Jetson
4. Integrate with ROS2
5. Monitor via ROS2 topics

---

## Troubleshooting

### GUI Won't Start

```bash
# Check PyGame installed
pip3 list | grep pygame

# Install if missing
pip3 install pygame

# Check serial port
ls -la /dev/ttyUSB*
```

### No Sensor Data in GUI

1. Check ESP32 is connected: `ls /dev/ttyUSB*`
2. Verify firmware is running: `screen /dev/ttyUSB0 115200`
3. Should see JSON output every 100ms
4. Exit screen: Ctrl+A, K, Y

### CircuitPython Code Won't Upload

```bash
# Make sure CircuitPython is running
screen /dev/ttyUSB0 115200

# Press Ctrl+C - should see ">>>"
# If not, re-flash CircuitPython firmware
```

### Sensors Show Wrong Distances

1. Check 5V power supply voltage (should be 4.8-5.2V)
2. Verify level shifter wiring for ECHO pins
3. Enable DEBUG_MODE and check for timeouts
4. Try slower update rate (increase READ_INTERVAL)

---

## Files Summary

| File | Purpose | When to Use |
|------|---------|-------------|
| `ultrasonic_gui.py` | **Graphical debugging tool** | Sensor testing, debugging |
| `ultrasonic_visualizer.py` | Terminal ASCII radar | SSH sessions, simple testing |
| `ultrasonic_monitor.py` | Raw JSON monitor | Data format validation |
| `circuitpython_ultrasonic/code.py` | CircuitPython firmware | Experimentation, ESP32-S3 |
| `esp32_ultrasonic_raw.ino` | Arduino firmware | Production, maximum performance |

---

## Next Steps

1. **Wire up sensors** following `ESP32_ULTRASONIC_WIRING_GUIDE.md`
2. **Test with GUI**: `python3 ultrasonic_gui.py`
3. **Verify all 6 sensors** respond correctly
4. **Switch to Arduino firmware** for production (if using CircuitPython now)
5. **Deploy to Jetson** using `ULTRASONIC_SETUP_GUIDE.md`

---

**Created:** November 9, 2025
**Status:** Ready for sensor testing
**Recommended:** Use Arduino firmware + GUI tool for best results
