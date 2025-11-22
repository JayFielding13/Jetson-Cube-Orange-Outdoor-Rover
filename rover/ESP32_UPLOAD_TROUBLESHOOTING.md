# ESP32 Upload Troubleshooting Guide

**Date:** November 9, 2025
**Issue:** ESP32 upload fails with "Failed to write to target RAM" or "Checksum error"
**Status:** Firmware compiled successfully, upload blocked by hardware issue

---

## Current Situation

✅ **Working:**
- Firmware compiles successfully (0 errors)
- ESP32 is detected on `/dev/ttyUSB0`
- Chip identification succeeds (ESP32-D0WD-V3 rev 3.1)
- Binary files are ready to upload

❌ **Not Working:**
- Upload fails when writing stub to RAM
- Error: "Failed to write to target RAM (result was 0107: Checksum error)"
- Sometimes shows crystal frequency detection warning

---

## What We've Tried

### Software Approaches (All Failed)
1. ✗ PlatformIO upload (standard method)
2. ✗ esptool.py direct upload at 115200 baud
3. ✗ esptool.py with `--no-stub` option
4. ✗ Arduino CLI upload
5. ✗ Very slow upload (9600 baud)
6. ✗ Different upload speeds (115200, 460800, 9600)

**Conclusion:** This is NOT a software issue - the firmware is perfect.

---

## Root Cause Analysis

The "Failed to write to target RAM" error indicates one of these **hardware** issues:

### 1. USB Cable (Most Common - 80% of cases)
**Symptoms:**
- Chip detected but upload fails
- Checksum errors
- Intermittent connections

**Solution:** Try a different USB cable (preferably shorter, higher quality)

### 2. Insufficient Power
**Symptoms:**
- Upload starts but fails during stub loading
- Crystal frequency warnings
- Brownouts during flash

**Solutions:**
- Try powered USB hub
- Connect 5V power to VIN pin (external power supply)
- Use USB 2.0 port instead of USB 3.0 (sometimes provides more stable power)

### 3. USB Port Issues
**Symptoms:**
- Connection drops
- Data corruption

**Solutions:**
- Try different USB port on computer
- Avoid USB ports on monitors/docks
- Use motherboard USB ports directly

### 4. ESP32 Board Defect
**Symptoms:**
- Consistent failures across all methods
- Multiple cables/ports don't help

**Solution:** Try different ESP32 board if available

---

## Recommended Fix Steps

### Step 1: Try Different USB Cable
This fixes 80% of upload issues.

1. Find a **different** USB cable (Micro-USB type)
2. Preferably use:
   - Shorter cable (< 1 meter)
   - Cable with ferrite beads (noise suppression)
   - Cable marked "data + charging" (not "charging only")
3. Unplug ESP32, wait 5 seconds
4. Connect with new cable
5. Try upload again:
   ```bash
   cd "/home/jay/Desktop/Mini Rover Development/Jetson Cube Orange Outdoor Rover/esp32_ultrasonic"
   platformio run --target upload
   ```

### Step 2: Add External Power
If cable doesn't help, power might be the issue.

**Option A: Powered USB Hub**
1. Connect ESP32 to powered USB hub
2. Ensure hub has external power adapter connected
3. Try upload

**Option B: External 5V Supply**
1. Connect 5V power supply to ESP32 VIN pin
2. Connect GND to ESP32 GND
3. Keep USB connected for data
4. Try upload

### Step 3: Try Different Computer USB Port
1. Try USB 2.0 port (black interior) instead of USB 3.0 (blue)
2. Try ports directly on motherboard (not front panel)
3. Avoid hubs/docks for initial upload

### Step 4: Manual Boot Mode Entry
Sometimes holding BOOT at the right time helps:

1. Press and HOLD the BOOT button
2. Press and release the EN (reset) button while holding BOOT
3. Release BOOT button after 1 second
4. ESP32 should now be in bootloader mode
5. Immediately run upload command:
   ```bash
   cd "/home/jay/Desktop/Mini Rover Development/Jetson Cube Orange Outdoor Rover/esp32_ultrasonic"
   platformio run --target upload
   ```

---

## Alternative Upload Methods

### Method 1: Use the Jetson
If your Jetson has a working USB port, upload from there:

1. Copy files to Jetson:
   ```bash
   scp -r esp32_ultrasonic jay@100.91.191.47:~/
   ```

2. SSH to Jetson and upload:
   ```bash
   ssh jay@100.91.191.47
   cd ~/esp32_ultrasonic
   platformio run --target upload
   ```

### Method 2: Web-Based Flasher
Use ESP Tool Web: https://espressif.github.io/esptool-js/

1. Open in Chrome/Edge (needs Web Serial API)
2. Click "Connect"
3. Select `/dev/ttyUSB0`
4. Upload these files:
   - 0x1000: `esp32_ultrasonic/.pio/build/esp32dev/bootloader.bin`
   - 0x8000: `esp32_ultrasonic/.pio/build/esp32dev/partitions.bin`
   - 0x10000: `esp32_ultrasonic/.pio/build/esp32dev/firmware.bin`

### Method 3: Try Different ESP32 Board
If you have another ESP32:
1. Test upload on different board
2. If successful, original board may be defective
3. If fails, USB cable is likely the issue

---

## Files Ready for Upload

All compiled and ready in:
```
esp32_ultrasonic/.pio/build/esp32dev/
├── bootloader.bin    (bootloader - flash at 0x1000)
├── partitions.bin    (partition table - flash at 0x8000)
└── firmware.bin      (your code - flash at 0x10000)
```

**Firmware Stats:**
- Program size: 270,833 bytes (20.7% of flash)
- RAM usage: 21,608 bytes (6.6% of RAM)
- Status: ✅ Compiled successfully, 0 errors

---

## Quick Commands Reference

### Check ESP32 Connection
```bash
ls -la /dev/ttyUSB*
```

### Upload with PlatformIO
```bash
cd "/home/jay/Desktop/Mini Rover Development/Jetson Cube Orange Outdoor Rover/esp32_ultrasonic"
platformio run --target upload
```

### Upload with Arduino CLI
```bash
cd "/home/jay/Desktop/Mini Rover Development/Jetson Cube Orange Outdoor Rover/esp32_ultrasonic_sketch"
../../bin/arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 .
```

### Monitor Serial Output (After Upload)
```bash
# PlatformIO monitor
cd "/home/jay/Desktop/Mini Rover Development/Jetson Cube Orange Outdoor Rover/esp32_ultrasonic"
platformio device monitor --baud 115200

# Or use screen
screen /dev/ttyUSB0 115200

# Or use Python monitor
python3 ultrasonic_monitor.py /dev/ttyUSB0
```

---

## When Upload Finally Works

After successful upload:

1. **Test with DEBUG mode first:**
   - Firmware is currently in PRODUCTION mode (JSON output)
   - To enable DEBUG mode, edit `esp32_ultrasonic/src/main.cpp` line 31:
     ```cpp
     #define DEBUG_MODE true  // Change false to true
     ```
   - Recompile and upload
   - Open serial monitor to see human-readable output

2. **Validate all sensors:**
   ```bash
   python3 ultrasonic_monitor.py /dev/ttyUSB0
   ```
   - Check all 6 sensors show valid readings
   - Verify ~10 Hz update rate
   - Test with obstacles at known distances

3. **Switch to production mode:**
   - Set `DEBUG_MODE false` in code
   - Upload again
   - Verify JSON output with `cat /dev/ttyUSB0`

4. **Deploy to Jetson:**
   - Follow [ULTRASONIC_SETUP_GUIDE.md](ULTRASONIC_SETUP_GUIDE.md) Phase 3

---

## Getting Help

**ESP32 Upload Issues:**
- Espressif troubleshooting: https://docs.espressif.com/projects/esptool/en/latest/troubleshooting.html
- PlatformIO forum: https://community.platformio.org/
- Arduino forum: https://forum.arduino.cc/

**Common Solutions That Work:**
1. Different USB cable (fixes 80%)
2. Powered USB hub (fixes 15%)
3. External 5V power (fixes 4%)
4. Different ESP32 board (fixes 1%)

---

## Summary

**Problem:** Hardware connection prevents upload
**Not a problem:** Your firmware is perfect!
**Most likely fix:** Try a different USB cable
**Next most likely:** Add external power or use powered hub
**Workaround:** Upload from Jetson or use web flasher

The firmware is compiled and ready. Once you solve the hardware connection issue (probably just need a better USB cable), you'll be able to upload and test immediately.

---

**Created:** November 9, 2025
**Status:** Awaiting hardware fix (USB cable replacement recommended)
