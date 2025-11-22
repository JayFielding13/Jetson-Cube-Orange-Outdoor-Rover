# Side Right Sensor - Current Status

**Date:** November 10, 2025
**Issue:** Side Right sensor not working after multiple GPIO pin attempts

---

## Current Configuration (SWAPPED PINS TO TEST)

**Side Right Sensor:**
- TRIG: GPIO 33 (R7 - RIGHT side, pin 7)
- ECHO: GPIO 32 (R6 - RIGHT side, pin 6, via Level Shifter LV5)

**Note:** Pins are SWAPPED to test if wires got crossed

---

## What We've Tried

### GPIO Pins Attempted:
1. ❌ GPIO 9/10 - Caused boot loop (flash pins)
2. ❌ GPIO 16/17 - No LED activity, timeout
3. ❌ GPIO 18/19 - No LED activity, timeout (likely SPI flash conflict)
4. ❌ GPIO 32/33 - Currently testing with SWAPPED assignment

### Working Sensors:
- ✅ Side Left (GPIO 14/4) - Worked perfectly
- ✅ Rear (GPIO 25/26) - Worked in earlier tests

---

## Files Updated

### CircuitPython:
`circuitpython_ultrasonic/code.py` - Line 107:
```python
UltrasonicSensor('side_right', 90, board.IO33, board.IO32),  # SWAPPED
```

### Arduino:
`esp32_ultrasonic_raw/esp32_ultrasonic_raw.ino` - Lines 64-65:
```c
#define SIDE_RIGHT_TRIG_PIN 33  // SWAPPED
#define SIDE_RIGHT_ECHO_PIN 32  // SWAPPED
```

---

## Next Steps After Power Cycle

1. **Power cycle the ESP32 and sensor boards**
2. **Check if Side Right LED starts blinking**
3. **Run test script:**
   ```bash
   cd "/home/jay/Desktop/Mini Rover Development/Jetson Cube Orange Outdoor Rover"
   python3 -c "
   import serial, json, time
   ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
   time.sleep(1)
   for _ in range(5):
       line = ser.readline().decode('utf-8', errors='ignore').strip()
       if '{' in line:
           data = json.loads(line)
           for s in data['sensors']:
               marker = '✅' if s['valid'] else '❌'
               print(f'{marker} {s[\"name\"]:15} - {s[\"distance\"]:.2f}m')
           break
   ser.close()
   "
   ```

4. **If still not working:**
   - Consider GPIO 32/33 may also have conflicts
   - Try different GPIO pair (e.g., GPIO 21/22, GPIO 23)
   - Use Arduino IDE upload instead of CircuitPython

---

## Pin Reference

```
ESP32 RIGHT SIDE:
R6  [GPIO 32] ← Side Right ECHO (via Level Shifter LV5) - SWAPPED
R7  [GPIO 33] ← Side Right TRIG - SWAPPED
```

---

**Status:** Awaiting power cycle test
**Firmware:** CircuitPython uploaded with swapped pins
**Physical Wiring:** Confirmed connected to GPIO 32/33

