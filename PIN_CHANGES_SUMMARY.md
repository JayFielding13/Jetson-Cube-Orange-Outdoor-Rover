# Side Right Sensor Pin Changes - Summary

**Date:** November 9, 2025
**Reason:** GPIO 9/10 are flash pins and cause boot loops

---

## What Changed

### ❌ OLD (Caused Boot Loop)
```
Side Right Sensor:
- TRIG: GPIO 9  (R16 - RIGHT side, 16th pin)
- ECHO: GPIO 10 (R17 - RIGHT side, 17th pin)
```

### ✅ NEW (Fixed - Now Working)
```
Side Right Sensor:
- TRIG: GPIO 16 (L12 - LEFT side, 12th pin)
- ECHO: GPIO 17 (L11 - LEFT side, 11th pin)
```

---

## Visual Reference

```
                        ESP32 DevKit
                  (Micro-USB pointing back)

LEFT SIDE                                    RIGHT SIDE
─────────                                    ──────────
L1  = GND                                    R1  = 3V3
L2  = 23                                     R2  = EN
L3  = 22                                     R3  = VP
...                                          ...
L10 = 5  ← Corner Right TRIG                R10 = 26 ← Rear ECHO (LV2)
L11 = 17 ← Side Right ECHO (LV5) ✓ NEW      R11 = 27 ← Corner Right ECHO (LV3)
L12 = 16 ← Side Right TRIG ✓ NEW            R12 = 14 ← Side Left TRIG
L13 = 4  ← Side Left ECHO (LV4)             R13 = 12 ← Corner Left ECHO (LV6)
L14 = 0                                      R14 = GND
L15 = 2  ← Front ECHO (LV1)                 R15 = 13 ← Corner Left TRIG
L16 = 15 ← Front TRIG                        R16 = 9  ← ⚠️ DON'T USE (Flash)
L17 = 8                                      R17 = 10 ← ⚠️ DON'T USE (Flash)
L18 = 7                                      R18 = 11
L19 = 6                                      R19 = VIN
```

---

## Complete Pin Assignments (All Sensors)

| Sensor | Angle | TRIG Pin | ECHO Pin (via Level Shifter) | Physical Pins |
|--------|-------|----------|------------------------------|---------------|
| Front | 0° | GPIO 15 | GPIO 2 (LV1) | L16, L15 |
| Corner Left | -45° | GPIO 13 | GPIO 12 (LV6) | R15, R13 |
| Corner Right | +45° | GPIO 5 | GPIO 27 (LV3) | L10, R11 |
| Side Left | -90° | GPIO 14 | GPIO 4 (LV4) | R12, L13 |
| **Side Right** | **+90°** | **GPIO 16** | **GPIO 17 (LV5)** | **L12, L11** ⚠️ CHANGED |
| Rear | 180° | GPIO 25 | GPIO 26 (LV2) | R9, R10 |

---

## Wiring Instructions for Side Right

### Step 1: Power Connections
```
Side Right Sensor → Power Rails
├─ VCC (red wire)   → 5V power rail
└─ GND (black wire) → GND rail
```

### Step 2: Signal Connections
```
Side Right Sensor → Level Shifter → ESP32
├─ TRIG (yellow wire) ────────────────→ GPIO 16 (L12)
└─ ECHO (blue wire)   → HV5 → LV5 ───→ GPIO 17 (L11)
```

### How to Find the Pins

**With Micro-USB port pointing towards you:**

**GPIO 16 (L12) - Side Right TRIG:**
- LEFT side of ESP32
- Count down 12 pins from top
- Pin labeled "16"

**GPIO 17 (L11) - Side Right ECHO (via LV5):**
- LEFT side of ESP32
- Count down 11 pins from top
- Pin labeled "17"
- Goes through level shifter channel 5

---

## Why This Change Was Necessary

### Problem with GPIO 9/10

GPIO 9 and GPIO 10 are directly connected to the ESP32's internal flash memory chip:
- **GPIO 9:** Flash data line 2 (SD_DATA2)
- **GPIO 10:** Flash data line 3 (SD_DATA3)

Using these pins as regular GPIO causes:
```
rst:0x8 (TG1WDT_SYS_RESET)
boot:0x13 (SPI_FAST_FLASH_BOOT)
[Continuous reset loop]
```

The ESP32 watchdog timer detects flash interference and resets the system.

### Solution

GPIO 16 and 17 are safe general-purpose pins:
- Not connected to flash
- No special boot functions
- Fully available for ultrasonic sensors
- Located conveniently on the left side

---

## Files Updated

All documentation has been updated with the new pin assignments:

✅ **Firmware:**
- `esp32_ultrasonic_raw/esp32_ultrasonic_raw.ino`
- `esp32_ultrasonic/src/main.cpp`
- `circuitpython_ultrasonic/code.py`

✅ **Documentation:**
- `ESP32_PINOUT_DIAGRAM.md` (this file)
- `SIDE_RIGHT_REWIRE_NOTE.md`
- `ESP32_ULTRASONIC_WIRING_GUIDE.md`

✅ **Tools:**
- `ultrasonic_gui.py`
- `ultrasonic_visualizer.py`
- `ultrasonic_monitor.py`

All tools work with the updated pin assignments automatically!

---

## Testing Status

### ✅ Verified Working:
- ESP32 boots successfully (no more reset loop)
- Firmware runs at 10 Hz
- JSON output streaming correctly
- All 6 sensors configured (showing `"valid":false` without sensors wired)

### ⏳ Next Steps:
1. Wire Side Right sensor to GPIO 16/17
2. Test with GUI: `python3 ultrasonic_gui.py`
3. Verify correct sensor detection
4. Wire remaining sensors

---

## Quick Reference Card

**Print and keep this with your rover:**

```
┌──────────────────────────────────────────────────┐
│     SIDE RIGHT SENSOR - NEW PIN ASSIGNMENT       │
├──────────────────────────────────────────────────┤
│                                                  │
│  TRIG:  GPIO 16 (LEFT side, pin L12)            │
│  ECHO:  GPIO 17 → LV5 (LEFT side, pin L11)      │
│                                                  │
│  ⚠️ DON'T USE: GPIO 9/10 (Flash pins)            │
│                                                  │
└──────────────────────────────────────────────────┘
```

---

**Created:** November 9, 2025
**Status:** All documentation updated, firmware tested
**Action Required:** Wire Side Right sensor to GPIO 16/17
