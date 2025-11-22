# Side Right Sensor - Pin Change Required

**Date:** November 9, 2025
**Reason:** GPIO 9/10 cause boot loop (used by ESP32 flash)
**Solution:** Move to GPIO 16/17

---

## IMPORTANT: Rewiring Required

**Side Right Sensor must be moved:**

| Connection | OLD Pin (BAD) | NEW Pin (GOOD) | Physical Location |
|------------|---------------|----------------|-------------------|
| TRIG | GPIO 9 (R16) | **GPIO 16 (R18)** | Right side, pin 18 |
| ECHO (via LV5) | GPIO 10 (R17) | **GPIO 17 (R19)** | Right side, pin 19 |

---

## Updated Pin Assignments (All Sensors)

| Sensor | Angle | TRIG Pin | ECHO Pin (via Level Shifter) | Physical Pins |
|--------|-------|----------|------------------------------|---------------|
| Front | 0° | GPIO 15 | GPIO 2 (LV1) | L16, L15 |
| Corner Left | -45° | GPIO 13 | GPIO 12 (LV6) | R15, R13 |
| Corner Right | +45° | GPIO 5 | GPIO 27 (LV3) | L10, R11 |
| Side Left | -90° | GPIO 14 | GPIO 4 (LV4) | R12, L13 |
| **Side Right** | **+90°** | **GPIO 16** | **GPIO 17 (LV5)** | **R18, R19** ⚠️ CHANGED |
| Rear | 180° | GPIO 25 | GPIO 26 (LV2) | R9, R10 |

---

## Why GPIO 9/10 Don't Work

GPIO 9 and GPIO 10 are connected to the ESP32's internal flash memory:
- GPIO 9: Flash data line 2 (SD_DATA2)
- GPIO 10: Flash data line 3 (SD_DATA3)

Using these pins as GPIO causes the ESP32 to enter a watchdog reset loop:
```
rst:0x8 (TG1WDT_SYS_RESET)
```

---

## Rewiring Steps

1. **Locate Side Right sensor wires:**
   - Currently on GPIO 9 (R16) and GPIO 10 (R17)

2. **Move TRIG wire:**
   - From GPIO 9 (R16) → **GPIO 16 (R18)**

3. **Move ECHO wire (through level shifter):**
   - From GPIO 10 (R17) → **GPIO 17 (R19)**

4. **Double-check:**
   - GPIO 16 and 17 are the last two pins on the right side
   - Count from the top: R18 and R19

---

## After Rewiring

Upload the updated firmware (already modified):
- File: `esp32_ultrasonic_raw/esp32_ultrasonic_raw.ino`
- Side Right now uses GPIO 16/17
- Should boot normally without watchdog resets

---

## Quick Reference - Right Side Pins

```
        ESP32 DevKit (Right Side)
        ========================
R1  [ EN  ]
R2  [ VP  ]
R3  [ VN  ]
R4  [ 34  ]
R5  [ 35  ]
R6  [ 32  ]
R7  [ 33  ]
R8  [ 25  ] ← REAR TRIG
R9  [ 26  ] ← REAR ECHO (LV2)
R10 [ 27  ] ← CORNER RIGHT ECHO (LV3)
R11 [ 14  ] ← SIDE LEFT TRIG
R12 [ 12  ] ← CORNER LEFT ECHO (LV6)
R13 [ GND ]
R14 [ 13  ] ← CORNER LEFT TRIG
R15 [  9  ] ← DON'T USE (Flash pin)
R16 [ 10  ] ← DON'T USE (Flash pin)
R17 [ 11  ] ← DON'T USE (Flash pin)
R18 [ 16  ] ← SIDE RIGHT TRIG ✓ NEW
R19 [ 17  ] ← SIDE RIGHT ECHO (LV5) ✓ NEW
```

---

**Status:** Firmware updated, waiting for hardware rewire
**Next Step:** Move Side Right wires to GPIO 16/17, then upload
