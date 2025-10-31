# Obstacle-Avoidance Rover - Wiring Reference

## System Overview
- **Main Controller:** Raspberry Pi 4
- **Motor Controller:** Arduino Nano
- **Motor Driver:** Cytron MDDS30 (R/C mode)
- **Sensors:** 4× HC-SR04 Ultrasonic Sensors (2 front, 2 side)

---

## Raspberry Pi 4 GPIO Connections

### Front Sensors (Obstacle Avoidance)

#### Left Front HC-SR04 Ultrasonic Sensor
| Sensor Pin | Connection | Pi Pin | Notes |
|------------|------------|--------|-------|
| VCC | 5V Power | Physical Pin 2 | 5V rail |
| GND | Ground | Physical Pin 6 | Ground rail |
| TRIG | GPIO 23 | Physical Pin 16 | Direct connection (3.3V output safe for sensor) |
| ECHO | GPIO 24 | Physical Pin 18 | **REQUIRES VOLTAGE DIVIDER** (see below) |

#### Right Front HC-SR04 Ultrasonic Sensor
| Sensor Pin | Connection | Pi Pin | Notes |
|------------|------------|--------|-------|
| VCC | 5V Power | Physical Pin 4 | Can share 5V rail |
| GND | Ground | Physical Pin 9 | Can share ground |
| TRIG | GPIO 17 | Physical Pin 11 | Direct connection (3.3V output safe for sensor) |
| ECHO | GPIO 27 | Physical Pin 13 | **REQUIRES VOLTAGE DIVIDER** (see below) |

### Voltage Divider Circuits (CRITICAL!)

**Build TWO identical circuits - one for each ECHO pin**

```
Sensor ECHO (5V output)
       |
   [2kΩ] [2kΩ]  ← Two 2kΩ resistors in PARALLEL (= 1kΩ equivalent)
       |
       ├─────────> TO Raspberry Pi GPIO (reads ~3.3V here)
       |
     [2kΩ]  ← Single 2kΩ resistor to ground
       |
      GND
```

**Circuit 1:** Left Front Sensor ECHO → Voltage Divider → GPIO 24
**Circuit 2:** Right Front Sensor ECHO → Voltage Divider → GPIO 27

**WARNING:** Connecting 5V ECHO directly to Pi GPIO will damage the Pi!

### Side Sensors (Wall Detection)

#### Left Side HC-SR04 Ultrasonic Sensor
| Sensor Pin | Connection | Pi Pin | Notes |
|------------|------------|--------|-------|
| VCC | 5V Power | Physical Pin 2 or 4 | Can share 5V rail with front sensors |
| GND | Ground | Physical Pin 6, 9, or 14 | Can share ground |
| TRIG | GPIO 5 | Physical Pin 29 | Direct connection (3.3V output safe for sensor) |
| ECHO | GPIO 6 | Physical Pin 31 | **REQUIRES VOLTAGE DIVIDER** (same as front sensors) |

#### Right Side HC-SR04 Ultrasonic Sensor
| Sensor Pin | Connection | Pi Pin | Notes |
|------------|------------|--------|-------|
| VCC | 5V Power | Physical Pin 2 or 4 | Can share 5V rail |
| GND | Ground | Physical Pin 6, 9, or 14 | Can share ground |
| TRIG | GPIO 22 | Physical Pin 15 | Direct connection (3.3V output safe for sensor) |
| ECHO | GPIO 25 | Physical Pin 22 | **REQUIRES VOLTAGE DIVIDER** (same as front sensors) |

**Circuit 3:** Left Side Sensor ECHO → Voltage Divider → GPIO 6
**Circuit 4:** Right Side Sensor ECHO → Voltage Divider → GPIO 25

**Sensor Mounting:** Mount side sensors perpendicular (90°) to rover's sides for wall detection.

### Arduino Nano Connection
| Arduino | Connection | Pi Pin | Notes |
|---------|------------|--------|-------|
| USB Port | USB cable | Any USB port | Provides serial communication + power for Arduino |

---

## Arduino Nano Pin Assignments

### MDDS30 Motor Driver Connections

#### Motor Pin Assignments (VERIFIED - MIXED MODE)
| Arduino Pin | Function | MDDS30 Pin | Purpose | Notes |
|-------------|----------|------------|---------|-------|
| D5 (PWM) | STEERING | CH2 Signal | Left/Right control | R/C servo-style PWM signal |
| D6 (PWM) | THROTTLE | CH1 Signal | Forward/Backward control | R/C servo-style PWM signal |
| GND | Ground | CH1/CH2 GND | Common reference | Common ground |

**CRITICAL DISCOVERY (2025-10-13):** The MDDS30 is configured in **MIXED R/C MODE**, not independent motor mode!

**How Mixing Works:**
- **Channel 1 (D6)**: Acts as THROTTLE (forward/backward for both motors)
- **Channel 2 (D5)**: Acts as STEERING (differential speed for turning)
- The MDDS30 automatically mixes these signals to control both motors

**Control Mapping:**
- **Forward**: Steering channel forward (D5 > 1500), Throttle centered (D6 = 1500)
- **Backward**: Steering channel backward (D5 < 1500), Throttle centered (D6 = 1500)
- **Turn Left**: Throttle forward (D6 > 1500), Steering centered (D5 = 1500)
- **Turn Right**: Throttle backward (D6 < 1500), Steering centered (D5 = 1500)

This mixing allows the motor driver to automatically calculate the correct speeds for both motors to achieve the desired movement.

### Arduino Power
| Pin | Connection | Notes |
|-----|------------|-------|
| USB Port | Raspberry Pi USB | Powers Arduino + provides serial communication |

---

## MDDS30 Motor Driver Configuration

### DIP Switch Settings
⚠️ **IMPORTANT:** Configure MDDS30 DIP switches for **R/C mode** before use.
Consult MDDS30 user manual for correct DIP switch positions for R/C input mode.

### Motor Outputs
| MDDS30 Terminal | Connection | Notes |
|-----------------|------------|-------|
| M1+ / M1- | Left Motor | Channel 1 motor terminals |
| M2+ / M2- | Right Motor | Channel 2 motor terminals |

### Power Input
| MDDS30 Terminal | Connection | Notes |
|-----------------|------------|-------|
| Vin+ | Battery Positive | 7V-35V DC supply |
| Vin- | Battery Negative | Common ground with Arduino GND |

---

## Power Supply Summary

| Component | Power Source | Voltage | Notes |
|-----------|-------------|---------|-------|
| Raspberry Pi 4 | Dedicated power supply | 5V 3A | Via USB-C or GPIO pins |
| Arduino Nano | Raspberry Pi USB | 5V | Powered by Pi |
| HC-SR04 Sensors | Raspberry Pi 5V pins | 5V | Shares Pi's 5V rail |
| MDDS30 + Motors | Battery pack | 7V-35V | Separate high-current supply |

**CRITICAL:** Ensure MDDS30 GND is connected to Arduino GND for common reference.

---

## Wire Checklist

### Raspberry Pi Wiring - Front Sensors
- [ ] Left front sensor VCC → Pi Pin 2 (5V)
- [ ] Left front sensor GND → Pi Pin 6 (GND)
- [ ] Left front sensor TRIG → Pi Pin 16 (GPIO 23)
- [ ] Left front sensor ECHO → Voltage divider → Pi Pin 18 (GPIO 24)
- [ ] Right front sensor VCC → Pi Pin 4 (5V)
- [ ] Right front sensor GND → Pi Pin 9 (GND)
- [ ] Right front sensor TRIG → Pi Pin 11 (GPIO 17)
- [ ] Right front sensor ECHO → Voltage divider → Pi Pin 13 (GPIO 27)

### Raspberry Pi Wiring - Side Sensors
- [ ] Left side sensor VCC → Pi 5V (Pin 2 or 4)
- [ ] Left side sensor GND → Pi GND (Pin 6, 9, or 14)
- [ ] Left side sensor TRIG → Pi Pin 29 (GPIO 5)
- [ ] Left side sensor ECHO → Voltage divider → Pi Pin 31 (GPIO 6)
- [ ] Right side sensor VCC → Pi 5V (Pin 2 or 4)
- [ ] Right side sensor GND → Pi GND (Pin 6, 9, or 14)
- [ ] Right side sensor TRIG → Pi Pin 15 (GPIO 22)
- [ ] Right side sensor ECHO → Voltage divider → Pi Pin 22 (GPIO 25)

### Other Connections
- [ ] Arduino USB → Pi USB port

### Arduino Wiring
- [ ] Arduino D5 → MDDS30 CH1 Signal
- [ ] Arduino D6 → MDDS30 CH2 Signal
- [ ] Arduino GND → MDDS30 CH1/CH2 GND

### MDDS30 Wiring
- [ ] MDDS30 M1+/M1- → Left motor
- [ ] MDDS30 M2+/M2- → Right motor
- [ ] MDDS30 Vin+/Vin- → Battery pack (7V-35V)
- [ ] MDDS30 DIP switches configured for R/C mode

---

## Quick Reference Diagrams

### GPIO Pin Numbering (Physical)
```
    3.3V  [ 1] [ 2]  5V        ← All sensor VCC (shared 5V rail)
   GPIO2  [ 3] [ 4]  5V
   GPIO3  [ 5] [ 6]  GND       ← All sensor GND (shared ground)
   GPIO4  [ 7] [ 8]  GPIO14
     GND  [ 9][10]  GPIO15
  GPIO17  [11][12]  GPIO18     ← Right FRONT TRIG
  GPIO27  [13][14]  GND        ← Right FRONT ECHO (via divider)
  GPIO22  [15][16]  GPIO23     ← Right SIDE TRIG | Left FRONT TRIG
    3.3V  [17][18]  GPIO24     ← Left FRONT ECHO (via divider)
  GPIO10  [19][20]  GND
   GPIO9  [21][22]  GPIO25     ← Right SIDE ECHO (via divider)
  GPIO11  [23][24]  GPIO8
     GND  [25][26]  GPIO7
   GPIO0  [27][28]  GPIO1
   GPIO5  [29][30]  GND        ← Left SIDE TRIG
   GPIO6  [31][32]  GPIO12     ← Left SIDE ECHO (via divider)
  GPIO13  [33][34]  GND
  GPIO19  [35][36]  GPIO16
  GPIO26  [37][38]  GPIO20
     GND  [39][40]  GPIO21
```

**Sensor Summary:**
- **Left Front**: GPIO23 (TRIG), GPIO24 (ECHO) - Pins 16, 18
- **Right Front**: GPIO17 (TRIG), GPIO27 (ECHO) - Pins 11, 13
- **Left Side**: GPIO5 (TRIG), GPIO6 (ECHO) - Pins 29, 31
- **Right Side**: GPIO22 (TRIG), GPIO25 (ECHO) - Pins 15, 22

### R/C Signal Specification
- **Center (Stop):** 1500μs pulse
- **Forward:** 1500-2000μs (higher = faster)
- **Reverse:** 1000-1500μs (lower = faster)
- **Period:** 20ms (50Hz)

---

## Testing Steps

1. **Power off everything**
2. **Double-check all connections** against this reference
3. **Verify voltage dividers** with multimeter (should read ~3.3V when sensor ECHO is high)
4. **Check MDDS30 DIP switches** for R/C mode
5. **Power on Raspberry Pi** (Arduino will power via USB)
6. **Upload Arduino sketch** to Nano
7. **Test sensors** with Python script
8. **Test motors** with simple serial commands before running full autonomous mode

---

---

## Troubleshooting Notes

### Motor Control Issues Resolved (2025-10-13)

**Issue:** Motors were moving sequentially instead of simultaneously during initial tests.

**Root Cause:** The MDDS30 motor driver is configured in **MIXED R/C MODE**, not independent dual-channel mode. Initial code attempted to control motors independently, which doesn't work with mixing enabled.

**Solution:** Modified Arduino code to work with the MDDS30's mixing mode:
- D5 (Steering) controls forward/backward for both motors
- D6 (Throttle) controls left/right differential for turning
- The driver automatically mixes these to achieve proper tank steering

**Implementation:**
1. Installed Arduino CLI on Raspberry Pi
2. Installed Servo library: `arduino-cli lib install Servo`
3. Created mixed-mode motor controller: `arduino_motor_control.ino`
4. Tested and verified all movements work correctly

**Current Status:** All motor movements verified working correctly with MIXED MODE:
- **FORWARD**: Steering forward (D5), Throttle center (D6) → Both motors forward
- **BACKWARD**: Steering backward (D5), Throttle center (D6) → Both motors backward
- **TURN LEFT**: Steering center (D5), Throttle forward (D6) → Left back, Right forward
- **TURN RIGHT**: Steering center (D5), Throttle backward (D6) → Left forward, Right back

**Key Insight:** The MDDS30 DIP switches must be set for MIXED R/C mode for this control scheme to work.

---

## Recent Updates

**2025-10-16:** Added side sensor wiring (2× additional HC-SR04 sensors for wall detection)
- Left side sensor: GPIO5 (TRIG), GPIO6 (ECHO)
- Right side sensor: GPIO22 (TRIG), GPIO25 (ECHO)
- Same voltage divider configuration as front sensors (1kΩ:2kΩ ratio)

**2025-10-13:** Documented MDDS30 mixed R/C mode configuration

---

Last Updated: 2025-10-16
