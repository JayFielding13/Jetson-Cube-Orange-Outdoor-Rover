# ESP32 DevKit Ultrasonic Sensor Network - Wiring Guide

**Project:** Jetson Cube Orange Outdoor Rover
**Sensor Model:** AJ-SR04M Waterproof Ultrasonic Sensors (6 units)
**Controller:** ESP32 DevKit (38-pin, 19 pins per side)
**Level Shifter:** Bidirectional 5V ↔ 3.3V voltage shifter (for ECHO pins)
**Connection to Jetson:** USB Serial
**Created:** November 7, 2025

---

## Sensor Layout & Mounting

### Rover Sensor Placement (Top-Down View)

```
                    FRONT

      [Corner L] [Front] [Corner R]
            45°      0°       45°
              \      |      /
               \     |     /
                \    |    /
                 \   |   /
      [Side L]    ┌──┴──┐    [Side R]
       90°        │ ESP │      90°
       ◄──────────┤ 32  ├──────────►
                  │     │
                  └─────┘
                     |
                     |
                   [Rear]
                    180°

Legend:
[Front] = Front (0° - straight ahead)
[Corner L] = Corner Left (45° angled outward-left)
[Corner R] = Corner Right (45° angled outward-right)
[Side L] = Side Left (90° - perpendicular left)
[Side R] = Side Right (90° - perpendicular right)
[Rear]  = Rear (180° - straight back)
```

### Sensor Coverage Map

```
                360° Coverage

        315°  0°/360°  45°
           \    |    /
            \   |   /
    ────[Corner L][Front][Corner R]────
   |                                    |
270°[Side L]      ESP32      [Side R]90°
   |                                    |
    ────────────[Rear]──────────────────
                 180°
```

---

## ESP32 DevKit Pin Assignments

### Recommended GPIO Pin Configuration

| Sensor Position | Angle | TRIG Pin | ECHO Pin | Notes |
|----------------|-------|----------|----------|-------|
| **Front** | 0° | GPIO 15 | GPIO 2 | Straight ahead detection |
| **Corner Left** | 45° left | GPIO 13 | GPIO 12 | Left-forward coverage |
| **Corner Right** | 45° right | GPIO 5 | GPIO 27 | Right-forward coverage |
| **Side Left** | 90° left | GPIO 14 | GPIO 4 | Perpendicular left |
| **Side Right** | 90° right | GPIO 9 | GPIO 10 | Perpendicular right |
| **Rear** | 180° | GPIO 25 | GPIO 26 | Rear obstacle detection |

### Power Connections

| Pin | Function | Wire Color | Notes |
|-----|----------|------------|-------|
| **VIN** | Sensor VCC (5V) | Red | All 6 sensors (check current) |
| **GND** | Common Ground | Black | All 6 sensors + ESP32 |

---

## Detailed Pin Configuration

### ESP32 DevKit GPIO Characteristics

**Safe GPIO Pins for Ultrasonic Sensors:**
- All GPIO pins used are safe for general purpose I/O
- All pins are 3.3V tolerant
- **IMPORTANT:** Level shifter installed on all ECHO pins for 5V → 3.3V conversion

### ⚡ Voltage Level Shifter Configuration

**Protection Setup:**
```
AJ-SR04M ECHO (5V) → Level Shifter HV Side → Level Shifter LV Side → ESP32 GPIO (3.3V)
ESP32 TRIG (3.3V) → Direct connection OK (sensors accept 3.3V input)
```

**Level Shifter Connections:**
- **HV (High Voltage Side):** Connect to 5V power rail
- **LV (Low Voltage Side):** Connect to 3.3V power rail
- **GND:** Common ground for both sides
- **6 channels:** One for each ECHO pin (Front, Corner Left, Corner Right, Side Left, Side Right, Rear)

**Why level shifter needed:**
- AJ-SR04M sensors powered at 5V may output 5V on ECHO
- ESP32-S3 GPIO pins are 3.3V max tolerant
- Level shifter prevents damage to ESP32 inputs
- TRIG pins can connect directly (3.3V is sufficient to trigger sensors)

### Pin Definitions (Arduino Code)

```cpp
// AJ-SR04M Ultrasonic Sensor Pin Definitions
// ESP32 DevKit GPIO Assignments

// Front - Straight ahead (0°)
#define FRONT_TRIG_PIN 15  // L16
#define FRONT_ECHO_PIN 2   // L15

// Corner Left - Angled 45° outward-left
#define CORNER_LEFT_TRIG_PIN 13  // R15
#define CORNER_LEFT_ECHO_PIN 12  // R13

// Corner Right - Angled 45° outward-right
#define CORNER_RIGHT_TRIG_PIN 5   // L10
#define CORNER_RIGHT_ECHO_PIN 27  // R11

// Side Left - Perpendicular left (90°)
#define SIDE_LEFT_TRIG_PIN 14  // R12
#define SIDE_LEFT_ECHO_PIN 4   // L13

// Side Right - Perpendicular right (90°)
#define SIDE_RIGHT_TRIG_PIN 9   // R16
#define SIDE_RIGHT_ECHO_PIN 10  // R17

// Rear - Straight back (180°)
#define REAR_TRIG_PIN 25  // R9
#define REAR_ECHO_PIN 26  // R10

// Power
#define VCC_PIN  VIN  // ESP32 VIN for 5V power
#define GND_PIN  GND  // Common ground
```

---

## Complete Wiring Diagram with Level Shifter

### System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    POWER DISTRIBUTION                            │
└─────────────────────────────────────────────────────────────────┘
                │                    │
           5V Power Rail         3.3V Power Rail
                │                    │
                ├─────────────────────────────┐
                │                             │
        ┌───────▼──────────┐        ┌────────▼─────────┐
        │  AJ-SR04M × 6    │        │   ESP32 DevKit  │
        │  VCC (5V input)  │        │  3.3V Logic     │
        └───────┬──────────┘        └────────┬─────────┘
                │                             │
         ┌──────┴──────┐             ┌───────┴────────┐
         │             │             │                │
    TRIG Pins    ECHO Pins      TRIG Pins       ECHO Pins
    (Output)     (Output 5V!)    (Output 3.3V)  (Input 3.3V)
         │             │             │                │
         │             │             │                │
         └─────────────┼─────────────┘                │
              Direct   │                              │
              Connection                               │
                       │                              │
                       │        ┌──────────────┐      │
                       └───────►│ Level Shifter│◄─────┘
                                │  (6 Channel) │
                                │ 5V ↔ 3.3V   │
                                └──────────────┘
                                HV Side  LV Side
                                 (5V)    (3.3V)
```

### Detailed Connection Path (Per Sensor)

**Example: Front Sensor**

```
AJ-SR04M Sensor                Level Shifter              ESP32 DevKit
┌──────────────┐              ┌──────────────┐         ┌──────────┐
│              │              │              │         │          │
│  VCC ────────┼──────────────┼──> 5V Rail   │         │          │
│              │              │              │         │          │
│  GND ────────┼──────────────┼──> GND Rail  │         │          │
│              │              │              │         │          │
│  TRIG◄───────┼──────────────┼──────────────┼─────────┤ GPIO 15  │
│  (Input)     │              │              │ 3.3V    │ (Output) │
│              │              │              │         │          │
│  ECHO────────┼──> HV1 ───── │ ──── LV1 ────┼─────────┤ GPIO 2   │
│  (Output 5V) │    (5V side) │  (3.3V side) │         │ (Input)  │
│              │              │              │         │          │
└──────────────┘              └──────────────┘         └──────────┘

TRIG: Direct 3.3V connection (sensors accept 3.3V trigger)
ECHO: Through level shifter (5V → 3.3V conversion)
```

### Level Shifter Channel Assignments

| Sensor | ECHO Output | Level Shifter Channel | ESP32 GPIO Input |
|--------|-------------|----------------------|------------------|
| Front | Front ECHO (5V) | HV1 → LV1 | GPIO 2 (3.3V) |
| Corner Left | Corner L ECHO (5V) | HV6 → LV6 | GPIO 12 (3.3V) |
| Corner Right | Corner R ECHO (5V) | HV3 → LV3 | GPIO 27 (3.3V) |
| Side Left | Side L ECHO (5V) | HV4 → LV4 | GPIO 4 (3.3V) |
| Side Right | Side R ECHO (5V) | HV5 → LV5 | GPIO 10 (3.3V) |
| Rear | Rear ECHO (5V) | HV2 → LV2 | GPIO 26 (3.3V) |

### Level Shifter Power Connections

**Typical 6-Channel Bidirectional Level Shifter:**
```
┌────────────────────────────────────┐
│   Voltage Level Shifter Board      │
├────────────────────────────────────┤
│ HV (5V Side)     │  LV (3.3V Side) │
├──────────────────┼─────────────────┤
│ HV  → 5V         │  LV  → 3.3V     │
│ GND → Common     │  GND → Common   │
│ HV1 ← Front ECHO │  LV1 → GPIO 2   │
│ HV2 ← Rear ECHO  │  LV2 → GPIO 26  │
│ HV3 ← Corner R   │  LV3 → GPIO 27  │
│ HV4 ← Side L     │  LV4 → GPIO 4   │
│ HV5 ← Side R     │  LV5 → GPIO 10  │
│ HV6 ← Corner L   │  LV6 → GPIO 12  │
└──────────────────┴─────────────────┘
```

---

## Wiring Checklist

### Pre-Wiring Preparation

- [ ] ESP32 DevKit development board ready (38-pin, 19 per side)
- [ ] 6× AJ-SR04M sensors
- [ ] 6-channel bidirectional logic level shifter (5V ↔ 3.3V)
- [ ] 24× Female-to-Female jumper wires (or custom harness)
- [ ] Additional wires for level shifter connections
- [ ] Breadboard or perfboard for power distribution
- [ ] Micro-USB cable for ESP32 to Jetson connection
- [ ] Multimeter for continuity testing

### Level Shifter Setup (Do This First!)

- [ ] Mount level shifter on breadboard/perfboard
- [ ] Connect HV (high voltage) pin to 5V power rail
- [ ] Connect LV (low voltage) pin to 3.3V power rail
- [ ] Connect both GND pins to common ground rail
- [ ] Verify power connections with multimeter:
  - HV side: 5V ±0.25V
  - LV side: 3.3V ±0.15V
- [ ] Label each channel (HV1-HV6, LV1-LV6)

### Sensor #1 - Front

| AJ-SR04M Pin | Wire Color | Connection Path | ESP32 Pin |
|--------------|------------|-----------------|-----------|
| VCC | Red | → 5V rail | - |
| GND | Black | → GND rail | - |
| TRIG | Yellow | → Direct connection | GPIO 15 (L16) |
| ECHO | Blue | → Level Shifter HV1 → LV1 | GPIO 2 (L15) |

**Mounting:** Front center of rover, facing straight ahead (0°)
**Level Shifter:** Channel 1 (HV1 ← ECHO, LV1 → GPIO 2)

---

### Sensor #2 - Corner Left

| AJ-SR04M Pin | Wire Color | Connection Path | ESP32 Pin |
|--------------|------------|-----------------|-----------|
| VCC | Red | → 5V rail | - |
| GND | Black | → GND rail | - |
| TRIG | Yellow | → Direct connection | GPIO 13 (R15) |
| ECHO | Blue | → Level Shifter HV6 → LV6 | GPIO 12 (R13) |

**Mounting:** Front-left corner, angled 45° outward from centerline
**Level Shifter:** Channel 6 (HV6 ← ECHO, LV6 → GPIO 12)

---

### Sensor #3 - Corner Right

| AJ-SR04M Pin | Wire Color | Connection Path | ESP32 Pin |
|--------------|------------|-----------------|-----------|
| VCC | Red | → 5V rail | - |
| GND | Black | → GND rail | - |
| TRIG | Yellow | → Direct connection | GPIO 5 (L10) |
| ECHO | Blue | → Level Shifter HV3 → LV3 | GPIO 27 (R11) |

**Mounting:** Front-right corner, angled 45° outward from centerline
**Level Shifter:** Channel 3 (HV3 ← ECHO, LV3 → GPIO 27)

---

### Sensor #4 - Side Left

| AJ-SR04M Pin | Wire Color | Connection Path | ESP32 Pin |
|--------------|------------|-----------------|-----------|
| VCC | Red | → 5V rail | - |
| GND | Black | → GND rail | - |
| TRIG | Yellow | → Direct connection | GPIO 14 (R12) |
| ECHO | Blue | → Level Shifter HV4 → LV4 | GPIO 4 (L13) |

**Mounting:** Left side of rover, perpendicular to body (90° from centerline)
**Level Shifter:** Channel 4 (HV4 ← ECHO, LV4 → GPIO 4)

---

### Sensor #5 - Side Right

| AJ-SR04M Pin | Wire Color | Connection Path | ESP32 Pin |
|--------------|------------|-----------------|-----------|
| VCC | Red | → 5V rail | - |
| GND | Black | → GND rail | - |
| TRIG | Yellow | → Direct connection | GPIO 9 (R16) |
| ECHO | Blue | → Level Shifter HV5 → LV5 | GPIO 10 (R17) |

**Mounting:** Right side of rover, perpendicular to body (90° from centerline)
**Level Shifter:** Channel 5 (HV5 ← ECHO, LV5 → GPIO 10)

---

### Sensor #6 - Rear

| AJ-SR04M Pin | Wire Color | Connection Path | ESP32 Pin |
|--------------|------------|-----------------|-----------|
| VCC | Red | → 5V rail | - |
| GND | Black | → GND rail | - |
| TRIG | Yellow | → Direct connection | GPIO 25 (R9) |
| ECHO | Blue | → Level Shifter HV2 → LV2 | GPIO 26 (R10) |

**Mounting:** Rear center of rover, facing straight back (180°)
**Level Shifter:** Channel 2 (HV2 ← ECHO, LV2 → GPIO 26)

---

## Power Distribution

### Power Budget Calculation

**AJ-SR04M Current Draw:**
- Idle: ~5mA per sensor
- Active (measuring): ~30mA per sensor
- Peak: All 6 sensors = 6 × 30mA = **180mA max**

**ESP32 DevKit VIN Pin:**
- Can supply: Up to **500mA** (from USB)
- Required: 180mA + ESP32 consumption (~200mA)
- **Total: ~380mA** ✓ Safe

### Recommended Power Setup

**Option 1: Power from ESP32 DevKit (Recommended for testing)**
```
ESP32 VIN Pin → Power Rail → All 6 sensor VCC pins
ESP32 GND     → Ground Rail → All 6 sensor GND pins
```

**Option 2: External 5V Supply (Recommended for production)**
```
External 5V PSU → Power Rail → All 6 sensor VCC pins + ESP32 VIN
Common GND      → Ground Rail → All sensors + ESP32 GND
```

**Why external power?**
- Reduces noise on ESP32 power rail
- More stable voltage for sensors
- Allows for higher current if needed

---

## AJ-SR04M Specifications

### Technical Details

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Operating Voltage** | 3.0V - 5.5V | Works with 3.3V or 5V |
| **Current Draw** | 30mA max | Per sensor |
| **Measurement Range** | 20cm - 600cm | 0.2m - 6m |
| **Detection Angle** | 70° cone | Wide coverage |
| **Accuracy** | ±1cm | High precision |
| **Trigger Pulse** | 10μs | Standard |
| **Echo Signal** | 3.3V or 5V | 3.3V safe for ESP32 |
| **Waterproof Rating** | IP67 | Outdoor capable |
| **Operating Temp** | -10°C to +70°C | All-weather |

### AJ-SR04M vs HC-SR04 Advantages

✅ **Waterproof (IP67)** - Can handle rain, dust, splash
✅ **Longer range** - Up to 6m vs 4m
✅ **Better accuracy** - ±1cm vs ±3cm
✅ **Small size** - Easier to mount
✅ **3.3V compatible** - No voltage divider needed for ECHO
✅ **Industrial grade** - More reliable in harsh conditions

---

## ESP32 DevKit to Jetson Connection

### USB Serial Interface

**Hardware:**
```
ESP32 DevKit Micro-USB Port → USB-A Cable → Jetson Orin Nano USB Port
```

**Serial Configuration:**
- **Baud Rate:** 115200 (recommended) or 921600 (high-speed)
- **Data Format:** 8N1 (8 data bits, no parity, 1 stop bit)
- **Flow Control:** None
- **Device Path:** `/dev/ttyUSB0` or `/dev/ttyACM0` (on Jetson)

### Protocol Design

**Data Packet Format (JSON):**
```json
{
  "timestamp": 1234567890,
  "sensors": {
    "front": 150,         // Front distance (cm)
    "corner_left": 200,   // Corner Left distance (cm)
    "corner_right": 180,  // Corner Right distance (cm)
    "side_left": 120,     // Side Left distance (cm)
    "side_right": 110,    // Side Right distance (cm)
    "rear": 300           // Rear distance (cm)
  },
  "valid": {
    "front": true,
    "corner_left": true,
    "corner_right": true,
    "side_left": true,
    "side_right": true,
    "rear": true
  }
}
```

**Update Rate:** 10-20 Hz (all sensors, sequential read)

---

## Arduino Code Template (ESP32 DevKit)

### Basic Sensor Reading Code

```cpp
#include <Arduino.h>

// Pin definitions for ESP32 DevKit (38-pin)
#define FRONT_TRIG_PIN 15        // L16 - Front TRIG
#define FRONT_ECHO_PIN 2         // L15 - Front ECHO (via level shifter)
#define CORNER_LEFT_TRIG_PIN 25  // R9  - Corner Left TRIG
#define CORNER_LEFT_ECHO_PIN 26  // R10 - Corner Left ECHO (via level shifter)
#define CORNER_RIGHT_TRIG_PIN 5  // L10 - Corner Right TRIG
#define CORNER_RIGHT_ECHO_PIN 27 // R11 - Corner Right ECHO (via level shifter)
#define SIDE_LEFT_TRIG_PIN 14    // R12 - Side Left TRIG
#define SIDE_LEFT_ECHO_PIN 4     // L13 - Side Left ECHO (via level shifter)
#define SIDE_RIGHT_TRIG_PIN 9    // R16 - Side Right TRIG
#define SIDE_RIGHT_ECHO_PIN 10   // R17 - Side Right ECHO (via level shifter)
#define REAR_TRIG_PIN 13         // R15 - Rear TRIG
#define REAR_ECHO_PIN 12         // R13 - Rear ECHO (via level shifter)

// Sensor parameters
#define MAX_DISTANCE 600  // cm (6m)
#define MIN_DISTANCE 20   // cm
#define TIMEOUT_US 35000  // 35ms timeout (for 6m range)

// Sensor structure
struct Sensor {
  int trig;
  int echo;
  const char* name;
  float distance;
  bool valid;
};

// Define all 6 sensors
Sensor sensors[6] = {
  {FRONT_TRIG_PIN, FRONT_ECHO_PIN, "front", 0, false},              // Front
  {CORNER_LEFT_TRIG_PIN, CORNER_LEFT_ECHO_PIN, "corner_left", 0, false},  // Corner Left
  {CORNER_RIGHT_TRIG_PIN, CORNER_RIGHT_ECHO_PIN, "corner_right", 0, false}, // Corner Right
  {SIDE_LEFT_TRIG_PIN, SIDE_LEFT_ECHO_PIN, "side_left", 0, false},  // Side Left
  {SIDE_RIGHT_TRIG_PIN, SIDE_RIGHT_ECHO_PIN, "side_right", 0, false}, // Side Right
  {REAR_TRIG_PIN, REAR_ECHO_PIN, "rear", 0, false}                  // Rear
};

void setup() {
  Serial.begin(115200);

  // Initialize all sensor pins
  for (int i = 0; i < 6; i++) {
    pinMode(sensors[i].trig, OUTPUT);
    pinMode(sensors[i].echo, INPUT);
    digitalWrite(sensors[i].trig, LOW);
  }

  delay(100);
  Serial.println("AJ-SR04M Sensor Array Ready");
}

void loop() {
  // Read all sensors
  for (int i = 0; i < 6; i++) {
    readSensor(&sensors[i]);
    delay(10);  // Small delay between sensors
  }

  // Send JSON data to Jetson
  sendSensorData();

  delay(50);  // 20Hz update rate
}

void readSensor(Sensor* sensor) {
  // Send trigger pulse
  digitalWrite(sensor->trig, LOW);
  delayMicroseconds(2);
  digitalWrite(sensor->trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor->trig, LOW);

  // Measure echo pulse
  unsigned long duration = pulseIn(sensor->echo, HIGH, TIMEOUT_US);

  // Calculate distance (speed of sound = 343 m/s = 0.0343 cm/μs)
  // Distance = (duration / 2) * 0.0343
  float distance = (duration / 2.0) * 0.0343;

  // Validate measurement
  if (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE) {
    sensor->distance = distance;
    sensor->valid = true;
  } else {
    sensor->distance = 0;
    sensor->valid = false;
  }
}

void sendSensorData() {
  // Create JSON packet
  Serial.print("{");
  Serial.print("\"timestamp\":");
  Serial.print(millis());
  Serial.print(",\"sensors\":{");

  for (int i = 0; i < 6; i++) {
    Serial.print("\"");
    Serial.print(sensors[i].name);
    Serial.print("\":");
    Serial.print(sensors[i].distance, 1);
    if (i < 5) Serial.print(",");
  }

  Serial.print("},\"valid\":{");

  for (int i = 0; i < 6; i++) {
    Serial.print("\"");
    Serial.print(sensors[i].name);
    Serial.print("\":");
    Serial.print(sensors[i].valid ? "true" : "false");
    if (i < 5) Serial.print(",");
  }

  Serial.println("}}");
}
```

---

## Physical Mounting Guidelines

### Sensor Placement Rules

**Height from Ground:**
- Mount sensors 10-15cm above ground level
- Ensures detection of low obstacles
- Avoids ground reflections

**Sensor Angles:**
- Front: 0° (straight ahead)
- Corner Left/Right: 45° angled outward
- Side Left/Right: 90° perpendicular
- Rear: 180° (straight back)

**Mounting Method:**
- Use waterproof sensor mounts (AJ-SR04M includes mounting bracket)
- Secure with M2 or M3 screws
- Ensure sensors are level and stable
- Keep sensor face clear of obstructions

**Cable Management:**
- Use cable ties or wire guides
- Avoid sharp bends in sensor cables
- Keep power and signal wires separated if possible
- Label each sensor cable at both ends

---

## Integration with ROS2 (Jetson Side)

### ROS2 Node Overview

The ESP32-S3 will connect to the Jetson via USB and communicate sensor data using a ROS2 node.

**ROS2 Topic Structure:**
```
/ultrasonic/front_center       (sensor_msgs/Range)
/ultrasonic/front_left         (sensor_msgs/Range)
/ultrasonic/front_right        (sensor_msgs/Range)
/ultrasonic/left_corner        (sensor_msgs/Range)
/ultrasonic/right_corner       (sensor_msgs/Range)
/ultrasonic/rear               (sensor_msgs/Range)
/ultrasonic/scan               (sensor_msgs/LaserScan) - Combined view
```

### Python Serial Reader (Jetson)

```python
#!/usr/bin/env python3
import serial
import json
import time

# Serial port configuration
SERIAL_PORT = '/dev/ttyUSB0'  # or /dev/ttyACM0
BAUD_RATE = 115200

def read_ultrasonic_data():
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        print(f"Connected to ESP32-S3 on {SERIAL_PORT}")

        while True:
            line = ser.readline().decode('utf-8').strip()

            if line:
                try:
                    data = json.loads(line)
                    print(f"Timestamp: {data['timestamp']}")
                    print(f"Sensors: {data['sensors']}")
                    print(f"Valid: {data['valid']}")
                    print("-" * 40)

                    # Publish to ROS2 topics here
                    # publish_sensor_data(data)

                except json.JSONDecodeError:
                    print(f"Invalid JSON: {line}")

if __name__ == "__main__":
    read_ultrasonic_data()
```

---

## Testing Procedure

### 1. Bench Test (Before Mounting)

- [ ] Power ESP32-S3 via USB
- [ ] Upload test firmware
- [ ] Open Serial Monitor (115200 baud)
- [ ] Test each sensor individually:
  - Hold hand 20cm in front
  - Move hand to 50cm, 100cm, 200cm
  - Verify readings are accurate
- [ ] Test all sensors simultaneously
- [ ] Check for crosstalk or interference

### 2. Connectivity Test

- [ ] Connect ESP32-S3 to Jetson via USB
- [ ] Check device appears: `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`
- [ ] Test serial communication:
  ```bash
  sudo chmod 666 /dev/ttyUSB0
  python3 serial_reader.py
  ```
- [ ] Verify JSON data received correctly

### 3. Mounted Test

- [ ] Mount all sensors on rover
- [ ] Verify sensor angles and heights
- [ ] Test in outdoor environment
- [ ] Check for obstacles detection:
  - Wall at 1m - all front sensors should detect
  - Person walking on left side - LC should detect
  - Obstacle behind - Rear sensor should detect
- [ ] Verify no false positives from rover body

---

## Troubleshooting

### Problem: No readings from sensor

**Check:**
1. Wiring connections (TRIG, ECHO, VCC, GND)
2. Sensor power (5V present?)
3. GPIO pin numbers in code match physical wiring
4. Serial monitor shows sensor output?

### Problem: Erratic or invalid readings

**Check:**
1. Sensor mounting (stable, not vibrating?)
2. Obstacle surface (absorbent materials give poor readings)
3. Crosstalk between sensors (add delay between readings)
4. Power supply noise (add decoupling capacitors)

### Problem: Short range only

**Check:**
1. Timeout value in code (increase TIMEOUT_US)
2. Sensor blocked or dirty
3. Operating in very hot/cold environment

### Problem: ESP32 not detected by Jetson

**Check:**
1. USB cable (data cable, not charging-only)
2. Jetson USB port functional
3. ESP32 driver installed: `sudo apt install python3-serial`
4. Permissions: `sudo usermod -a -G dialout $USER`

---

## Safety & Best Practices

### Electrical Safety

- [ ] Double-check polarity before powering sensors
- [ ] Use common ground for ESP32 and all sensors
- [ ] Add 100μF capacitor across power rails for stability
- [ ] Protect ESP32 from reverse polarity (diode on VIN)

### Environmental Protection

- [ ] Seal cable connections with heat shrink or silicone
- [ ] Mount ESP32 in waterproof enclosure
- [ ] Use cable glands for wire entry points
- [ ] Protect USB connector from moisture

### Operational Safety

- [ ] Test all sensors before outdoor use
- [ ] Verify emergency stop still works with sensors
- [ ] Don't rely solely on ultrasonics (use with LiDAR + camera)
- [ ] Set conservative obstacle thresholds (50cm+ for moving)

---

## Next Steps

1. **Wire all sensors** according to this guide
2. **Upload test firmware** to ESP32-S3
3. **Verify serial communication** with Jetson
4. **Create ROS2 node** for sensor integration
5. **Integrate with autonomous navigation** system
6. **Test in outdoor environment**

---

## File References

- ESP32 Firmware: `esp32_ultrasonic_array.ino` (to be created)
- Python ROS2 Node: `ultrasonic_ros2_node.py` (to be created)
- Configuration: Update `config.py` with sensor parameters
- Integration docs: See `README.md` for system architecture

---

**Created:** November 7, 2025
**Author:** Jay
**Project:** Jetson Cube Orange Outdoor Rover
**Status:** Wiring phase - sensors ready for connection
**Protection:** 6-channel bidirectional level shifter for ECHO pin protection (5V → 3.3V)

---

## Important Notes

✅ **Level Shifter is Installed** - All ECHO pins pass through bidirectional 5V ↔ 3.3V level shifter
✅ **ESP32 Protected** - No 5V signals reach ESP32-S3 GPIO inputs
✅ **TRIG Pins Direct** - 3.3V output from ESP32 is sufficient to trigger AJ-SR04M sensors
✅ **Waterproof Sensors** - AJ-SR04M rated IP67 for outdoor use
✅ **6m Range** - Superior range compared to standard HC-SR04 (4m)

**Last Updated:** November 7, 2025
