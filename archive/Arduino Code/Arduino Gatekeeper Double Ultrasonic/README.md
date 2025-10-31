# Arduino Rover Gatekeeper v2.0 - Dual Ultrasonic Sensor System

An advanced Arduino-based safety controller for autonomous rovers featuring dual ultrasonic sensors with intelligent obstacle detection and directional avoidance capabilities.

## 🚀 Features

### Core Functionality
- **Tri-mode operation**: Failsafe, Manual, and Autonomous modes
- **RC signal processing** with PinChangeInterrupt library
- **Motor control** via H-bridge drivers (L298N compatible)
- **Serial communication** with Raspberry Pi or other controllers
- **Real-time telemetry** and command acknowledgment

### Advanced Safety Systems
- **Dual ultrasonic sensors** with 30° angled coverage
- **Sensor fusion algorithm** for enhanced spatial awareness  
- **Directional obstacle detection** (left/center/right)
- **Intelligent avoidance maneuvers** 
- **Configurable emergency stop distance**
- **Visual status indication** via LED patterns

## 🔧 Hardware Requirements

### Arduino Board
- **Arduino Nano** (or compatible)
- USB programming cable

### Sensors
- **2x HC-SR04 Ultrasonic Sensors**
  - Positioned at front of rover
  - Angled 30° left and 30° right from center
  - Separated by 4.5 inches (11.43 cm)

### RC System
- **FS-IA10B Receiver** (or compatible)
- 3-channel minimum (throttle, steering, mode switch)

### Motor Control
- **Dual motor setup** (left/right drive)
- **H-bridge motor driver** (L298N recommended)

### Power Supply
- **5V for sensors and receiver** (Arduino 5V rail sufficient)
- **Separate motor power supply** recommended

## 📋 Pin Configuration

### RC Inputs
```
CH1_PIN = 2   // Forward/Reverse (Throttle)
CH2_PIN = 3   // Steering  
CH9_PIN = 4   // Mode Switch (3-position)
```

### Motor Outputs
```
LEFT_IN1 = 5    // Left motor direction 1
LEFT_IN2 = 6    // Left motor direction 2  
LEFT_ENA = 9    // Left motor PWM speed
RIGHT_IN1 = 7   // Right motor direction 1
RIGHT_IN2 = 10  // Right motor direction 2
RIGHT_ENA = 11  // Right motor PWM speed
```

### Ultrasonic Sensors
```
// Left Sensor (30° left angle)
LEFT_SENSOR_TRIG_PIN = 8   // Trigger
LEFT_SENSOR_ECHO_PIN = 12  // Echo

// Right Sensor (30° right angle)  
RIGHT_SENSOR_TRIG_PIN = A0 // Trigger (analog pin as digital)
RIGHT_SENSOR_ECHO_PIN = A1 // Echo (analog pin as digital)
```

### Status Indicator
```
LED_PIN = 13  // Built-in LED
```

## 🔌 Wiring Diagram

### HC-SR04 Connections
```
Left Sensor:
├── VCC → Arduino 5V
├── GND → Arduino GND  
├── Trig → Pin D8
└── Echo → Pin D12

Right Sensor:
├── VCC → Arduino 5V
├── GND → Arduino GND
├── Trig → Pin A0
└── Echo → Pin A1
```

### FS-IA10B Receiver
```
├── VCC → Arduino 5V
├── GND → Arduino GND
├── CH1 → Pin D2 (Throttle)
├── CH2 → Pin D3 (Steering)  
└── CH9 → Pin D4 (Mode Switch)
```

### Motor Driver (L298N)
```
Arduino → L298N:
├── D5 → IN1 (Left motor)
├── D6 → IN2 (Left motor)
├── D9 → ENA (Left motor PWM)
├── D7 → IN3 (Right motor)  
├── D10 → IN4 (Right motor)
└── D11 → ENB (Right motor PWM)

Power:
├── 12V → Motor power input
└── 5V → Logic power (or use Arduino 5V)
```

## 🎛️ Operating Modes

### Mode Selection (CH9 Channel)
- **Failsafe** (CH9 < -500): Emergency stop, motors disabled
- **Manual** (CH9 -500 to +500): Full RC control, safety overrides disabled
- **Autonomous** (CH9 > +500): Pi/Computer control with full safety systems

### LED Status Indicators
- **Solid**: Autonomous mode, valid Pi commands
- **Fast blink**: Emergency stop (obstacle detected)
- **Variable blink**: Directional obstacle indication
  - Slower = Left obstacle
  - Faster = Right obstacle  
- **Double blink**: Manual mode
- **Slow blink**: Valid RC signal, not autonomous
- **Very slow blink**: No RC signal

## 🧠 Sensor Fusion Algorithm

### Spatial Coverage
- **Left sensor**: 30° left coverage zone
- **Right sensor**: 30° right coverage zone  
- **Overlap zone**: Central area covered by both sensors
- **Total coverage**: ~60° forward-facing arc

### Obstacle Detection Logic
1. **Both sensors detect**: Center obstacle or narrow passage → Full stop
2. **Left sensor only**: Obstacle on left → Turn right
3. **Right sensor only**: Obstacle on right → Turn left  
4. **No detection**: Path clear → Normal operation

### Avoidance Maneuvers
```cpp
Left obstacle:  Turn right (L:50, R:-50)
Right obstacle: Turn left  (L:-50, R:50)  
Center/Both:    Full stop  (L:0, R:0)
```

## 📡 Serial Communication Protocol

### Command Format (Pi → Arduino)
```json
// Simple format
{"left": 100, "right": -50}

// Extended format  
{
  "motor": {"left": 100, "right": -50},
  "telemetry": {...}
}
```

### Telemetry Format (Arduino → Pi)
```json
{
  "ch1": 0,
  "ch2": 0, 
  "ch9": -1000,
  "valid": true,
  "mode": 0,
  "emergency": false,
  "sensors": {
    "left_distance": 25.3,
    "right_distance": 18.7,
    "min_distance": 18.7,
    "obstacle_direction": 1,
    "left_valid": true,
    "right_valid": true
  },
  "distance": 18.7,
  "pi_command_valid": false,
  "gatekeeper_mode": "FAILSAFE"
}
```

### Acknowledgment Format (Arduino → Pi)  
```json
{
  "pi_ack": {
    "left": 100,
    "right": -50, 
    "applied": true
  }
}
```

## ⚙️ Configuration Parameters

### Safety Settings
```cpp
#define EMERGENCY_DISTANCE 10.0    // Emergency stop distance (cm)
#define ULTRASONIC_TIMEOUT 30000   // Sensor timeout (µs)
#define PI_COMMAND_TIMEOUT 500     // Pi command timeout (ms)
```

### Physical Dimensions
```cpp
#define SENSOR_SEPARATION_CM 11.43  // 4.5 inches sensor spacing
#define SENSOR_ANGLE_DEG 30        // Sensor angle from center
```

### RC Parameters
```cpp  
#define RC_MIN_PULSE 1000    // Minimum RC pulse width
#define RC_MAX_PULSE 2000    // Maximum RC pulse width
#define RC_DEADBAND 50       // RC input deadband
#define RC_TIMEOUT 250       // RC signal timeout (ms)
```

## 🛠️ Installation & Setup

### Arduino IDE Setup
1. Install Arduino IDE 2.x
2. Install **PinChangeInterrupt** library
   - Library Manager → Search "PinChangeInterrupt"
   - Install by NicoHood

### Upload Process
1. Connect Arduino Nano via USB
2. Select **Board**: Arduino Nano
3. Select **Processor**: ATmega328P (Old Bootloader) 
4. Select correct **Port**: /dev/ttyUSB0 (Linux) or COM port (Windows)
5. Upload sketch

### Calibration
1. Power on system with RC transmitter
2. Verify mode switching on CH9
3. Test emergency stop with obstacle at 10cm
4. Confirm directional detection with angled obstacles

## 🔧 Troubleshooting

### Sensor Issues
- **"null" readings**: Check wiring, increase timeout value
- **Inconsistent readings**: Ensure sensors are firmly mounted
- **No directional detection**: Verify 30° sensor angles

### RC Problems  
- **Mode not switching**: Check CH9 wiring and transmitter setup
- **Erratic control**: Verify RC signal quality and connections
- **No response**: Confirm RC_TIMEOUT and pulse width ranges

### Motor Issues
- **Motors don't move**: Check H-bridge connections and power
- **Wrong direction**: Swap motor wires or adjust code polarity
- **Weak response**: Verify PWM pins and motor driver capacity

## 🤝 Contributing

Feel free to submit issues, fork the repository, and create pull requests for improvements.

### Potential Enhancements
- Multiple sensor arrays (side/rear sensors)
- IMU integration for heading control  
- GPS waypoint navigation
- Bluetooth/WiFi communication
- LIDAR integration
- Machine learning obstacle classification

## 📄 License

This project is open source. Please credit original authors when redistributing.

## 📞 Support

For questions or issues:
- Create GitHub issue with detailed description
- Include serial output and wiring photos  
- Specify hardware versions and modifications

---

**Version**: 2.0  
**Last Updated**: August 2025  
**Compatibility**: Arduino Nano, HC-SR04, FS-IA10B