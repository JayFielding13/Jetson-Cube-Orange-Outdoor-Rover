# Modular Rover Code

**Clean, foundational robotics architecture for the Mini Rover Development Project**

## 🎯 Overview

This modular code structure provides a solid foundation for rover development by separating concerns into independent, testable components. Perfect for building from basic motor control to advanced autonomous behaviors.

## 📁 Architecture

```
Modular Pi Code/
├── main.py              # Main program coordinator
├── arduino_interface.py # Serial communication with Arduino
├── motor_controller.py  # High-level movement commands
├── config.py           # Centralized configuration
└── README.md           # This file
```

## 🚀 Quick Start

### Prerequisites
- Python 3.7+
- `pyserial` library: `pip install pyserial`
- Arduino connected via USB/Serial

### Running the System
```bash
cd "Mini Rover Development/Modular Pi Code"
python3 main.py
```

### Controls (when running)
- `w/s` - Forward/Backward
- `a/d` - Turn Left/Right  
- `q/e` - Curve Left/Right
- `x` - Stop
- `ESC` - Emergency Stop & Exit

## 🧩 Module Details

### `arduino_interface.py` - Core Communication
**Purpose:** Low-level serial communication with Arduino gatekeeper

**Key Features:**
- Thread-safe JSON protocol
- Connection health monitoring
- Automatic error recovery
- Data callbacks for real-time processing

**Usage:**
```python
from arduino_interface import ArduinoInterface

arduino = ArduinoInterface('/dev/ttyUSB0')
arduino.connect()
arduino.send_motor_command(100, -50)  # Left, Right speeds
data = arduino.read_data()  # Get sensor readings
```

### `motor_controller.py` - Movement Control
**Purpose:** High-level movement commands with safety controls

**Key Features:**
- Intuitive movement functions
- Speed validation and limits
- Safety thresholds
- State tracking

**Usage:**
```python
from motor_controller import MotorController

motors = MotorController(arduino_interface)
motors.forward()           # Move forward at cruise speed
motors.turn_left(120)      # Turn left at custom speed
motors.curve_right()       # Gentle right curve
motors.timed_movement(80, -80, 2.0)  # Turn for 2 seconds
```

### `config.py` - Configuration Management
**Purpose:** Centralized settings for all system parameters

**Key Features:**
- Dataclass-based configuration
- Environment variable support
- Validation system
- Easy parameter tuning

**Usage:**
```python
from config import config

# Access settings
port = config.hardware.arduino_port
speed = config.motor.cruise_speed

# Environment variables (optional)
export ROVER_CRUISE_SPEED=120
export ROVER_ARDUINO_PORT=/dev/ttyACM0
```

### `main.py` - System Coordinator
**Purpose:** Integrates all modules into a working rover system

**Key Features:**
- Clean startup/shutdown
- Real-time keyboard control
- Background monitoring threads
- Error handling and recovery

## ⚙️ Configuration Options

### Hardware Settings
- `arduino_port` - Serial port path (default: `/dev/ttyUSB0`)
- `arduino_baud` - Baud rate (default: `115200`)
- `arduino_timeout` - Communication timeout (default: `1.0s`)

### Motor Settings
- `cruise_speed` - Normal forward speed (default: `100`)
- `turn_speed` - Turning speed (default: `80`)
- `speed_limit` - Safety speed limit (default: `200`)
- `min_speed` - Minimum effective speed (default: `30`)

### System Settings
- `status_update_interval` - Status display frequency (default: `3.0s`)
- `sensor_read_rate` - Arduino data reading rate (default: `50Hz`)
- `main_loop_rate` - Main control loop rate (default: `10Hz`)

## 🛡️ Safety Features

- **Speed Validation** - All motor commands are clamped to safe ranges
- **Connection Monitoring** - Automatic detection of Arduino disconnection
- **Emergency Stops** - Multiple levels of emergency stopping
- **Timeout Handling** - Graceful handling of communication failures
- **Thread Safety** - Safe concurrent access to shared resources

## 🔧 Extending the System

### Adding New Sensors
```python
# Example: Adding ultrasonic sensor module
class UltrasonicSensor:
    def __init__(self, arduino_interface):
        self.arduino = arduino_interface
        self.distance = 200.0
    
    def update(self, arduino_data):
        self.distance = arduino_data.get('distance', 200.0)
    
    def get_distance(self):
        return self.distance
```

### Adding Autonomous Behaviors
```python
# Example: Adding simple obstacle avoidance
class ObstacleAvoidance:
    def __init__(self, motors, sensor):
        self.motors = motors
        self.sensor = sensor
        self.danger_threshold = 30.0
    
    def update(self):
        if self.sensor.get_distance() < self.danger_threshold:
            self.motors.turn_left()
        else:
            self.motors.forward()
```

## 🧪 Testing Individual Modules

Each module can be tested independently:

```bash
# Test Arduino communication
python3 arduino_interface.py

# Test motor control
python3 motor_controller.py

# Test configuration
python3 config.py
```

## 📊 Monitoring and Debugging

The system provides real-time status information:
- Arduino connection health
- Current motor speeds
- Sensor readings (if available)
- System timing and performance

Enable debug mode:
```bash
export ROVER_DEBUG=true
python3 main.py
```

## 🚧 Future Expansion Ideas

This foundation makes it easy to add:
- **Sensors** - Cameras, LIDAR, GPS, IMU
- **Navigation** - Path planning, SLAM, mapping
- **Communication** - WiFi control, Bluetooth, telemetry  
- **Autonomy** - Behavior trees, state machines
- **Safety** - Watchdog timers, failsafe systems

## 📋 Requirements

- **Python 3.7+**
- **pyserial** - For Arduino communication
- **Arduino with compatible sketch** - JSON protocol support
- **Linux/Raspberry Pi** - Tested on Raspberry Pi OS

## 🐛 Troubleshooting

**Arduino not connecting:**
- Check USB cable and port permissions
- Verify baud rate matches Arduino sketch
- Try different port: `/dev/ttyACM0`, `/dev/ttyUSB1`

**Permission denied:**
```bash
sudo usermod -a -G dialout $USER  # Add user to dialout group
sudo chmod 666 /dev/ttyUSB0       # Temporary permission fix
```

**Module import errors:**
- Ensure all files are in the same directory
- Check Python path and working directory

## 📝 License

Part of the Mini Rover Development Project - Open for educational and development use.

---

**Ready to build something awesome? Start with this solid foundation! 🤖**