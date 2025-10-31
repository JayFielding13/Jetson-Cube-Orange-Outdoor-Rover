# Rover Autonomous Exploration System

**Complete modular robotics system demonstrating obstacle avoidance and exploration**

## 🎯 What This System Does

Your rover will autonomously:
1. **Move forward** when path is clear
2. **Detect obstacles** with ultrasonic sensor
3. **Turn left or right** to avoid obstacles  
4. **Resume forward movement** when clear
5. **Make random exploration turns** to explore new areas
6. **Recover from stuck situations** automatically

## 🏗️ Modular Architecture

```
Modular Pi Code/
├── Main/                    # Core foundation
│   ├── arduino_interface.py # Arduino communication
│   ├── motor_controller.py  # Movement commands
│   ├── config.py           # System configuration
│   └── main.py             # Manual control system
├── Sensors/                 # Sensor processing
│   └── ultrasonic_sensor.py # Distance measurement & filtering
├── Navigation/              # Navigation algorithms  
│   └── obstacle_avoidance.py # Reactive obstacle avoidance
├── Autonomous Behaviors/    # High-level behaviors
│   └── simple_exploration.py # Exploration with stuck recovery
└── rover_exploration_test.py # Complete integration test
```

## 🚀 Quick Start

### Prerequisites
- Arduino connected via USB (`/dev/ttyUSB0` or `/dev/ttyACM0`)
- Ultrasonic sensor connected to Arduino
- Arduino running compatible sketch (JSON protocol)

### Running the System
```bash
cd "Mini Rover Development/Modular Pi Code"
python3 rover_exploration_test.py
```

### What You'll See
```
🤖 ROVER EXPLORATION SYSTEM
============================
🔧 Initializing rover components...
  📡 Setting up Arduino interface...
  🎛️ Setting up motor controller...
  📏 Setting up ultrasonic sensor...
  🧭 Setting up obstacle avoidance...
  🎯 Setting up exploration behavior...
✅ All components initialized successfully

🚀 Starting autonomous exploration...
🤖 Rover exploration active - Press Ctrl+C to stop
```

## 📊 Real-Time Monitoring

The system provides detailed status updates:

```
📊 === Rover Status ===
🎯 Exploration: EXPLORING | Runtime: 45.2s
🧭 Navigation: FORWARD | Distance: 87.3cm (SAFE)
📈 Performance: 3 turns | 2 obstacles | 0 recoveries
📊 Sensor: 98.5% accuracy | 234 readings
```

## ⚙️ Configuration

Edit settings in `Main/config.py`:

```python
# Motor speeds
cruise_speed = 100        # Normal forward speed
turn_speed = 80          # Turning speed

# Sensor thresholds  
obstacle_threshold = 30.0  # Stop and turn below this (cm)
safe_threshold = 80.0     # Safe to move above this (cm)

# Exploration behavior
exploration_turn_interval = 8.0  # Seconds between random turns
stuck_threshold = 5.0            # Seconds close to obstacle = stuck
```

## 🧩 How the Modules Work Together

### Data Flow
```
Arduino → ArduinoInterface → UltrasonicSensor
                                    ↓
SimpleExploration ← ObstacleAvoidance ← Sensor Data
        ↓
MotorController → ArduinoInterface → Arduino
```

### Module Responsibilities

**UltrasonicSensor** (`Sensors/`)
- Filters noisy distance readings
- Detects obstacles vs. safe zones
- Provides callbacks for real-time updates

**ObstacleAvoidance** (`Navigation/`)
- State machine: FORWARD → TURNING → FORWARD
- Alternating turn directions
- Configurable speeds and timing

**SimpleExploration** (`Autonomous Behaviors/`)
- Adds random exploration turns
- Detects stuck conditions
- Coordinates with obstacle avoidance

**MotorController** (`Main/`)
- High-level movement commands
- Speed validation and safety limits
- Emergency stop capability

## 🎛️ Testing Individual Modules

Each module can be tested independently:

```bash
# Test ultrasonic sensor
cd Sensors
python3 ultrasonic_sensor.py

# Test obstacle avoidance
cd Navigation  
python3 obstacle_avoidance.py

# Test exploration behavior
cd "Autonomous Behaviors"
python3 simple_exploration.py
```

## 📈 Performance Features

**Smart Sensor Processing:**
- Moving average filter for stable readings
- Invalid reading rejection
- Accuracy statistics tracking

**Robust Navigation:**
- Alternating turn directions prevent loops
- Configurable speeds for different conditions
- Safety timeouts and error recovery

**Exploration Intelligence:**
- Random exploration turns when safe
- Stuck detection and recovery
- Area exploration tracking

**Comprehensive Monitoring:**
- Real-time status display
- Performance statistics
- Error detection and reporting

## 🛡️ Safety Features

- **Emergency stops** on sensor failure
- **Connection monitoring** with timeouts
- **Speed limits** and validation
- **Graceful shutdown** on Ctrl+C
- **Error recovery** for stuck conditions

## 🔧 Customization Examples

**Change exploration behavior:**
```python
# More aggressive exploration
explorer.set_exploration_parameters(
    turn_interval=5.0,      # Turn every 5 seconds
    turn_chance=0.9,        # 90% chance to turn
    stuck_threshold=3.0     # Detect stuck faster
)
```

**Adjust navigation speeds:**
```python
# Faster navigation
navigator.set_speeds(
    forward=120,    # Faster forward
    turn=100,       # Faster turns
    backup=80       # Faster backup
)
```

**Modify sensor thresholds:**
```python
# More cautious obstacle detection
sensor.set_thresholds(
    obstacle=40.0,  # Detect obstacles farther away
    warning=80.0,   # Warning zone
    safe=120.0      # Need more space to feel safe
)
```

## 🐛 Troubleshooting

**Arduino not connecting:**
- Check USB cable and port (`ls /dev/tty*`)
- Verify baud rate matches Arduino sketch
- Try different port: `/dev/ttyACM0`

**Sensor readings invalid:**
- Check ultrasonic sensor wiring
- Verify Arduino sketch sends `distance` in JSON
- Check for electrical interference

**Robot doesn't move:**
- Verify motor controller connections
- Check Arduino receives motor commands
- Test with manual control first (`Main/main.py`)

**Exploration seems random:**
- This is normal! It's a reactive system
- For better navigation, add mapping/planning modules
- Current system is intentionally simple

## 🚧 Future Enhancements

The modular design makes it easy to add:

**Sensors/**
- Camera vision processing
- LIDAR mapping
- IMU orientation tracking
- GPS positioning

**Navigation/**
- SLAM (mapping)
- Path planning algorithms
- Waypoint following
- Formation control

**Autonomous Behaviors/**
- Object following
- Area patrol patterns
- Search and rescue behaviors
- Swarm coordination

---

**Ready to explore! Your rover will navigate autonomously while avoiding obstacles. 🤖🧭**