# Dual Pi Visualization System Setup Guide

## 🎯 System Overview

**Dual Pi Architecture for Enhanced Performance:**
- **Navigation Pi (192.168.254.65)**: Focused purely on navigation decisions and motor control
- **Companion Pi (192.168.254.70)**: Handles sensor visualization and streams to development PC  
- **Development PC**: Receives and displays real-time sensor visualization

## 🔧 Hardware Requirements

### Both Raspberry Pis
- Raspberry Pi 4 (recommended) or Pi 3B+
- SD card with Raspberry Pi OS
- Network connectivity (WiFi or Ethernet)

### Arduino System  
- Arduino Nano with dual ultrasonic sensor gatekeeper code
- 2x HC-SR04 sensors positioned at 30° angles
- USB connection to both Pis

### Development PC
- Python 3.7+ with tkinter support
- Network connection to Pi network

## 📋 Installation Steps

### 1. Upload Arduino Code
Flash the dual sensor gatekeeper code:
```bash
# Upload: Arduino Code/Arduino Gatekeeper Double Ultrasonic/rover_arduino_gatekeeper_dual_sensor/rover_arduino_gatekeeper_dual_sensor.ino
```

### 2. Setup Navigation Pi (192.168.254.65)
```bash
# Install dependencies
sudo apt update
sudo apt install python3-serial

# Run navigation controller
cd "/path/to/Pi Code/Navigation V7 - Dual Pi/"
python3 navigation_pi_dual_sensor_controller.py
```

### 3. Setup Companion Pi (192.168.254.70)
```bash
# Install dependencies
sudo apt update  
sudo apt install python3-matplotlib python3-pil python3-serial

# Run visualization system
cd "/path/to/Pi Code/Navigation V7 - Dual Pi/"
python3 companion_pi_sensor_visualizer.py
```

### 4. Setup Development PC
```bash
# Install dependencies
pip3 install pillow

# Run visualization receiver
cd "/path/to/Dashboard/Dashboard V3.5 - Dual Pi Management/"
python3 dev_pc_visualization_receiver.py
```

## 🚀 Usage Instructions

### Starting the System

1. **Power up Arduino and connect to both Pis** via USB hubs or separate Arduino units
2. **Start Navigation Pi** - Run navigation controller first
3. **Start Companion Pi** - Run visualization system  
4. **Connect Development PC** - Start receiver and click "Connect to Companion Pi"

### What You'll See

**Navigation Pi Console:**
```
🤖 Navigation Pi - Dual Sensor Autonomous Controller
🔍 Using dual 30° ultrasonic sensors with directional avoidance
📊 STATUS UPDATE - 14:32:15
   Mode: EXPLORE
   Sensors: L=45.2cm R=32.1cm  
   Emergency: NO
   Runtime: 127.3s | Commands: 1273 | Obstacles: 8
```

**Companion Pi Visualization:**
- Real-time radar display with 30° sensor cones
- Obstacle positions plotted in real-time
- Color-coded sensor status and emergency alerts

**Development PC Display:**
- Live radar visualization streamed from Companion Pi
- Sensor telemetry data with statistics
- Connection status and frame rate monitoring

## 🔧 Configuration Options

### Network Settings
Edit IP addresses in scripts if needed:
- Navigation Pi: Modify `ARDUINO_PORT` in navigation script
- Companion Pi: Modify `DEV_PC_IP` in visualization script
- Development PC: Modify `COMPANION_PI_IP` in receiver script

### Sensor Parameters
Adjust in Arduino code or Python scripts:
- `EMERGENCY_DISTANCE`: Obstacle detection threshold (default: 10cm)
- `SENSOR_ANGLE_DEG`: Sensor cone angle (default: 30°)
- `UPDATE_RATE`: Visualization frame rate (default: 10Hz)

### Navigation Behavior
Modify in navigation script:
- `NORMAL_SPEED`: Forward movement speed (default: 120)
- `TURN_SPEED`: Turning maneuver speed (default: 80)
- `BACKUP_DURATION`: Time to back up from obstacles (default: 1.5s)

## 🎮 Testing Scenarios

### Directional Obstacle Tests
1. **Left Obstacle**: Place object 30° to left, 8cm away
   - **Expected**: Navigation Pi turns right, Companion Pi shows left sensor detection
   
2. **Right Obstacle**: Place object 30° to right, 8cm away
   - **Expected**: Navigation Pi turns left, Companion Pi shows right sensor detection
   
3. **Center Obstacle**: Place object directly ahead, 8cm away
   - **Expected**: Navigation Pi backs up then turns, both sensors detect

4. **Narrow Passage**: Create narrow corridor
   - **Expected**: System detects both sides, initiates backup maneuver

## 🔍 Monitoring and Debugging

### Navigation Pi Monitoring
- Watch console for mode changes and obstacle detection
- Monitor command send rate and navigation decisions
- Check sensor data freshness and validity

### Companion Pi Monitoring  
- Verify Arduino connection and data parsing
- Check visualization generation and network streaming
- Monitor client connections from development PC

### Development PC Monitoring
- Verify network connection to Companion Pi
- Check frame rate and data reception statistics
- Monitor sensor telemetry accuracy

## 📊 Performance Benefits

**Navigation Pi (No Visualization Overhead):**
- ✅ Dedicated CPU for navigation decisions
- ✅ Faster response times to obstacles  
- ✅ More reliable motor control
- ✅ Better real-time performance

**Companion Pi (Specialized Visualization):**
- ✅ Dedicated visualization processing
- ✅ Real-time radar display generation
- ✅ Network streaming to development PC
- ✅ No impact on navigation performance

**Development PC (Remote Monitoring):**
- ✅ Real-time visualization without Pi overhead
- ✅ Enhanced debugging capabilities
- ✅ Professional development environment
- ✅ Data logging and analysis tools

## 🛠️ Troubleshooting

### Common Issues

**Arduino Not Detected:**
- Check USB connections on both Pis
- Verify `/dev/ttyUSB0` device exists: `ls -la /dev/tty*`
- Ensure Arduino is flashed with dual sensor code

**Network Connection Issues:**
- Verify Pi IP addresses: `hostname -I`
- Check firewall settings: `sudo ufw status`
- Test network connectivity: `ping 192.168.254.70`

**Visualization Not Updating:**
- Check Python dependencies: `pip3 list | grep matplotlib`
- Verify sensor data reception in console logs
- Monitor network traffic: `netstat -an | grep 5555`

**Navigation Erratic:**
- Check sensor wiring and 30° mounting angles
- Verify emergency distance threshold settings
- Monitor sensor data validity in telemetry

## 🔄 Next Steps

1. **Test the complete system** with various obstacle scenarios
2. **Fine-tune navigation parameters** based on rover performance
3. **Add data logging** for analysis and improvement
4. **Implement GPS waypoint integration** for advanced navigation
5. **Add machine learning** for obstacle classification

---

**System Status**: ✅ Complete dual Pi implementation ready for testing  
**Performance**: 🚀 Optimized for real-time navigation with professional visualization  
**Scalability**: 📈 Foundation for advanced autonomous rover features