# Mobile RTK Station - Development Workflow
**Optimized Development Process Using Existing Professional Hardware**

**Development Platform**: Pi 4B + SparkFun GPS-RTK-SMA + 7" Touchscreen  
**Integration Target**: Outdoor Rover Platform + existing RTK base station  
**Timeline**: 1-2 weeks to working prototype

---

## 🎯 **DEVELOPMENT STRATEGY**

### **Advantages of Your Hardware Setup**
**Development Speed Benefits**:
- **Pi 4B Performance** - Fast compilation, testing, and debugging
- **Professional GPS** - Reliable RTK performance, no hardware debugging needed
- **Quality Antenna** - Multi-band L1/L2/L5 for faster convergence and better accuracy
- **Large Screen** - Excellent for development interface and real-time debugging

**Production Readiness**:
- **Field-Grade Components** - Hardware proven for outdoor professional use
- **Integrated Display** - No separate smartphone/laptop needed for operation
- **Professional Appearance** - Looks like commercial surveying equipment
- **Expansion Capability** - Pi 4B can handle advanced features and multi-rover coordination

---

## 📅 **OPTIMIZED DEVELOPMENT TIMELINE**

### **Day 1-2: Hardware Integration & Basic Testing**
**Morning: Hardware Assembly**
- [ ] **GPS Connection** - Wire GPS-RTK-SMA to Pi 4B UART (30 minutes)
- [ ] **Touchscreen Setup** - Connect 7" display and test touch response (30 minutes)
- [ ] **Status Components** - Add emergency button and status LEDs (1 hour)
- [ ] **Power System** - Test with 20,000mAh power bank (30 minutes)

**Afternoon: Software Foundation**  
- [ ] **GPS Library** - Install and test pyserial, pynmea2, pyubx2 (1 hour)
- [ ] **UART Testing** - Verify GPS communication and NMEA message reception (1 hour)
- [ ] **RTK Connection** - Configure for corrections from your base station (2 hours)
- [ ] **Basic GUI** - Create simple status display with GPS info (2 hours)

**Evening: RTK Validation**
- [ ] **RTK Fix Testing** - Achieve RTK fix and measure accuracy (1 hour)
- [ ] **Base Station Integration** - Verify correction data flow (30 minutes)
- [ ] **Performance Baseline** - Document accuracy and update rates (30 minutes)

### **Day 3-4: Communication & Basic Following**
**Morning: Rover Communication**
- [ ] **UDP Implementation** - Create position broadcast to outdoor rover (2 hours)
- [ ] **Rover Integration** - Modify outdoor rover to receive mobile position (3 hours)
- [ ] **Communication Testing** - Verify reliable data flow and range (1 hour)

**Afternoon: Basic Following Algorithm**  
- [ ] **Distance Calculation** - Implement distance/bearing math between positions (1 hour)
- [ ] **Following Logic** - Basic "maintain 3m distance" algorithm (2 hours)
- [ ] **Safety Integration** - Emergency stop and lost communication handling (2 hours)
- [ ] **LIDAR Coordination** - Ensure following works with obstacle avoidance (1 hour)

**Evening: Initial Field Testing**
- [ ] **Static Testing** - Test accuracy with mobile station stationary (30 minutes)
- [ ] **Walking Test** - Simple following behavior while walking (1 hour)
- [ ] **Safety Testing** - Emergency stop and communication loss scenarios (30 minutes)

### **Day 5-7: GUI Enhancement & Advanced Features**
**Days 5-6: Professional GUI Development**
- [ ] **Main Interface** - Clean status screen with GPS/battery/network info (4 hours)
- [ ] **Control Panel** - Start/stop following, distance adjustment, emergency stop (3 hours)
- [ ] **Diagnostics Screen** - GPS status, satellite view, accuracy graphs (4 hours)
- [ ] **Settings Interface** - Configuration for rover IP, follow patterns, safety margins (3 hours)
- [ ] **Data Logging** - Record position data, performance metrics, issues (2 hours)

**Day 7: Integration & Testing**
- [ ] **Extended Field Testing** - Multi-hour operation with various scenarios (4 hours)
- [ ] **Battery Optimization** - Power management and life testing (2 hours)
- [ ] **Performance Tuning** - Optimize update rates, accuracy, responsiveness (2 hours)

### **Week 2: Advanced Features & Production Polish**
**Optional enhancements based on Week 1 results**

---

## 💻 **DEVELOPMENT ENVIRONMENT SETUP**

### **Pi 4B Software Configuration**
```bash
# System Setup
sudo apt update && sudo apt upgrade -y
sudo apt install python3-pip python3-venv python3-dev
sudo apt install git vim screen

# Enable UART for GPS
sudo raspi-config
# Interface Options → Serial Port → No (login) → Yes (hardware) → Reboot

# GPS Communication Libraries
pip3 install pyserial pynmea2 pyubx2

# GUI Development  
sudo apt install python3-tkinter python3-matplotlib python3-numpy
pip3 install pillow

# Network Communication
pip3 install asyncio websockets

# Development Tools
sudo apt install minicom htop iotop
pip3 install ipython jupyter
```

### **Development Directory Structure**
```bash
mkdir -p ~/mobile_rtk_station
cd ~/mobile_rtk_station

# Create project structure
mkdir -p {src,config,logs,tests,docs,gui}
mkdir -p src/{gps,communication,safety,utils}
mkdir -p gui/{screens,widgets,assets}

# Initialize git repository
git init
```

### **Project File Organization**
```
~/mobile_rtk_station/
├── main.py                    # Main application entry point
├── config/
│   ├── settings.json          # Configuration parameters
│   ├── rover_endpoints.json   # Rover IP addresses and ports  
│   └── gps_config.json        # GPS-specific settings
├── src/
│   ├── gps/
│   │   ├── gps_handler.py     # ZED-F9P interface
│   │   ├── rtk_manager.py     # RTK correction handling
│   │   └── position_filter.py # Position smoothing/filtering
│   ├── communication/
│   │   ├── rover_comm.py      # UDP/TCP rover communication
│   │   ├── base_station.py    # RTK correction reception
│   │   └── network_monitor.py # Connection monitoring
│   ├── safety/
│   │   ├── emergency_stop.py  # Emergency stop system
│   │   ├── health_monitor.py  # System health monitoring
│   │   └── failsafe.py        # Failure mode handling
│   └── utils/
│       ├── math_utils.py      # Position calculation utilities
│       ├── logging_utils.py   # Logging and data collection
│       └── gpio_utils.py      # LED and button handling
├── gui/
│   ├── main_window.py         # Main GUI application
│   ├── screens/
│   │   ├── status_screen.py   # Main status display
│   │   ├── gps_screen.py      # GPS diagnostics
│   │   ├── rover_screen.py    # Rover control and monitoring
│   │   └── settings_screen.py # Configuration interface
│   ├── widgets/
│   │   ├── status_bar.py      # Status bar widget
│   │   ├── gps_display.py     # GPS information widget
│   │   └── control_buttons.py # Control button widgets
│   └── assets/
│       ├── icons/             # GUI icons and images
│       └── styles.css         # GUI styling (if using Qt)
├── tests/
│   ├── test_gps.py           # GPS functionality tests
│   ├── test_communication.py # Communication tests
│   └── test_integration.py   # Integration tests
├── logs/                     # Runtime logs and data
└── docs/                     # Documentation
```

---

## 🔧 **DEVELOPMENT WORKFLOW**

### **Daily Development Cycle**
```bash
# Morning: Start development session
cd ~/mobile_rtk_station
source venv/bin/activate
git pull  # If working with collaborators

# Development session
python3 main.py  # Run current version
# Make changes, test, debug

# Evening: End session  
git add .
git commit -m "Description of changes"
git push  # If using remote repository

# Document progress in development log
```

### **Testing Workflow**
**Unit Testing** (Indoor, no GPS required):
```bash
# Test individual components
python3 -m pytest tests/test_communication.py
python3 -m pytest tests/test_gps.py
python3 tests/test_gui.py  # Test GUI components
```

**Integration Testing** (Outdoor, with GPS):
```bash
# Test GPS functionality
python3 src/gps/gps_handler.py  # Direct GPS test
python3 tests/test_rtk_connection.py  # RTK base station test

# Test rover communication  
python3 tests/test_rover_comm.py  # Network communication test
```

**Field Testing** (Full system):
```bash
# Run full system in field conditions
python3 main.py --field-test-mode
# Monitor logs in real-time
tail -f logs/mobile_rtk.log
```

### **Debugging Tools & Techniques**

#### **GPS Debugging**
```bash
# Direct UART monitoring
sudo minicom -D /dev/serial0 -b 38400
# Should see continuous NMEA messages

# GPS message analysis
python3 -c "
import serial
port = serial.Serial('/dev/serial0', 38400)
while True:
    line = port.readline().decode('ascii', errors='ignore')
    if 'GGA' in line:  # Position data
        print(f'Position: {line.strip()}')
"
```

#### **Network Debugging**
```bash
# Test UDP communication
# Terminal 1 (receiver - on rover)
nc -u -l 12345

# Terminal 2 (sender - on mobile station)  
echo "test position data" | nc -u [rover_ip] 12345

# Monitor network traffic
sudo tcpdump -i wlan0 port 12345
```

#### **Performance Monitoring**
```bash
# System resource monitoring
htop  # CPU and memory usage
iotop  # Disk I/O monitoring  
iwconfig  # WiFi signal strength

# GPS performance monitoring
python3 -c "
import time
# Log position updates per second
# Log RTK fix quality over time  
# Monitor communication latency
"
```

---

## 🎨 **GUI DEVELOPMENT APPROACH**

### **Touchscreen-Optimized Design Principles**
**Large Touch Targets**:
- Minimum button size: 60x60 pixels
- Comfortable spacing between controls  
- High contrast colors for outdoor visibility
- Clear visual feedback on button press

**Information Hierarchy**:
- Critical info (GPS status, battery) always visible
- Secondary info (detailed diagnostics) on separate screens
- Emergency controls prominently placed and easily accessible

### **Screen Layout Strategy**
```python
# Main Status Screen Layout
┌─────────────────────────────────────────────────────────┐
│ GPS: RTK FIXED │ Battery: 85% │ Signal: ████▓ │ 14:23 │
├─────────────────────────────────────────────────────────┤
│                                                         │
│         ROVER FOLLOWING STATUS                          │
│                                                         │
│    Distance: 3.2m    Bearing: 180°    Speed: 1.1m/s   │
│                                                         │
│  ┌─────────┐  ┌─────────┐  ┌─────────────────────────┐ │
│  │  START  │  │  STOP   │  │    EMERGENCY STOP       │ │
│  │FOLLOWING│  │FOLLOWING│  │        (RED)            │ │
│  └─────────┘  └─────────┘  └─────────────────────────┘ │
│                                                         │
├─────────────────────────────────────────────────────────┤
│ Lat: 40.123456  Lon: -105.987654  Alt: 1234.5m        │
│ [GPS] [Rover] [Settings] [Help]                        │
└─────────────────────────────────────────────────────────┘
```

### **GUI Framework Selection**
**Recommended: Tkinter** (Built into Python)
- ✅ **Advantages**: No additional dependencies, good touch support, fast development
- ✅ **Pi Integration**: Excellent performance on Pi 4B, optimized for Raspberry Pi OS
- ✅ **Customization**: Easy to create custom widgets and layouts

**Alternative: PyQt5** (If more advanced features needed)
- ✅ **Advantages**: Professional appearance, advanced widgets, better styling
- ❌ **Disadvantages**: Larger memory footprint, more complex deployment

### **GUI Development Workflow**
```python
# Start with simple tkinter prototype
import tkinter as tk
from tkinter import ttk

class MobileRTKGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Mobile RTK Station")  
        self.root.geometry("800x480")  # 7" screen resolution
        self.root.configure(bg='black')  # Better for outdoor use
        
        self.create_status_screen()
        
    def create_status_screen(self):
        # Large, touch-friendly interface
        pass
    
    def run(self):
        self.root.mainloop()

# Iterative development - add features gradually
if __name__ == "__main__":
    app = MobileRTKGUI()
    app.run()
```

---

## 🔄 **INTEGRATION TESTING STRATEGY**

### **Phase 1: Component Integration**
**GPS Integration Testing**:
```python
# Test GPS communication and RTK fix
def test_gps_integration():
    gps = GPSHandler('/dev/serial0')
    
    # Test basic communication
    assert gps.read_position() is not None
    
    # Test RTK correction reception
    assert gps.configure_rtk_base('192.168.8.100')
    
    # Wait for RTK fix (may take 1-5 minutes)  
    timeout = 300  # 5 minutes
    start_time = time.time()
    while time.time() - start_time < timeout:
        if gps.get_rtk_status() == 'RTK_FIXED':
            break
        time.sleep(1)
    
    assert gps.get_rtk_status() in ['RTK_FIXED', 'RTK_FLOAT']
    print(f"GPS accuracy: {gps.get_accuracy()}m")
```

**Communication Testing**:
```python
# Test rover communication
def test_rover_communication():
    comm = RoverComm('192.168.8.70', 12345)  # Outdoor rover IP
    
    position_data = {
        'lat': 40.123456,
        'lon': -105.987654,
        'accuracy': 0.015
    }
    
    # Test position updates
    success = comm.send_position_update(position_data)
    assert success == True
    
    # Test emergency stop
    success = comm.send_emergency_stop()
    assert success == True
```

### **Phase 2: System Integration**
**Follow Behavior Testing**:
```python
# Test complete follow system
def test_follow_behavior():
    # Initialize mobile station at known position
    mobile_pos = get_current_position()
    
    # Command rover to start following
    start_following(distance=3.0, bearing=180)
    
    # Move mobile station 5 meters north
    target_pos = offset_position(mobile_pos, bearing=0, distance=5)
    
    # Wait for rover to adjust position
    time.sleep(10)
    
    # Verify rover maintained correct distance and bearing
    rover_pos = get_rover_position()
    actual_distance = calculate_distance(target_pos, rover_pos)
    assert 2.5 <= actual_distance <= 3.5  # Within 50cm tolerance
```

### **Phase 3: Field Validation**
**Real-World Testing Scenarios**:
1. **Walking Follow Test** - Person walks normal pace, rover maintains distance
2. **Obstacle Navigation** - Following behavior with LIDAR obstacle avoidance
3. **Communication Range** - Maximum reliable operating distance  
4. **Battery Endurance** - Continuous operation duration measurement
5. **Weather Resistance** - Operation in various weather conditions

---

## 📊 **PERFORMANCE MONITORING**

### **Key Metrics to Track**
```python
# Performance monitoring during development
class PerformanceMonitor:
    def __init__(self):
        self.metrics = {
            'gps_update_rate': [],
            'rtk_fix_time': [],
            'position_accuracy': [],
            'communication_latency': [],
            'rover_following_error': [],
            'battery_consumption': [],
            'system_uptime': 0
        }
    
    def log_metrics(self):
        # Collect and log performance data
        pass
    
    def generate_report(self):
        # Generate performance report
        pass
```

### **Success Criteria Validation**
```
Target Performance:
✅ RTK fix time: <2 minutes in open sky
✅ Position accuracy: <2cm horizontal  
✅ Update rate: 5Hz sustained
✅ Following accuracy: ±10cm at 3m distance
✅ Emergency stop: <200ms response
✅ Battery life: >12 hours continuous
✅ Communication range: >100m line of sight
```

---

**🎯 DEVELOPMENT WORKFLOW STATUS: OPTIMIZED FOR RAPID PROTOTYPING**  
**Professional Hardware + Structured Workflow = Fast Development**  
**Timeline: 1-2 weeks to working prototype with professional-grade performance**

---

*Development Workflow Created: September 13, 2025*  
*Optimized for: Pi 4B + Professional GPS hardware + Touchscreen interface*  
*Integration: Outdoor rover platform + existing RTK base station*