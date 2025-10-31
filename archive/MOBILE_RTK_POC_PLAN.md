# Mobile RTK Station - Proof of Concept Implementation Plan
**Phase 1: Basic GPS Beacon Development**

**Timeline**: 2-3 weeks  
**Budget**: $150-200  
**Goal**: Demonstrate centimeter-accurate position tracking and rover following

---

## 🎯 **PROOF OF CONCEPT OBJECTIVES**

### **Primary Success Criteria**
1. **RTK GPS Integration** - Achieve RTK fix using your existing base station
2. **Position Communication** - Stream accurate position data to outdoor rover
3. **Basic Following** - Rover maintains 3-meter following distance
4. **Safety Integration** - Emergency stop and lost communication handling
5. **Field Operation** - 4+ hours portable operation on battery power

### **Demonstration Scenarios**
1. **Static Positioning Test** - Verify cm-level accuracy compared to surveyed point
2. **Walking Follow Test** - Person walks, rover follows maintaining distance
3. **Emergency Stop Test** - Immediate rover stop on button press
4. **Communication Range Test** - Determine reliable operating range
5. **Battery Life Test** - Continuous operation duration measurement

---

## 🛒 **COMPONENT ACQUISITION PLAN**

### **Week 1: Order Components**
**Immediate Orders** (2-3 day shipping):
```
Essential Components:
☐ Raspberry Pi Zero 2 W + microSD (32GB) - $25 total
☐ USB power bank (10,000mAh with USB-A output) - $20
☐ Breadboard (half-size) + jumper wire kit - $15
☐ Basic project box/enclosure (IP54 minimum) - $15

Status Indicators:
☐ RGB LED (common cathode) + resistors (220Ω) - $5
☐ Small buzzer (5V compatible) - $3
☐ Push button (momentary) - $2

Connectivity:
☐ USB-A to micro-USB cables - $5
☐ Breadboard power supply (3.3V/5V) - $8
```

**GPS Module** (Longer shipping, order immediately):
```
☐ SparkFun GPS-RTK2 ZED-F9P Board - $100
  - Alternative: Order from SparkFun directly
  - Include: GPS antenna (survey grade if possible)
  - Shipping: 5-7 days typical
```

**Total Phase 1 Cost: ~$198**

### **Week 2: Additional Supplies** (If needed)
```
Development Tools:
☐ Digital multimeter (if not available) - $20
☐ Micro-USB to USB-A adapters - $5
☐ Extra microSD cards for backup - $10
☐ Basic soldering supplies (if needed) - $15
```

---

## 📅 **DEVELOPMENT SCHEDULE**

### **Week 1: Hardware Assembly & Basic GPS**
**Day 1-2: Initial Setup**
- [ ] Receive and inventory components
- [ ] Set up Pi Zero 2 W with latest Raspberry Pi OS
- [ ] Install Python development environment
- [ ] Test basic WiFi connectivity and SSH access

**Day 3-4: GPS Integration**
- [ ] Wire ZED-F9P to Pi Zero (UART connection)
- [ ] Install GPS libraries and test basic positioning
- [ ] Configure for RTK corrections from your base station
- [ ] Verify RTK fix achievement and accuracy

**Day 5-7: Basic Communication**
- [ ] Implement UDP position broadcast
- [ ] Create simple rover position receiver
- [ ] Test communication range and reliability
- [ ] Add basic status LED indicators

### **Week 2: Following Behavior & Integration**
**Day 8-10: Rover Integration**
- [ ] Modify outdoor rover to receive mobile position data
- [ ] Implement basic following algorithm (maintain distance)
- [ ] Test LIDAR integration (obstacle avoidance during following)
- [ ] Add Mission Planner display of mobile station position

**Day 11-12: Safety Systems**
- [ ] Implement emergency stop functionality
- [ ] Add lost communication detection and rover stop
- [ ] Test battery monitoring and low battery warnings
- [ ] Validate safety response times

**Day 13-14: Field Testing**
- [ ] Conduct outdoor following tests
- [ ] Measure accuracy and performance
- [ ] Test edge cases and failure modes
- [ ] Document results and improvements needed

### **Week 3: Refinement & Documentation**
**Day 15-17: Improvements**
- [ ] Address issues found in field testing
- [ ] Optimize battery life and performance
- [ ] Improve user interface and status reporting
- [ ] Add configuration options and settings

**Day 18-21: Documentation & Demo**
- [ ] Create operation manual and setup guide
- [ ] Document lessons learned and next phase planning
- [ ] Prepare demonstration for proof of concept
- [ ] Plan Phase 2 enhancements

---

## 🔧 **HARDWARE ASSEMBLY GUIDE**

### **Power System**
```
Pi Zero 2 W Power:
- Connect USB power bank to Pi micro-USB power
- Consider: USB-A splitter for simultaneous Pi + GPS power
- Monitor: Add voltage divider for battery monitoring

Status LED Wiring:
GPIO 18 (Pin 12) → Red LED → 220Ω resistor → GND
GPIO 19 (Pin 35) → Green LED → 220Ω resistor → GND  
GPIO 20 (Pin 38) → Blue LED → 220Ω resistor → GND

Control Button:
GPIO 21 (Pin 40) → Push button → GND (with internal pullup)
```

### **GPS Module Connection**
```
ZED-F9P to Pi Zero 2 W:
VCC → 3.3V (Pin 1)
GND → GND (Pin 6) 
TX → GPIO 15 (Pin 10) - UART0 RX
RX → GPIO 14 (Pin 8) - UART0 TX

Note: Enable UART in raspi-config, disable serial console
```

### **Physical Assembly**
```
Breadboard Layout:
- Pi Zero mounted on breadboard or beside
- ZED-F9P on breadboard with header pins
- LEDs and button on breadboard
- Power bank connected via cable
- Everything fits in project box with access holes
```

---

## 💻 **SOFTWARE DEVELOPMENT PLAN**

### **Core Python Modules**
```
mobile_rtk_station/
├── main.py                 # Main application loop
├── gps_handler.py          # ZED-F9P interface and RTK processing
├── communication.py        # WiFi/UDP position broadcasting
├── safety_system.py        # Emergency stop and monitoring
├── status_display.py       # LED indicators and user feedback
├── config.py              # Configuration and settings
└── utils/
    ├── position_math.py    # GPS coordinate utilities
    └── network_tools.py    # Network connectivity helpers
```

### **Key Software Components**

#### **GPS Handler** (`gps_handler.py`)
```python
import serial
import pynmea2
from pyubx2 import UBXReader

class GPSHandler:
    def __init__(self, port='/dev/serial0', baudrate=38400):
        self.serial_port = serial.Serial(port, baudrate)
        self.current_position = None
        self.rtk_status = "NO_FIX"
        
    def read_position(self):
        # Parse NMEA messages from ZED-F9P
        # Extract position, accuracy, RTK status
        # Return standardized position data
        
    def get_rtk_status(self):
        # Return current RTK fix status
        # Types: NO_FIX, 3D_FIX, DGPS, RTK_FLOAT, RTK_FIXED
        
    def configure_rtk(self, base_station_ip):
        # Configure for RTCM corrections from base station
```

#### **Communication** (`communication.py`)
```python
import socket
import json
import threading

class MobileStationComm:
    def __init__(self, rover_ip, position_port=12345):
        self.rover_ip = rover_ip
        self.position_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.emergency_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
    def broadcast_position(self, position_data):
        # Send UDP position updates at 5Hz
        message = {
            "timestamp": time.time(),
            "position": position_data,
            "status": self.get_system_status()
        }
        self.position_socket.sendto(json.dumps(message).encode(), 
                                   (self.rover_ip, self.position_port))
    
    def send_emergency_stop(self):
        # Send immediate TCP emergency stop command
        
    def monitor_connection(self):
        # Monitor rover connection status
        # Handle reconnection and failover
```

#### **Safety System** (`safety_system.py`)
```python
class SafetySystem:
    def __init__(self, comm_handler):
        self.comm = comm_handler
        self.emergency_active = False
        self.battery_level = 100
        
    def check_gps_quality(self, gps_data):
        # Validate GPS accuracy is sufficient for safe following
        # Return True/False for following enable
        
    def monitor_battery(self):
        # Check battery level and warn on low power
        
    def handle_emergency_button(self):
        # Process emergency button press
        # Send immediate stop to rover
        # Set emergency state
        
    def validate_safe_operation(self):
        # Overall safety check
        # GPS quality, battery, communication, etc.
```

---

## 🧪 **TESTING PROTOCOLS**

### **Phase 1: Component Testing**
```
GPS Module Testing:
☐ Basic position acquisition (outdoor, clear sky)
☐ RTK correction reception from base station
☐ RTK fix achievement time measurement
☐ Position accuracy validation (compare to known point)

Communication Testing:
☐ UDP packet transmission to rover
☐ Reliable delivery over WiFi
☐ Range testing in open field
☐ Interference and reliability testing

Hardware Testing:
☐ Battery life measurement
☐ Power consumption optimization
☐ LED status indicator functionality  
☐ Button response testing
```

### **Phase 2: Integration Testing**
```
Rover Following Testing:
☐ Basic following behavior (stationary mobile station)
☐ Moving follow test (walking pace)
☐ Distance accuracy (maintain 3m separation)
☐ Integration with LIDAR obstacle avoidance

Safety System Testing:
☐ Emergency stop response time
☐ Lost communication rover behavior
☐ GPS accuracy degradation handling
☐ Low battery warning and shutdown

Performance Testing:
☐ Continuous operation duration
☐ Position update rate and latency
☐ System reliability over extended operation
☐ Environmental condition tolerance
```

### **Phase 3: Field Validation**
```
Real-World Scenarios:
☐ Survey following simulation (walking transects)
☐ Complex terrain navigation
☐ Multiple stop/start cycles
☐ Different weather conditions

Edge Case Testing:
☐ GPS multipath environments
☐ WiFi interference conditions
☐ Battery depletion scenarios
☐ Hardware failure simulation
```

---

## 📊 **SUCCESS METRICS**

### **Performance Targets**
```
Positioning:
- RTK fix achievement: <2 minutes in open sky
- Position accuracy: <5cm horizontal, <10cm vertical
- Update rate: 5Hz sustained position updates

Following Behavior:
- Distance accuracy: ±10cm at 3m separation
- Response time: <500ms from position change to rover movement
- Following stability: No oscillation or hunting behavior

Battery & Reliability:
- Continuous operation: 4+ hours minimum
- Communication reliability: >95% packet success rate
- Safety response: <200ms emergency stop response
```

### **Qualitative Assessment**
```
Usability:
- Simple one-button operation
- Clear status indication (LED colors intuitive)
- Comfortable wearing/carrying for extended periods

Robustness:
- Reliable operation in outdoor conditions
- Graceful handling of edge cases and failures
- Consistent performance across different environments

Integration:
- Works seamlessly with existing rover systems
- Compatible with current RTK base station
- Mission Planner integration functional
```

---

## 🔄 **ITERATIVE DEVELOPMENT APPROACH**

### **Version 0.1: Basic GPS Beacon**
- [ ] GPS position acquisition working
- [ ] UDP broadcast to rover
- [ ] Basic LED status indicators

### **Version 0.2: Communication Protocol**
- [ ] Structured JSON message format
- [ ] Rover receives and processes position data
- [ ] Basic error handling and reconnection

### **Version 0.3: Following Behavior**
- [ ] Rover maintains distance to mobile station
- [ ] Basic safety features (emergency stop)
- [ ] Status monitoring and reporting

### **Version 0.4: Safety Integration**
- [ ] Lost communication handling
- [ ] GPS quality monitoring
- [ ] LIDAR integration compatibility

### **Version 0.5: Field Ready**
- [ ] Battery optimization
- [ ] Environmental protection
- [ ] User interface refinements
- [ ] Documentation and operation guide

---

## 🚀 **IMMEDIATE NEXT STEPS**

### **This Week**
1. **Order Components** - Place orders for all Phase 1 hardware
2. **Prepare Development Environment** - Set up Pi Zero with development tools
3. **Research Integration** - Study your outdoor rover's current software architecture
4. **Plan Testing Area** - Identify suitable outdoor area for RTK and following tests

### **Week 1 Goals**
1. **Basic GPS Working** - ZED-F9P providing position data
2. **RTK Base Connection** - Receiving corrections from your existing base station
3. **Communication Test** - UDP messages reaching your outdoor rover
4. **Hardware Assembly** - All components wired and functioning

### **Decision Points**
1. **GPS Module Choice** - Confirm ZED-F9P or consider alternatives based on availability
2. **Power Strategy** - USB power bank vs custom battery pack for Phase 1
3. **Enclosure Approach** - Simple project box vs custom 3D printed enclosure
4. **Communication Protocol** - UDP vs TCP vs mixed approach based on testing

---

**🎯 PROOF OF CONCEPT STATUS: DETAILED IMPLEMENTATION PLAN READY**  
**Estimated Timeline: 2-3 weeks | Budget: $150-200 | High Success Probability**  
**Key Advantage: Leverages existing RTK base station and outdoor rover platform**

---

*Implementation Plan Created: September 13, 2025*  
*Ready for: Immediate component ordering and development start*  
*Integration Target: Outdoor Rover Platform + existing RTK base station infrastructure*