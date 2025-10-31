# Mobile RTK Station Design Document
**High-Accuracy GPS Beacon for Rover Following Applications**

**Project Goal**: Create a portable, wearable RTK GPS beacon that enables rovers to follow a person with centimeter-level accuracy  
**Application**: Field work, surveying, inspection, collaborative robotics

---

## 🎯 **SYSTEM OVERVIEW**

### **Core Concept**
A compact, battery-powered RTK GPS receiver that:
1. Receives RTK corrections from your base station
2. Achieves centimeter-level positioning accuracy
3. Transmits real-time position data to rovers
4. Provides "follow me" waypoint generation
5. Maintains safety controls and emergency stop capability

### **Use Cases**
- **Field Surveying**: Rover follows surveyor with precision positioning
- **Construction Monitoring**: Rover tracks inspector around construction sites
- **Agricultural Applications**: Rover follows farmer for crop monitoring
- **Search and Rescue**: Rover maintains specific distance/formation with human operator
- **Research Applications**: Autonomous data collection following research team

---

## 🔧 **HARDWARE ARCHITECTURE**

### **Core Components**

#### **GPS Receiver** ⭐ **PRIMARY COMPONENT**
**Recommendation**: SparkFun GPS-RTK2 ZED-F9P (Rover Mode)
- **Accuracy**: 10mm horizontal, 20mm vertical with RTK corrections
- **Interface**: UART for position data, USB for configuration
- **Power**: 3.3V, ~50mA continuous operation
- **Update Rate**: Up to 20Hz position updates
- **Form Factor**: Compact module suitable for portable integration

#### **Computing Platform**
**Option 1 - Raspberry Pi Zero 2 W** (Recommended for full functionality)
- **Pros**: Full Linux, Python support, WiFi built-in, familiar development
- **Cons**: Higher power consumption (~150mA), larger size
- **Use Case**: Full-featured mobile station with advanced behaviors

**Option 2 - ESP32-S3** (Alternative for ultra-compact design)
- **Pros**: Very low power (~30mA), built-in WiFi, smaller form factor
- **Cons**: Arduino programming, limited processing for complex features
- **Use Case**: Basic beacon functionality with minimal features

#### **Communication Systems**
**Primary**: **WiFi 802.11n** 
- **Range**: 100-200m in open field (compatible with travel router)
- **Protocol**: UDP for real-time position streaming
- **Fallback**: TCP for reliable command/control messages

**Secondary**: **LoRa Module** (Optional for extended range)
- **Range**: 1-5km in open terrain
- **Use Case**: Operations beyond WiFi coverage
- **Module**: RFM95W or similar 915MHz module

#### **Power Management**
**Battery**: **18650 Li-ion Battery Pack** (2-3 cells)
- **Capacity**: 6000-9000mAh total
- **Runtime**: 8-12 hours continuous operation
- **Charging**: USB-C PD for field charging from power bank

**Power Management IC**: MCP73871 or similar
- **Features**: Battery charging, power path management, low battery detection
- **Protection**: Over-discharge, over-current, thermal protection

#### **User Interface**
**Status Indicators**: RGB LED + Buzzer
- **GPS Status**: Green = RTK Fix, Yellow = 3D Fix, Red = No Fix
- **Communication**: Blue = Connected to rover, Flashing = Transmitting
- **Battery**: Slow blink = Good, Fast blink = Low, Solid red = Critical

**Controls**: Single pushbutton
- **Functions**: Power on/off, emergency stop, mode selection
- **Emergency**: Long press for immediate rover stop command

#### **Environmental Protection**
**Enclosure**: IP65-rated weather-resistant housing
- **Material**: ABS or polycarbonate plastic
- **Sealing**: O-ring seals for connectors
- **Mounting**: Belt clip, carabiner attachment, optional chest harness

---

## 📊 **TECHNICAL SPECIFICATIONS**

### **Positioning Performance**
```
Accuracy (with RTK corrections):
  - Horizontal: ±10mm (95% confidence)
  - Vertical: ±20mm (95% confidence)
  
Update Rate: 
  - GPS: 10Hz position updates
  - Transmission: 5Hz to rover (balance accuracy vs bandwidth)
  
Acquisition Time:
  - Cold Start: <30 seconds to 3D fix
  - RTK Fix: 30-120 seconds (depends on base station distance)
```

### **Communication Performance**
```
WiFi Range: 
  - Line of sight: 200m+ 
  - With obstacles: 50-100m
  - Indoor: 20-50m

Data Rate:
  - Position updates: 5Hz (minimal bandwidth)
  - Status messages: 1Hz
  - Emergency commands: Immediate

Protocol:
  - UDP for real-time position (low latency)
  - TCP for control commands (reliable delivery)
  - JSON message format for compatibility
```

### **Power Specifications**
```
Power Consumption:
  - GPS Receiver: ~50mA
  - Pi Zero 2 W: ~150mA (idle), ~300mA (active)
  - WiFi Transmission: +50mA (when transmitting)
  - Total Average: ~250mA continuous

Battery Life:
  - 6000mAh pack: ~24 hours
  - 9000mAh pack: ~36 hours
  - With power saving: Up to 48 hours

Charging:
  - USB-C PD: 2-4 hour full charge
  - Field charging: Compatible with standard power banks
```

### **Physical Specifications**
```
Dimensions (Target):
  - Device: 120mm x 80mm x 30mm
  - Weight: <300g including battery
  - Attachment: Universal belt clip + carabiner

Environmental:
  - Operating temp: -10°C to +50°C  
  - Water resistance: IP65 (rain/splash proof)
  - Drop resistance: 1m drop onto concrete
```

---

## 💻 **SOFTWARE ARCHITECTURE**

### **Mobile Station Software Stack**
```
Mobile RTK Station Application
├── GPS Interface Layer
│   ├── ZED-F9P communication (UART)
│   ├── RTK correction handling
│   ├── Position quality assessment
│   └── Coordinate system management
├── Communication Layer  
│   ├── WiFi network management
│   ├── UDP position streaming
│   ├── TCP command/control
│   └── Connection monitoring
├── Safety & Control Layer
│   ├── Emergency stop functionality
│   ├── Battery monitoring
│   ├── GPS quality validation
│   └── User interface management
└── Application Layer
    ├── Follow-me mode logic
    ├── Waypoint generation
    ├── Formation control
    └── Status reporting
```

### **Rover Integration Software**
```
Rover Following System (Addition to Outdoor Rover)
├── Mobile Station Interface
│   ├── Position data receiver
│   ├── Command message handler
│   ├── Connection monitoring
│   └── Failsafe management
├── Following Behaviors
│   ├── Direct following (maintain distance/bearing)
│   ├── Offset following (parallel path)
│   ├── Formation following (specific patterns)
│   └── Adaptive following (terrain/obstacle aware)
├── Safety Integration
│   ├── Emergency stop response
│   ├── Lost communication handling
│   ├── LIDAR override coordination
│   └── RTK GPS validation
└── Mission Planner Integration
    ├── Real-time following status
    ├── Dynamic waypoint display
    ├── Safety system monitoring
    └── Manual override capability
```

---

## 🔄 **COMMUNICATION PROTOCOL**

### **Position Update Messages** (UDP, 5Hz)
```json
{
  "message_type": "position_update",
  "timestamp": 1694712345123,
  "station_id": "mobile_rtk_001",
  "gps_data": {
    "latitude": 40.123456789,
    "longitude": -105.987654321,
    "altitude": 1234.567,
    "accuracy_horizontal": 0.015,
    "accuracy_vertical": 0.025,
    "fix_type": "RTK_FIXED",
    "satellites": 18,
    "hdop": 0.8
  },
  "motion_data": {
    "speed": 1.25,
    "heading": 45.2,
    "acceleration": 0.15
  },
  "battery_level": 85,
  "signal_strength": -45
}
```

### **Control Messages** (TCP, as needed)
```json
{
  "message_type": "follow_command",
  "timestamp": 1694712345123,
  "station_id": "mobile_rtk_001",
  "command": {
    "action": "start_following",
    "follow_distance": 3.0,
    "follow_bearing": 180,
    "formation_type": "direct_follow",
    "safety_distance": 2.0
  }
}

{
  "message_type": "emergency_stop",
  "timestamp": 1694712345123,
  "station_id": "mobile_rtk_001",
  "emergency_type": "user_initiated",
  "stop_all_motion": true
}
```

### **Status Messages** (TCP, 1Hz)
```json
{
  "message_type": "station_status",
  "timestamp": 1694712345123,
  "station_id": "mobile_rtk_001",
  "system_status": {
    "gps_health": "RTK_FIXED",
    "battery_percentage": 85,
    "wifi_signal": -45,
    "uptime": 14523,
    "temperature": 25.3
  },
  "connected_rovers": ["outdoor_rover_001"],
  "active_mode": "follow_me"
}
```

---

## 🚀 **IMPLEMENTATION PHASES**

### **Phase 1: Basic GPS Beacon** (2-3 weeks)
**Goal**: Prove core GPS positioning and communication

**Hardware**:
- Pi Zero 2 W + ZED-F9P on breadboard
- Basic power supply (USB power bank)
- Simple LED status indicators

**Software**:
- GPS data acquisition and RTK processing
- Basic WiFi communication with rover
- Simple position streaming protocol
- Battery monitoring

**Success Criteria**:
- [ ] Achieves RTK fix from base station
- [ ] Transmits position data to rover at 5Hz
- [ ] Rover can receive and process position updates
- [ ] 4+ hour operation on battery power

### **Phase 2: Following Behaviors** (2-3 weeks)
**Goal**: Implement basic rover following functionality

**Rover Software Additions**:
- Mobile station position receiver
- Basic following algorithm (maintain distance)
- Safety integration with LIDAR system
- Mission Planner display integration

**Mobile Station Enhancements**:
- User control interface (start/stop following)
- Emergency stop functionality
- Improved status reporting
- Connection monitoring and recovery

**Success Criteria**:
- [ ] Rover follows mobile station maintaining 3m distance
- [ ] Emergency stop works reliably
- [ ] Following works with LIDAR obstacle avoidance
- [ ] Mission Planner shows real-time following status

### **Phase 3: Advanced Features** (3-4 weeks)
**Goal**: Production-ready mobile station with advanced behaviors

**Hardware Integration**:
- Custom PCB design (optional)
- Proper enclosure with IP65 rating
- Optimized battery management
- Enhanced user interface

**Software Features**:
- Multiple following modes (direct, offset, formation)
- Adaptive following with terrain awareness
- Advanced safety features
- Configuration management
- Data logging and analysis

**Success Criteria**:
- [ ] Weather-resistant operation
- [ ] 12+ hour battery life
- [ ] Multiple following patterns working
- [ ] Production-ready reliability

### **Phase 4: Extended Capabilities** (Future)
**Goal**: Advanced features and multi-rover coordination

**Potential Features**:
- LoRa communication for extended range
- Multiple mobile stations coordination  
- Smartphone app integration
- Voice control capabilities
- Advanced formation patterns
- Autonomous charging integration

---

## 🛠️ **DEVELOPMENT TOOLS & RESOURCES**

### **Hardware Development**
**Prototyping**:
- Breadboard and jumper wires for initial testing
- Perfboard for semi-permanent connections
- 3D printer for custom enclosure prototypes
- Basic electronics tools (multimeter, oscilloscope)

**PCB Design** (Optional Phase 3):
- KiCad for open-source PCB design
- JLCPCB or similar for low-cost PCB fabrication
- Basic SMD assembly tools if using surface-mount components

### **Software Development**
**Development Environment**:
- Pi Zero 2 W with SSH access for remote development
- VS Code with remote SSH for code editing
- Git repository for version control
- Python virtual environment for dependency management

**Testing Tools**:
- Mission Planner for rover integration testing
- GPS testing software for accuracy validation
- Network monitoring tools for communication testing
- Battery monitoring for power optimization

### **Integration Testing**
**Test Setup**:
- Your existing RTK base station for corrections
- Outdoor rover with LIDAR for integration testing
- Travel router for network infrastructure
- Open field testing area for validation

---

## 📋 **COMPONENT SHOPPING LIST**

### **Phase 1 Prototype** (Est. $150-200)
```
Core Components:
- Raspberry Pi Zero 2 W: $15
- SparkFun GPS-RTK2 ZED-F9P: $100
- MicroSD card (32GB): $10
- USB power bank (10000mAh): $20
- Breadboard and jumpers: $15
- LEDs and resistors: $5
- Basic enclosure/project box: $10

Optional:
- Small OLED display: $15
- Buzzer for audio feedback: $5
- Push buttons: $5
```

### **Phase 2 Development** (Additional $100-150)
```
Enhancements:
- Custom cable assemblies: $20
- Improved enclosure materials: $30
- Better battery pack (18650 + holder): $40
- Power management circuit components: $25
- Environmental sealing materials: $15
- Mounting hardware (clips, straps): $20
```

### **Phase 3 Production** (Additional $200-300)
```
Production Components:
- Custom PCB fabrication: $50-100
- Professional enclosure (weatherproof): $75
- High-quality battery management system: $50
- Professional cable assemblies: $40
- IP65 connectors and seals: $30
- Final mounting and carrying solutions: $50
```

---

## 🎯 **EXPECTED PERFORMANCE**

### **Following Accuracy**
- **Position Accuracy**: ±2cm with RTK corrections
- **Following Distance Accuracy**: ±5cm at 3m separation
- **Response Time**: <200ms from position change to rover command
- **Update Rate**: 5Hz position updates, smooth following motion

### **Operational Characteristics**
- **Battery Life**: 12+ hours continuous operation
- **Range**: 200m+ in open terrain with WiFi
- **Weather Resistance**: Rain and splash proof operation
- **User Friendliness**: Single button operation, clear status indicators

### **Safety Features**
- **Emergency Stop**: <100ms response time to stop command
- **Lost Communication**: Automatic rover stop after 2 seconds
- **Low Battery**: 30-minute warning before automatic shutdown
- **GPS Quality**: Automatic following disable if accuracy degrades

---

## 🚨 **RISK ANALYSIS & MITIGATION**

### **Technical Risks**
**Risk**: RTK corrections unreliable at distance from base station  
**Mitigation**: Implement graceful degradation to standard GPS, range testing

**Risk**: WiFi communication drops in field conditions  
**Mitigation**: Robust reconnection logic, LoRa backup option, failsafe behaviors

**Risk**: Battery life insufficient for work day  
**Mitigation**: Power optimization, larger battery pack, USB-C field charging

### **Safety Risks**
**Risk**: Rover continues following with degraded GPS accuracy  
**Mitigation**: Continuous accuracy monitoring, automatic stop on degradation

**Risk**: Emergency stop fails or delayed  
**Mitigation**: Multiple communication paths, hardware watchdog, fail-safe design

**Risk**: Rover lost communication results in unpredictable behavior  
**Mitigation**: Clear communication timeouts, default safe behaviors

### **Operational Risks**
**Risk**: Device damaged by weather or rough handling  
**Mitigation**: Robust enclosure design, drop testing, weather sealing

**Risk**: Complex setup reduces field usability  
**Mitigation**: Simple user interface, automatic configuration, clear documentation

---

**🎯 MOBILE RTK STATION DESIGN STATUS: COMPREHENSIVE CONCEPT READY FOR IMPLEMENTATION**  
**Phase 1 Prototype: 2-3 weeks | Est. Cost: $150-200 | High Success Probability**

---

*Design Document Created: September 13, 2025*  
*Ready for: Phase 1 prototype development*  
*Integration Target: Outdoor Rover Platform with existing RTK base station*