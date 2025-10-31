# Rover Project - Platform-Based Organization
**Multi-Platform Rover Development Framework**

**Last Updated**: September 13, 2025  
**Organization Method**: Platform-based with specific use cases and development directions

---

## 🏗️ **PLATFORM ARCHITECTURE OVERVIEW**

### **Platform Categories**
1. **🌍 Outdoor Rover** - Cube Orange+ based autonomous navigation platform
2. **🏠 Indoor Rover** - Raspberry Pi + Arduino Gatekeeper architecture  
3. **📡 RTK GPS Base Station** - Stationary precision positioning infrastructure
4. **📱 Mobile RTK Station** - Portable "follow me" functionality platform
5. **💻 Base Station Setup & Data** - Configuration, networking, and operational data

---

## 🌍 **OUTDOOR ROVER PLATFORM**

### **Platform Purpose**
High-precision autonomous navigation for outdoor environments with obstacle avoidance, GPS waypoint navigation, and advanced sensor fusion.

### **Hardware Architecture**
```
OUTDOOR ROVER (Tank Track Platform)
├── Flight Controller: Cube Orange+ (ArduRover 4.6.2)
├── Navigation Computer: NavigationPi (192.168.8.70)
├── Sensors:
│   ├── RPLidar A1 (360° obstacle detection, 8m range)
│   ├── RTK GPS ZED-F9P (cm-level positioning)
│   ├── Logitech C920 (vision-based object tracking)
│   └── BerryGPS-IMU V4 (backup navigation)
├── Actuators: MDDS30 motor controller + tank tracks
└── Control: Xbox 360 controller + Mission Planner
```

### **Current Capabilities**
- ✅ **360° LIDAR Obstacle Avoidance** - 5Hz real-time collision prevention
- ✅ **RTK GPS Navigation** - Sub-decimeter positioning accuracy  
- ✅ **Vision-Based Object Tracking** - Tennis ball detection and following
- ✅ **Mission Planner Integration** - Waypoint navigation and monitoring
- 🔄 **Multi-System Integration** - Combining LIDAR + GPS + Vision

### **Development Status**
- **LIDAR System**: Production ready with auto-start service
- **RTK GPS**: Hardware working, Auto mode EKF calibration needed
- **Vision Tracking**: Functional, needs integration with obstacle avoidance
- **Integration**: Architecture design needed for multi-system coordination

---

## 🏠 **INDOOR ROVER PLATFORM**

### **Platform Purpose**  
Agile indoor navigation with ultrasonic sensors, modular Python architecture, and real-time telemetry for controlled environment testing.

### **Hardware Architecture**
```
INDOOR ROVER (Tank Track Platform)
├── Control Computer: Raspberry Pi (Navigation + Companion Pi setup)
├── Motor Controller: Arduino Nano ATmega328 (Gatekeeper firmware)
├── Sensors:
│   ├── Dual HC-SR04 Ultrasonics (left/right obstacle detection)
│   ├── BerryGPS-IMU V4 (heading and navigation)
│   └── Optional: Camera for indoor object tracking
├── Actuators: Motor controller + tank tracks
└── Interface: Direct Python control + web visualization
```

### **Current Capabilities**
- ✅ **Progressive Obstacle Avoidance** - 80cm→15cm adaptive navigation
- ✅ **Real-time Telemetry** - UDP streaming to companion Pi
- ✅ **Web Visualization** - Live sensor data at http://192.168.254.70:8090
- ✅ **Modular Python Architecture** - Sensor/actuator/navigation modules
- ✅ **High Success Rate** - 90% navigation success in testing

### **Development Status**
- **Core Navigation**: Production ready (run_enhanced_exploration.py)
- **Arduino Gatekeeper**: Stable JSON communication protocol
- **Python Architecture**: Complete modular design deployed
- **Visualization**: Real-time web interface operational

---

## 📡 **RTK GPS BASE STATION PROJECT**

### **Platform Purpose**
Stationary RTK GPS base station providing centimeter-level positioning corrections to all rover platforms and mobile units.

### **Hardware Architecture**
```
RTK BASE STATION
├── GPS Receiver: SparkFun GPS-RTK2 ZED-F9P
├── Host Computer: Laptop or dedicated Pi
├── Communication: 
│   ├── USB connection for configuration
│   ├── WiFi/Ethernet for correction distribution
│   └── Radio link capability (future expansion)
├── Power: External power supply for continuous operation
└── Mounting: Survey-grade tripod or permanent installation
```

### **Current Capabilities**
- ✅ **Survey-In Mode** - Automatic position determination (60s, 2m accuracy)
- ✅ **RTCM3 Corrections** - Real-time correction data generation
- ✅ **Mission Planner Integration** - Visual status monitoring
- ✅ **Multi-Rover Support** - Corrections available to all platforms

### **Development Status**
- **Hardware Setup**: Complete and tested
- **Software Configuration**: Mission Planner RTK base station mode working
- **Correction Distribution**: RTCM3 messages flowing to rovers
- **Documentation**: Setup procedures need documentation

### **Future Enhancements**
- [ ] CAD models for permanent mounting solutions
- [ ] Radio link for extended range operations  
- [ ] Automated startup and monitoring scripts
- [ ] Multi-base station network capability

---

## 📱 **MOBILE RTK STATION PROJECT**

### **Platform Purpose**
Portable RTK GPS unit for "follow me" functionality, person tracking, and mobile precision positioning applications.

### **Hardware Architecture** (Planned)
```
MOBILE RTK STATION
├── GPS Receiver: SparkFun GPS-RTK2 ZED-F9P (rover mode)
├── Host Computer: Raspberry Pi Zero/4 (compact form factor)
├── Communication:
│   ├── WiFi for rover coordination
│   ├── Bluetooth for device pairing
│   └── Radio link to base station
├── Power: Battery pack for portable operation
├── Sensors: IMU for orientation tracking
└── Interface: Mobile app or simple display
```

### **Planned Capabilities**
- [ ] **Person Following** - Attach to person for rover to follow
- [ ] **Mobile Waypoint** - Dynamic waypoint generation for rovers
- [ ] **Precision Tracking** - cm-level position of mobile targets
- [ ] **Multi-Device Coordination** - Multiple mobile stations and rovers

### **Development Ideas**
- Wearable form factor for hands-free operation
- Integration with smartphone apps for control
- Mesh networking between mobile stations
- Advanced tracking patterns (orbit, maintain distance, etc.)
- Emergency stop and safety features

---

## 💻 **BASE STATION SETUP & DATA**

### **Platform Purpose**
Central configuration, networking, and operational data repository for all rover platforms and support systems.

### **Configuration Categories**

#### **Network Infrastructure**
- Travel router setup and configuration
- Internet bridge configuration (laptop WiFi → ethernet)
- Pi network access and routing tables
- SSH key management and access control
- IP address assignments and DHCP configuration

#### **Platform Configuration**
- Mission Planner parameter files for each rover
- Arduino firmware versions and configurations
- Python environment setup and dependencies
- Service configurations (systemd auto-start)
- Calibration data and procedures

#### **Operational Data**
- Test session logs and results
- Performance metrics and benchmarks
- Troubleshooting guides and solutions
- Hardware inventory and specifications
- Software version tracking and update logs

### **Current Documentation Needs**
- [ ] **SSH Setup Guide** - Key generation and deployment for each rover
- [ ] **Internet Bridge Configuration** - Step-by-step laptop→Pi setup
- [ ] **Mission Planner Parameters** - Complete parameter sets for each platform
- [ ] **Service Management** - systemd service creation and monitoring
- [ ] **Hardware Inventory** - Components, versions, and configurations

---

## 📁 **RECOMMENDED DIRECTORY STRUCTURE**

### **New Platform-Based Organization**
```
/Mini Rover Development/
├── 01-Outdoor-Rover-Platform/
│   ├── Hardware/
│   │   ├── Cube-Orange-Config/
│   │   ├── LIDAR-Setup/
│   │   ├── RTK-GPS-Integration/
│   │   └── Camera-Systems/
│   ├── Software/
│   │   ├── LIDAR-Bridge/
│   │   ├── Vision-Tracking/
│   │   ├── GPS-Navigation/
│   │   └── Integration-Scripts/
│   ├── Testing/
│   │   ├── Field-Tests/
│   │   ├── Integration-Tests/
│   │   └── Performance-Data/
│   └── Documentation/
│       ├── Setup-Guides/
│       ├── Troubleshooting/
│       └── Development-Log/
│
├── 02-Indoor-Rover-Platform/
│   ├── Hardware/
│   │   ├── Arduino-Gatekeeper/
│   │   ├── Pi-Configuration/
│   │   └── Sensor-Setup/
│   ├── Software/
│   │   ├── Navigation-System/
│   │   ├── Visualization/
│   │   ├── Modular-Architecture/
│   │   └── Test-Scripts/
│   ├── Testing/
│   │   ├── Indoor-Navigation/
│   │   └── Performance-Metrics/
│   └── Documentation/
│
├── 03-RTK-Base-Station/
│   ├── Hardware/
│   │   ├── GPS-Configuration/
│   │   ├── Mounting-Solutions/
│   │   └── Power-Systems/
│   ├── Software/
│   │   ├── Mission-Planner-Setup/
│   │   ├── Correction-Distribution/
│   │   └── Monitoring-Scripts/
│   ├── CAD-Models/           # Future CAD files
│   ├── Installation-Data/    # Site survey and setup records
│   └── Documentation/
│
├── 04-Mobile-RTK-Station/
│   ├── Hardware/             # Future hardware designs
│   ├── Software/             # Follow-me applications
│   ├── Mobile-Apps/          # Smartphone integration
│   ├── Prototypes/           # Development iterations
│   └── Documentation/
│
├── 05-Base-Station-Setup/
│   ├── Network-Configuration/
│   │   ├── SSH-Keys/
│   │   ├── Router-Configs/
│   │   ├── Internet-Bridge/
│   │   └── IP-Management/
│   ├── Platform-Configurations/
│   │   ├── Mission-Planner-Params/
│   │   ├── Arduino-Firmware/
│   │   ├── Pi-Environments/
│   │   └── Service-Configs/
│   ├── Operational-Data/
│   │   ├── Test-Logs/
│   │   ├── Performance-Data/
│   │   ├── Hardware-Inventory/
│   │   └── Software-Versions/
│   └── Documentation/
│       ├── Setup-Procedures/
│       ├── Troubleshooting/
│       └── Configuration-Guides/
│
└── Archive/                  # Deprecated/historical files
```

---

## 🎯 **PLATFORM DEVELOPMENT PRIORITIES**

### **Immediate Focus (Next 2 Weeks)**
1. **Outdoor Rover**: Resolve LIDAR ground detection + RTK Auto mode issues
2. **Indoor Rover**: Extended testing and stability validation
3. **Base Station Setup**: Document SSH and internet bridge configurations

### **Medium Term (Next Month)**  
1. **Outdoor Rover**: Multi-system integration (LIDAR + GPS + Vision)
2. **Mobile RTK Station**: Initial prototype and concept development
3. **RTK Base Station**: CAD models and installation documentation

### **Long Term (Next Quarter)**
1. **Platform Integration**: Coordinate outdoor and indoor rover capabilities
2. **Mobile Applications**: Follow-me functionality implementation
3. **Base Station Network**: Multi-base station coordination

---

## 🤝 **COLLABORATION BY PLATFORM**

### **Platform Ownership Strategy**
- **Outdoor Rover**: Jay leading advanced sensor integration
- **Indoor Rover**: Ready for Jarrett to extend and enhance
- **RTK Base Station**: Shared setup and documentation effort
- **Mobile RTK Station**: Collaborative design and prototyping
- **Base Station Setup**: Shared knowledge base and documentation

### **Cross-Platform Benefits**
- Software modules can be shared between rover platforms
- RTK base station serves both rover types
- Configuration templates reduce setup time
- Testing procedures can be standardized across platforms

---

**🎯 PLATFORM ORGANIZATION STATUS: READY FOR IMPLEMENTATION**  
**5 Distinct Platforms | Clear Development Directions | Collaborative Framework**

---

*Platform Organization Created: September 13, 2025*  
*For: Multi-platform rover development coordination*  
*Next Step: Create platform-specific documentation templates*