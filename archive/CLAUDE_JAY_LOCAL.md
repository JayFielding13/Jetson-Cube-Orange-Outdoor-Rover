# Jay Robot Project - Jay's Development Environment

## Overview
**IMPORTANT FOR CLAUDE**: This is Jay's personal development environment for the Jay Robot Project. This project is a **collaboration between Jay and Jarrett**.

- **This file**: Jay's local configuration and workflow (NOT in repository)
- **Jarrett's setup**: Jarrett maintains his own local CLAUDE.md file with his Pi configuration
- **Shared project**: Both developers work on the same repository but with different local setups

## Jay's Raspberry Pi Fleet
### Navigation Pi (Jay's Primary Pi)
- **Hostname**: Navigation Pi
- **Ethernet IP**: 192.168.254.65
- **WiFi IP**: 192.168.254.65
- **Username**: jay
- **Password**: jay
- **Owner**: Jay
- **Python Version**: Python 3.11.2
- **Status**: ✅ SSH configured, passwordless access working

### Companion Pi (Jay's Secondary Pi)
- **Hostname**: Companion Pi
- **Ethernet IP**: 192.168.254.70
- **WiFi IP**: 192.168.254.70
- **Username**: jay
- **Password**: jay
- **Owner**: Jay
- **Python Version**: Python 3.11.2
- **Status**: ✅ SSH configured, passwordless access working

## Jay's SSH Configuration  
### SSH Shortcuts (configured and working):
```bash
# Jay's Navigation Pi
ssh jay-nav-pi    # Navigation Pi connection (192.168.254.65)

# Jay's Companion Pi
ssh jay-comp-pi   # Companion Pi connection (192.168.254.70)

# Direct connections (backup)
ssh jay@192.168.254.65  # Navigation Pi direct
ssh jay@192.168.254.70  # Companion Pi direct
```

**SSH Key Status**: ✅ SSH keys generated and deployed to both Pis
**SSH Config File**: `/home/jay/.ssh/config` configured with aliases

## Jay's Local Development Tools
### Arduino Configuration (Jay's Setup)
- **Board**: Arduino Nano ATmega328 (New Bootloader)
- **Port**: [To be determined when Arduino is connected]
- **Upload Method**: PlatformIO CLI
- **Status**: ✅ PlatformIO Core 6.1.18 installed

### Quick PlatformIO Commands
```bash
# Navigate to project
cd "/home/jay/Desktop/Mini Rover Development/Rover"

# Build project
platformio run

# Upload code to Arduino Nano (when connected)
platformio run --target upload

# Monitor serial output (adjust port as needed)
platformio device monitor -p /dev/ttyUSB0 -b 115200

# List connected devices
platformio device list
```

### Dashboard Quick Launch (Jay's Setup)
```bash
# Navigate to dashboard
cd "/home/jay/Desktop/Mini Rover Development/Rover/Rover-Dashboard"

# Run dashboard
python3 main_enhanced_v3.py

# Dependencies status: ✅ All installed
```

## Python Dependencies Status
### Local Machine (Development PC)
- ✅ **paramiko**: 2.9.3 (SSH connections)
- ✅ **scp**: 0.15.0 (secure file copy)
- ✅ **pyserial**: 3.5 (Arduino communication)
- ✅ **matplotlib**: 3.10.5 (data visualization)
- ✅ **numpy**: 2.2.6 (numerical computing)
- ✅ **rplidar-roboticia**: 0.9.5 (LIDAR interface)
- ✅ **platformio**: 6.1.18 (Arduino development)

### Navigation Pi (192.168.254.65)
- ✅ **python3-matplotlib**: System package (visualization)
- ✅ **python3-numpy**: System package (numerical computing)
- ✅ **python3-paramiko**: System package (SSH client)
- ✅ **python3-serial**: System package (Arduino communication)
- ✅ **scp**: 0.15.0 (user package - secure file copy)
- ✅ **rplidar-roboticia**: 0.9.5 (user package - LIDAR interface)

### Companion Pi (192.168.254.70)
- ✅ **python3-matplotlib**: System package (visualization)
- ✅ **python3-numpy**: System package (numerical computing)
- ✅ **python3-paramiko**: System package (SSH client)
- ✅ **python3-serial**: System package (Arduino communication)
- ✅ **scp**: 0.15.0 (user package - secure file copy)
- ✅ **rplidar-roboticia**: 0.9.5 (user package - LIDAR interface)

## Jay's Development Notes
- **Local Machine**: Linux (Ubuntu-based distribution)
- **Arduino Setup**: CH340 USB-Serial chip, requires "nanoatmega328new" board setting
- **SSH Access**: ✅ Passwordless SSH keys configured for both Pis
- **Working Branch**: Jay-Development
- **Collaboration**: Jarrett maintains RoboPi2/RoboPi3 fleet with his own local CLAUDE.md configuration
- **Project Location**: `/home/jay/Desktop/Mini Rover Development/Rover`

## Hardware Configuration
### Microcontroller
- **Board**: Arduino Nano ATmega328 (New Bootloader)
- **USB-Serial**: CH340 chip
- **Programming**: PlatformIO with Arduino framework

### Sensors  
- **RPLIDAR A1M8**: 360° LIDAR with 12m range, 8000 samples/second
- **HC-SR04**: Ultrasonic distance sensors for close-range navigation
- **RC Receiver**: Flysky FS-iA10B for manual control

### Motor Control
- **Motor Controller**: DRV-1011 Dual H-Bridge
- **Power Management**: DROK Buck Converter (12V to 5V)

## Control Modes
The rover operates in three distinct modes based on RC Channel 9:

1. **FAILSAFE** (CH9 < -500): Emergency stop, no motor movement allowed
2. **MANUAL** (CH9 -500 to +500): Full RC control, safety overrides disabled  
3. **AUTONOMOUS** (CH9 > +500): Pi control with full safety systems active

## Quick Start Commands
### Test SSH Connections
```bash
# Test both Pi connections
ssh jay-nav-pi "echo 'Navigation Pi connected'"
ssh jay-comp-pi "echo 'Companion Pi connected'"
```

### Test Python Environment
```bash
# Test local Python packages
python3 -c "import matplotlib, numpy, paramiko, serial, scp, rplidar; print('✅ Local: All packages available')"

# Test Navigation Pi packages
ssh jay-nav-pi "python3 -c 'import matplotlib, numpy, paramiko, serial, scp, rplidar; print(\"✅ Navigation Pi: All packages available\")'"

# Test Companion Pi packages  
ssh jay-comp-pi "python3 -c 'import matplotlib, numpy, paramiko, serial, scp, rplidar; print(\"✅ Companion Pi: All packages available\")'"
```

### Arduino Development
```bash
# Build and test Arduino code
cd "/home/jay/Desktop/Mini Rover Development/Rover"
platformio run

# When Arduino is connected, upload with:
# platformio run --target upload
```

### Dashboard Launch
```bash
# Start rover management dashboard
cd "/home/jay/Desktop/Mini Rover Development/Rover/Rover-Dashboard"
python3 main_enhanced_v3.py
```

## Project Structure
```
/home/jay/Desktop/Mini Rover Development/Rover/
├── README.md                    # Main project documentation (updated with Jay's Pi details)
├── CLAUDE.md                    # Shared project documentation 
├── platformio.ini              # Arduino build configuration
├── src/main.cpp                # Arduino gatekeeper code
├── autonomous_ultrasonic_navigator.py   # Pi autonomous navigation
├── rplidar_realtime.py         # Real-time LIDAR processing
├── CAD/                        # SolidWorks models
└── Rover-Dashboard/            # SSH dashboard for Pi management
    ├── main_enhanced_v3.py     # Dashboard application
    ├── pi_settings.json        # Saved IP configuration
    ├── requirements.txt        # Python dependencies
    └── README.md               # Dashboard documentation
```

## Network Configuration
- **WiFi Network**: House of Googlers
- **Development Network**: 192.168.254.x subnet
- **SSH Ports**: Standard port 22 for both Pis
- **Connection Method**: Passwordless SSH with RSA keys

## Jarrett's Setup (For Reference)
Jarrett maintains his own Pi fleet and local configuration:
- **RoboPi3** (Primary): 192.168.1.123/124 
- **RoboPi2** (Secondary): 192.168.1.221/132
- **Local CLAUDE.md**: Separate file with his Pi configuration and SSH shortcuts

## Development Workflow
1. **Code Development**: Use local machine for Python script development
2. **Arduino Programming**: PlatformIO CLI for microcontroller development
3. **Pi Deployment**: SSH to deploy and test code on Navigation/Companion Pis
4. **Testing**: Use dashboard for integrated system testing
5. **Collaboration**: Coordinate with Jarrett via shared repository

## Status Summary
✅ **All Jarrett's assigned tasks completed:**
1. ✅ Pi configuration details filled in README.md
2. ✅ SSH aliases configured and working
3. ✅ Passwordless SSH set up for both Pis
4. ✅ PlatformIO Core installed (v6.1.18)
5. ✅ Python dependencies installed on all systems
6. ✅ SSH connections tested and verified
7. ✅ Local CLAUDE.md file created (this file)

**Ready for rover development and testing!**