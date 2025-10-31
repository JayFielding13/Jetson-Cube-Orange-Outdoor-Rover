# Simple Rover Project

## Overview
Autonomous obstacle-avoidance rover using Raspberry Pi, Arduino motor controller, and LoRa wireless cutoff switch. Features bendy ruler navigation algorithm for smooth, fluid obstacle avoidance.

### Current Capabilities
- ✅ 4-sensor ultrasonic navigation (2 front, 2 side)
- ✅ Bendy ruler algorithm for smooth steering
- ✅ Sensor fusion for center obstacle detection
- ✅ Corner trap detection and escape
- ✅ Side wall avoidance and gentle steering
- ✅ LoRa wireless STOP/AUTONOMOUS control
- ✅ Auto-start on boot via systemd

---

## Hardware

### Main Components
- **Raspberry Pi 4** - Main controller
- **Arduino Nano** - Motor controller (USB serial)
- **Cytron MDDS30** - Dual motor driver (Mixed R/C mode)
- **Heltec LoRa WiFi 32 V3 (2×)** - Wireless control (transmitter + receiver)
- **HC-SR04 Ultrasonic Sensors (4×)** - Obstacle and wall detection

### Sensor Configuration
```
        [LF] 30°   30° [RF]     LF = Left Front (angled outward)
              \   /             RF = Right Front (angled outward)
               \ /              LS = Left Side (perpendicular)
                🤖              RS = Right Side (perpendicular)
         [LS] ← → [RS]
```

---

## Documentation

### Quick Start
📄 **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** - Essential info, parameters, troubleshooting

### Hardware Setup
📄 **[WIRING_REFERENCE.md](WIRING_REFERENCE.md)** - Complete wiring diagrams and GPIO pins
📄 **[SIDE_SENSOR_WIRING.md](SIDE_SENSOR_WIRING.md)** - Side sensor installation guide

### Software & Configuration
📄 **[SETUP_AND_TESTING.md](SETUP_AND_TESTING.md)** - Initial setup and testing procedures
📄 **[NETWORK_CONFIG_SUMMARY.md](NETWORK_CONFIG_SUMMARY.md)** - Network and SSH setup
📄 **[TRAVEL_ROUTER_SETUP.md](TRAVEL_ROUTER_SETUP.md)** - GL.iNet router configuration

### Recent Updates
📄 **[UPDATE_LOG.md](UPDATE_LOG.md)** - Detailed changelog and system improvements (Oct 2025)

---

## Quick Reference

### System Control
```bash
# Start/Stop/Restart
sudo systemctl start simple-rover.service
sudo systemctl stop simple-rover.service
sudo systemctl restart simple-rover.service

# View logs
journalctl -u simple-rover.service -f
```

### Testing
```bash
# Test all 4 sensors
cd "/home/jay/Simple Rover"
python3 test_all_sensors.py

# Manual run (for debugging)
python3 rover_controller_with_lora.py
```

### Navigation Parameters
| Parameter | Value | Description |
|-----------|-------|-------------|
| CRITICAL_THRESHOLD | 10 cm | Emergency stop |
| OBSTACLE_THRESHOLD | 50 cm | Start steering away |
| SENSOR_FUSION_DIFF | 8 cm | Center obstacle detection |
| SIDE_WARNING_THRESHOLD | 10 cm | Side wall steering |

---

## Operation

### LoRa Control
1. **Power on rover** - Starts in STOP mode (motors disabled)
2. **Press transmitter button** - Switches to AUTONOMOUS mode
3. **Press button again** - Returns to STOP mode
4. **Both OLEDs display current mode** (STOP or AUTONOMOUS)

### Navigation Behavior
- **Open space**: Full speed forward
- **Approaching obstacle (20-50cm)**: Slow down and steer away
- **Side wall detected (<10cm)**: Gentle steering correction
- **Very close (<10cm)**: Emergency stop
- **Corner trap (both sensors <20cm)**: Rotate to find exit

---

## Troubleshooting

### Rover Won't Move
1. Check transmitter battery
2. Verify both Heltec OLEDs show "AUTONOMOUS"
3. Check service: `systemctl status simple-rover.service`
4. **Try different USB port for Heltec** (common issue!)

### Rover Gets Stuck Near Walls
1. Verify thresholds: CRITICAL=10, FUSION_DIFF=8
2. Run sensor test: `python3 test_all_sensors.py`
3. Check side sensor mounting (perpendicular to body)

### LoRa Not Working
1. Check USB ports: `ls -la /dev/ttyUSB*`
2. Identify devices: `udevadm info /dev/ttyUSB0 | grep ID_MODEL`
   - Arduino = CH340 chip
   - Heltec = CP2102 chip
3. **Move Heltec to different USB port if needed**

---

## File Structure

### Code (on Raspberry Pi)
```
/home/jay/Simple Rover/
├── rover_controller_with_lora.py  # Main controller
└── test_all_sensors.py            # Sensor verification
```

### Documentation (this folder)
```
/home/jay/Desktop/Mini Rover Development/Simple Rover/
├── README.md                    # This file
├── QUICK_REFERENCE.md           # Quick lookup guide
├── UPDATE_LOG.md                # Detailed changelog
├── WIRING_REFERENCE.md          # Hardware connections
├── SIDE_SENSOR_WIRING.md        # Side sensor guide
├── SETUP_AND_TESTING.md         # Initial setup
├── NETWORK_CONFIG_SUMMARY.md    # Network info
└── TRAVEL_ROUTER_SETUP.md       # Router config
```

### Arduino Code
```
/home/jay/Desktop/Mini Rover Development/Arduino Code/
├── Heltec Code/
│   ├── LoRa_KillSwitch_Transmitter_Channel/  # Transmitter
│   └── LoRa_KillSwitch_Receiver_Channel/     # Receiver
└── Simple Rover/
    └── Mixed_RC_mode_motor_controller/        # Motor control
```

---

## Recent Updates (October 2025)

### Added Side Sensors
- 2× additional HC-SR04 sensors for wall detection
- Gentle steering away from walls within 10cm
- No hard stops - smooth navigation adjustments

### Fixed Navigation Issues
- Reduced CRITICAL_THRESHOLD from 15cm → 10cm
- Tightened SENSOR_FUSION_DIFF from 15cm → 8cm
- Resolved "stuck at walls" behavior
- Improved angled approach handling (70-80° to wall)

### USB Port Reliability
- Identified Heltec USB data transmission issues
- Solution: Use reliable USB ports for consistent operation

---

## Credits & Resources

### Navigation Algorithm
- **Bendy Ruler**: Preemptive smooth obstacle avoidance
- **Sensor Fusion**: Center obstacle detection
- **Corner Escape**: Rotation-based trap exit strategy

### Hardware
- Raspberry Pi 4 Model B
- Arduino Nano (CH340 USB)
- Heltec LoRa WiFi 32 V3
- Cytron MDDS30 Motor Driver
- HC-SR04 Ultrasonic Sensors

---

## Contact & Support

For issues or questions:
1. Check [QUICK_REFERENCE.md](QUICK_REFERENCE.md) for common solutions
2. Review [UPDATE_LOG.md](UPDATE_LOG.md) for recent changes
3. Consult [WIRING_REFERENCE.md](WIRING_REFERENCE.md) for hardware questions

**Last Updated**: October 16, 2025
