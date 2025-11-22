# Rover Hardware Deployment

This directory contains all code and configuration that runs on the actual Jetson Orin Nano rover.

## Directory Structure

```
rover/
├── scripts/                    # Python scripts deployed to Jetson
│   ├── jetson_rover_server.py  # Main control server (port 5000)
│   ├── jetson_rover_server_mavlink.py  # MAVLink variant
│   ├── config.py               # GPIO pins, thresholds, speeds
│   ├── ultrasonic_bridge.py    # ROS2 ultrasonic sensor node
│   └── ...                     # Other utility scripts
├── firmware/                   # Microcontroller firmware
│   ├── esp32_ultrasonic/       # ESP32 PlatformIO project
│   └── circuitpython/          # CircuitPython alternative
├── config/                     # Configuration files
│   └── xbox_rover.config.yaml  # Gamepad button mappings
├── systemd/                    # Systemd service files
├── *.md                        # Hardware-specific documentation
└── *.sh                        # Deployment/test scripts
```

## Deployment Target

- **Device:** Jetson Orin Nano
- **IP Address:** 100.91.191.47
- **ROS Domain:** ROS_DOMAIN_ID=42

## Key Components

### Control Server
The `jetson_rover_server.py` provides HTTP REST API on port 5000 for:
- Motor control via Cube Orange (MAVLink)
- GPS telemetry
- ARM/DISARM commands
- Waypoint navigation

### Ultrasonic Sensors
- 6x AJ-SR04M sensors in 360-degree coverage
- ESP32 reads sensors and streams via USB serial
- `ultrasonic_bridge.py` publishes to ROS2 topics

### Configuration
Edit `scripts/config.py` to adjust:
- GPIO pin assignments
- Navigation thresholds
- Movement speeds
- Safety parameters

## Deploying to Jetson

```bash
# Copy scripts to Jetson
scp -r scripts/* jay@100.91.191.47:~/rover/

# Copy firmware (if updating ESP32)
scp -r firmware/esp32_ultrasonic jay@100.91.191.47:~/firmware/
```

## Documentation

- `ESP32_PINOUT_DIAGRAM.md` - Wiring diagram
- `ULTRASONIC_SETUP_GUIDE.md` - Sensor setup
- `SYSTEMD_SERVICES.md` - Auto-start configuration
