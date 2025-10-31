# Mobile RTK Beacon Station

## Overview
Raspberry Pi-based mobile RTK beacon providing centimeter-accurate positioning for rover operations. Receives RTK corrections wirelessly and serves as QGroundControl interface.

## Hardware Configuration
- **Platform**: Raspberry Pi 4B
- **GPS Receiver**: u-blox F9P connected via USB (/dev/ttyACM0)
- **SiK Radio**: Connected via USB (/dev/ttyUSB0) for wireless RTK corrections
- **Network**: Ethernet connection (192.168.8.131)

## System Architecture
```
[Base Station] → [SiK Radio] → [Pi Mobile Beacon] → [QGroundControl] → [Rover]
```

## Key Files

### Core RTK Components

#### `sik_rtk_receiver.py`
Receives RTK corrections via SiK radio network:
- Listens on SiK radio for RTCM3 data from base station
- Forwards corrections to GPS receiver for RTK fix
- Network ID: 25, Node ID: 131

#### `rtk_quality_test.py`
Comprehensive RTK performance monitoring:
- GPS fix quality analysis (Fix=2 target for RTK)
- Satellite count and signal strength (HDOP monitoring)
- Position accuracy validation
- Real-time status reporting

#### `mavlink_bridge.py`
TCP bridge for QGroundControl integration:
- Bridges SiK radio (/dev/ttyUSB0) to TCP port 5760
- Enables QGroundControl connection for Follow Me operations
- Handles MAVLink protocol translation

### Configuration Files

#### `hardware_config_new.json`
Hardware configuration for new u-blox GPS:
```json
{
  "gps_device": "/dev/ttyACM0",
  "gps_baud": 115200,
  "sik_device": "/dev/ttyUSB0",
  "sik_baud": 57600
}
```

#### `rtk_client.conf`
RTKLib client configuration:
- Rover mode settings for RTK positioning
- Input stream configuration for corrections
- Position output format settings

### GPS Testing Tools

#### `demonstrate_rtk.py`
RTK demonstration and validation:
- Shows RTK fix acquisition process
- Displays positioning accuracy improvements
- Real-time coordinate and quality monitoring

#### `gps_test_usb.py`
Hardware validation for new GPS board:
- USB GPS device detection and configuration
- NMEA sentence parsing and validation
- Satellite signal analysis

## SiK Radio Configuration
```
Network ID: 25
Node ID: 131 (Mobile Beacon)
Air Data Rate: 128 kbps
Serial Speed: 57600 baud
Transmit Power: 20 dBm
Destination Node: 26 (Base Station)
```

## QGroundControl Integration

### MAVLink Bridge Setup
```bash
python3 mavlink_bridge.py
```
- Connects QGroundControl to TCP port 5760
- Enables Follow Me mode operations
- Provides real-time telemetry and control

### Connection Steps
1. Start MAVLink bridge on Pi
2. Connect QGroundControl to 192.168.8.131:5760
3. Verify GPS and position data
4. Enable Follow Me mode for rover operations

## Usage

### Start RTK Reception
```bash
python3 sik_rtk_receiver.py
```

### Monitor RTK Quality
```bash
python3 rtk_quality_test.py
```

### Start QGroundControl Bridge
```bash
python3 mavlink_bridge.py
```

### GPS Hardware Test
```bash
python3 gps_test_usb.py
```

## Performance Metrics
- **RTK Fix Quality**: Fix=2 (RTK Fixed)
- **Satellite Count**: 12+ satellites typical
- **HDOP**: <1.0 for optimal accuracy
- **Position Accuracy**: Centimeter-level with RTK
- **Update Rate**: 10 Hz GPS, 1 Hz RTK corrections

## Status Indicators
- **GPS Fix Types**: 0=No Fix, 1=GPS, 2=RTK Fixed, 3=RTK Float
- **HDOP Values**: <1.0=Excellent, 1-2=Good, >2=Poor
- **Satellite Count**: 8+=Good, 12+=Excellent
- **RTK Age**: <10s for fresh corrections

## Integration Notes
- Compatible with laptop RTK base station
- Supports QGroundControl Follow Me operations
- Ready for rover integration via MAVLink
- Designed for outdoor autonomous operations
- Dual battery system compatible (electronics vs motor power)