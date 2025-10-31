# RTK Base Station Setup

## Overview
Laptop-based RTK base station providing RTCM3 correction data for rover operations. Uses u-blox F9P GPS receiver for centimeter-level positioning accuracy.

## Hardware Configuration
- **GPS Receiver**: u-blox F9P connected via USB (/dev/ttyACM0)
- **SiK Radio**: Holybro SiK radio on /dev/ttyUSB0 for wireless RTK relay
- **Baud Rate**: 115200 for GPS, 57600 for SiK radio

## System Architecture
```
[u-blox GPS] → [RTKLib] → [NTRIP Server] → [SiK Radio] → [Mobile Beacon]
```

## Key Files

### `quick_rtcm_server.sh`
Main startup script for RTK base station operations:
- Configures GPS receiver for base station mode
- Starts str2str for RTCM3 correction generation
- Serves corrections via NTRIP on port 2101
- Generates ~13kbps correction data stream

### `base_station.conf`
RTKLib configuration file with:
- Survey-in parameters for base position
- RTCM message types (1005, 1077, 1087, 1097, 1127, 1230)
- Positioning accuracy settings

### `sik_rtk_relay.py`
Python script for wireless RTK correction transmission:
- Receives RTCM data from NTRIP stream
- Transmits corrections via SiK radio network
- Network ID: 25, Node ID: 26

## SiK Radio Configuration
```
Network ID: 25
Node ID: 26 (Base Station)
Air Data Rate: 128 kbps
Serial Speed: 57600 baud
Transmit Power: 20 dBm
```

## Usage

### Start Base Station
```bash
cd ~/rtk_base_station
./quick_rtcm_server.sh
```

### Monitor RTCM Stream
```bash
curl http://localhost:2101/RTCM3
```

### Start SiK RTK Relay
```bash
python3 sik_rtk_relay.py
```

## Performance
- **Correction Rate**: ~13 kbps RTCM3 data
- **Position Accuracy**: Survey-in to <1m accuracy
- **Update Rate**: 1 Hz corrections
- **Wireless Range**: ~1km with SiK radios

## Status Monitoring
- GPS status via `str2str.trace` log file
- NTRIP connections on port 2101
- SiK radio status via AT commands
- RTK fix quality indicators

## Integration Notes
- Works with Mobile RTK Beacon on Pi (192.168.8.131)
- Compatible with QGroundControl via MAVLink bridge
- Supports Follow Me operations with centimeter accuracy
- Dual battery system ready (electronics vs motor power)