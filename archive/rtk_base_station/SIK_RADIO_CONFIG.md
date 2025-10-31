# SiK Radio Network Configuration

## Network Overview
Mesh network configuration for RTK correction data transmission and MAVLink communication between base station, mobile beacon, and rover.

## Network Topology
```
Base Station (Laptop)     Mobile Beacon (Pi)        Rover
Node ID: 26          ←→   Node ID: 131          ←→   Node ID: [TBD]
/dev/ttyUSB0              /dev/ttyUSB0              /dev/ttyUSB0
```

## Radio Configuration Parameters

### Common Settings (All Nodes)
```
Network ID: 25
Air Data Rate: 128 kbps
Serial Speed: 57600 baud
Transmit Power: 20 dBm
Duty Cycle: 100%
LBT RSSI: 0
Max Window: 131ms
Min Frequency: 915000 kHz
Max Frequency: 928000 kHz
Number of Channels: 50
```

### Node-Specific Settings

#### Base Station (Node 26)
```
Node ID: 26
Destination: 65535 (Broadcast)
Role: Base station RTK correction transmitter
```

#### Mobile Beacon (Node 131)
```
Node ID: 131
Destination: 26 (Base Station)
Role: RTK correction receiver and MAVLink bridge
```

## Configuration Commands

### Using AT Commands
Connect to radio via serial terminal (57600 baud):

```
AT&F                    # Factory reset
ATS3=55                 # Set network ID to 25 (decimal 25 = hex 0x19, serial value 55)
ATS5=26                 # Set node ID (26 for base, 131 for beacon)
ATS6=65535              # Set destination (65535 for broadcast, 26 for base station)
ATS1=57                 # Set serial speed to 57600
ATS2=128                # Set air data rate to 128kbps
ATS14=20                # Set transmit power to 20dBm
AT&W                    # Write settings to EEPROM
ATZ                     # Reset radio
```

### Using Python Configuration Script
See `configure_sik_radio.py` in mobile_rtk_station directory.

## Data Flow

### RTK Corrections (Base → Beacon)
```
[RTKLib str2str] → [NTRIP:2101] → [sik_rtk_relay.py] → [SiK Radio] → [Pi SiK Radio] → [sik_rtk_receiver.py] → [GPS Receiver]
```

### MAVLink Commands (QGC → Rover)
```
[QGroundControl] → [TCP:5760] → [mavlink_bridge.py] → [SiK Radio] → [Rover SiK Radio] → [Flight Controller]
```

## Performance Characteristics

### Range and Reliability
- **Line of Sight**: ~1km typical range
- **Urban Environment**: 200-500m typical
- **Data Rate**: ~13kbps for RTK corrections
- **Latency**: <100ms typical
- **Packet Loss**: <1% in good conditions

### Signal Quality Monitoring
```bash
# Check signal strength
screen /dev/ttyUSB0 57600
+++
AT         # Enter AT mode
ATI5       # Show signal strength
ATI6       # Show error statistics
ATI7       # Show version info
```

## Troubleshooting

### Common Issues
1. **No Communication**: Check network ID and node IDs
2. **Poor Range**: Verify transmit power and antenna connections
3. **High Packet Loss**: Check for interference, adjust frequency range
4. **Slow Data**: Verify air data rate and serial speed settings

### Diagnostic Commands
```
ATI        # Basic radio info
ATI5       # RSSI and noise levels
ATI6       # Error counters
ATI7       # Version and build info
ATS?       # Show all parameters
```

### Reset Procedure
```
AT&F       # Factory defaults
AT&W       # Write to EEPROM
ATZ        # Reset radio
```

## Network Security
- Network ID provides basic access control
- Consider frequency hopping for additional security
- Monitor for unauthorized nodes with ATI commands
- Change default network ID for production use

## Integration Notes
- Compatible with ArduPilot MAVLink protocol
- Supports RTKLib RTCM3 correction format
- Works with QGroundControl TCP bridge
- Ready for rover flight controller integration