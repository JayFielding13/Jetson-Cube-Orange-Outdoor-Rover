# Simple Rover - Quick Reference

## Current Configuration (October 2025)

### Sensor Layout
```
        [LF] 30°   30° [RF]     LF = Left Front
              \   /             RF = Right Front
               \ /              LS = Left Side
                🤖              RS = Right Side
         [LS] ← → [RS]
```

### GPIO Pin Summary
| Sensor | TRIG | ECHO | Voltage Divider |
|--------|------|------|-----------------|
| Left Front | GPIO 23 | GPIO 24 | Required |
| Right Front | GPIO 17 | GPIO 27 | Required |
| Left Side | GPIO 5 | GPIO 6 | Required |
| Right Side | GPIO 22 | GPIO 25 | Required |

### Navigation Thresholds
| Parameter | Value | Description |
|-----------|-------|-------------|
| CRITICAL_THRESHOLD | 10 cm | Emergency stop distance |
| OBSTACLE_THRESHOLD | 50 cm | Start steering away |
| CORNER_TRAP_THRESHOLD | 20 cm | Both sensors = trapped |
| SENSOR_FUSION_DIFF | 8 cm | Perpendicular wall detection |
| SIDE_WARNING_THRESHOLD | 10 cm | Side wall steering trigger |

### Speed Settings
| Parameter | Value | Range |
|-----------|-------|-------|
| MAX_SPEED | 150 | 0-255 |
| BASE_SPEED | 100 | 0-255 |
| MIN_SPEED | 60 | 0-255 |
| ESCAPE_TURN_SPEED | 100 | 0-255 |

---

## Navigation Behavior

### Distance Zones
- **> 50cm**: Full speed forward
- **20-50cm**: Bendy ruler (slow down + steer)
- **10-20cm**: Aggressive steering
- **< 10cm**: Emergency STOP

### Sensor Fusion
- **Activated when**: Front sensors within 8cm of each other
- **Effect**: Treats as center obstacle, uses minimum distance for both
- **Example**: Left=25cm, Right=27cm → Both become 25cm

### Side Sensor Influence
- **Triggered when**: Side wall < 10cm
- **Effect**: Adds steering bias (30% threshold)
- **Behavior**: Gently steers away from wall (no stop)

### Corner Escape
- **Triggered when**: Both front sensors < 20cm
- **Behavior**: Rotate in place until clear path (> 50cm) found
- **Timeout**: 8 seconds maximum rotation

---

## System Management

### Service Control
```bash
# Start/Stop/Restart
sudo systemctl start simple-rover.service
sudo systemctl stop simple-rover.service
sudo systemctl restart simple-rover.service

# Status and Logs
sudo systemctl status simple-rover.service
journalctl -u simple-rover.service -f
```

### Manual Run (for testing)
```bash
cd "/home/jay/Simple Rover"
python3 rover_controller_with_lora.py
```

### Test All Sensors
```bash
cd "/home/jay/Simple Rover"
python3 test_all_sensors.py
```

---

## LoRa Communication

### Transmitter Commands
- **Button Press**: Toggle AUTONOMOUS ↔ STOP
- **OLED Display**: Shows current mode

### Receiver Behavior
- **STOP Mode**: Motors disabled, rover stationary
- **AUTONOMOUS Mode**: Bendy ruler navigation active
- **Default**: Always starts in STOP (safety)

### USB Port Assignments
- **Arduino (CH340)**: /dev/ttyUSB0 - Motor control
- **Heltec (CP2102)**: /dev/ttyUSB1 - LoRa receiver

**Note**: If LoRa not working, try different USB port for Heltec!

---

## Troubleshooting Checklist

### Rover Won't Move
- [ ] Transmitter battery charged?
- [ ] Both OLEDs show "AUTONOMOUS"?
- [ ] Service running? `systemctl status simple-rover.service`
- [ ] Try different USB port for Heltec

### Rover Gets Stuck at Walls
- [ ] Verify CRITICAL_THRESHOLD = 10 (not 15)
- [ ] Verify SENSOR_FUSION_DIFF = 8 (not 15)
- [ ] Run `test_all_sensors.py` to check readings
- [ ] Check side sensor mounting (perpendicular?)

### Sensor Not Reading
- [ ] Voltage divider correct? (1kΩ:2kΩ ratio)
- [ ] TRIG/ECHO connections secure?
- [ ] 5V power connected?
- [ ] Ground connected?

### USB Device Issues
```bash
# Identify devices
ls -la /dev/ttyUSB*

# Check device type
udevadm info /dev/ttyUSB0 | grep ID_MODEL
udevadm info /dev/ttyUSB1 | grep ID_MODEL

# Arduino = CH340, Heltec = CP2102
```

---

## Voltage Divider Configuration

### Resistor Setup (Per ECHO Pin)
```
5V ECHO Output
      |
  [2kΩ] [2kΩ] ← Parallel = 1kΩ equivalent
      |
      ├────→ To Pi GPIO (3.3V)
      |
    [2kΩ] ← To Ground
      |
     GND
```

### Voltage Calculation
- Input: 5V
- Top resistance: 1kΩ (2× 2kΩ parallel)
- Bottom resistance: 2kΩ
- **Output: 5V × (2kΩ / 3kΩ) = 3.33V ✓**

---

## File Locations

### Code Files
- Main Controller: `/home/jay/Simple Rover/rover_controller_with_lora.py`
- Test Script: `/home/jay/Simple Rover/test_all_sensors.py`

### Documentation
- This File: `/home/jay/Desktop/Mini Rover Development/Simple Rover/QUICK_REFERENCE.md`
- Update Log: `/home/jay/Desktop/Mini Rover Development/Simple Rover/UPDATE_LOG.md`
- Wiring Guide: `/home/jay/Desktop/Mini Rover Development/Simple Rover/WIRING_REFERENCE.md`
- Side Sensors: `/home/jay/Desktop/Mini Rover Development/Simple Rover/SIDE_SENSOR_WIRING.md`

### Arduino Code
- Transmitter: `/home/jay/Desktop/Mini Rover Development/Arduino Code/Heltec Code/LoRa_KillSwitch_Transmitter_Channel/`
- Receiver: `/home/jay/Desktop/Mini Rover Development/Arduino Code/Heltec Code/LoRa_KillSwitch_Receiver_Channel/`
- Motor Controller: `/home/jay/Desktop/Mini Rover Development/Arduino Code/Simple Rover/Mixed_RC_mode_motor_controller/`

---

## Recent Updates Summary

### October 16, 2025
1. **Added Side Sensors**
   - GPIO 5/6 (left), GPIO 22/25 (right)
   - Gentle wall avoidance within 10cm

2. **Fixed "Stuck" Bug**
   - CRITICAL_THRESHOLD: 15cm → 10cm
   - SENSOR_FUSION_DIFF: 15cm → 8cm
   - Rover now handles wall approaches smoothly

3. **USB Port Fix**
   - Heltec moved to reliable USB port
   - Data transmission working correctly

### System Status
- ✅ All 4 sensors operational
- ✅ LoRa communication working
- ✅ Auto-start on boot enabled
- ✅ Navigation smooth and reliable
