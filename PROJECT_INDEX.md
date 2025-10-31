# Mini Rover Development - Project Index

Quick reference for all robot projects and systems.

---

## 🎯 Active Projects

### RTK GPS Base Station ✅ OPERATIONAL
**Location:** [Static RTK Base Station](Static%20RTK%20Base%20Station/)

**Status:** Fully configured and running (Oct 26, 2025)

**What it does:**
- Provides centimeter-accurate RTK GPS corrections via MQTT
- Supports multiple robots simultaneously
- ±2cm positioning accuracy

**Quick Access:**
- **Connection:** `ssh jay@192.168.254.165` (password: jay)
- **MQTT Broker:** `192.168.254.165:1883`
- **Topic:** `rtk/base/corrections`
- **Documentation:** See [README.md](Static%20RTK%20Base%20Station/README.md)

**Key Files:**
- [README.md](Static%20RTK%20Base%20Station/README.md) - Start here
- [RTK_BASE_REFERENCE.md](Static%20RTK%20Base%20Station/RTK_BASE_REFERENCE.md) - Complete reference
- [SYSTEM_STATUS.md](Static%20RTK%20Base%20Station/SYSTEM_STATUS.md) - Current status

---

### Jetson Simple Rover
**Location:** [Jetson Simple Rover](Jetson%20Simple%20Rover/)

**Platform:** NVIDIA Jetson Nano
**Purpose:** Basic rover with camera and motor control

---

### Mini Race Robots
**Location:** [Mini Race Robots](Mini%20Race%20Robots/)

**Purpose:** Autonomous racing robots
**Features:** Vision-based navigation, racing algorithms

---

### Cube Orange Based Rover
**Location:** [Cube Orange Based Rover](Cube%20Orange%20Based%20Rover/)

**Platform:** Cube Orange flight controller
**Purpose:** Advanced rover with ArduPilot

---

## 🔧 Support Projects

### GPIO Arduino
**Location:** [GPIO Arduino](GPIO%20Arduino/)

**Purpose:** Arduino-based GPIO expansion and control

---

### Mobile RTK Station
**Location:** [mobile_rtk_station](mobile_rtk_station/)

**Purpose:** Earlier RTK experiments and mobile beacon work

---

## 📚 Resources

### Data Sheets
**Location:** [Data Sheets](Data%20Sheets/)

**Contents:** Hardware datasheets and specifications

---

### Archive
**Location:** [archive](archive/)

**Contents:** Historical projects and old code

---

## 🤖 System Integration

### RTK GPS for Robots

To integrate RTK GPS into any robot project:

1. **Connect to RTK Base Station:**
   ```python
   import paho.mqtt.client as mqtt
   import serial

   MQTT_BROKER = "192.168.254.165"
   robot_gps = serial.Serial("/dev/ttyACM0", 115200)

   def on_message(client, userdata, msg):
       robot_gps.write(msg.payload)

   client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
   client.on_message = on_message
   client.connect(MQTT_BROKER, 1883)
   client.subscribe("rtk/base/corrections")
   client.loop_forever()
   ```

2. **Expected Results:**
   - RTK Fixed: ±2cm accuracy
   - Time to fix: 30-60 seconds
   - Range: 10-20km

3. **Full Documentation:**
   See [Static RTK Base Station/RTK_BASE_REFERENCE.md](Static%20RTK%20Base%20Station/RTK_BASE_REFERENCE.md)

---

## 📋 Quick Reference

### Active Systems

| System | IP Address | Status | Access |
|--------|-----------|--------|--------|
| RTK Base Station (RTKPi) | 192.168.254.165 | ✅ Running | SSH: jay@192.168.254.165 |
| Jetson Nano Rover | 192.168.8.110 | 🔄 Variable | SSH: jay@192.168.8.110 |

### Common Services

| Service | Host | Port | Protocol |
|---------|------|------|----------|
| RTK MQTT Broker | 192.168.254.165 | 1883 | MQTT |
| RTK Corrections Topic | - | - | rtk/base/corrections |

---

## 🛠️ Common Tools

### Utilities Location
**Path:** [bin](bin/)

Scripts and utilities used across projects

---

## 📝 Notes for Future Sessions

### RTK Base Station
- **Fully configured and tested** (Oct 26, 2025)
- Auto-starts on boot
- 12+ satellites locked
- Streaming corrections continuously
- All documentation complete

### Integration Pattern
Most robot projects will benefit from RTK GPS integration:
1. Install paho-mqtt on robot
2. Use code example above
3. Forward corrections to robot's GPS module
4. Achieve ±2cm accuracy

### Documentation Standard
Each project should have:
- README.md (overview and quick start)
- Technical reference documentation
- Connection/configuration details
- Code examples

---

**This index is maintained to help navigate between projects and understand system architecture.**

*Last Updated: October 26, 2025*
