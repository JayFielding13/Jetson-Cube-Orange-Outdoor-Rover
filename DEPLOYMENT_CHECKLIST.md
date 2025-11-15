# Jetson Simple Rover - Deployment Checklist

Complete checklist for building and deploying the Jetson Simple Rover with battery monitoring.

**Print this document and check off each step as you complete it.**

---

## Phase 1: Hardware Preparation

### Battery Monitoring Hardware

#### Option A: INA3221 (Recommended)
- [ ] Acquire INA3221 Triple-Channel Power Monitor
- [ ] Verify I2C pins on module (VCC, GND, SDA, SCL)
- [ ] Test module with multimeter before connecting

#### Option B: Voltage Divider + ADC
- [ ] Acquire resistors: 10kΩ and 4.7kΩ (1% tolerance)
- [ ] Build voltage divider circuit
- [ ] Test divider ratio with multimeter
- [ ] Verify output voltage < 3.3V with 12.6V input

### Ultrasonic Sensors
- [ ] Acquire 4× HC-SR04 ultrasonic sensors
- [ ] Label sensors: LF, RF, LS, RS
- [ ] Test each sensor individually
- [ ] Build/acquire 4× voltage dividers for ECHO pins
  - [ ] 1kΩ + 2kΩ resistors for each sensor
  - [ ] Test divider output (5V → 3.3V)

### LoRa Boards
- [ ] 2× Heltec WiFi LoRa 32 V3 boards
- [ ] Label: "TRANSMITTER" and "RECEIVER"
- [ ] 2× LoRa antennas (915MHz or appropriate frequency)
- [ ] USB cables for programming

### Additional Hardware
- [ ] Arduino Nano (motor controller)
- [ ] Cytron MDDS30 motor driver
- [ ] Logitech C920X webcam
- [ ] 3S LiPo battery (fully charged)
- [ ] Fuse for battery protection (recommended: 5A)
- [ ] Jumper wires (male-male, male-female)
- [ ] Breadboard (optional, for testing)

---

## Phase 2: Software Installation

### On Development Laptop

#### Arduino IDE Setup
- [ ] Install Arduino IDE 2.x
- [ ] Install Heltec ESP32 board support:
  - [ ] File → Preferences → Additional Board URLs
  - [ ] Add: `https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series/releases/download/0.0.7/package_heltec_esp32_index.json`
  - [ ] Tools → Board → Boards Manager → Search "Heltec" → Install
- [ ] Install libraries via Library Manager:
  - [ ] RadioLib
  - [ ] ArduinoJson
  - [ ] Heltec ESP32 Dev-Boards

#### Verify Code is Ready
- [ ] All Python files present in `Jetson Simple Rover/`
  - [ ] `rover_controller_jetson.py`
  - [ ] `battery_monitor.py`
  - [ ] `config.py`
  - [ ] `test_sensors.py`
  - [ ] `test_camera.py`
- [ ] All Arduino files present in `Jetson Heltec TX RX/`
  - [ ] `Transmitter_with_Battery_Display.ino`
  - [ ] `Receiver_with_Battery_Monitor.ino`

### On Jetson Orin Nano

#### System Setup
- [ ] SSH access working: `ssh jay@192.168.8.110`
- [ ] Static IP configured: 192.168.8.110
- [ ] System updated: `sudo apt update && sudo apt upgrade`

#### Python Dependencies
```bash
sudo pip3 install Jetson.GPIO
sudo pip3 install pyserial
sudo pip3 install opencv-python
sudo pip3 install adafruit-circuitpython-ina3221  # If using INA3221
```
- [ ] Jetson.GPIO installed
- [ ] pyserial installed
- [ ] opencv-python installed
- [ ] INA3221 library installed (if using)

#### GPIO Permissions
```bash
sudo groupadd -f -r gpio
sudo usermod -aG gpio jay
sudo usermod -aG dialout jay
```
- [ ] GPIO group created
- [ ] User added to gpio group
- [ ] User added to dialout group
- [ ] **Logged out and back in** (required for group changes)

#### Transfer Code
```bash
# From laptop
cd "/home/jay/Desktop/Mini Rover Development/Jetson Simple Rover"
scp *.py jay@192.168.8.110:~/rover/
```
- [ ] Created `~/rover/` directory on Jetson
- [ ] Transferred all Python files
- [ ] Verified files present: `ssh jay@192.168.8.110 "ls ~/rover/"`

---

## Phase 3: Electronics Assembly

### ⚠️ POWER OFF EVERYTHING BEFORE WIRING ⚠️

### Battery Monitoring Wiring

#### If Using INA3221:
```
INA3221          Jetson 40-pin Header
VCC     ───────→ Pin 1 (3.3V)
GND     ───────→ Pin 6 (GND)
SDA     ───────→ Pin 3 (I2C_2_SDA)
SCL     ───────→ Pin 5 (I2C_2_SCL)

3S LiPo (+)  ──→ IN+ (Channel 1)
3S LiPo (-)  ──→ IN- (Channel 1) & GND
```

**Checklist**:
- [ ] INA3221 VCC → Jetson Pin 1
- [ ] INA3221 GND → Jetson Pin 6
- [ ] INA3221 SDA → Jetson Pin 3
- [ ] INA3221 SCL → Jetson Pin 5
- [ ] Battery (+) → INA3221 IN+
- [ ] Battery (-) → INA3221 IN- & GND
- [ ] **Verify with multimeter before powering on**

#### If Using Voltage Divider:
```
3S LiPo (+) ──┬─── 10kΩ ───┬─── Jetson Pin 29 (AO_DMIC_IN_DAT)
              │            │
              │          4.7kΩ
              │            │
             GND          GND
```

**Checklist**:
- [ ] Built voltage divider (10kΩ + 4.7kΩ)
- [ ] Tested with multimeter (12.6V → ~4.0V)
- [ ] Battery (+) → divider input
- [ ] Divider output → Jetson Pin 29
- [ ] GND → Jetson GND (Pin 6, 9, 14, 20, 25, 30, 34, or 39)

### Ultrasonic Sensor Wiring

**Reference**: See `WIRING_GUIDE.md` for complete pin-by-pin instructions.

**Left Front Sensor**:
- [ ] VCC → Jetson Pin 2 (5V)
- [ ] GND → Jetson Pin 14 (GND)
- [ ] TRIG → Jetson Pin 26 (GPIO 7)
- [ ] ECHO → Voltage Divider → Jetson Pin 24 (GPIO 8)
- [ ] **Voltage divider installed and tested**

**Right Front Sensor**:
- [ ] VCC → Jetson Pin 4 (5V)
- [ ] GND → Jetson Pin 20 (GND)
- [ ] TRIG → Jetson Pin 21 (GPIO 9)
- [ ] ECHO → Voltage Divider → Jetson Pin 19 (GPIO 10)
- [ ] **Voltage divider installed and tested**

**Left Side Sensor**:
- [ ] VCC → Jetson Pin 2 (5V)
- [ ] GND → Jetson Pin 25 (GND)
- [ ] TRIG → Jetson Pin 23 (GPIO 11)
- [ ] ECHO → Voltage Divider → Jetson Pin 32 (GPIO 12)
- [ ] **Voltage divider installed and tested**

**Right Side Sensor**:
- [ ] VCC → Jetson Pin 4 (5V)
- [ ] GND → Jetson Pin 30 (GND)
- [ ] TRIG → Jetson Pin 33 (GPIO 13)
- [ ] ECHO → Voltage Divider → Jetson Pin 8 (GPIO 14)
- [ ] **Voltage divider installed and tested**

### USB Connections
- [ ] Arduino Nano → Jetson USB (motor controller)
- [ ] Heltec Receiver → Jetson USB (LoRa control)
- [ ] Logitech C920X → Jetson USB (camera)
- [ ] Identified USB ports: `ls -la /dev/ttyUSB* /dev/ttyACM*`
- [ ] Verified devices:
  - [ ] Arduino: `/dev/ttyUSB0` (or ttyACM0)
  - [ ] Heltec: `/dev/ttyUSB1` (or different port)
  - [ ] Camera: `/dev/video0`

### Power Mode Configuration
```bash
# Set to 15W mode
ssh jay@192.168.8.110
chmod +x ~/rover/setup_power_mode.sh
sudo ~/rover/setup_power_mode.sh
```
- [ ] Power mode script executed
- [ ] Verified mode: `sudo nvpmodel -q`
- [ ] Expected: `NV Power Mode: MODE_15W`

---

## Phase 4: LoRa Programming

### Program Receiver (On Rover)

**Arduino IDE Setup**:
- [ ] Connect Heltec receiver to laptop via USB
- [ ] Tools → Board → Heltec WiFi LoRa 32(V3)
- [ ] Tools → Port → Select correct COM port
- [ ] Open `Jetson Heltec TX RX/Receiver_with_Battery_Monitor.ino`

**Configuration Check**:
```cpp
#define SYNC_WORD   0x12  // Must match transmitter
#define LORA_FREQ   915.0 // 915MHz for US, adjust for region
```
- [ ] Sync word set (default: 0x12)
- [ ] Frequency correct for your region
- [ ] Sketch → Upload
- [ ] Upload successful
- [ ] Open Serial Monitor (115200 baud)
- [ ] Verify output: "LoRa Kill Switch - Receiver"

### Program Transmitter (Handheld)

**Arduino IDE Setup**:
- [ ] Connect Heltec transmitter to laptop via USB
- [ ] Tools → Board → Heltec WiFi LoRa 32(V3)
- [ ] Tools → Port → Select correct COM port
- [ ] Open `Jetson Heltec TX RX/Transmitter_with_Battery_Display.ino`

**Configuration Check**:
```cpp
#define SYNC_WORD   0x12    // Must match receiver
#define LORA_FREQ   915.0   // Same as receiver
#define BUZZER_PIN  -1      // Set GPIO pin or -1 to disable
```
- [ ] Sync word matches receiver
- [ ] Frequency matches receiver
- [ ] Buzzer pin configured (or disabled)
- [ ] Sketch → Upload
- [ ] Upload successful
- [ ] Open Serial Monitor (115200 baud)
- [ ] Verify output: "LoRa Kill Switch - Transmitter"

### Test LoRa Communication
- [ ] Both Heltec boards powered on
- [ ] Both OLEDs showing "STOP" mode
- [ ] Press transmitter button
- [ ] Transmitter OLED changes to "AUTONOMOUS"
- [ ] Receiver OLED changes to "AUTONOMOUS"
- [ ] Press button again
- [ ] Both return to "STOP"
- [ ] **LoRa communication working!**

---

## Phase 5: Testing

### Test 1: Jetson Boot and SSH
```bash
# Power on Jetson
# Wait 30-60 seconds
ping 192.168.8.110
ssh jay@192.168.8.110
```
- [ ] Jetson boots successfully
- [ ] Network accessible (ping works)
- [ ] SSH works

### Test 2: GPIO Access
```bash
ssh jay@192.168.8.110
cd ~/rover
python3 -c "import Jetson.GPIO as GPIO; GPIO.setmode(GPIO.BOARD); print('GPIO OK')"
```
- [ ] No permission errors
- [ ] Output: "GPIO OK"

### Test 3: Camera Detection
```bash
ssh jay@192.168.8.110
cd ~/rover
python3 test_camera.py
```
- [ ] Camera detected at /dev/video0
- [ ] Resolution test passes
- [ ] FPS acceptable (>20 fps at 1080p)
- [ ] Test image saved

### Test 4: Ultrasonic Sensors
```bash
ssh jay@192.168.8.110
cd ~/rover
python3 test_sensors.py
```
- [ ] All 4 sensors detected
- [ ] Readings reasonable (2-400 cm)
- [ ] No timeout errors
- [ ] Move hand in front of sensors - readings change

### Test 5: Battery Monitor Standalone
```bash
ssh jay@192.168.8.110
cd ~/rover
python3 battery_monitor.py
```
- [ ] Battery voltage reads correctly
- [ ] State determined correctly (FULL/GOOD/MODERATE/etc.)
- [ ] LoRa alerts sent (check Serial Monitor on receiver)
- [ ] Ctrl+C stops cleanly

### Test 6: Battery LoRa Integration

**On Jetson**:
```bash
ssh jay@192.168.8.110
cd ~/rover
python3 battery_monitor.py &
```

**On Receiver** (Serial Monitor):
- [ ] "Battery Update: X.XXV | STATE | XX%" appears
- [ ] Updates every 5 seconds (or configured interval)
- [ ] "Battery status sent to transmitter" appears

**On Transmitter** (OLED and Serial Monitor):
- [ ] Display alternates between status and battery
- [ ] Battery voltage matches actual voltage
- [ ] Battery state matches (FULL/GOOD/etc.)
- [ ] Percentage displayed correctly

### Test 7: Low Battery Alert (Simulated)

**Modify battery_monitor.py temporarily**:
```python
# In get_battery_voltage(), force low voltage
return 9.95  # Simulates CRITICAL state
```

**Expected behavior**:
- [ ] Jetson battery monitor shows CRITICAL state
- [ ] Receiver receives critical alert
- [ ] Transmitter receives critical alert
- [ ] Both OLEDs lock to battery display
- [ ] Visual warnings (!!!) appear on both screens
- [ ] Buzzer beeps (if enabled) on transmitter

**Restore normal operation**:
- [ ] Remove test code
- [ ] Restart battery monitor

### Test 8: Motor Controller
```bash
# Test Arduino motor controller responds to commands
# This will be part of rover_controller_jetson.py testing
```
- [ ] Arduino detected on USB
- [ ] Motor commands accepted
- [ ] Motors respond (if connected and safe to test)

---

## Phase 6: Integration Testing

### Full System Test - STOP Mode
```bash
# On Jetson
ssh jay@192.168.8.110
cd ~/rover
python3 battery_monitor.py &
python3 rover_controller_jetson.py
```

**Expected behavior**:
- [ ] Rover controller starts in STOP mode
- [ ] Battery monitor running
- [ ] Transmitter and receiver show "STOP"
- [ ] Motors disabled
- [ ] Battery data updating on transmitter

### Full System Test - AUTONOMOUS Mode
- [ ] Press transmitter button → "AUTONOMOUS"
- [ ] Receiver forwards mode to Jetson
- [ ] Rover controller enables autonomous navigation
- [ ] Sensors reading continuously
- [ ] Motors respond to navigation logic
- [ ] Battery data continues updating
- [ ] Press button again → returns to "STOP"

### Navigation Test (Bench Test - No Motion)
- [ ] Place rover on blocks (wheels off ground)
- [ ] Enable AUTONOMOUS mode
- [ ] Move obstacles near sensors
- [ ] Verify wheel speeds change appropriately
- [ ] Verify stuck detection works
- [ ] Return to STOP mode

### Navigation Test (Floor Test)
- [ ] Clear test area
- [ ] Place rover on floor
- [ ] Enable AUTONOMOUS mode
- [ ] Rover navigates autonomously
- [ ] Avoids obstacles smoothly
- [ ] Escapes corners
- [ ] Responds to side walls
- [ ] Battery updates continue
- [ ] Return to STOP mode

---

## Phase 7: Production Deployment

### Auto-Start Service (Optional)
```bash
ssh jay@192.168.8.110
sudo nano /etc/systemd/system/jetson-rover.service
```

**Service file content**:
```ini
[Unit]
Description=Jetson Simple Rover Controller
After=network.target

[Service]
Type=simple
User=jay
WorkingDirectory=/home/jay/rover
ExecStartPre=/usr/bin/python3 /home/jay/rover/battery_monitor.py &
ExecStart=/usr/bin/python3 /home/jay/rover/rover_controller_jetson.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Enable service**:
```bash
sudo systemctl daemon-reload
sudo systemctl enable jetson-rover.service
sudo systemctl start jetson-rover.service
sudo systemctl status jetson-rover.service
```

- [ ] Service file created
- [ ] Service enabled
- [ ] Service starts on boot
- [ ] Check logs: `journalctl -u jetson-rover.service -f`

### Final Safety Checks
- [ ] Battery fully charged
- [ ] All connections secure
- [ ] Voltage dividers tested
- [ ] Fuse installed in battery line
- [ ] Emergency stop (LoRa) tested
- [ ] Low battery alerts working
- [ ] Camera secured to rover
- [ ] All USB cables secured
- [ ] Power switch accessible

---

## Phase 8: Documentation

### Update Logs
- [ ] Record deployment date in DEVELOPMENT_LOG.md
- [ ] Note any issues encountered
- [ ] Document any configuration changes
- [ ] Update wiring if different from guide

### Performance Baseline
- [ ] Record battery runtime (full charge → LOW state)
- [ ] Note average sensor readings in open space
- [ ] Document navigation behavior in test environment
- [ ] Capture example battery voltage curve

---

## Troubleshooting Quick Reference

### Issue: Jetson won't boot
- Check power supply (5V 4A minimum)
- Check connections
- Verify LED activity on board

### Issue: GPIO permission denied
```bash
sudo usermod -aG gpio jay
# Log out and back in
```

### Issue: Sensors return 0 or timeout
- Check voltage divider wiring
- Verify GPIO pin numbers
- Test individual sensors
- Check 5V power supply to sensors

### Issue: Battery monitor shows 0.0V
- Check INA3221 wiring (I2C)
- Verify battery connections
- Test with multimeter
- Check I2C: `sudo i2cdetect -y -r 1`

### Issue: LoRa not communicating
- Check sync words match
- Verify antennas connected
- Check frequency setting
- Test range (start close)

### Issue: Camera not detected
- Check USB connection
- Verify with: `ls /dev/video*`
- Try different USB port
- Check permissions

---

## Success Criteria

### ✅ Deployment Complete When:
- [ ] All hardware connected and tested
- [ ] All software installed and running
- [ ] LoRa control working (STOP/AUTONOMOUS toggle)
- [ ] Battery monitoring working with LoRa alerts
- [ ] Ultrasonic sensors reading correctly
- [ ] Camera detected and functioning
- [ ] Autonomous navigation working
- [ ] Low battery alerts functioning
- [ ] Documentation updated

---

## Next Phase: Vision Integration (Phase 2)

After successful deployment of Phase 1, proceed to:
- [ ] Install YOLOv8 for object detection
- [ ] Implement vision processing module
- [ ] Create sensor fusion system
- [ ] Test vision-enhanced navigation

---

**Deployment Date**: _______________
**Deployed By**: _______________
**Notes**:
_________________________________________
_________________________________________
_________________________________________

**Last Updated**: 2025-10-19
**Version**: 1.0
