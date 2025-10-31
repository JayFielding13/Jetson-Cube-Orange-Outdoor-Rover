# Simple Rover - Complete Setup and Testing Guide

## Table of Contents
1. [System Overview](#system-overview)
2. [Hardware Setup](#hardware-setup)
3. [Software Installation](#software-installation)
4. [LoRa Wireless Cutoff Switch](#lora-wireless-cutoff-switch)
5. [Network Configuration](#network-configuration)
6. [System Testing](#system-testing)
7. [Troubleshooting](#troubleshooting)

---

## System Overview

### Components
- **Main Controller:** Raspberry Pi 4
- **Motor Controller:** Arduino Nano
- **Motor Driver:** Cytron MDDS30 (R/C mode)
- **Sensors:** 2× HC-SR04 Ultrasonic Sensors (left and right)
- **Wireless Control:** 2× Heltec LoRa WiFi 32 V3 (transmitter and receiver)
- **Drive System:** Tank-style differential drive with tracks

### Control Architecture
```
┌─────────────────┐
│ LoRa Transmitter│ (Wireless remote with button)
│  (Heltec V3)    │
└────────┬────────┘
         │ LoRa Radio
         ▼
┌─────────────────┐
│ LoRa Receiver   │
│  (Heltec V3)    │
└────────┬────────┘
         │ USB Serial
         ▼
┌─────────────────┐      USB Serial      ┌──────────────┐
│  Raspberry Pi 4 │◄────────────────────►│ Arduino Nano │
│  (Main Control) │                      │ (Motor PWM)  │
└────────┬────────┘                      └──────┬───────┘
         │                                       │
         │ GPIO                                  │ R/C PWM
         ▼                                       ▼
┌─────────────────┐                      ┌──────────────┐
│  Ultrasonic     │                      │  MDDS30      │
│  Sensors (L+R)  │                      │  Motor Driver│
└─────────────────┘                      └──────┬───────┘
                                                 │
                                                 ▼
                                          ┌──────────────┐
                                          │ Left + Right │
                                          │    Motors    │
                                          └──────────────┘
```

---

## Hardware Setup

### Step 1: Wire Ultrasonic Sensors
See [WIRING_REFERENCE.md](WIRING_REFERENCE.md) for detailed pin connections.

**CRITICAL:** ECHO pins require voltage dividers! 5V from sensor will damage Pi GPIO.

### Step 2: Connect Arduino to Motor Driver
- Arduino D5 → MDDS30 CH2 (Right Motor)
- Arduino D6 → MDDS30 CH1 (Left Motor)
- Arduino GND → MDDS30 GND (common ground)

### Step 3: Configure MDDS30
Set DIP switches for **R/C mode** (consult MDDS30 manual).

### Step 4: Connect Power
- Raspberry Pi: 5V 3A supply (USB-C or GPIO)
- Arduino: Powered via USB from Pi
- MDDS30: 7-35V battery pack (must share GND with Arduino)

### Step 5: Connect USB Devices to Pi
- Arduino Nano → USB (typically /dev/ttyUSB0)
- Heltec LoRa Receiver → USB (typically /dev/ttyUSB1)

---

## Software Installation

### On Raspberry Pi

#### 1. Install Required Python Libraries
```bash
cd "/home/jay/Desktop/Mini Rover Development/Simple Rover"

# Install dependencies
pip3 install pyserial RPi.GPIO
```

#### 2. Install Arduino CLI (for remote Arduino programming)
```bash
# Download and install Arduino CLI
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Add to PATH (already in ~/.bashrc)
export PATH=$PATH:$HOME/.local/bin

# Update Arduino CLI
arduino-cli core update-index

# Install Arduino AVR core
arduino-cli core install arduino:avr

# Install Servo library
arduino-cli lib install Servo
```

#### 3. Upload Arduino Sketch
```bash
# From Raspberry Pi, upload motor control code
arduino-cli compile --fqbn arduino:avr:nano ~/Simple\ Rover/arduino_motor_control.ino
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano ~/Simple\ Rover/arduino_motor_control.ino
```

### On Heltec Boards

#### 1. Upload Transmitter Code
Upload `LoRa_KillSwitch_Transmitter_Channel.ino` to transmitter Heltec board using Arduino IDE.

**Hardware:**
- Button: GPIO 47 (internal pull-up, active LOW)
- LED: GPIO 48 (illuminates button)
- Sync Word: 0x12 (channel separation)

#### 2. Upload Receiver Code
Upload `LoRa_KillSwitch_Receiver_Channel.ino` to receiver Heltec board.

**Configuration:**
- Same Sync Word: 0x12 (must match transmitter)
- Serial output: Commands sent to Pi via USB

---

## LoRa Wireless Cutoff Switch

### Button Operation
- **Press button:** Toggle between STOP and AUTONOMOUS modes
- **LED indicator:**
  - Blinking: STOP mode (rover disabled)
  - Solid: AUTONOMOUS mode (rover active)

### Mode Behavior
- **STOP Mode (default):**
  - Rover motors disabled
  - All movement commands ignored
  - Safe state for handling rover

- **AUTONOMOUS Mode:**
  - Obstacle avoidance enabled
  - Motors respond to sensor inputs
  - Rover navigates autonomously

### Channel Separation
Multiple rover/transmitter pairs can operate simultaneously using different sync words:
- Rover 1: 0x12
- Rover 2: 0x34
- Rover 3: 0x56
- etc.

Change `SYNC_WORD` in both transmitter and receiver code to match.

---

## Network Configuration

### Connect to Travel Router

See [TRAVEL_ROUTER_SETUP.md](TRAVEL_ROUTER_SETUP.md) for detailed network setup.

**Quick Setup:**
```bash
# On Simple Rover Pi
sudo nmcli connection add \
    type wifi \
    con-name "RoverNet" \
    ssid "RoverNet" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "YOUR_PASSWORD" \
    ipv4.method manual \
    ipv4.addresses 192.168.8.100/24 \
    ipv4.gateway 192.168.8.1 \
    connection.autoconnect yes

# Activate connection
nmcli connection up "RoverNet"
```

**SSH Access:**
```bash
# From laptop (on same network)
ssh jay@192.168.8.100
```

### Internet Bridging from Laptop (Optional)

If Pi needs internet access while on travel router network:

```bash
# On laptop (one-time setup)
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -o wlp3s0 -j MASQUERADE
sudo iptables -A FORWARD -i enp5s0 -o wlp3s0 -j ACCEPT

# On Pi, set laptop as gateway
sudo ip route add default via 192.168.8.182  # Laptop's IP on travel router
```

---

## System Testing

### Pre-Flight Checklist
- [ ] All power connections secure
- [ ] MDDS30 DIP switches set to R/C mode
- [ ] USB devices connected (Arduino + Heltec)
- [ ] Sensors wired with voltage dividers
- [ ] Battery charged
- [ ] LoRa transmitter powered on
- [ ] Pi connected to travel router network

### Test 1: USB Device Detection
```bash
ssh jay@192.168.8.100
ls /dev/ttyUSB*
```

Expected output:
```
/dev/ttyUSB0  (Arduino Nano)
/dev/ttyUSB1  (Heltec LoRa Receiver)
```

### Test 2: Systems Check
Run the comprehensive systems check:
```bash
cd "/home/jay/Desktop/Mini Rover Development/Simple Rover"
python3 systems_check.py
```

Expected results:
- ✓ USB devices found (2/2)
- ✓ Arduino responding
- ✓ Left sensor reading distance
- ✓ Right sensor reading distance

### Test 3: Motor Test
**IMPORTANT:** Elevate rover on a box (wheels/tracks off ground) before motor testing!

```bash
python3 motor_test_auto.py
```

**Expected behavior:**
1. **FORWARD:** Both motors spin forward together
2. **BACKWARD:** Both motors spin backward together
3. **TURN LEFT:** Right motor forward, left motor backward
4. **TURN RIGHT:** Left motor forward, right motor backward

### Test 4: LoRa Communication Test
```bash
# Monitor LoRa receiver output
python3 -c "
import serial
ser = serial.Serial('/dev/ttyUSB1', 115200)
print('Listening for LoRa commands... Press transmitter button.')
while True:
    if ser.in_waiting:
        print(ser.readline().decode().strip())
"
```

Press the button on the LoRa transmitter and verify:
- `MODE:STOP` or `MODE:AUTONOMOUS` messages appear
- LED on transmitter changes state

### Test 5: Full System Test
**Still keep rover elevated for initial test!**

```bash
cd "/home/jay/Desktop/Mini Rover Development/Simple Rover"
python3 rover_controller_with_lora.py
```

**Test sequence:**
1. Rover starts in STOP mode (motors disabled)
2. Press transmitter button → LED solid, "AUTONOMOUS MODE ENABLED"
3. Move hand near left sensor → Rover turns right
4. Move hand near right sensor → Rover turns left
5. Press transmitter button → LED blinking, "STOP MODE"
6. Verify motors disabled in STOP mode

---

## Troubleshooting

### Motors Not Moving

**Check 1: Serial Communication**
```bash
# Test Arduino directly
python3 -c "
import serial, time
ser = serial.Serial('/dev/ttyUSB0', 115200)
time.sleep(2)
ser.write(b'F150\n')
print(ser.readline().decode())
"
```
Expected: `OK: F150`

**Check 2: Motor Driver Power**
- Verify battery connected to MDDS30
- Check battery voltage (should be 7V-35V)
- Confirm motor LEDs on MDDS30 are lit

**Check 3: DIP Switches**
- MDDS30 must be in R/C mode
- Consult MDDS30 manual for correct switch positions

**Check 4: Pin Connections**
- Verify D5 and D6 connections to MDDS30
- Confirm common ground between Arduino and MDDS30

### LoRa Not Receiving

**Check 1: Sync Word Mismatch**
- Both transmitter and receiver must have same `SYNC_WORD`
- Verify in both .ino files

**Check 2: Range**
- Transmitter should be within ~100m line-of-sight
- Obstacles reduce range

**Check 3: Receiver USB Connection**
```bash
ls /dev/ttyUSB*
# Should show Heltec board (typically ttyUSB1)
```

**Check 4: Monitor Receiver Serial**
```bash
python3 -c "import serial; ser=serial.Serial('/dev/ttyUSB1',115200); [print(ser.readline().decode().strip()) for _ in range(10)]"
```

### Sensors Not Reading

**Check 1: Voltage Dividers**
- CRITICAL: ECHO pins MUST have voltage dividers!
- Test voltage at Pi GPIO (should be ~3.3V, not 5V)

**Check 2: GPIO Permissions**
```bash
groups | grep gpio
# If not in gpio group:
sudo usermod -a -G gpio jay
# Log out and back in
```

**Check 3: Test Sensors Individually**
```python
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
TRIG = 23  # Left sensor
ECHO = 24

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

GPIO.output(TRIG, GPIO.HIGH)
time.sleep(0.00001)
GPIO.output(TRIG, GPIO.LOW)

while GPIO.input(ECHO) == GPIO.LOW:
    start = time.time()
while GPIO.input(ECHO) == GPIO.HIGH:
    end = time.time()

distance = (end - start) * 17150
print(f"Distance: {distance:.2f} cm")
GPIO.cleanup()
```

### Network Connection Issues

**Check 1: WiFi Status**
```bash
nmcli connection show --active
ip addr show wlan0
```

**Check 2: Reconnect to Travel Router**
```bash
sudo nmcli connection up "RoverNet"
```

**Check 3: Ping Test**
```bash
# From laptop
ping 192.168.8.100
```

### Arduino Upload Failures

**Error: "Servo.h: No such file or directory"**
```bash
# Install Servo library
arduino-cli lib install Servo
```

**Error: "Permission denied" on /dev/ttyUSB0**
```bash
# Add user to dialout group
sudo usermod -a -G dialout jay
# Log out and back in
```

**Error: "Can't open device"**
```bash
# Check which port Arduino is on
ls /dev/ttyUSB*
# Update port in compile command
```

---

## Maintenance

### Before Each Use
- [ ] Charge batteries (Pi + MDDS30 + Travel Router + LoRa Transmitter)
- [ ] Check all USB connections secure
- [ ] Verify sensors clean (no dust/debris)
- [ ] Test LoRa transmitter battery

### After Each Use
- [ ] Power down gracefully: `sudo shutdown -h now`
- [ ] Disconnect power
- [ ] Store batteries properly
- [ ] Check for loose wires or connections

### Periodic
- [ ] Update software: `git pull` (if using version control)
- [ ] Check motor brushes/gears for wear
- [ ] Clean sensors with soft cloth
- [ ] Backup configuration files

---

## File Locations

### Raspberry Pi
- Main controller: `/home/jay/Desktop/Mini Rover Development/Simple Rover/rover_controller_with_lora.py`
- Arduino code: `/home/jay/Desktop/Mini Rover Development/Simple Rover/arduino_motor_control.ino`
- Systems check: `/tmp/systems_check.py`
- Motor test: `/tmp/motor_test_auto.py`

### Heltec LoRa Boards
- Transmitter: `Simple Rover/Simple Rover Cutoff Switch/Transmitter/LoRa_KillSwitch_Transmitter_Channel.ino`
- Receiver: `Simple Rover/Simple Rover Cutoff Switch/Receiver/LoRa_KillSwitch_Receiver_Channel.ino`

---

## Command Reference

### Start Rover
```bash
cd "/home/jay/Desktop/Mini Rover Development/Simple Rover"
python3 rover_controller_with_lora.py
```

### Run Systems Check
```bash
python3 systems_check.py
```

### Test Motors
```bash
python3 motor_test_auto.py
```

### Monitor LoRa Receiver
```bash
python3 -c "import serial; ser=serial.Serial('/dev/ttyUSB1',115200); [print(ser.readline().decode().strip()) for _ in iter(int, 1)]"
```

### Check USB Devices
```bash
ls -la /dev/ttyUSB*
```

### Upload Arduino Code
```bash
arduino-cli compile --fqbn arduino:avr:nano arduino_motor_control/
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano arduino_motor_control/
```

---

## Safety Guidelines

### Operating Safety
1. **Always start in STOP mode** (default)
2. **Test on elevated platform first** before floor operation
3. **Keep LoRa transmitter in hand** while rover is active
4. **Press button immediately** if unexpected behavior occurs
5. **Never reach toward moving rover** - use STOP mode first

### Electrical Safety
1. **Disconnect power** before wiring changes
2. **Never bypass voltage dividers** on sensor ECHO pins
3. **Check polarity** on motor driver and battery connections
4. **Ensure proper grounding** between all components
5. **Monitor temperatures** during extended operation

### Battery Safety
1. **Use appropriate charger** for battery type
2. **Never over-discharge** LiPo/Li-ion batteries
3. **Store in fire-safe location**
4. **Inspect for damage** before each use
5. **Dispose properly** when end-of-life

---

## Performance Tuning

### Adjust Obstacle Detection Distance
Edit `rover_controller_with_lora.py`:
```python
OBSTACLE_THRESHOLD = 30  # Distance in cm (default: 30)
```

### Adjust Turn Speed
Edit `rover_controller_with_lora.py`:
```python
TURN_SPEED = 120  # Speed 0-255 (default: 120)
```

### Adjust Forward Speed
Edit `rover_controller_with_lora.py`:
```python
FORWARD_SPEED = 100  # Speed 0-255 (default: 100)
```

### Change LoRa Channel
Edit both transmitter and receiver .ino files:
```cpp
#define SYNC_WORD 0x12  // Change to 0x34, 0x56, etc.
```

---

## Version History

### v1.0 - 2025-10-13
- Initial setup and testing completed
- LoRa wireless cutoff switch integrated
- Motor control verified working
- Network configuration completed
- All systems tested and operational

---

## Support

For issues or questions:
1. Check this guide's [Troubleshooting](#troubleshooting) section
2. Review [WIRING_REFERENCE.md](WIRING_REFERENCE.md) for connections
3. Check [TRAVEL_ROUTER_SETUP.md](TRAVEL_ROUTER_SETUP.md) for network issues
4. Verify all checklist items completed

---

Last Updated: 2025-10-13
