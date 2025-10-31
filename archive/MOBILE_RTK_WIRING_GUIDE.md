# Mobile RTK Station - Wiring & Connection Guide
**Pi 4B + SparkFun GPS-RTK-SMA + 7" Touchscreen Integration**

**Purpose**: Detailed wiring diagrams and connection procedures for hardware integration  
**Hardware**: Using your existing professional-grade GPS components

---

## 🔌 **MAIN SYSTEM CONNECTIONS**

### **SparkFun GPS-RTK-SMA to Pi 4B Wiring**

#### **Primary UART Connection** (Recommended)
```
SparkFun GPS-RTK-SMA → Raspberry Pi 4B GPIO
────────────────────────────────────────────
VCC (3.3V) → Pin 1  (3.3V Power)
GND        → Pin 6  (Ground)  
TXO        → Pin 10 (GPIO 15 - UART0 RX)
RXI        → Pin 8  (GPIO 14 - UART0 TX)

Additional Connections:
SAFEBOOT   → Not connected (leave floating)
EXTINT     → Not connected (leave floating)
PPS        → Pin 12 (GPIO 18) - Optional for precision timing
```

#### **Alternative I2C Connection** (If UART needed elsewhere)
```
SparkFun GPS-RTK-SMA → Raspberry Pi 4B GPIO
────────────────────────────────────────────
VCC (3.3V) → Pin 1  (3.3V Power)
GND        → Pin 6  (Ground)
SDA        → Pin 3  (GPIO 2 - I2C1 SDA)  
SCL        → Pin 5  (GPIO 3 - I2C1 SCL)

Note: I2C may be slower for high-rate position updates
Recommended for: Configuration only, use UART for data
```

### **7" Touchscreen Display Connection**
```
Official Pi 7" Touchscreen → Raspberry Pi 4B
──────────────────────────────────────────────
Power: DSI connector provides power to Pi 4B
Display: DSI ribbon cable (included with screen)
Touch: USB connection (included with screen)

GPIO Usage: Display uses DSI and power pins only
Available GPIO: All standard GPIO pins remain available
```

### **Status LED Indicators** (Optional backup to touchscreen)
```
Status LEDs → Raspberry Pi 4B GPIO
─────────────────────────────────────
GPS Status LED (Green):
  LED Anode → Pin 16 (GPIO 23)
  LED Cathode → 220Ω resistor → Pin 14 (Ground)

Communication LED (Blue):  
  LED Anode → Pin 18 (GPIO 24)
  LED Cathode → 220Ω resistor → Pin 20 (Ground)

Emergency LED (Red):
  LED Anode → Pin 22 (GPIO 25) 
  LED Cathode → 220Ω resistor → Pin 24 (Ground)
```

### **Hardware Controls**
```
Emergency Button → Raspberry Pi 4B GPIO
────────────────────────────────────────────
Button Terminal 1 → Pin 26 (GPIO 7)
Button Terminal 2 → Pin 30 (Ground)
Internal Pull-up: Enabled in software (GPIO.PUD_UP)

Buzzer (Optional Audio Feedback):
Buzzer Positive → Pin 32 (GPIO 12)
Buzzer Negative → Pin 34 (Ground)
```

---

## 📋 **COMPLETE GPIO PIN ASSIGNMENT**

### **Pi 4B GPIO Allocation**
```
Pin  GPIO  Function                    Connection
─────────────────────────────────────────────────────────
1    3.3V  Power Supply               GPS VCC  
2    5V    Power Supply               (Available)
3    2     I2C1 SDA                   (GPS Alt/Available)
4    5V    Power Supply               (Available)
5    3     I2C1 SCL                   (GPS Alt/Available)
6    GND   Ground                     GPS GND
7    4     GPIO                       (Available)
8    14    UART0 TX                   GPS RXI ⭐
9    GND   Ground                     (Common Ground)
10   15    UART0 RX                   GPS TXO ⭐
11   17    GPIO                       (Available)
12   18    GPIO                       GPS PPS (Optional) ⭐
13   27    GPIO                       (Available)
14   GND   Ground                     LED Common Ground
15   22    GPIO                       (Available)
16   23    GPIO                       GPS Status LED ⭐
17   3.3V  Power Supply               (Available)
18   24    GPIO                       Comm Status LED ⭐
19   10    SPI0 MOSI                  (Available)
20   GND   Ground                     LED Common Ground
21   9     SPI0 MISO                  (Available)
22   25    GPIO                       Emergency LED ⭐
23   11    SPI0 CLK                   (Available)
24   8     SPI0 CE0                   (Available)
25   GND   Ground                     (Available)
26   7     GPIO                       Emergency Button ⭐
27   0     I2C0 SDA                   (Reserved EEPROM)
28   1     I2C0 SCL                   (Reserved EEPROM)
29   5     GPIO                       (Available)
30   GND   Ground                     Button Ground
31   6     GPIO                       (Available)  
32   12    GPIO                       Buzzer Control ⭐
33   13    GPIO                       (Available)
34   GND   Ground                     Buzzer Ground
35   19    GPIO                       (Available)
36   16    GPIO                       (Available)
37   26    GPIO                       (Available)
38   20    GPIO                       (Available)
39   GND   Ground                     (Available)
40   21    GPIO                       (Available)
```

### **Summary of Used Pins**
```
Core GPS Interface:
- Pin 1, 6: Power and Ground
- Pin 8, 10: UART communication (primary)
- Pin 12: PPS signal (optional precision timing)

Status Interface:
- Pin 16, 18, 22: Status LEDs (GPS, Comm, Emergency)
- Pin 14, 20, 24: LED ground connections

User Interface:
- Pin 26, 30: Emergency button and ground  
- Pin 32, 34: Buzzer and ground

Available: 20+ GPIO pins for future expansion
```

---

## 🔧 **PHYSICAL ASSEMBLY INSTRUCTIONS**

### **Step 1: GPS Module Connection**
**Components**: SparkFun GPS-RTK-SMA, Pi 4B, jumper wires

```bash
# 1. Enable UART on Pi 4B
sudo raspi-config
# → Interface Options → Serial Port
# → "Would you like login shell accessible over serial?" → No  
# → "Would you like the serial port hardware enabled?" → Yes
# → Reboot required

# 2. Verify UART is available
ls -l /dev/serial*
# Should show: /dev/serial0 → ttyAMA0
```

**Physical Connections**:
1. **Power Connection**: GPS VCC (3.3V) to Pi Pin 1, GND to Pi Pin 6
2. **Data Connection**: GPS TXO to Pi Pin 10 (RX), GPS RXI to Pi Pin 8 (TX)  
3. **Optional PPS**: GPS PPS to Pi Pin 12 (for precise timing)
4. **Antenna**: Connect helical antenna to GPS SMA connector

**Verification**:
```bash
# Test UART communication
sudo apt install minicom
sudo minicom -D /dev/serial0 -b 38400

# Should see NMEA messages streaming:
# $GNGGA,123456.00,4012.12345,N,10512.12345,W,1,12,0.8,1234.5,M,0.0,M,,*hh
```

### **Step 2: Touchscreen Integration**
**Components**: Official Pi 7" Touchscreen, DSI cable, USB cable

**Physical Assembly**:
1. **DSI Connection**: Connect DSI ribbon cable from Pi to display  
2. **USB Touch**: Connect USB cable from display to Pi USB port
3. **Power**: Display can power Pi through DSI connector
4. **Mounting**: Secure Pi to back of display using standoffs

**Software Setup**:
```bash
# Update system for touchscreen support
sudo apt update && sudo apt upgrade -y

# Install touchscreen drivers (usually automatic)
sudo apt install xinput-calibrator

# Test touchscreen
xinput list
# Should show touchscreen device

# Rotate display if needed (landscape/portrait)
sudo nano /boot/config.txt
# Add: display_rotate=1 (for 90° rotation)
```

### **Step 3: Status LEDs and Controls**
**Components**: RGB LEDs, 220Ω resistors, momentary button, buzzer

**LED Assembly** (Breadboard/Perfboard):
```
For each LED:
GPIO Pin → LED Anode (long leg)
LED Cathode (short leg) → 220Ω Resistor → Ground Pin

Colors:
- Green LED: GPS Status (Pin 16 → GPIO 23)
- Blue LED: Communication (Pin 18 → GPIO 24)  
- Red LED: Emergency (Pin 22 → GPIO 25)
```

**Button Connection**:
```
Momentary Button:
Terminal 1 → Pin 26 (GPIO 7)
Terminal 2 → Pin 30 (Ground)
Note: Use internal pull-up resistor in software
```

**Buzzer Connection**:
```
Active Buzzer (3.3V):
Positive → Pin 32 (GPIO 12)
Negative → Pin 34 (Ground)
```

---

## 🔌 **POWER SYSTEM DESIGN**

### **Power Requirements Analysis**
```
Component Power Consumption:
─────────────────────────────────────
Raspberry Pi 4B:        600mA (idle) - 1200mA (active)
7" Touchscreen:          400mA (full brightness)
GPS-RTK Module:          50mA (continuous)
Status LEDs:             10mA (all on)
Buzzer:                  20mA (when active)
─────────────────────────────────────
Total Average:           ~1100mA
Total Peak:              ~1700mA
```

### **Battery Specification**
```
Recommended: 20,000mAh USB Power Bank
─────────────────────────────────────────
Dual USB Output:         2A total capacity
USB-C PD:                For Pi 4B (3A capability)
USB-A:                   For display power (1A)

Expected Runtime:
At 1100mA average:       ~18 hours continuous
At 1700mA peak:          ~12 hours continuous
With power management:   24+ hours typical use

Features Required:
- Pass-through charging (use while charging)
- LCD power indicator
- Multiple output ports
- Overcharge/discharge protection
```

### **Power Distribution**
```
Power Bank → Pi 4B (USB-C) → Display (DSI Power)
          ↓
          → Backup USB-A port available for accessories
```

---

## 🌐 **NETWORK CONFIGURATION**

### **WiFi Setup for Rover Communication**
**Network Architecture**:
```
[RTK Base Station] → [Travel Router] → [Mobile RTK Station]
                                    ↓
                              [Outdoor Rover]
```

**WiFi Configuration**:
```bash
# Configure for existing travel router network
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf

network={
    ssid="RoverNetwork"
    psk="your_password"
    priority=10
}

# Set static IP for consistent rover communication  
sudo nano /etc/dhcpcd.conf
interface wlan0
static ip_address=192.168.8.150/24
static routers=192.168.8.1
static domain_name_servers=8.8.8.8
```

### **Communication Ports**
```
UDP Position Updates:    Port 12345 (to rover)
TCP Control Messages:    Port 12346 (to rover)
RTK Corrections (in):    Port 2101 (from base station)
Status Monitoring:       Port 8080 (web interface)
```

---

## 🧪 **TESTING & VERIFICATION PROCEDURES**

### **Hardware Testing Checklist**
```
GPS Module Testing:
☐ UART communication established (/dev/serial0)
☐ NMEA messages received and parsed
☐ RTK corrections from base station received
☐ RTK fix achieved (may take 1-5 minutes)
☐ Position accuracy <10cm verified against known point

Display Testing:
☐ Touchscreen responsive and calibrated
☐ Display orientation correct for handheld use
☐ Touch sensitivity good for field use
☐ Screen readable in outdoor lighting

Status Indicators:
☐ LEDs illuminate correctly on GPIO commands
☐ Emergency button triggers GPIO interrupt
☐ Buzzer produces clear audio feedback
☐ All connections secure and strain-relieved

Power System:
☐ Power consumption measured and within expectations
☐ Battery life testing >12 hours continuous
☐ Charging while operating functional
☐ Low battery warnings and safe shutdown
```

### **Software Integration Testing**
```python
# Basic GPIO testing script
import RPi.GPIO as GPIO
import time

# Test status LEDs
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)  # GPS LED
GPIO.setup(24, GPIO.OUT)  # Comm LED  
GPIO.setup(25, GPIO.OUT)  # Emergency LED

# LED sequence test
for led in [23, 24, 25]:
    GPIO.output(led, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(led, GPIO.LOW)

# Test emergency button
GPIO.setup(7, GPIO.IN, pull_up_down=GPIO.PUD_UP)
print("Button state:", GPIO.input(7))

GPIO.cleanup()
```

---

## 🔧 **TROUBLESHOOTING GUIDE**

### **Common GPS Connection Issues**
**Issue**: No NMEA messages received
```
Check: 
- UART enabled in raspi-config
- Correct pin connections (TXO→RX, RXI→TX)
- 38400 baud rate setting
- 3.3V power supply stable

Solution:
sudo minicom -D /dev/serial0 -b 38400
# Should see continuous NMEA output
```

**Issue**: RTK corrections not received
```
Check:
- Base station operating and broadcasting
- Network connection to base station
- Firewall not blocking port 2101
- GPS module configured for RTK mode

Solution: 
# Test base station connection
telnet [base_station_ip] 2101
# Should receive RTCM3 data stream
```

### **Touchscreen Issues**
**Issue**: Touch not responsive
```
Check:
- USB connection secure
- Touch drivers installed
- Display calibration

Solution:
xinput list  # Verify touch device present
xinput-calibrator  # Recalibrate if needed
```

**Issue**: Display orientation wrong
```
Solution:
sudo nano /boot/config.txt
# Add: display_rotate=1 (90°) or display_rotate=3 (270°)
sudo reboot
```

### **Power Issues**  
**Issue**: System unstable or reboots
```
Check:
- Power supply capacity (need 2A minimum)
- USB cable quality (voltage drop)
- Overheating (adequate ventilation)

Solution:
# Monitor power supply voltage
vcgencmd measure_volts
# Should be >4.8V under load
```

---

**🎯 WIRING GUIDE STATUS: COMPLETE INTEGRATION INSTRUCTIONS**  
**Hardware Integration: Professional GPS + Touchscreen + Pi 4B**  
**Ready for: Immediate assembly and testing**

---

*Wiring Guide Created: September 13, 2025*  
*Hardware Target: Pi 4B + SparkFun GPS-RTK-SMA + 7" Touchscreen*  
*Integration: Optimized for your existing professional-grade components*