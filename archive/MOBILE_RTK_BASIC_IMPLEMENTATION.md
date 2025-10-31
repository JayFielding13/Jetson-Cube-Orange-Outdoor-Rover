# Mobile RTK Station - Basic Implementation Guide
**Core Functionality: Accurate Position Data for Rover Following**

**Goal**: Get your Pi 4B + SparkFun GPS-RTK-SMA providing RTK-accurate position data to your outdoor rover  
**Timeline**: 2-3 days for basic working system  
**Focus**: Minimal viable product for rover following

---

## 🎯 **STEP-BY-STEP IMPLEMENTATION**

### **Step 1: Hardware Connection & GPS Setup**

#### **Physical Wiring** (15 minutes)
```bash
# Your SparkFun GPS-RTK-SMA to Pi 4B connections:
GPS Module        Pi 4B GPIO
──────────────    ──────────────
VCC (3.3V)    →   Pin 1  (3.3V Power)
GND           →   Pin 6  (Ground)
TXO           →   Pin 10 (GPIO 15 - UART RX)
RXI           →   Pin 8  (GPIO 14 - UART TX)

# Connect your multi-band antenna to GPS SMA connector
# Power: Use your existing power bank or desktop power supply
```

#### **Pi 4B UART Configuration** (5 minutes)
```bash
# Enable UART for GPS communication
sudo raspi-config
# → Interface Options → Serial Port
# → "Login shell over serial?" → No
# → "Serial port hardware enabled?" → Yes
# → Reboot when prompted

# Verify UART is available
ls -l /dev/serial*
# Should show: /dev/serial0 -> ttyAMA0
```

#### **Test Basic GPS Communication** (10 minutes)
```bash
# Install communication tools
sudo apt update
sudo apt install minicom

# Test raw GPS data
sudo minicom -D /dev/serial0 -b 38400

# You should see continuous NMEA messages like:
# $GNGGA,123456.00,4012.12345,N,10512.12345,W,1,12,0.8,1234.5,M,0.0,M,,*hh
# $GNRMC,123456.00,A,4012.12345,N,10512.12345,W,1.234,45.67,130925,,,A*hh

# Press Ctrl+A then X to exit minicom
```

---

### **Step 2: Python GPS Interface** (30 minutes)

#### **Install Required Libraries**
```bash
# Install GPS parsing libraries
pip3 install pyserial pynmea2 pyubx2
```

#### **Create Basic GPS Handler**
```python
# Save as: ~/mobile_rtk_station/gps_basic.py
import serial
import pynmea2
import time
from datetime import datetime

class BasicGPSHandler:
    def __init__(self, port='/dev/serial0', baudrate=38400):
        self.port = serial.Serial(port, baudrate, timeout=1)
        self.current_position = None
        self.fix_quality = 0
        self.satellites = 0
        
    def read_gps_data(self):
        """Read and parse GPS data from UART"""
        try:
            line = self.port.readline().decode('ascii', errors='ignore').strip()
            
            if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                # Parse GGA message (position and fix quality)
                msg = pynmea2.parse(line)
                
                if msg.latitude and msg.longitude:
                    self.current_position = {
                        'latitude': float(msg.latitude),
                        'longitude': float(msg.longitude),
                        'altitude': float(msg.altitude) if msg.altitude else 0.0,
                        'fix_quality': int(msg.gps_qual),
                        'satellites': int(msg.num_sats) if msg.num_sats else 0,
                        'hdop': float(msg.horizontal_dil) if msg.horizontal_dil else 0.0,
                        'timestamp': datetime.now().isoformat()
                    }
                    self.fix_quality = int(msg.gps_qual)
                    self.satellites = int(msg.num_sats) if msg.num_sats else 0
                    
                    return self.current_position
                    
        except Exception as e:
            print(f"GPS parsing error: {e}")
            
        return None
        
    def get_fix_type_string(self):
        """Convert fix quality number to readable string"""
        fix_types = {
            0: "NO_FIX",
            1: "GPS_FIX", 
            2: "DGPS_FIX",
            4: "RTK_FIXED",
            5: "RTK_FLOAT"
        }
        return fix_types.get(self.fix_quality, "UNKNOWN")
        
    def is_rtk_ready(self):
        """Check if we have RTK-level accuracy"""
        return self.fix_quality in [4, 5]  # RTK Fixed or Float
        
    def get_status_summary(self):
        """Get human-readable status"""
        if self.current_position:
            return {
                'fix_type': self.get_fix_type_string(),
                'satellites': self.satellites,
                'position': f"{self.current_position['latitude']:.6f}, {self.current_position['longitude']:.6f}",
                'accuracy_estimate': self.estimate_accuracy()
            }
        return {'fix_type': 'NO_DATA', 'satellites': 0}
        
    def estimate_accuracy(self):
        """Estimate position accuracy based on fix type"""
        if self.fix_quality == 4:  # RTK Fixed
            return "±2cm"
        elif self.fix_quality == 5:  # RTK Float  
            return "±10cm"
        elif self.fix_quality == 2:  # DGPS
            return "±1m"
        elif self.fix_quality == 1:  # GPS
            return "±3m"
        else:
            return "Unknown"

# Test the GPS handler
if __name__ == "__main__":
    gps = BasicGPSHandler()
    print("Starting GPS test... Press Ctrl+C to stop")
    
    try:
        while True:
            position = gps.read_gps_data()
            if position:
                status = gps.get_status_summary()
                print(f"Fix: {status['fix_type']} | "
                      f"Sats: {status['satellites']} | "
                      f"Pos: {status['position']} | "
                      f"Acc: {status['accuracy_estimate']}")
            time.sleep(0.2)  # 5Hz update rate
            
    except KeyboardInterrupt:
        print("\nGPS test stopped")
        gps.port.close()
```

#### **Test Basic GPS Functionality**
```bash
# Run the GPS test
cd ~/mobile_rtk_station
python3 gps_basic.py

# Expected output:
# Fix: GPS_FIX | Sats: 12 | Pos: 40.123456, -105.987654 | Acc: ±3m
# Fix: GPS_FIX | Sats: 13 | Pos: 40.123457, -105.987655 | Acc: ±3m
# (Should update every 200ms)
```

---

### **Step 3: RTK Base Station Integration** (45 minutes)

#### **Configure GPS for RTK Corrections**
The ZED-F9P needs to be configured to receive RTCM corrections from your base station. We'll use UBX configuration messages:

```python
# Save as: ~/mobile_rtk_station/rtk_config.py
import serial
from pyubx2 import UBXMessage, POLL, SET
import time

class RTKConfigurator:
    def __init__(self, port='/dev/serial0', baudrate=38400):
        self.port = serial.Serial(port, baudrate, timeout=2)
        
    def configure_for_rtk_rover(self):
        """Configure ZED-F9P as RTK rover"""
        print("Configuring ZED-F9P for RTK rover mode...")
        
        # Enable high precision mode
        msg = UBXMessage('CFG', 'CFG-VALSET', SET, 
                        cfgData=[
                            (0x10110011, 1),  # CFG-NAVHPG-DGNSSMODE: RTK
                        ])
        self.send_ubx_message(msg)
        time.sleep(0.1)
        
        # Set dynamic model to pedestrian (for walking)
        msg = UBXMessage('CFG', 'CFG-VALSET', SET,
                        cfgData=[
                            (0x20110021, 3),  # CFG-NAVSPG-DYNMODEL: Pedestrian
                        ])
        self.send_ubx_message(msg)
        time.sleep(0.1)
        
        print("RTK rover configuration complete")
        
    def send_ubx_message(self, msg):
        """Send UBX message to GPS"""
        self.port.write(msg.serialize())
        
    def close(self):
        self.port.close()

# Configuration helper
if __name__ == "__main__":
    config = RTKConfigurator()
    config.configure_for_rtk_rover()
    config.close()
    print("GPS configured for RTK rover mode")
```

#### **RTK Correction Reception Setup**
Your GPS needs to receive RTCM corrections from your base station. We'll set up a simple correction forwarder:

```python
# Save as: ~/mobile_rtk_station/rtk_corrections.py
import socket
import threading
import serial
import time

class RTKCorrectionHandler:
    def __init__(self, base_station_ip='192.168.8.100', base_station_port=2101, 
                 gps_port='/dev/serial0'):
        self.base_station_ip = base_station_ip
        self.base_station_port = base_station_port
        self.gps_serial = serial.Serial(gps_port, 38400, timeout=1)
        self.correction_socket = None
        self.running = False
        
    def start_corrections(self):
        """Start receiving and forwarding RTCM corrections"""
        self.running = True
        correction_thread = threading.Thread(target=self.correction_forwarder)
        correction_thread.daemon = True
        correction_thread.start()
        print(f"Started RTCM correction reception from {self.base_station_ip}:{self.base_station_port}")
        
    def correction_forwarder(self):
        """Receive RTCM data from base station and forward to GPS"""
        try:
            # Connect to base station
            self.correction_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.correction_socket.connect((self.base_station_ip, self.base_station_port))
            print("Connected to RTK base station")
            
            while self.running:
                # Receive RTCM data from base station
                data = self.correction_socket.recv(1024)
                if data:
                    # Forward directly to GPS module
                    self.gps_serial.write(data)
                    # Optional: print correction data rate
                    if len(data) > 0:
                        print(f"Forwarded {len(data)} bytes of RTCM corrections")
                        
        except Exception as e:
            print(f"RTCM correction error: {e}")
            
    def stop_corrections(self):
        """Stop correction reception"""
        self.running = False
        if self.correction_socket:
            self.correction_socket.close()
        print("Stopped RTCM corrections")

# Test correction reception
if __name__ == "__main__":
    # Update this IP to your base station's IP
    rtk = RTKCorrectionHandler('192.168.8.100')  # Your base station IP
    rtk.start_corrections()
    
    try:
        print("Receiving RTCM corrections... Press Ctrl+C to stop")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        rtk.stop_corrections()
        print("Correction test stopped")
```

---

### **Step 4: Basic Position Broadcasting** (30 minutes)

#### **Simple UDP Position Broadcaster**
```python
# Save as: ~/mobile_rtk_station/position_broadcaster.py
import socket
import json
import time
import threading
from gps_basic import BasicGPSHandler
from rtk_corrections import RTKCorrectionHandler

class PositionBroadcaster:
    def __init__(self, rover_ip='192.168.8.70', rover_port=12345, 
                 base_station_ip='192.168.8.100'):
        self.rover_ip = rover_ip
        self.rover_port = rover_port
        
        # Initialize GPS and RTK corrections
        self.gps = BasicGPSHandler()
        self.rtk = RTKCorrectionHandler(base_station_ip)
        
        # UDP socket for sending to rover
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.running = False
        
    def start(self):
        """Start the position broadcasting system"""
        print("Starting Mobile RTK Station...")
        
        # Start RTK corrections
        self.rtk.start_corrections()
        
        # Wait for RTK corrections to stabilize
        print("Waiting for RTK corrections to stabilize...")
        time.sleep(5)
        
        # Start position broadcasting
        self.running = True
        broadcast_thread = threading.Thread(target=self.broadcast_loop)
        broadcast_thread.daemon = True
        broadcast_thread.start()
        
        print(f"Broadcasting position to rover at {self.rover_ip}:{self.rover_port}")
        
    def broadcast_loop(self):
        """Main broadcasting loop - sends position at 5Hz"""
        while self.running:
            try:
                # Get current GPS position
                position = self.gps.read_gps_data()
                
                if position and self.gps.is_rtk_ready():
                    # Create position message for rover
                    message = {
                        'message_type': 'mobile_position',
                        'timestamp': time.time(),
                        'mobile_station_id': 'mobile_rtk_001',
                        'position': {
                            'latitude': position['latitude'],
                            'longitude': position['longitude'],
                            'altitude': position['altitude'],
                            'accuracy': position.get('hdop', 0.0)
                        },
                        'gps_status': {
                            'fix_type': self.gps.get_fix_type_string(),
                            'satellites': position['satellites'],
                            'rtk_ready': True
                        }
                    }
                    
                    # Send to rover via UDP
                    message_json = json.dumps(message)
                    self.udp_socket.sendto(
                        message_json.encode('utf-8'),
                        (self.rover_ip, self.rover_port)
                    )
                    
                    # Print status
                    status = self.gps.get_status_summary()
                    print(f"Sent: {status['fix_type']} | "
                          f"{status['satellites']} sats | "
                          f"{status['position']} | "
                          f"{status['accuracy_estimate']}")
                          
                elif position:
                    # Have GPS but not RTK - still send position but mark as low accuracy
                    print(f"GPS only (no RTK): {self.gps.get_fix_type_string()} | "
                          f"{self.gps.satellites} sats - waiting for RTK...")
                else:
                    print("No GPS fix - waiting for satellite lock...")
                    
                time.sleep(0.2)  # 5Hz update rate
                
            except Exception as e:
                print(f"Broadcasting error: {e}")
                time.sleep(1)
                
    def stop(self):
        """Stop the broadcasting system"""
        print("Stopping Mobile RTK Station...")
        self.running = False
        self.rtk.stop_corrections()
        self.udp_socket.close()
        self.gps.port.close()

# Main application
if __name__ == "__main__":
    # Configure for your network
    broadcaster = PositionBroadcaster(
        rover_ip='192.168.8.70',      # Your outdoor rover IP
        base_station_ip='192.168.8.100'  # Your base station IP  
    )
    
    try:
        broadcaster.start()
        print("Mobile RTK Station running... Press Ctrl+C to stop")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        broadcaster.stop()
        print("Mobile RTK Station stopped")
```

---

### **Step 5: Basic Rover Integration** (30 minutes)

#### **Rover-Side Position Receiver**
Create this on your outdoor rover (NavigationPi):

```python
# Save as: ~/rover_code/mobile_station_receiver.py (on outdoor rover)
import socket
import json
import threading
import time

class MobileStationReceiver:
    def __init__(self, listen_port=12345):
        self.listen_port = listen_port
        self.mobile_position = None
        self.last_update = 0
        self.running = False
        
        # UDP socket for receiving position updates
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(('0.0.0.0', listen_port))
        
    def start_receiving(self):
        """Start receiving mobile station position updates"""
        self.running = True
        receive_thread = threading.Thread(target=self.receive_loop)
        receive_thread.daemon = True
        receive_thread.start()
        print(f"Listening for mobile station position on port {self.listen_port}")
        
    def receive_loop(self):
        """Main receiving loop"""
        while self.running:
            try:
                # Receive position update
                data, addr = self.udp_socket.recvfrom(1024)
                message = json.loads(data.decode('utf-8'))
                
                if message.get('message_type') == 'mobile_position':
                    self.mobile_position = message
                    self.last_update = time.time()
                    
                    pos = message['position']
                    gps = message['gps_status']
                    
                    print(f"Mobile station: {pos['latitude']:.6f}, {pos['longitude']:.6f} | "
                          f"{gps['fix_type']} | {gps['satellites']} sats")
                          
            except Exception as e:
                print(f"Receive error: {e}")
                
    def get_mobile_position(self):
        """Get latest mobile station position"""
        if self.mobile_position and (time.time() - self.last_update) < 2.0:
            return self.mobile_position
        return None
        
    def is_mobile_position_fresh(self):
        """Check if we have recent position data"""
        return (time.time() - self.last_update) < 2.0 if self.mobile_position else False
        
    def stop(self):
        """Stop receiving"""
        self.running = False
        self.udp_socket.close()

# Test the receiver
if __name__ == "__main__":
    receiver = MobileStationReceiver()
    
    try:
        receiver.start_receiving()
        print("Mobile station receiver running... Press Ctrl+C to stop")
        
        while True:
            if receiver.is_mobile_position_fresh():
                pos = receiver.get_mobile_position()
                if pos:
                    print(f"Fresh position available: "
                          f"{pos['gps_status']['fix_type']} accuracy")
            else:
                print("No fresh mobile station position")
            time.sleep(2)
            
    except KeyboardInterrupt:
        receiver.stop()
        print("Receiver stopped")
```

---

### **Step 6: Test the Complete System** (30 minutes)

#### **System Test Procedure**
1. **Mobile Station Test**:
```bash
# On Pi 4B (mobile station)
cd ~/mobile_rtk_station
python3 position_broadcaster.py

# Expected output:
# Starting Mobile RTK Station...
# Started RTCM correction reception from 192.168.8.100:2101
# Connected to RTK base station
# Broadcasting position to rover at 192.168.8.70:12345
# Sent: RTK_FIXED | 18 sats | 40.123456, -105.987654 | ±2cm
```

2. **Rover Receiver Test**:
```bash
# On outdoor rover (NavigationPi)
cd ~/rover_code  
python3 mobile_station_receiver.py

# Expected output:
# Listening for mobile station position on port 12345
# Mobile station: 40.123456, -105.987654 | RTK_FIXED | 18 sats
```

3. **Network Connectivity Test**:
```bash
# Test UDP connectivity from mobile station to rover
# On mobile station:
echo "test message" | nc -u 192.168.8.70 12345

# On rover, should receive the test message
nc -u -l 12345
```

---

## 🎯 **SUCCESS CRITERIA**

### **Basic System Working When:**
- [ ] Mobile station achieves RTK fix (RTK_FIXED or RTK_FLOAT)
- [ ] Position updates sent to rover at 5Hz
- [ ] Rover receives and displays mobile station position
- [ ] Position accuracy ±10cm or better with RTK
- [ ] Communication reliable over local network

### **Expected Performance:**
```
GPS Performance:
- Time to RTK fix: 30-120 seconds in open sky
- Position accuracy: ±2cm (RTK Fixed) or ±10cm (RTK Float)
- Update rate: 5Hz sustained

Communication:
- UDP packet delivery: >95% success rate
- Latency: <50ms on local network
- Range: 100-200m with travel router
```

---

## 🔧 **TROUBLESHOOTING GUIDE**

### **GPS Issues**
**Problem**: No GPS messages in minicom
```bash
# Check UART configuration
sudo raspi-config  # Ensure serial enabled
ls -l /dev/serial*  # Should show serial0

# Check wiring
# Verify 3.3V power, ground, TXO→Pin10, RXI→Pin8
```

**Problem**: GPS fix but no RTK
```bash
# Test base station connection
telnet 192.168.8.100 2101  # Should receive RTCM data

# Check correction forwarding
python3 rtk_corrections.py  # Should show correction bytes
```

### **Communication Issues**
**Problem**: Rover not receiving position
```bash
# Check rover IP
ping 192.168.8.70  # From mobile station

# Check firewall
sudo ufw status  # Should allow port 12345
```

### **Network Issues**
**Problem**: Can't reach base station
```bash
# Check network configuration
ip route  # Should show route to base station
ping 192.168.8.100  # Base station reachable
```

---

## 📋 **NEXT STEPS AFTER BASIC SYSTEM WORKS**

### **Immediate Enhancements** (Week 2)
1. **Add touchscreen GUI** - Visual status display and controls
2. **Implement basic following** - Rover maintains distance to mobile station
3. **Add emergency stop** - Hardware button for immediate rover stop
4. **Battery monitoring** - Power management and low battery warnings

### **Advanced Features** (Week 3+)
1. **Mission Planner integration** - MAVLink communication
2. **Following patterns** - Different following behaviors (offset, formation)
3. **Data logging** - Performance tracking and analysis
4. **Weatherproof enclosure** - Physical protection for field use

---

**🎯 BASIC IMPLEMENTATION STATUS: READY FOR DEVELOPMENT**  
**Focus: RTK position accuracy + reliable rover communication**  
**Timeline: 2-3 days for working position broadcasting system**

---

*Basic Implementation Guide Created: September 13, 2025*  
*Hardware: Pi 4B + SparkFun GPS-RTK-SMA + Multi-band antenna*  
*Goal: Centimeter-accurate position data for rover following*