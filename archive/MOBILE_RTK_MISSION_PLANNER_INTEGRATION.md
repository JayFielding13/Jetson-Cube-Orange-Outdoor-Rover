# Mobile RTK Station - Mission Planner Integration
**MAVLink Interface for Professional Rover Control**

**Integration Goal**: Mobile RTK Station as intelligent MAVLink companion to Mission Planner  
**Key Benefit**: Emergency stop, rover control, and status monitoring through established Mission Planner infrastructure

---

## 🎯 **INTEGRATION ARCHITECTURE OVERVIEW**

### **System Communication Flow**
```
Mission Planner (Base Station) ←→ Mobile RTK Station ←→ Outdoor Rover
                ↓                                           ↓
        MAVLink Commands                              MAVLink Commands
        Status Updates                                Direct Control
        Emergency Stop                               Position Following
```

### **Mission Planner Integration Benefits**
**Centralized Control**:
- Emergency stop from mobile station propagated through Mission Planner
- Rover mode changes (Manual/Guided/Auto) controlled from mobile station
- Unified status monitoring and logging in Mission Planner

**Professional Operation**:
- Mission Planner's established safety systems and protocols  
- Integration with existing parameter management and configuration
- Compatibility with Mission Planner's flight planning and monitoring tools

**Enhanced Capabilities**:
- Mobile station appears as "ground control station" in Mission Planner
- Real-time position data integrated into Mission Planner mapping
- Follow-me missions can be created and modified in Mission Planner

---

## 🔧 **MAVLINK INTEGRATION DESIGN**

### **Mobile RTK Station as MAVLink Node**
**System Identity**:
```python
# MAVLink system configuration
MOBILE_STATION_SYSTEM_ID = 200  # Unique system ID
MOBILE_STATION_COMPONENT_ID = 1  # Ground Control Station component
TARGET_ROVER_SYSTEM_ID = 1      # Your outdoor rover's system ID
MISSION_PLANNER_SYSTEM_ID = 255  # Mission Planner GCS system ID
```

**MAVLink Message Types Used**:
```python
# Messages FROM mobile station TO Mission Planner/Rover
HEARTBEAT                    # System alive and mode status
GLOBAL_POSITION_INT         # Mobile station GPS position
RC_CHANNELS_OVERRIDE        # Direct rover control (emergency stop)
COMMAND_LONG               # Commands (arm/disarm, mode change, emergency)
SET_MODE                   # Change rover flight mode
STATUSTEXT                 # Status messages and alerts

# Messages FROM Mission Planner/Rover TO mobile station  
HEARTBEAT                  # Rover and Mission Planner status
GLOBAL_POSITION_INT        # Rover GPS position
SYS_STATUS                 # System health and battery status
COMMAND_ACK                # Command acknowledgments
MISSION_ITEM_INT          # Waypoint/mission data
```

### **Communication Architecture**
```python
# Network topology
Mobile RTK Station (192.168.8.150)
├── MAVLink TCP → Mission Planner (192.168.8.100:14550)
├── MAVLink UDP → Outdoor Rover (192.168.8.70:14551)
└── RTK Corrections ← Base Station (192.168.8.100:2101)

# Message routing
Mission Planner ←→ Mobile Station ←→ Outdoor Rover
     ↓                    ↓                ↓
- Monitoring          - Relay            - Direct Control
- Logging            - Position          - Emergency Stop
- Commands           - Emergency         - Follow Commands
```

---

## 🚨 **EMERGENCY STOP INTEGRATION**

### **Multi-Level Emergency Stop System**
**Level 1: Mobile Station Hardware Button**
```python
# Immediate hardware response on mobile station
def hardware_emergency_button_pressed():
    # 1. Immediate local indicator (LED, buzzer)
    activate_emergency_indicators()
    
    # 2. Send MAVLink emergency command to rover (fastest path)
    send_emergency_stop_to_rover()
    
    # 3. Notify Mission Planner of emergency state
    send_emergency_status_to_mission_planner()
    
    # 4. Log emergency event with timestamp and GPS position
    log_emergency_event("HARDWARE_BUTTON", get_current_position())
```

**Level 2: Mission Planner Emergency Commands**
```python
# Route Mission Planner emergency commands to rover
def handle_mission_planner_emergency(mavlink_msg):
    if mavlink_msg.command == MAV_CMD_COMPONENT_ARM_DISARM:
        # Disarm command from Mission Planner
        relay_disarm_to_rover()
        update_mobile_station_status("ROVER_DISARMED")
        
    elif mavlink_msg.command == MAV_CMD_DO_FLIGHTTERMINATION:
        # Flight termination from Mission Planner
        send_immediate_stop_to_rover()
        update_mobile_station_status("EMERGENCY_STOP")
```

**Level 3: Communication Loss Failsafe**
```python
# Handle lost communication scenarios  
def monitor_communication_health():
    if time_since_rover_heartbeat() > 3.0:  # 3 second timeout
        # Lost communication with rover
        send_emergency_status_to_mission_planner("ROVER_COMM_LOST")
        # Rover should have its own failsafe behavior
        
    if time_since_mission_planner_heartbeat() > 10.0:  # 10 second timeout
        # Lost communication with base station
        # Continue local operation but with reduced capability
        set_autonomous_emergency_mode()
```

---

## 📡 **MAVLINK MESSAGE IMPLEMENTATION**

### **Core MAVLink Interface Class**
```python
from pymavlink import mavutil
import time
import threading

class MobileStationMAVLink:
    def __init__(self, mission_planner_connection, rover_connection):
        # Connection to Mission Planner
        self.mp_conn = mavutil.mavlink_connection(
            f'tcp:{mission_planner_connection}',
            source_system=200,  # Mobile station system ID
            source_component=1   # Ground control component
        )
        
        # Connection to rover
        self.rover_conn = mavutil.mavlink_connection(
            f'udp:{rover_connection}',
            source_system=200,
            source_component=1
        )
        
        self.running = True
        self.last_position = None
        self.rover_status = {}
        
    def start_communication(self):
        # Start message handling threads
        threading.Thread(target=self.mp_message_handler, daemon=True).start()
        threading.Thread(target=self.rover_message_handler, daemon=True).start()
        threading.Thread(target=self.send_heartbeat, daemon=True).start()
        
    def send_heartbeat(self):
        """Send periodic heartbeat to identify mobile station"""
        while self.running:
            # Send to Mission Planner
            self.mp_conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,           # Ground Control Station
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,  # No autopilot
                0, 0, 0  # Base mode, custom mode, system status
            )
            
            # Send to rover  
            self.rover_conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            
            time.sleep(1)  # 1Hz heartbeat
            
    def send_position_update(self, lat, lon, alt, accuracy):
        """Send mobile station position to Mission Planner and rover"""
        # Convert to MAVLink format
        lat_int = int(lat * 1e7)
        lon_int = int(lon * 1e7)
        alt_int = int(alt * 1000)  # mm
        
        # Send to Mission Planner for display
        self.mp_conn.mav.global_position_int_send(
            int(time.time() * 1000),  # timestamp
            lat_int, lon_int, alt_int, alt_int,  # position
            0, 0, 0,  # velocity (vx, vy, vz)
            0         # heading
        )
        
        # Send to rover for following behavior
        self.rover_conn.mav.global_position_int_send(
            int(time.time() * 1000),
            lat_int, lon_int, alt_int, alt_int,
            0, 0, 0, 0
        )
        
    def send_emergency_stop(self):
        """Send immediate emergency stop to rover"""
        # Method 1: RC override (immediate)
        self.rover_conn.mav.rc_channels_override_send(
            1,    # target_system (rover)
            1,    # target_component  
            1500, # channel 1 (steering) - center
            1000, # channel 3 (throttle) - minimum/stop
            65535, 65535, 65535, 65535, 65535, 65535  # unused channels
        )
        
        # Method 2: Command long (arm/disarm)
        self.rover_conn.mav.command_long_send(
            1, 1,  # target system, component
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,     # confirmation
            0,     # param1: 0=disarm, 1=arm
            21196, # param2: force disarm magic number
            0, 0, 0, 0, 0  # unused params
        )
        
        # Method 3: Mode change to HOLD
        self.rover_conn.mav.set_mode_send(
            1,  # target_system
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4   # HOLD mode for ArduRover
        )
        
    def send_follow_command(self, distance=3.0, bearing=180.0):
        """Send follow-me waypoint to rover through Mission Planner"""
        if self.last_position:
            # Calculate follow position
            follow_lat, follow_lon = calculate_offset_position(
                self.last_position['lat'],
                self.last_position['lon'], 
                bearing, distance
            )
            
            # Send as guided mode waypoint
            self.rover_conn.mav.mission_item_int_send(
                1, 1,  # target system, component
                0,     # sequence number
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                2,     # current (2 = guided mode waypoint)
                1,     # autocontinue
                0, 0, 0, 0,  # params 1-4
                int(follow_lat * 1e7),  # x (latitude)
                int(follow_lon * 1e7),  # y (longitude)  
                0      # z (altitude)
            )
```

### **Mission Planner Display Integration**
```python
def send_status_text(self, message, severity=mavutil.mavlink.MAV_SEVERITY_INFO):
    """Send status messages to Mission Planner HUD"""
    self.mp_conn.mav.statustext_send(
        severity,
        message.encode('utf-8')[:50]  # 50 character limit
    )
    
def send_gps_status_to_mission_planner(self):
    """Update Mission Planner with mobile station GPS status"""
    gps_status = self.gps_handler.get_status()
    
    if gps_status['fix_type'] == 'RTK_FIXED':
        self.send_status_text("Mobile RTK: RTK FIXED", 
                            mavutil.mavlink.MAV_SEVERITY_INFO)
    elif gps_status['fix_type'] == 'RTK_FLOAT':
        self.send_status_text("Mobile RTK: RTK FLOAT",
                            mavutil.mavlink.MAV_SEVERITY_WARNING)
    else:
        self.send_status_text("Mobile RTK: NO RTK FIX",
                            mavutil.mavlink.MAV_SEVERITY_ERROR)

def update_mission_planner_with_follow_status(self, rover_position):
    """Send following status to Mission Planner"""
    if self.last_position and rover_position:
        distance = calculate_distance(self.last_position, rover_position)
        bearing = calculate_bearing(rover_position, self.last_position)
        
        status_msg = f"Follow: {distance:.1f}m @ {bearing:.0f}°"
        self.send_status_text(status_msg, mavutil.mavlink.MAV_SEVERITY_INFO)
```

---

## 🎮 **MISSION PLANNER INTERFACE ENHANCEMENTS**

### **Custom Mission Planner Setup**
**MAVLink Connection Configuration**:
```
Mission Planner → Setup → Optional Hardware → MAVLink
├── Add Connection: TCP 192.168.8.150:14550 (Mobile RTK Station)
├── System ID Filter: Show system 200 (Mobile station)
└── Message Rate: GLOBAL_POSITION_INT at 5Hz
```

**HUD Display Integration**:
```python
# Mobile station will appear in Mission Planner as:
# - "Mobile RTK" in connected systems list
# - Real-time position on map with different icon/color
# - Status messages in HUD text area
# - Emergency stop commands available in Action tab
```

### **Mission Planner Action Tab Integration**
**Custom Actions Available**:
```
Mission Planner → Actions Tab (when mobile station connected)
├── Emergency Stop Mobile Following
├── Start Follow Mode (distance configurable)  
├── Stop Follow Mode
├── Request Mobile Station Status
└── View Mobile Station GPS Quality
```

### **Mission Planner Map Display**
**Visual Elements**:
```python
# Mobile station position shown as:
# - Green circle: RTK fixed, good accuracy
# - Yellow circle: RTK float or GPS 3D fix
# - Red circle: GPS issues or no fix
# - Line connecting mobile station to rover (when following)
# - Follow distance circle around mobile station
```

---

## 📱 **ENHANCED MOBILE STATION GUI**

### **Mission Planner Integration Screen**
```python
# Additional GUI screen for Mission Planner integration
class MissionPlannerScreen:
    def create_mp_interface(self):
        # Connection status
        self.mp_connection_status = tk.Label(text="MP: Connected")
        
        # Rover control through Mission Planner
        self.mode_buttons = [
            tk.Button(text="MANUAL", command=self.set_rover_manual),
            tk.Button(text="GUIDED", command=self.set_rover_guided),
            tk.Button(text="AUTO", command=self.set_rover_auto),
            tk.Button(text="HOLD", command=self.set_rover_hold)
        ]
        
        # Emergency controls
        self.emergency_frame = tk.Frame(bg='red')
        self.emergency_stop_btn = tk.Button(
            self.emergency_frame,
            text="EMERGENCY STOP\n(Mission Planner)",
            command=self.emergency_stop_via_mp,
            bg='red', fg='white', font=('Arial', 16, 'bold')
        )
        
        # Status display
        self.rover_status_display = tk.Text(height=10)
        
    def emergency_stop_via_mp(self):
        """Emergency stop with Mission Planner integration"""
        # Local immediate action
        self.activate_local_emergency_indicators()
        
        # Send through MAVLink to both rover and Mission Planner
        self.mavlink.send_emergency_stop()
        
        # Update GUI
        self.update_emergency_status("EMERGENCY STOP ACTIVATED")
        
        # Log event
        self.log_emergency("EMERGENCY_STOP_VIA_MP", self.get_position())
```

### **Real-Time Status Integration**
```python
class IntegratedStatusDisplay:
    def update_status(self):
        """Update display with Mission Planner integration status"""
        status = {
            'mobile_gps': self.gps_handler.get_status(),
            'rover_connection': self.mavlink.rover_status,
            'mp_connection': self.mavlink.mp_status,
            'rover_mode': self.mavlink.rover_mode,
            'following_active': self.following_status
        }
        
        # Update touchscreen display
        self.display_integrated_status(status)
        
        # Send status to Mission Planner
        self.mavlink.send_system_status(status)
```

---

## 🔧 **IMPLEMENTATION INTEGRATION**

### **Enhanced Software Architecture**
```python
mobile_rtk_station/
├── src/
│   ├── mavlink/
│   │   ├── mavlink_interface.py    # Core MAVLink communication
│   │   ├── mission_planner.py      # Mission Planner specific integration
│   │   ├── rover_control.py        # Rover control via MAVLink
│   │   └── emergency_system.py     # Integrated emergency stop
│   ├── integration/
│   │   ├── mp_status_monitor.py    # Mission Planner status monitoring
│   │   ├── rover_following.py      # Following behavior via MAVLink
│   │   └── command_relay.py        # Command routing between systems
└── gui/
    ├── screens/
    │   ├── mission_planner_screen.py  # MP integration interface
    │   └── integrated_control.py      # Unified rover control
```

### **Configuration Integration**
```json
// config/mission_planner_integration.json
{
    "mission_planner": {
        "ip_address": "192.168.8.100",
        "tcp_port": 14550,
        "system_id": 255,
        "component_id": 190
    },
    "mobile_station": {
        "system_id": 200,
        "component_id": 1,
        "heartbeat_rate_hz": 1,
        "position_rate_hz": 5
    },
    "rover_control": {
        "system_id": 1,
        "component_id": 1,
        "emergency_stop_timeout_ms": 100,
        "follow_mode_default_distance": 3.0
    },
    "safety": {
        "communication_timeout_s": 3.0,
        "emergency_stop_channels": [1, 3],
        "failsafe_mode": "HOLD"
    }
}
```

---

## 🚀 **IMPLEMENTATION TIMELINE**

### **Week 1: Basic MAVLink Integration**
**Day 1-2: MAVLink Setup**
- [ ] Install pymavlink and test basic communication
- [ ] Establish TCP connection to Mission Planner
- [ ] Implement heartbeat and basic status messages

**Day 3-4: Emergency Stop Integration**
- [ ] Implement hardware emergency button → MAVLink command
- [ ] Test emergency stop propagation to rover via Mission Planner
- [ ] Add Mission Planner status text integration

**Day 5-7: Position Integration**  
- [ ] Send mobile station position to Mission Planner for display
- [ ] Integrate follow-me commands via MAVLink waypoints
- [ ] Test rover following behavior with Mission Planner monitoring

### **Week 2: Advanced Integration**
- [ ] Enhanced Mission Planner GUI integration
- [ ] Rover mode control from mobile station
- [ ] Complete status monitoring and logging
- [ ] Field testing with full Mission Planner integration

---

## 📊 **INTEGRATION BENEFITS**

### **Operational Advantages**
1. **Centralized Control** - All rover operations managed through familiar Mission Planner interface
2. **Professional Logging** - Mission Planner's comprehensive data logging for analysis
3. **Safety Integration** - Emergency stop works through established Mission Planner safety systems
4. **Status Monitoring** - Real-time rover and mobile station status in Mission Planner HUD
5. **Mission Planning** - Create follow-me missions using Mission Planner's planning tools

### **Development Benefits**
1. **Proven Protocol** - MAVLink is field-tested and reliable
2. **Extensive Documentation** - Well-documented message formats and procedures  
3. **Tool Integration** - Compatible with Mission Planner's parameter management and configuration
4. **Future Expansion** - Easy to add more features using existing MAVLink infrastructure

### **Field Operation Benefits**
1. **Familiar Interface** - Operators already know Mission Planner
2. **Unified System** - Single interface for rover + mobile station management
3. **Enhanced Safety** - Multiple emergency stop paths and failsafe behaviors
4. **Professional Appearance** - Integration with commercial-grade GCS software

---

**🎯 MISSION PLANNER INTEGRATION STATUS: COMPREHENSIVE MAVLINK INTERFACE DESIGNED**  
**Key Benefit: Professional rover control through established Mission Planner infrastructure**  
**Emergency Stop: Multi-level safety system with Mission Planner integration**

---

*Integration Design Created: September 13, 2025*  
*Protocol: MAVLink for Mission Planner + rover communication*  
*Safety: Hardware button → MAVLink → Mission Planner → rover emergency stop*