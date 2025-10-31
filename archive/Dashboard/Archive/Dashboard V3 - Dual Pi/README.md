# Dashboard V3 - Dual Pi Setup

Enhanced dashboard for managing dual-Pi rover setup with real-time telemetry monitoring.

## Features

### 🤖 Dual-Pi Management
- **Navigation Pi (192.168.1.10)**: Autonomous navigation, Bluetooth tracking, motor control
- **Visualization Pi (192.168.1.11)**: Telemetry relay, data visualization, base station communication
- Independent SSH connections to both Pi systems
- Tabbed interface for organized management

### 📡 Real-Time Telemetry
- Live telemetry streaming via Pi-to-Pi ethernet communication
- Real-time data visualization with matplotlib
- System overview with connection status monitoring
- Telemetry data logging and export

### 🎮 Enhanced Controls
- File upload targeting specific Pi systems
- Emergency stop functionality for all systems
- Dark/Light theme support
- Integrated terminals for both Pi systems

## Setup Requirements

### Hardware
- Navigation Pi: Raspberry Pi 4B (recommended)
- Visualization Pi: Raspberry Pi 4B (recommended)  
- Ethernet cable for Pi-to-Pi connection
- Standard rover hardware (Arduino, motors, sensors)

### Network Configuration

#### Direct Ethernet Connection
Connect the Pis with an ethernet cable and configure static IPs:

**Navigation Pi (/etc/dhcpcd.conf):**
```bash
interface eth0
static ip_address=192.168.1.10/24
static routers=192.168.1.1
```

**Visualization Pi (/etc/dhcpcd.conf):**
```bash
interface eth0
static ip_address=192.168.1.11/24
static routers=192.168.1.1
```

After configuration, restart networking:
```bash
sudo systemctl restart dhcpcd
```

Test connectivity:
```bash
# From Navigation Pi
ping 192.168.1.11

# From Visualization Pi  
ping 192.168.1.10
```

### Software Installation

#### Desktop (Windows/Linux/Mac)
1. Install Python 3.8+ and required packages:
```bash
pip install -r requirements.txt
```

2. Run the dual-Pi dashboard:
```bash
python dual_pi_dashboard.py
```

#### Navigation Pi
1. Copy the navigation code:
```bash
scp bluetooth_navigator_dual_pi_v7.py jay@192.168.1.10:/home/jay/rover_project/
scp pi_communication.py jay@192.168.1.10:/home/jay/rover_project/
```

2. Install dependencies:
```bash
ssh jay@192.168.1.10
cd ~/rover_project
pip install bleak
```

#### Visualization Pi
1. Copy the visualization code:
```bash
scp visualization_pi_receiver.py jay@192.168.1.11:/home/jay/rover_project/
scp pi_communication.py jay@192.168.1.11:/home/jay/rover_project/
```

2. Install dependencies:
```bash
ssh jay@192.168.1.11
cd ~/rover_project
pip install matplotlib numpy
```

## Usage

### 1. Start Visualization Pi Receiver
On the Visualization Pi:
```bash
cd ~/rover_project
python3 visualization_pi_receiver.py
```

This will:
- Start telemetry server on port 8888
- Begin logging telemetry data
- Show real-time visualization (if display connected)

### 2. Start Navigation Pi
On the Navigation Pi:
```bash
cd ~/rover_project
python3 bluetooth_navigator_dual_pi_v7.py
```

Optional: Specify Visualization Pi IP if different:
```bash
python3 bluetooth_navigator_dual_pi_v7.py 192.168.1.11
```

### 3. Use Dashboard V3
On your desktop:
```bash
python dual_pi_dashboard.py
```

1. **Connect to both Pis**: Use the Navigation Pi and Visualization Pi tabs
2. **Upload code**: Target specific Pi systems for code deployment
3. **Monitor telemetry**: Use System Overview tab for live data
4. **Emergency stop**: Use the emergency stop button to halt all systems

## File Structure

```
Dashboard V3 - Dual Pi/
├── dual_pi_dashboard.py          # Main dashboard application
├── pi_communication.py           # Pi-to-Pi communication module
├── requirements.txt               # Python dependencies
└── README.md                     # This file

Navigation V7 - Dual Pi/
├── bluetooth_navigator_dual_pi_v7.py    # Enhanced navigation with telemetry
├── visualization_pi_receiver.py          # Visualization Pi receiver
└── pi_communication.py                   # Communication module (copy)
```

## Telemetry Data Format

The telemetry system sends JSON messages containing:

```json
{
  "timestamp": 1642857600.123,
  "nav_state": "FORWARD_TRACKING",
  "ultrasonic_distance": 85.2,
  "bluetooth_distance": 2.3,
  "bluetooth_smoothed_distance": 2.1,
  "bluetooth_rssi": -65,
  "motor_speeds": {
    "left": 80,
    "right": 80
  },
  "robot_mode": 2,
  "emergency_stop": false,
  "stuck_counter": 0,
  "bluetooth_modes": {
    "tracking": true,
    "approaching": false,
    "holding": false,
    "signal_improving": true,
    "signal_degrading": false
  },
  "stats": {
    "telemetry_messages_sent": 1234,
    "bluetooth_detections": 567,
    ...
  }
}
```

## Troubleshooting

### Connection Issues
- Verify ethernet cable and IP configuration
- Check firewall settings (port 8888 must be open)
- Test with `ping` and `telnet <ip> 8888`

### Dashboard Connection Problems
- Ensure SSH credentials are correct
- Verify Pi systems are powered and accessible
- Check network connectivity from desktop to Pis

### Telemetry Issues
- Start Visualization Pi receiver before Navigation Pi
- Check telemetry logs: `/home/jay/rover_project/telemetry.log`
- Verify Pi-to-Pi network connectivity

### Performance Issues
- Reduce telemetry frequency by increasing `telemetry_send_interval`
- Limit data history with `max_telemetry_points` setting
- Use wired ethernet instead of WiFi for Pi-to-Pi communication

## Advanced Configuration

### Custom Network Setup
Edit the IP addresses in the code files:
- `dual_pi_dashboard.py`: Default Pi IP addresses
- `bluetooth_navigator_dual_pi_v7.py`: Visualization Pi IP
- `visualization_pi_receiver.py`: Listening port

### Telemetry Customization
Modify the telemetry data in `send_telemetry_data()` method to include additional sensor data or reduce message size.

### Visualization Customization
Edit `visualization_pi_receiver.py` to add custom plots, change update intervals, or modify data logging format.

## Migration from V2

### Preserving Existing Work
- V2 dashboard remains unchanged in `Dashboard V2/` directory
- V6 navigation code preserved in original location
- All existing logs and data are maintained

### Key Differences
- V3 adds dual-Pi management and telemetry
- V7 navigation adds telemetry publishing to V6 base
- Pi-to-Pi communication is new functionality
- Dashboard now has tabbed interface for multiple Pis

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review telemetry and dashboard logs
3. Verify network connectivity between all systems
4. Test individual components (Pi connections, telemetry receiver, navigation)

The dual-Pi setup provides enhanced capability and redundancy while maintaining all existing navigation functionality.