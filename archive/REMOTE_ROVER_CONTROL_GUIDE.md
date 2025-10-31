# Remote Rover Control Setup Guide

## Overview
This guide enables your friend to securely control the rover remotely through a WireGuard VPN connection.

## Network Architecture
```
[Friend's Computer] → [Internet] → [WireGuard VPN] → [Base Station] → [Mobile Beacon] → [Rover]
     QGroundControl                                     RTK System      SiK Radio     Flight Controller
```

## Setup Steps

### 1. Install WireGuard on Base Station
Run this command on your base station computer:
```bash
sudo bash ~/wireguard_setup.sh
```

### 2. Configure Network Access
After running the setup script:

#### Find Your Public IP
```bash
curl ifconfig.me
```

#### Update Client Configuration
Edit `/home/jay/friend_client.conf` and replace `YOUR_PUBLIC_IP` with the actual public IP.

#### Router Port Forwarding
Forward UDP port 51820 from your router to this computer's local IP.

### 3. Send Configuration to Friend
Send the file `friend_client.conf` to your friend securely (encrypted email, Signal, etc.).

## Friend's Setup Instructions

### Install WireGuard Client
**Windows**: Download from https://www.wireguard.com/install/
**macOS**: `brew install wireguard-tools` or download from website
**Linux**: `sudo apt install wireguard` (Ubuntu/Debian)

### Import Configuration
1. Open WireGuard client
2. Import the `friend_client.conf` file
3. Activate the tunnel
4. Verify connection: `ping 10.8.0.1`

### Install QGroundControl
Download QGroundControl from: https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

### Connect to Rover System
In QGroundControl, add TCP connections:

#### Mobile Beacon Connection
- **Type**: TCP
- **Host**: 10.8.0.1 (base station via VPN)
- **Port**: 5760
- **Description**: "Mobile RTK Beacon"

#### Base Station Connection (Optional)
- **Type**: TCP
- **Host**: 10.8.0.1
- **Port**: 5761
- **Description**: "RTK Base Station"

## VPN Network Layout

### IP Addresses
- **Base Station**: 10.8.0.1 (WireGuard server)
- **Remote Friend**: 10.8.0.2 (WireGuard client)
- **Local Pi**: 192.168.8.131 (accessible via base station)

### Accessible Services
Through the VPN, your friend can access:
- **QGroundControl**: 10.8.0.1:5760 (Mobile beacon control)
- **Base Station**: 10.8.0.1:5761 (Base station monitoring)
- **NTRIP Stream**: 10.8.0.1:2101 (RTK corrections)

## Security Features
- **Encryption**: ChaCha20Poly1305 authenticated encryption
- **Key Exchange**: Curve25519 elliptic curve
- **No Open Ports**: Only WireGuard port 51820 exposed
- **Network Isolation**: VPN clients can only access rover services

## Operational Procedures

### Starting Remote Session
1. **You (Base Station)**:
   ```bash
   # Start RTK base station
   cd ~/rtk_base_station && ./quick_rtcm_server.sh

   # Start base station position broadcaster (optional)
   python3 base_station_position_broadcaster.py

   # Verify mobile beacon is running
   ssh jay@192.168.8.131 "ps aux | grep mavlink_bridge"
   ```

2. **Friend (Remote)**:
   ```bash
   # Connect VPN
   wg-quick up friend_client

   # Test connection
   ping 10.8.0.1

   # Launch QGroundControl
   # Connect to 10.8.0.1:5760
   ```

### During Operation
- **Real-time RTK positioning** with centimeter accuracy
- **Full rover control** via QGroundControl Follow Me mode
- **Live telemetry** including GPS, battery, system status
- **Mission planning** and waypoint navigation

### Ending Session
1. **Friend**: Disconnect QGroundControl, deactivate VPN
2. **You**: Stop RTK services and WireGuard if desired

## Troubleshooting

### Connection Issues
```bash
# Check WireGuard status
sudo wg show

# Check firewall
sudo ufw status

# Test VPN connectivity
ping 10.8.0.2  # From base station to friend
ping 10.8.0.1  # From friend to base station
```

### QGroundControl Issues
- Verify VPN connection first
- Check that MAVLink bridge is running on Pi
- Confirm RTK system has GPS fix
- Test local connection before remote

### Performance Optimization
- **Bandwidth**: ~50kbps typical for telemetry + video
- **Latency**: <200ms for good responsiveness
- **Update Rate**: 10 Hz GPS, 1 Hz RTK corrections

## Safety Considerations
- **Emergency Stop**: Always have local override capability
- **Communication Loss**: Configure failsafe behavior in rover
- **GPS Reliability**: Monitor RTK fix quality during operation
- **Legal Compliance**: Ensure remote operation complies with local regulations

## Advanced Features
- **Multi-client**: Support multiple simultaneous connections
- **Video Streaming**: Add camera feeds through VPN
- **Logging**: Remote access to flight logs and telemetry data
- **Mission Sharing**: Collaborative mission planning