# WireGuard VPN Configuration Reference

**Date Configured:** October 4, 2025
**Purpose:** Remote rover control via QGroundControl through secure VPN tunnel

---

## System Overview

### Network Topology
- **VPN Network:** 10.8.0.0/24
  - Server (Base Station): 10.8.0.1
  - Client (Friend): 10.8.0.2
- **Rover Network:** 192.168.8.0/24 (Ethernet, no internet)
- **Internet Connection:** WiFi (wlp3s0) at 192.168.254.129
- **Public IP:** 50.54.153.144
- **VPN Port:** UDP 51820

---

## Server Configuration

### Network Interfaces
- **WiFi (Internet):** wlp3s0 - 192.168.254.129/24 (Gateway: 192.168.254.254)
- **Ethernet (Rover):** enp5s0 - 192.168.8.182/24 (No internet, robotics only)
- **WireGuard VPN:** wg0 - 10.8.0.1/24

### Server Config File: /etc/wireguard/wg0.conf
```
[Interface]
# Base Station WireGuard Server
PrivateKey = AA5Hhufpvxv0/yzdFQotrTY2M6A1560Z+A7jLsbE3Ww=
Address = 10.8.0.1/24
ListenPort = 51820
PostUp = iptables -A FORWARD -i %i -j ACCEPT; iptables -A FORWARD -o %i -j ACCEPT; iptables -t nat -A POSTROUTING -o wlp3s0 -j MASQUERADE
PostDown = iptables -D FORWARD -i %i -j ACCEPT; iptables -D FORWARD -o %i -j ACCEPT; iptables -t nat -D POSTROUTING -o wlp3s0 -j MASQUERADE

[Peer]
# Remote Friend Client
PublicKey = ZeqgDTuZ95b20AVa0F7YpNFlm9V160knrfu6yeFQ11c=
AllowedIPs = 10.8.0.2/32
```

### Server Public Key
```
3tXP4VrAHqZAZqSxJP9M21acKTcXuH/jUxsYz+eAiF0=
```

---

## Client Configuration

### Client Config File: ~/friend_client.conf
```
[Interface]
# Remote Friend WireGuard Client
PrivateKey = WFenC9IQzKMXotT8QstyYkgKhZeW1TAqEVJmv/xvGFs=
Address = 10.8.0.2/24
DNS = 8.8.8.8

[Peer]
# Base Station Server
PublicKey = 3tXP4VrAHqZAZqSxJP9M21acKTcXuH/jUxsYz+eAiF0=
Endpoint = 50.54.153.144:51820
AllowedIPs = 10.8.0.0/24, 192.168.8.0/24
PersistentKeepalive = 25
```

---

## System Configuration

### IP Forwarding
**Status:** Enabled ✓
```bash
sysctl net.ipv4.ip_forward
# Output: net.ipv4.ip_forward = 1
```

To permanently enable (already configured):
```bash
echo "net.ipv4.ip_forward=1" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

### Firewall (UFW)
**Status:** Inactive
- UFW is not blocking any traffic
- All firewall rules managed through iptables via WireGuard PostUp/PostDown scripts

### iptables Rules
Applied automatically by WireGuard when interface starts:

**Filter Table:**
```
FORWARD chain:
- ACCEPT all traffic from wg0 interface
- ACCEPT all traffic to wg0 interface
```

**NAT Table:**
```
POSTROUTING chain:
- MASQUERADE all traffic going out through wlp3s0 (WiFi)
```

---

## Router Configuration

### Port Forwarding Rule
- **Protocol:** UDP
- **External/Global Port:** 51820
- **Internal IP:** 192.168.254.129 (WiFi interface)
- **Internal/Local Port:** 51820
- **Device:** Base Station Computer (MAC: 14:5a:fc:3d:92:89)
- **Service Name:** WireGuard (or custom name)

### Router Details
- **WiFi Gateway:** 192.168.254.254
- **Public IP:** 50.54.153.144

---

## Critical Issue Fixed

### Problem: Interface Mismatch
**Original Issue:** WireGuard config referenced `eth0` which doesn't exist on this system
- Config had: `POSTROUTING -o eth0 -j MASQUERADE`
- Actual interface: `wlp3s0` (WiFi)

**Solution:** Updated PostUp/PostDown rules to use `wlp3s0` instead of `eth0`

### Why This Matters
- NAT masquerading allows VPN clients to access internet through the server
- Without correct interface, traffic routing fails
- Client can connect to VPN but cannot access internet or rover network

---

## WireGuard Management Commands

### Start/Stop VPN
```bash
# Start VPN interface
sudo wg-quick up wg0

# Stop VPN interface
sudo wg-quick down wg0

# Restart VPN (stop then start)
sudo wg-quick down wg0 && sudo wg-quick up wg0
```

### Check Status
```bash
# Show WireGuard interface status and peers
sudo wg show

# Check if interface is up
ip addr show wg0

# View routing table
ip route show
```

### View Configuration
```bash
# Read config file
sudo cat /etc/wireguard/wg0.conf

# Check iptables rules
sudo iptables -L -v -n
sudo iptables -t nat -L -v -n
```

### Enable Auto-Start
```bash
# Enable WireGuard to start on boot
sudo systemctl enable wg-quick@wg0

# Check service status
sudo systemctl status wg-quick@wg0
```

---

## Troubleshooting

### Verify VPN Connection
When client connects, you should see handshake:
```bash
sudo wg show
```

Expected output with active connection:
```
interface: wg0
  public key: 3tXP4VrAHqZAZqSxJP9M21acKTcXuH/jUxsYz+eAiF0=
  private key: (hidden)
  listening port: 51820

peer: ZeqgDTuZ95b20AVa0F7YpNFlm9V160knrfu6yeFQ11c=
  endpoint: <client_public_ip>:<port>
  allowed ips: 10.8.0.2/32
  latest handshake: X seconds ago    <-- This line indicates successful connection
  transfer: X.XX KiB received, X.XX KiB sent
```

### Common Issues

#### 1. No Handshake Appearing
**Symptoms:** Client connects but no "latest handshake" appears
**Causes:**
- Port forwarding not configured correctly on router
- Firewall blocking UDP 51820
- Client using wrong endpoint IP/port

**Debug:**
```bash
# Check if port is listening
sudo ss -ulnp | grep 51820

# Check public IP
curl -4 ifconfig.me

# Verify NAT rules
sudo iptables -t nat -L -v -n | grep wlp3s0
```

#### 2. Client Can Ping VPN but Not Internet
**Symptoms:** ping 10.8.0.1 works, but no internet access
**Causes:**
- IP forwarding disabled
- NAT masquerading not working
- Wrong interface in MASQUERADE rule

**Debug:**
```bash
# Check IP forwarding
sysctl net.ipv4.ip_forward

# Check NAT rules are using correct interface
sudo iptables -t nat -L POSTROUTING -v -n
```

#### 3. VPN Won't Start
**Symptoms:** `wg-quick up wg0` fails
**Causes:**
- Config file syntax error
- Port already in use
- Permission issues

**Debug:**
```bash
# Check for syntax errors
sudo wg-quick up wg0

# Check if port is in use
sudo ss -ulnp | grep 51820

# Check config file permissions
ls -la /etc/wireguard/wg0.conf
```

#### 4. Interface Name Changed After Reboot
**Symptoms:** wlp3s0 interface name changes
**Causes:**
- Network interface enumeration changed
- Hardware changes

**Solution:**
```bash
# Find current WiFi interface
ip addr show | grep -A2 "state UP"

# Update WireGuard config with new interface name
sudo nano /etc/wireguard/wg0.conf
# Change wlp3s0 to new interface name in PostUp/PostDown

# Restart WireGuard
sudo wg-quick down wg0
sudo wg-quick up wg0
```

#### 5. Public IP Changed
**Symptoms:** Client cannot connect after working previously
**Causes:**
- ISP changed your public IP
- Router rebooted and got new IP from DHCP

**Solution:**
```bash
# Check current public IP
curl -4 ifconfig.me

# Update client config with new endpoint IP
# Send updated friend_client.conf to client
```

---

## Network Testing Commands

### Test VPN Connectivity (Server Side)
```bash
# Ping client through VPN
ping 10.8.0.2

# Check routing to VPN network
ip route get 10.8.0.2

# Monitor WireGuard traffic in real-time
watch -n 1 'sudo wg show'
```

### Test From Client Side
```bash
# Test VPN connection
ping 10.8.0.1

# Test access to rover network
ping 192.168.8.1

# Test internet through VPN (if AllowedIPs includes 0.0.0.0/0)
curl -4 ifconfig.me
```

---

## Security Notes

### Private Keys
- Server private key stored in: `/etc/wireguard/wg0.conf`
- Client private key stored in: `~/friend_client.conf`
- **NEVER share private keys publicly**
- Config file permissions: 600 (read/write owner only)

### Access Control
- Only authorized peer (friend) can connect via public key authentication
- Peer limited to IP 10.8.0.2/32
- Cannot connect without matching private/public key pair

### Monitoring
```bash
# View active connections
sudo wg show all

# Check for unauthorized access attempts (system logs)
sudo journalctl -u wg-quick@wg0 -f
```

---

## QGroundControl Connection

### Rover Control Setup
- **Connection Type:** TCP
- **Host Address:** 10.8.0.1
- **Port:** 5760

Client connects to VPN, then QGroundControl connects to rover through VPN tunnel:
```
Client PC → VPN (10.8.0.2) → Server (10.8.0.1) → Rover (192.168.8.x:5760)
```

---

## Important Files Reference

### Configuration Files
- **Server WireGuard Config:** `/etc/wireguard/wg0.conf`
- **Client Config (for friend):** `~/friend_client.conf`
- **Client Setup Guide:** `~/FRIEND_ROVER_CONTROL_SETUP.txt`
- **This Documentation:** `~/Desktop/Mini Rover Development/VPN_Configuration_Reference.md`

### System Settings
- **IP Forwarding:** `/etc/sysctl.conf` (net.ipv4.ip_forward=1)
- **WireGuard Service:** `wg-quick@wg0.service`

---

## Quick Reference Commands

```bash
# Check everything is working
sudo wg show                           # Shows VPN status and peers
ip addr show wg0                       # Shows VPN interface
curl -4 ifconfig.me                    # Shows public IP
sudo iptables -t nat -L -v -n          # Shows NAT rules

# Restart VPN
sudo wg-quick down wg0 && sudo wg-quick up wg0

# Monitor connection
watch -n 1 'sudo wg show'              # Real-time connection status
sudo journalctl -u wg-quick@wg0 -f     # Live logs
```

---

## Change Log

### October 4, 2025 - Initial Setup & Fix
- Installed WireGuard server
- Created VPN configuration (10.8.0.0/24 network)
- **Fixed critical issue:** Changed MASQUERADE interface from `eth0` to `wlp3s0`
- Configured router port forwarding (UDP 51820 → 192.168.254.129)
- Verified IP forwarding enabled
- Created client configuration files
- Documented complete setup

### Future Changes
*Document any configuration changes here with date and description*

---

**For support, refer to this document and run diagnostic commands above.**
