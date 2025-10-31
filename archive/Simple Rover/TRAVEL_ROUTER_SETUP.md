# Simple Rover - Travel Router Connection Guide

## Overview
Connect your Simple Rover Raspberry Pi to your travel router for wireless access and control. This allows you to:
- SSH into the rover remotely
- Monitor rover status
- View logs in real-time
- Update code wirelessly
- Control multiple devices on the same network

---

## Quick Setup

### Option 1: Assign Static IP for Simple Rover

Since you already have a travel router set up (RoverNet), let's add the Simple Rover Pi to it.

**Recommended IP:** `192.168.8.100` (avoiding your other Pis at .65 and .70)

### Step 1: Connect Simple Rover Pi to Travel Router

On the Simple Rover Raspberry Pi, run:

```bash
# Add travel router WiFi connection
sudo nmcli connection add \
    type wifi \
    con-name "RoverNet" \
    ssid "RoverNet" \
    wifi.mode infrastructure \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "YOUR_ROUTER_PASSWORD" \
    ipv4.method manual \
    ipv4.addresses 192.168.8.100/24 \
    ipv4.gateway 192.168.8.1 \
    ipv4.dns "192.168.8.1,8.8.8.8" \
    connection.autoconnect yes \
    connection.autoconnect-priority 100
```

### Step 2: Connect to the Network

```bash
# Activate the connection
nmcli connection up "RoverNet"

# Verify connection
ip addr show wlan0
ping 192.168.8.1
```

### Step 3: Test SSH Access

From your laptop (also connected to RoverNet):

```bash
ssh jay@192.168.8.100
```

---

## Network Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      Travel Router                          │
│                   (RoverNet / 192.168.8.1)                  │
│                                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │ Navigation Pi│  │ Companion Pi │  │  Simple Rover│     │
│  │ 192.168.8.65 │  │ 192.168.8.70 │  │ 192.168.8.100│     │
│  └──────────────┘  └──────────────┘  └──────────────┘     │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │            Your Laptop (Control Station)             │  │
│  │            192.168.8.x (DHCP assigned)               │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

---

## IP Address Assignments

| Device | IP Address | Purpose |
|--------|------------|---------|
| Travel Router | 192.168.8.1 | Gateway |
| Navigation Pi | 192.168.8.65 | Main rover (existing) |
| Companion Pi | 192.168.8.70 | Companion rover (existing) |
| **Simple Rover Pi** | **192.168.8.100** | Your new simple rover |
| Your Laptop | 192.168.8.x | Control station (DHCP) |

---

## Accessing the Simple Rover

### Via SSH
```bash
ssh jay@192.168.8.100
```

### Running the Rover Script Remotely
```bash
# SSH into the rover
ssh jay@192.168.8.100

# Navigate to the project
cd "/home/jay/Desktop/Mini Rover Development/Simple Rover"

# Run the rover controller
python3 rover_controller_with_lora.py
```

### View Logs in Real-Time
```bash
# SSH and run with output
ssh jay@192.168.8.100 "cd '/home/jay/Desktop/Mini Rover Development/Simple Rover' && python3 rover_controller_with_lora.py"
```

---

## Auto-Start Rover on Boot (Optional)

If you want the rover to start automatically when powered on:

### Create systemd Service

```bash
# On the Simple Rover Pi
sudo nano /etc/systemd/system/simple-rover.service
```

Add this content:

```ini
[Unit]
Description=Simple Rover Controller with LoRa
After=network.target

[Service]
Type=simple
User=jay
WorkingDirectory=/home/jay/Desktop/Mini Rover Development/Simple Rover
ExecStart=/usr/bin/python3 rover_controller_with_lora.py
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

### Enable and Start Service

```bash
# Reload systemd
sudo systemctl daemon-reload

# Enable auto-start on boot
sudo systemctl enable simple-rover.service

# Start the service now
sudo systemctl start simple-rover.service

# Check status
sudo systemctl status simple-rover.service

# View logs
journalctl -u simple-rover.service -f
```

### Control the Service

```bash
# Stop
sudo systemctl stop simple-rover.service

# Start
sudo systemctl start simple-rover.service

# Restart
sudo systemctl restart simple-rover.service

# Disable auto-start
sudo systemctl disable simple-rover.service
```

---

## Troubleshooting

### Can't Connect to RoverNet

**Check if travel router is on:**
```bash
nmcli device wifi list
```

**Check connection status:**
```bash
nmcli connection show
```

**Reconnect manually:**
```bash
sudo nmcli connection up "RoverNet"
```

### Wrong IP Address

**Verify IP:**
```bash
ip addr show wlan0
```

**Should show:** `inet 192.168.8.100/24`

**If wrong, delete and recreate:**
```bash
sudo nmcli connection delete "RoverNet"
# Then re-run the nmcli add command from Step 1
```

### Can't SSH from Laptop

**Make sure laptop is on RoverNet:**
```bash
# On your laptop
ip addr | grep 192.168.8
```

**Ping test:**
```bash
ping 192.168.8.100
```

**If ping fails:**
- Check travel router is powered on
- Verify Pi is connected: `nmcli connection show --active`
- Check firewall: `sudo ufw status`

### Serial Devices Not Found

**Check USB connections:**
```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

**Check permissions:**
```bash
sudo usermod -a -G dialout jay
# Log out and back in
```

---

## Security Considerations

### Change Default Password
```bash
passwd
```

### Setup SSH Keys (Recommended)
```bash
# On your laptop, generate key if you don't have one
ssh-keygen -t ed25519

# Copy to Simple Rover Pi
ssh-copy-id jay@192.168.8.100

# Now you can SSH without password
ssh jay@192.168.8.100
```

### Disable Password Authentication (Optional, after SSH keys work)
```bash
# On Simple Rover Pi
sudo nano /etc/ssh/sshd_config

# Change to:
# PasswordAuthentication no

# Restart SSH
sudo systemctl restart ssh
```

---

## Fallback Network (Optional)

If you want the Simple Rover to fall back to your home WiFi when travel router is off:

```bash
# Add home network with lower priority
sudo nmcli connection add \
    type wifi \
    con-name "HomeWiFi" \
    ssid "YOUR_HOME_SSID" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "YOUR_HOME_PASSWORD" \
    ipv4.method auto \
    connection.autoconnect yes \
    connection.autoconnect-priority 50
```

**Priority System:**
- **100**: Travel Router (RoverNet) - Primary
- **50**: Home WiFi - Fallback

The Pi will automatically switch based on availability.

---

## Monitoring Multiple Rovers

### SSH to All Rovers from One Terminal

Using **tmux** for multi-pane terminal:

```bash
# Install tmux on your laptop if not installed
sudo apt install tmux  # Linux
brew install tmux      # macOS

# Create multi-pane session
tmux new-session -s rovers

# Split panes (Ctrl+B then %)
# Ctrl+B then %  = vertical split
# Ctrl+B then "  = horizontal split

# In each pane, SSH to different rover:
# Pane 1: ssh jay@192.168.8.65  (Navigation Pi)
# Pane 2: ssh jay@192.168.8.70  (Companion Pi)
# Pane 3: ssh jay@192.168.8.100 (Simple Rover)

# Navigate between panes: Ctrl+B then arrow keys
# Detach: Ctrl+B then D
# Reattach: tmux attach -t rovers
```

---

## Testing Checklist

- [ ] Travel router powered on and broadcasting RoverNet
- [ ] Simple Rover Pi connected to RoverNet
- [ ] Pi has IP 192.168.8.100
- [ ] Can ping 192.168.8.1 (gateway)
- [ ] Can SSH from laptop: `ssh jay@192.168.8.100`
- [ ] USB devices detected: Arduino and Heltec
- [ ] Rover controller starts without errors
- [ ] LoRa transmitter in range and working
- [ ] Can switch modes (STOP/AUTONOMOUS)
- [ ] Motors respond correctly

---

## Quick Reference Commands

```bash
# Check WiFi connection
nmcli connection show --active

# Check IP address
ip addr show wlan0

# Reconnect to RoverNet
sudo nmcli connection up "RoverNet"

# SSH to Simple Rover
ssh jay@192.168.8.100

# Run rover controller
cd "/home/jay/Desktop/Mini Rover Development/Simple Rover"
python3 rover_controller_with_lora.py

# Check USB devices
ls /dev/ttyUSB* /dev/ttyACM*

# View system logs
journalctl -f
```

---

## Advanced: VPN Access (Like Your Other Rover)

If you want to access the Simple Rover from anywhere (not just local network), you can extend your existing WireGuard VPN setup:

### Add Route to Simple Rover

On your VPN server (base station), add routing:

```bash
# In /etc/wireguard/wg0.conf, add to PostUp:
iptables -A FORWARD -d 192.168.8.100 -j ACCEPT

# After adding, restart WireGuard
sudo systemctl restart wg-quick@wg0
```

Then your remote friend can access:
- Simple Rover Pi: `192.168.8.100` (via VPN tunnel through base station)

---

## Next Steps

1. Configure WiFi connection on Simple Rover Pi
2. Test SSH access
3. Run rover controller remotely
4. Optional: Set up auto-start service
5. Optional: Configure VPN access for remote control

---

## Notes

- Travel router range: Typically 30-50 meters outdoors
- Battery life: Check travel router battery regularly
- Interference: 2.4GHz can have interference - consider 5GHz if available
- Multiple rovers: All can coexist on same network with different IPs
