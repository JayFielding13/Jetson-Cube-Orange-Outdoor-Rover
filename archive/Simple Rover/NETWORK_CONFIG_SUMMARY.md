# Simple Rover - Network Configuration Summary

## Current Configuration (Verified 2025-10-13)

### WiFi Connection
- **SSID**: GL-AXT1800-f8d (Travel Router)
- **IP Address**: 192.168.8.100 (Static)
- **Gateway**: 192.168.8.1
- **DNS**: 192.168.8.1
- **Auto-connect**: Enabled
- **Priority**: 100 (High - preferred over ethernet)

### Ethernet Connection
- **Name**: Wired connection 1
- **IP Address**: DHCP or Static (depends on router)
- **Auto-connect**: Enabled
- **Priority**: -1 (Lower than WiFi)

## Connection Priority

The Pi is configured to prefer WiFi over Ethernet:
1. **WiFi (Priority 100)** - Will use this when available
2. **Ethernet (Priority -1)** - Fallback if WiFi unavailable

This means when you power on the rover, it will:
- First try to connect to WiFi (GL-AXT1800-f8d)
- Fall back to Ethernet if WiFi is unavailable
- Always accessible at **192.168.8.100** on either connection

## Testing Connectivity

### From Laptop (on same network):
```bash
# Ping test
ping 192.168.8.100

# SSH access
ssh jay@192.168.8.100

# View rover logs
ssh jay@192.168.8.100 "journalctl -u simple-rover.service -f"

# Check network status
ssh jay@192.168.8.100 "nmcli device status"
```

## Rover Service

The rover controller runs automatically on boot as a systemd service:
- **Service Name**: simple-rover.service
- **Status**: Enabled (starts automatically)
- **Script**: `/home/jay/Simple Rover/rover_controller_with_lora.py`

### Service Commands:
```bash
# Check status
sudo systemctl status simple-rover.service

# View live logs
journalctl -u simple-rover.service -f

# Restart service
sudo systemctl restart simple-rover.service

# Stop service
sudo systemctl stop simple-rover.service

# Start service
sudo systemctl start simple-rover.service
```

## Wireless Operation Workflow

### For Floor Testing:
1. **Power on the rover** (Pi + motors)
2. **Wait ~30 seconds** for Pi to boot and connect to WiFi
3. **Service auto-starts** - rover enters STOP mode
4. **Press LoRa transmitter button** to enable AUTONOMOUS mode
5. **Rover navigates autonomously** with corner escape and sensor fusion
6. **Press button again** to return to STOP mode

### For Development/Testing:
1. **Connect laptop to travel router** (WiFi or Ethernet)
2. **SSH to Pi**: `ssh jay@192.168.8.100`
3. **View live logs**: `journalctl -u simple-rover.service -f`
4. **Make code changes** and restart service
5. **Test without SSH** by disconnecting

## Network Troubleshooting

### Pi not accessible at 192.168.8.100:

**Check 1: Is Pi powered on and booted?**
- Wait 30-60 seconds after power on
- Check for activity LED on Pi

**Check 2: Is travel router on and accessible?**
```bash
# From laptop
ping 192.168.8.1
```

**Check 3: Is laptop on same network?**
```bash
# Check laptop IP
ip addr | grep 192.168.8
# Should show 192.168.8.x address
```

**Check 4: Check Pi's connection (if you can physically access)**
```bash
# On Pi's console/keyboard
nmcli device status
ip addr show wlan0
```

**Check 5: Try ethernet fallback**
- Connect ethernet cable between Pi and travel router
- Pi should get 192.168.8.100 via ethernet if WiFi fails

### WiFi Connection Issues:

**Reset WiFi connection:**
```bash
ssh jay@192.168.8.100  # Via ethernet if needed
sudo nmcli connection down GL-AXT1800-f8d
sudo nmcli connection up GL-AXT1800-f8d
```

**Check WiFi status:**
```bash
nmcli device wifi list
nmcli connection show GL-AXT1800-f8d
```

**Reconnect WiFi (if password changed):**
```bash
sudo nmcli connection modify GL-AXT1800-f8d wifi-sec.psk "NEW_PASSWORD"
sudo nmcli connection up GL-AXT1800-f8d
```

## Multi-Network Setup

If you want the Pi to connect to multiple networks (home + travel router):

### Current Networks:
1. **GL-AXT1800-f8d** (Travel Router) - Priority 100
2. **MikroTik-1DFF0C** - Priority 0

The Pi will automatically connect to whichever network is available, preferring the travel router.

### Add Another Network:
```bash
sudo nmcli connection add \
    type wifi \
    con-name "HomeWiFi" \
    ssid "YOUR_HOME_SSID" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "YOUR_PASSWORD" \
    ipv4.method auto \
    connection.autoconnect yes \
    connection.autoconnect-priority 50
```

## Quick Reference

| Component | Value |
|-----------|-------|
| Pi Hostname | simplerover |
| Pi IP | 192.168.8.100 |
| Travel Router IP | 192.168.8.1 |
| WiFi SSID | GL-AXT1800-f8d |
| SSH User | jay |
| Rover Service | simple-rover.service |
| Code Location | /home/jay/Simple Rover/ |

---

Last Updated: 2025-10-13
