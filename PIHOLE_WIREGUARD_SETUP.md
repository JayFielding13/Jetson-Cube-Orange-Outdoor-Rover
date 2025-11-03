# Pi-hole + WireGuard Integration Setup

**Date Configured:** November 2, 2025
**Server:** Raspberry Pi at 192.168.254.134
**Purpose:** Network-wide ad blocking for home network and WireGuard VPN clients

---

## Overview

This document details the Pi-hole DNS-based ad blocker integrated with our existing WireGuard VPN server running on a Raspberry Pi. This setup provides:

- Network-wide ad blocking for all home devices
- Ad blocking for WireGuard VPN clients when connected remotely
- DNS filtering and tracking protection
- Real-time statistics and monitoring via web interface

---

## System Configuration

### Hardware
- **Device:** Raspberry Pi (at 192.168.254.134)
- **Network:** 192.168.254.0/24 (home network)
- **WireGuard Network:** 10.8.0.0/24

### IP Addresses
- **Pi Static IP (WiFi):** 192.168.254.134/24
- **WireGuard Interface:** 10.8.0.1/24
- **Router/Gateway:** 192.168.254.254

### Services Running
1. **WireGuard VPN Server** - Port 51821/UDP
2. **Pi-hole DNS Server** - Port 53 (TCP/UDP)
3. **Pi-hole Web Interface** - Port 80 (HTTP)

---

## Pi-hole Installation Details

### Version Information
- **Pi-hole Core:** v6.2.2
- **Web Interface:** v6.3
- **FTL (DNS Engine):** v6.3.2

### Installation Date
November 2, 2025

### Installation Method
Automated installation using official Pi-hole installer:
```bash
curl -sSL https://install.pi-hole.net | sudo bash /dev/stdin --unattended
```

### Configuration Files
- **Main Config:** `/etc/pihole/pihole.toml`
- **Setup Variables:** `/etc/pihole/setupVars.conf` (migrated to pihole.toml)
- **Gravity Database:** `/etc/pihole/gravity.db`
- **Logs:** `/var/log/pihole/pihole.log`

---

## DNS Configuration

### Upstream DNS Servers
Pi-hole forwards queries to:
- **Primary:** 8.8.8.8 (Google DNS)
- **Secondary:** 8.8.4.4 (Google DNS)

### Listening Configuration
- **Interface:** wlan0 (also listens on all interfaces including wg0)
- **Listening Mode:** LOCAL (accepts queries from local subnets)
- **IPv4:** ✅ Enabled
- **IPv6:** ✅ Enabled

### Blocking Statistics
- **Initial Blocklist Domains:** 107,321 domains
- **Default Blocklist:** StevenBlack's Unified Hosts List
- **Blocking Status:** ENABLED

---

## Network Integration

### Home Network Setup

**Router DNS Configuration:**
The router (192.168.254.254) forwards all DNS queries to Pi-hole:

1. Router Advanced Connection Settings:
   - DNS Type: Static
   - Primary DNS: 192.168.254.134
   - Secondary DNS: 8.8.8.8

2. Client DNS Flow:
   ```
   Home Devices → Router (192.168.254.254) → Pi-hole (192.168.254.134) → Internet (8.8.8.8)
   ```

**Result:** All home network devices receive ad blocking automatically via router DNS forwarding.

### WireGuard Integration

**Server Configuration:** `/etc/wireguard/wg0.conf`
```ini
[Interface]
Address = 10.8.0.1/24
ListenPort = 51821
PrivateKey = <REDACTED>
PostUp = iptables -A FORWARD -i %i -j ACCEPT; iptables -A FORWARD -o %i -j ACCEPT; iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
PostDown = iptables -D FORWARD -i %i -j ACCEPT; iptables -D FORWARD -o %i -j ACCEPT; iptables -t nat -D POSTROUTING -o eth0 -j MASQUERADE

# Lana's device
[Peer]
PublicKey = Nqdh8YIIvk1ACGiyH3io8QD64A6xSpQGHG/rppVY+nw=
AllowedIPs = 10.8.0.2/32

# Test Client (Jay laptop)
[Peer]
PublicKey = EwnDcutQB/2xu8I5abNyLJLGygA6AbB3JGFJjwYP6DU=
AllowedIPs = 10.8.0.3/32
```

**Client Configuration Example:**

To enable Pi-hole DNS for WireGuard clients, add this to client configs:

```ini
[Interface]
PrivateKey = <CLIENT_PRIVATE_KEY>
Address = 10.8.0.2/32  # Unique per client: .2, .3, etc.
DNS = 10.8.0.1  # ← Routes DNS through Pi-hole

[Peer]
PublicKey = RcLQ1ur5dGpSkswTGjL2bGSElKe3dn6VE9Z++iCvJQQ=
Endpoint = <PUBLIC_IP_OR_DDNS>:51821
AllowedIPs = 0.0.0.0/0, ::/0  # Route all traffic through VPN
PersistentKeepalive = 25
```

**WireGuard DNS Flow:**
```
VPN Clients → WireGuard Tunnel → Pi-hole (10.8.0.1) → Internet (8.8.8.8)
```

---

## Static IP Configuration

The Pi's WiFi interface has been configured with a static IP to ensure Pi-hole remains accessible:

```bash
sudo nmcli connection modify 'preconfigured' \
  ipv4.method manual \
  ipv4.addresses 192.168.254.134/24 \
  ipv4.gateway 192.168.254.254 \
  ipv4.dns '8.8.8.8 8.8.4.4'
```

**Connection Name:** `preconfigured`
**Interface:** `wlan0`
**IP:** `192.168.254.134/24`

---

## Web Interface Access

### Login Information
- **URL (Home Network):** http://192.168.254.134/admin
- **URL (WireGuard):** http://10.8.0.1/admin
- **Password:** `pihole123` (⚠️ Consider changing this)

### Changing Password
```bash
sudo pihole setpassword 'your-new-password'
```

### Dashboard Features
- Real-time query statistics
- Top blocked domains
- Top clients making queries
- Query log viewer
- Blocklist/Allowlist management
- DHCP settings (if enabled)
- DNS settings configuration

---

## Management Commands

### Service Management
```bash
# Check Pi-hole status
pihole status

# Restart Pi-hole FTL service
sudo systemctl restart pihole-FTL

# Enable/disable blocking
pihole enable
pihole disable [time]  # Optionally disable for X seconds

# Restart WireGuard
sudo systemctl restart wg-quick@wg0
```

### DNS Management
```bash
# Reload DNS (update lists without restart)
pihole reloaddns

# Update gravity (blocklists)
pihole -g

# Flush logs
pihole -f

# Query a domain
pihole -q example.com
```

### Allowlist/Blocklist Management
```bash
# Allow a domain
pihole allow example.com

# Deny a domain
pihole deny ads.example.com

# Regex blocking
pihole --regex "ad[s]?\\..*\\.com"

# Regex allowing
pihole --allow-regex "api\\.trusted\\.com"

# Wildcard blocking
pihole --wild ads.example.com

# List all entries
pihole --list
```

### Logs and Debugging
```bash
# View live Pi-hole log
pihole -t

# View FTL log
sudo tail -f /var/log/pihole/FTL.log

# View dnsmasq queries
sudo tail -f /var/log/pihole/pihole.log

# Debug mode
pihole -d
```

---

## Testing and Verification

### Test DNS Resolution
```bash
# Test from any device on network
nslookup google.com

# Test Pi-hole directly
dig @192.168.254.134 google.com

# Test from WireGuard
dig @10.8.0.1 google.com
```

### Test Ad Blocking
1. **Online Test:** Visit http://canyoublockit.com/testing/
   - Should see placeholders instead of ads

2. **Command Line Test:**
   ```bash
   # Known ad domain - should return 0.0.0.0 or blocked message
   dig @192.168.254.134 doubleclick.net
   ```

3. **Check Pi-hole Dashboard:**
   - Visit http://192.168.254.134/admin
   - Monitor queries in real-time

### Verify Services
```bash
# SSH to Pi
ssh jay@192.168.254.134

# Check services
pihole status
sudo wg show
sudo systemctl status pihole-FTL
sudo systemctl status wg-quick@wg0

# Check listening ports
sudo netstat -tulpn | grep :53    # Pi-hole DNS
sudo netstat -tulpn | grep :51821 # WireGuard
```

---

## Troubleshooting

### Pi-hole Not Blocking Ads

1. **Check Pi-hole status:**
   ```bash
   pihole status
   ```

2. **Verify gravity database:**
   ```bash
   pihole -g
   ```

3. **Check if blocking is enabled:**
   ```bash
   pihole enable
   ```

4. **Verify DNS queries reaching Pi-hole:**
   ```bash
   sudo tail -f /var/log/pihole/pihole.log
   ```

### DNS Not Resolving

1. **Check Pi-hole FTL is running:**
   ```bash
   sudo systemctl status pihole-FTL
   ```

2. **Restart FTL:**
   ```bash
   sudo systemctl restart pihole-FTL
   ```

3. **Check upstream DNS:**
   ```bash
   dig @8.8.8.8 google.com
   ```

4. **Check firewall:**
   ```bash
   sudo ufw status
   ```

### WireGuard Clients Not Using Pi-hole

1. **Verify client config has DNS line:**
   ```ini
   DNS = 10.8.0.1
   ```

2. **Check WireGuard is running:**
   ```bash
   sudo wg show
   ```

3. **Test DNS from WireGuard network:**
   ```bash
   dig @10.8.0.1 google.com
   ```

### Pi Lost Static IP

1. **Reapply static IP configuration:**
   ```bash
   sudo nmcli connection modify 'preconfigured' \
     ipv4.method manual \
     ipv4.addresses 192.168.254.134/24 \
     ipv4.gateway 192.168.254.254 \
     ipv4.dns '8.8.8.8 8.8.4.4'

   sudo nmcli connection down 'preconfigured'
   sudo nmcli connection up 'preconfigured'
   ```

---

## Maintenance

### Regular Tasks

**Weekly:**
- Check Pi-hole dashboard for statistics
- Review top blocked domains
- Verify blocking is working

**Monthly:**
- Update gravity (blocklists):
  ```bash
  pihole -g
  ```
- Review and update allowlist if needed
- Check for Pi-hole updates:
  ```bash
  pihole -up --check-only
  ```

**As Needed:**
- Add domains to allowlist if legitimate sites break
- Add additional blocklists for enhanced protection
- Review query logs for anomalies

### Updating Pi-hole

```bash
# Check for updates
pihole -up --check-only

# Update Pi-hole
pihole -up

# Update operating system
sudo apt update && sudo apt upgrade -y
```

### Backup Configuration

```bash
# Backup Pi-hole configuration
sudo pihole -a -t

# Backup WireGuard config
sudo cp /etc/wireguard/wg0.conf ~/wg0.conf.backup

# Backup to remote location
scp jay@192.168.254.134:~/wg0.conf.backup ./
```

---

## Additional Blocklists (Optional)

To add more comprehensive blocking, consider these popular blocklists:

### Via Web Interface
1. Go to http://192.168.254.134/admin
2. Navigate to Adlists
3. Add URLs to additional blocklists

### Popular Blocklists
- **OISD:** https://big.oisd.nl/
- **1Hosts (Pro):** https://o0.pages.dev/Pro/adblock.txt
- **Hagezi Multi PRO:** https://cdn.jsdelivr.net/gh/hagezi/dns-blocklists@latest/wildcard/pro.txt
- **Developer Dan Ads & Tracking:** https://www.github.developerdan.com/hosts/lists/ads-and-tracking-extended.txt

After adding blocklists:
```bash
pihole -g  # Update gravity
```

---

## Security Considerations

### Current Security Posture
✅ Pi-hole only accepts queries from local networks (LOCAL mode)
✅ Static IP prevents DHCP conflicts
✅ WireGuard provides encrypted VPN tunnel
⚠️ Web interface password is default (`pihole123`)
⚠️ Pi-hole web interface accessible on local network without additional auth

### Recommended Security Improvements

1. **Change Web Interface Password:**
   ```bash
   sudo pihole setpassword
   ```

2. **Enable HTTPS for Web Interface** (optional):
   ```bash
   sudo apt install lighttpd-mod-openssl
   # Configure SSL certificate
   ```

3. **Firewall Configuration:**
   ```bash
   sudo ufw allow 53/tcp    # DNS
   sudo ufw allow 53/udp    # DNS
   sudo ufw allow 51821/udp # WireGuard
   sudo ufw allow 80/tcp    # Pi-hole web interface
   sudo ufw allow 22/tcp    # SSH
   sudo ufw enable
   ```

4. **Regular Updates:**
   - Keep Pi-hole updated
   - Keep Raspberry Pi OS updated
   - Monitor security advisories

---

## Performance and Monitoring

### Expected Performance
- **Query Response Time:** < 50ms
- **Memory Usage:** ~150-300 MB
- **CPU Usage:** < 5% idle, < 20% under load
- **Blocked Percentage:** Typically 15-30% of queries

### Monitoring Commands
```bash
# System resources
htop

# Pi-hole statistics via API
pihole api /stats/summary

# Check DNS query load
sudo tail -f /var/log/pihole/pihole.log | grep query
```

### Performance Tuning (if needed)
Edit `/etc/pihole/pihole.toml`:
```toml
[dns]
  # Increase cache size for better performance
  cacheSize = 20000  # Default is 10000
```

---

## Integration with Other Services

### Current Integrations
- ✅ WireGuard VPN Server
- ✅ Home Router DHCP/DNS forwarding

### Potential Future Integrations
- **Unbound Recursive DNS:** Use Pi as its own recursive DNS server
- **DoH/DoT:** Encrypted DNS to upstream servers
- **Home Assistant:** Integrate Pi-hole statistics
- **Grafana:** Advanced monitoring dashboards
- **Homebridge:** Network monitoring

---

## Quick Reference

### Essential URLs
- **Web Interface:** http://192.168.254.134/admin
- **API Endpoint:** http://192.168.254.134/admin/api.php
- **Ad Test Site:** http://canyoublockit.com/testing/

### Essential Commands
```bash
# Status check
pihole status

# Enable/disable blocking
pihole enable
pihole disable 5m

# Update blocklists
pihole -g

# View logs
pihole -t

# Allowlist domain
pihole allow example.com

# Blocklist domain
pihole deny ads.example.com

# Restart services
sudo systemctl restart pihole-FTL
sudo systemctl restart wg-quick@wg0
```

### SSH Access
```bash
ssh jay@192.168.254.134
# Password: jay
```

### Key IP Addresses
- **Pi (Home):** 192.168.254.134
- **Pi (WireGuard):** 10.8.0.1
- **Router:** 192.168.254.254
- **DNS Upstream:** 8.8.8.8, 8.8.4.4

---

## Changelog

### November 2, 2025 - Initial Setup
- Installed Pi-hole v6.2.2
- Configured static IP on Raspberry Pi WiFi interface
- Integrated with existing WireGuard VPN server
- Configured router to forward DNS to Pi-hole
- Set web interface password to `pihole123`
- Loaded StevenBlack blocklist (107,321 domains)
- Verified ad blocking working on home network
- Documented client configuration for WireGuard

---

## Support and Resources

### Official Documentation
- **Pi-hole Docs:** https://docs.pi-hole.net/
- **Pi-hole GitHub:** https://github.com/pi-hole/pi-hole
- **Pi-hole Discourse:** https://discourse.pi-hole.net/
- **WireGuard Docs:** https://www.wireguard.com/

### Community Resources
- **Reddit:** r/pihole
- **Discord:** Pi-hole Community Server

### Getting Help
1. Check Pi-hole dashboard for errors
2. Review logs: `/var/log/pihole/pihole.log`
3. Run debug mode: `pihole -d`
4. Search Pi-hole Discourse
5. Consult this documentation

---

## Notes

- Pi-hole blocks ads at the DNS level, so some cosmetic filtering may not occur (unlike browser extensions)
- Some applications/services may break if they serve ads and content from the same domain
- Use the allowlist feature to unblock legitimate services
- Query logs contain all DNS requests - consider privacy implications
- Regular gravity updates ensure blocklists stay current
- The setup uses DNS forwarding through the router, which simplifies management but means all queries appear to come from the router IP in Pi-hole logs

---

**Document Version:** 1.0
**Last Updated:** November 2, 2025
**Maintained By:** Jay
**Related Documentation:** WireGuard setup, Network infrastructure docs
