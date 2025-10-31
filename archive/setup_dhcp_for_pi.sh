#!/bin/bash
# Set up simple DHCP server for direct Pi connection

echo "Installing and configuring DHCP server for Pi..."

# Install dnsmasq for DHCP
apt-get update && apt-get install -y dnsmasq

# Stop dnsmasq service
systemctl stop dnsmasq

# Create simple DHCP config
cat > /etc/dnsmasq.d/pi-direct.conf << EOF
# Direct Pi connection DHCP
interface=enp5s0
dhcp-range=192.168.100.10,192.168.100.20,255.255.255.0,12h
dhcp-option=3,192.168.100.1
dhcp-option=6,8.8.8.8
EOF

# Start dnsmasq
systemctl start dnsmasq
systemctl enable dnsmasq

echo "DHCP server configured for Pi direct connection"
echo "Pi should get IP 192.168.100.10-20 automatically"