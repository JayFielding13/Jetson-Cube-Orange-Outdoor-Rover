#!/bin/bash
# Direct Pi Connection Setup Script
# Sets up direct Ethernet connection with internet sharing

echo "Setting up direct Pi connection with internet sharing..."

# Set up laptop Ethernet interface with static IP
ip addr add 192.168.100.1/24 dev enp5s0
ip link set enp5s0 up
echo "✓ Laptop Ethernet set to 192.168.100.1"

# Enable IP forwarding
echo 1 > /proc/sys/net/ipv4/ip_forward
echo "✓ IP forwarding enabled"

# Clear any existing iptables rules
iptables -t nat -D POSTROUTING -o wlp3s0 -j MASQUERADE 2>/dev/null
iptables -D FORWARD -i enp5s0 -o wlp3s0 -j ACCEPT 2>/dev/null
iptables -D FORWARD -i wlp3s0 -o enp5s0 -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null

# Set up NAT for internet sharing
iptables -t nat -A POSTROUTING -o wlp3s0 -j MASQUERADE
iptables -A FORWARD -i enp5s0 -o wlp3s0 -j ACCEPT
iptables -A FORWARD -i wlp3s0 -o enp5s0 -m state --state RELATED,ESTABLISHED -j ACCEPT
echo "✓ Internet sharing configured"

# Set up DHCP service for the Pi (optional - Pi can use static IP)
# We'll configure the Pi to use 192.168.100.10

echo ""
echo "Direct Pi connection ready!"
echo "Laptop IP: 192.168.100.1"
echo "Pi should be configured to: 192.168.100.10"
echo ""
echo "Configure Pi with:"
echo "  sudo ip addr add 192.168.100.10/24 dev eth0"
echo "  sudo ip route add default via 192.168.100.1"
echo "  echo 'nameserver 8.8.8.8' | sudo tee /etc/resolv.conf"