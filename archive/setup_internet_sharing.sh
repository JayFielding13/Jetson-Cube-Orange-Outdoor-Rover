#!/bin/bash
# Internet Connection Sharing Setup Script
# Shares WiFi internet connection through Ethernet to travel router

echo "Setting up Internet Connection Sharing..."

# Enable IP forwarding
echo 1 > /proc/sys/net/ipv4/ip_forward
echo "✓ IP forwarding enabled"

# Clear any existing iptables rules for these interfaces
iptables -t nat -D POSTROUTING -o wlp3s0 -j MASQUERADE 2>/dev/null
iptables -D FORWARD -i enp5s0 -o wlp3s0 -j ACCEPT 2>/dev/null
iptables -D FORWARD -i wlp3s0 -o enp5s0 -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null

# Set up NAT (Network Address Translation)
iptables -t nat -A POSTROUTING -o wlp3s0 -j MASQUERADE
echo "✓ NAT configured for WiFi interface (wlp3s0)"

# Allow forwarding from Ethernet to WiFi
iptables -A FORWARD -i enp5s0 -o wlp3s0 -j ACCEPT
echo "✓ Forward rule: Ethernet → WiFi"

# Allow established connections back
iptables -A FORWARD -i wlp3s0 -o enp5s0 -m state --state RELATED,ESTABLISHED -j ACCEPT
echo "✓ Forward rule: WiFi → Ethernet (established connections)"

echo ""
echo "Internet Connection Sharing is now active!"
echo "Travel router devices can now access the internet through your laptop's WiFi"
echo ""
echo "To test from the Pi:"
echo "  ping 8.8.8.8"
echo ""
echo "To disable internet sharing later, run:"
echo "  sudo iptables -t nat -F"
echo "  sudo iptables -F FORWARD"