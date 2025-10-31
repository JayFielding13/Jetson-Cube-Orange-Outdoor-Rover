#!/bin/bash
# Configure Ethernet Connection to Travel Router
# This connects your computer to the travel router network (192.168.8.x)

echo "=========================================="
echo "  Connect to Travel Router Network"
echo "=========================================="
echo ""

# Configure ethernet connection for travel router
echo "Configuring ethernet connection for 192.168.8.x network..."
sudo nmcli connection modify "Wired connection 2" \
    ipv4.addresses 192.168.8.182/24 \
    ipv4.gateway 192.168.8.1 \
    ipv4.dns "192.168.8.1" \
    ipv4.method manual \
    connection.autoconnect no

echo "Activating ethernet connection..."
sudo nmcli connection up "Wired connection 2"

echo ""
echo "Checking connection status..."
sleep 2

# Check if connected
ip addr show enp5s0 | grep "inet 192.168.8"

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Successfully connected to travel router network!"
    echo "✓ Your IP: 192.168.8.182"
    echo ""
    echo "Testing connection to travel router gateway..."
    ping -c 2 192.168.8.1

    echo ""
    echo "Testing connection to Simple Rover Pi..."
    ping -c 2 192.168.8.100

    echo ""
    echo "Attempting SSH to Simple Rover Pi..."
    echo "ssh jay@192.168.8.100"
else
    echo ""
    echo "✗ Failed to connect. Check that:"
    echo "  - Travel router is powered on"
    echo "  - Ethernet cable is connected"
    echo "  - Travel router is on 192.168.8.x network"
fi
