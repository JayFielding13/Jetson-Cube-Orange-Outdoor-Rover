#!/bin/bash
# Dual Network Setup: WiFi (Internet) + Ethernet (Travel Router)
# This configures your computer to use WiFi for internet while keeping
# ethernet connected to the travel router for rover communication

echo "=========================================="
echo "  Dual Network Configuration"
echo "=========================================="
echo ""
echo "This will configure:"
echo "  WiFi (wlp3s0)     = Internet connection (default route)"
echo "  Ethernet (enp5s0) = Travel router only (no default route)"
echo ""

# Configure ethernet for travel router - NO DEFAULT ROUTE
echo "Configuring ethernet for travel router (192.168.8.x)..."
sudo nmcli connection modify "Wired connection 2" \
    ipv4.addresses 192.168.8.182/24 \
    ipv4.method manual \
    ipv4.never-default yes \
    ipv4.dns "" \
    connection.autoconnect no

echo "✓ Ethernet configured (no default route - won't interfere with WiFi)"
echo ""

# Ensure WiFi has lower metric (preferred for default route)
echo "Ensuring WiFi remains primary internet connection..."
sudo nmcli connection modify "Ziply-B220" \
    ipv4.route-metric 100 \
    connection.autoconnect yes

echo "✓ WiFi set as primary internet connection"
echo ""

# Now bring up the ethernet connection
echo "Activating ethernet connection to travel router..."
sudo nmcli connection up "Wired connection 2"

echo ""
echo "Waiting for connection to establish..."
sleep 3

echo ""
echo "=========================================="
echo "  Connection Status"
echo "=========================================="
echo ""

# Show active connections
nmcli connection show --active

echo ""
echo "=========================================="
echo "  IP Addresses"
echo "=========================================="

# Show WiFi IP
WIFI_IP=$(ip addr show wlp3s0 | grep "inet " | awk '{print $2}')
if [ -n "$WIFI_IP" ]; then
    echo "✓ WiFi:     $WIFI_IP (Internet)"
else
    echo "✗ WiFi:     Not connected"
fi

# Show Ethernet IP
ETH_IP=$(ip addr show enp5s0 | grep "inet " | awk '{print $2}')
if [ -n "$ETH_IP" ]; then
    echo "✓ Ethernet: $ETH_IP (Travel Router)"
else
    echo "✗ Ethernet: Not connected"
fi

echo ""
echo "=========================================="
echo "  Routing Table"
echo "=========================================="
ip route | grep default

echo ""
echo "Default route should be via WiFi (wlp3s0)"
echo ""

# Test internet connectivity via WiFi
echo "=========================================="
echo "  Testing Internet (via WiFi)"
echo "=========================================="
ping -c 2 -I wlp3s0 8.8.8.8 > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✓ Internet working via WiFi"
else
    echo "✗ Internet not working - check WiFi connection"
fi

echo ""

# Test travel router connectivity via Ethernet
echo "=========================================="
echo "  Testing Travel Router (via Ethernet)"
echo "=========================================="

ping -c 2 192.168.8.1 > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✓ Travel router reachable at 192.168.8.1"

    echo ""
    echo "Testing Simple Rover Pi..."
    ping -c 2 192.168.8.100 > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "✓ Simple Rover Pi reachable at 192.168.8.100"
        echo ""
        echo "=========================================="
        echo "  Ready to SSH!"
        echo "=========================================="
        echo ""
        echo "Connect to Simple Rover Pi:"
        echo "  ssh jay@192.168.8.100"
        echo ""
    else
        echo "✗ Simple Rover Pi not responding at 192.168.8.100"
        echo ""
        echo "Possible reasons:"
        echo "  - Pi is not powered on"
        echo "  - Pi is not configured for 192.168.8.100"
        echo "  - Pi is not connected to travel router"
    fi
else
    echo "✗ Travel router not reachable"
    echo ""
    echo "Check:"
    echo "  - Travel router is powered on"
    echo "  - Ethernet cable is connected"
    echo "  - Travel router is using 192.168.8.1 gateway"
fi

echo ""
echo "=========================================="
echo "  Summary"
echo "=========================================="
echo ""
echo "Network Configuration:"
echo "  ✓ WiFi remains connected for internet"
echo "  ✓ Ethernet connects to travel router (no internet)"
echo "  ✓ Both networks can coexist"
echo ""
echo "You can now:"
echo "  - Browse internet via WiFi"
echo "  - SSH to rovers via ethernet"
echo "  - Both work simultaneously!"
echo ""
