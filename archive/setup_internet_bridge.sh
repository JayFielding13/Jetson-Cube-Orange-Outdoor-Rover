#!/bin/bash

echo "Setting up Internet Bridge from Laptop to Pi 5..."
echo "This will allow the Pi 5 to access internet through this laptop"
echo ""

# Enable IP forwarding
echo "Enabling IP forwarding..."
sudo sysctl net.ipv4.ip_forward=1

# Set up NAT masquerading from ethernet to WiFi
echo "Setting up NAT masquerading..."
sudo iptables -t nat -A POSTROUTING -o wlp3s0 -j MASQUERADE
sudo iptables -A FORWARD -i enp5s0 -o wlp3s0 -j ACCEPT
sudo iptables -A FORWARD -i wlp3s0 -o enp5s0 -m state --state RELATED,ESTABLISHED -j ACCEPT

echo ""
echo "Internet bridge configured!"
echo ""
echo "Now configuring Pi 5 DNS and gateway..."

# Configure Pi 5 to use this laptop as gateway
ssh jay@192.168.8.201 "sudo ip route del default 2>/dev/null || true"
ssh jay@192.168.8.201 "sudo ip route add default via 192.168.8.182"

# Set DNS on Pi 5
ssh jay@192.168.8.201 "echo 'nameserver 8.8.8.8' | sudo tee /etc/resolv.conf"

echo ""
echo "Testing Pi 5 internet connectivity..."
ssh jay@192.168.8.201 "ping -c 2 8.8.8.8"

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Internet bridge successful!"
    echo "Pi 5 now has internet access through this laptop"
    echo ""
    echo "You can now run the RTK installer on Pi 5:"
    echo "ssh jay@192.168.8.201"
    echo "./install_rtk_desktop_pi5.sh"
else
    echo ""
    echo "❌ Internet bridge failed - troubleshooting needed"
fi

echo ""
echo "To disable bridge later:"
echo "sudo iptables -t nat -D POSTROUTING -o wlp3s0 -j MASQUERADE"
echo "sudo iptables -D FORWARD -i enp5s0 -o wlp3s0 -j ACCEPT"
echo "sudo iptables -D FORWARD -i wlp3s0 -o enp5s0 -m state --state RELATED,ESTABLISHED -j ACCEPT"