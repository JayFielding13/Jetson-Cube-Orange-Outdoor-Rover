#!/bin/bash
# WireGuard VPN Setup Script for Remote Rover Control
# Run this script with: sudo bash wireguard_setup.sh

echo "Setting up WireGuard VPN for remote rover control..."

# Install WireGuard
apt update
apt install -y wireguard wireguard-tools

# Create WireGuard directory
mkdir -p /etc/wireguard
cd /etc/wireguard

# Generate server private and public keys
wg genkey | tee server_private.key | wg pubkey > server_public.key
chmod 600 server_private.key

# Generate client private and public keys
wg genkey | tee client_private.key | wg pubkey > client_public.key
chmod 600 client_private.key

# Get server keys
SERVER_PRIVATE=$(cat server_private.key)
SERVER_PUBLIC=$(cat server_public.key)
CLIENT_PRIVATE=$(cat client_private.key)
CLIENT_PUBLIC=$(cat client_public.key)

# Create server configuration
cat > /etc/wireguard/wg0.conf << EOF
[Interface]
# Base Station WireGuard Server
PrivateKey = $SERVER_PRIVATE
Address = 10.8.0.1/24
ListenPort = 51820
PostUp = iptables -A FORWARD -i %i -j ACCEPT; iptables -A FORWARD -o %i -j ACCEPT; iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
PostDown = iptables -D FORWARD -i %i -j ACCEPT; iptables -D FORWARD -o %i -j ACCEPT; iptables -t nat -D POSTROUTING -o eth0 -j MASQUERADE

[Peer]
# Remote Friend Client
PublicKey = $CLIENT_PUBLIC
AllowedIPs = 10.8.0.2/32
EOF

# Create client configuration
cat > /home/jay/friend_client.conf << EOF
[Interface]
# Remote Friend WireGuard Client
PrivateKey = $CLIENT_PRIVATE
Address = 10.8.0.2/24
DNS = 8.8.8.8

[Peer]
# Base Station Server
PublicKey = $SERVER_PUBLIC
Endpoint = YOUR_PUBLIC_IP:51820
AllowedIPs = 10.8.0.0/24, 192.168.8.0/24
PersistentKeepalive = 25
EOF

# Enable IP forwarding
echo 'net.ipv4.ip_forward=1' >> /etc/sysctl.conf
sysctl -p

# Configure firewall
ufw allow 51820/udp
ufw allow from 10.8.0.0/24 to any port 5760
ufw allow from 10.8.0.0/24 to any port 5761
ufw allow from 10.8.0.0/24 to any port 2101

# Enable and start WireGuard
systemctl enable wg-quick@wg0
systemctl start wg-quick@wg0

echo "WireGuard setup complete!"
echo "Server configuration: /etc/wireguard/wg0.conf"
echo "Client configuration: /home/jay/friend_client.conf"
echo ""
echo "Next steps:"
echo "1. Find your public IP: curl ifconfig.me"
echo "2. Edit /home/jay/friend_client.conf and replace YOUR_PUBLIC_IP with actual IP"
echo "3. Forward port 51820 UDP on your router to this computer"
echo "4. Send friend_client.conf to your friend"
echo ""
echo "WireGuard Status:"
systemctl status wg-quick@wg0
echo ""
echo "Connected clients:"
wg show