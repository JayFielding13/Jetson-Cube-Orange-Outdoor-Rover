#!/bin/bash

echo "Installing GPS Display Auto-Start for Pi Touchscreen..."
echo "======================================================="

# Make scripts executable
chmod +x /home/jay/mobile_rtk_station/console_gps_display.py
chmod +x /home/jay/mobile_rtk_station/setup_gps_display_service.sh

# Create improved systemd service
echo "Creating systemd service..."
sudo tee /etc/systemd/system/gps-display.service > /dev/null << 'EOF'
[Unit]
Description=Mobile Beacon GPS Display on Console
After=multi-user.target
Wants=getty@tty1.service

[Service]
Type=simple
User=root
Group=tty
WorkingDirectory=/home/jay/mobile_rtk_station
Environment=TERM=linux
ExecStartPre=/bin/chvt 1
ExecStart=/bin/bash -c 'exec /usr/bin/python3 /home/jay/mobile_rtk_station/console_gps_display.py < /dev/tty1 > /dev/tty1 2>&1'
StandardInput=tty-force
StandardOutput=tty
StandardError=tty
TTYPath=/dev/tty1
TTYReset=yes
TTYVHangup=yes
KillMode=process
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# Create a console autologin override to show GPS display
echo "Setting up console autologin with GPS display..."
sudo mkdir -p /etc/systemd/system/getty@tty1.service.d
sudo tee /etc/systemd/system/getty@tty1.service.d/override.conf > /dev/null << 'EOF'
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin jay --noclear %I $TERM
Type=idle
EOF

# Create profile script to auto-start GPS display on console login
echo "Creating auto-start profile script..."
sudo tee /home/jay/.bash_profile > /dev/null << 'EOF'
# Auto-start GPS display if on tty1 (main console)
if [ "$TTY" = "/dev/tty1" ] || [ "$(tty)" = "/dev/tty1" ]; then
    echo "Starting GPS Display on console..."
    cd /home/jay/mobile_rtk_station
    python3 console_gps_display.py
fi
EOF

# Alternative: Create a startup script in rc.local
echo "Adding backup startup in rc.local..."
if ! grep -q "gps_display" /etc/rc.local 2>/dev/null; then
    sudo sed -i '/^exit 0/i # Start GPS Display on console\n(sleep 10 && cd /home/jay/mobile_rtk_station && python3 console_gps_display.py < /dev/tty1 > /dev/tty1 2>&1) &\n' /etc/rc.local
fi

# Enable and start the service
echo "Enabling GPS display service..."
sudo systemctl daemon-reload
sudo systemctl enable gps-display.service
sudo systemctl stop gps-display.service 2>/dev/null || true
sudo systemctl start gps-display.service

# Set proper permissions
sudo chown jay:jay /home/jay/.bash_profile
sudo chmod +x /home/jay/.bash_profile

echo ""
echo "======================================================="
echo "GPS Display Auto-Start Installation Complete!"
echo "======================================================="
echo ""
echo "The GPS display will now automatically show on the Pi"
echo "touchscreen when the system boots up."
echo ""
echo "To test immediately:"
echo "  sudo systemctl status gps-display.service"
echo ""
echo "To manually start/stop:"
echo "  sudo systemctl start gps-display.service"
echo "  sudo systemctl stop gps-display.service"
echo ""
echo "The display should appear on tty1 (main console screen)"
echo "Reboot the Pi to see the GPS display start automatically!"
echo ""