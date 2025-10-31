#!/bin/bash

echo "Setting up GPS Display Service for Pi Screen..."

# Create systemd service file
sudo tee /etc/systemd/system/gps-display.service > /dev/null << 'EOF'
[Unit]
Description=Mobile Beacon GPS Display
After=multi-user.target

[Service]
Type=simple
User=jay
WorkingDirectory=/home/jay/mobile_rtk_station
ExecStart=/usr/bin/python3 /home/jay/mobile_rtk_station/console_gps_display.py
StandardOutput=tty
StandardError=tty
TTYPath=/dev/tty1
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# Make the GPS display script executable
chmod +x /home/jay/mobile_rtk_station/console_gps_display.py

# Reload systemd and enable the service
sudo systemctl daemon-reload
sudo systemctl enable gps-display.service

echo "GPS Display service created!"
echo ""
echo "To start the service now:"
echo "sudo systemctl start gps-display.service"
echo ""
echo "To check service status:"
echo "sudo systemctl status gps-display.service"
echo ""
echo "To stop the service:"
echo "sudo systemctl stop gps-display.service"