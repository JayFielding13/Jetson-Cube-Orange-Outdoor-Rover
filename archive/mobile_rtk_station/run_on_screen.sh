#!/bin/bash

echo "Starting GPS Display on Pi Screen..."

# Kill any existing displays
sudo pkill -f gps_display 2>/dev/null || true
sleep 1

# Make the console script executable
chmod +x /home/jay/mobile_rtk_station/console_gps_display.py

# Option 1: Run on main console (what user sees on screen)
echo "Running GPS display on console..."
sudo -u jay python3 /home/jay/mobile_rtk_station/console_gps_display.py

echo "GPS Display finished."