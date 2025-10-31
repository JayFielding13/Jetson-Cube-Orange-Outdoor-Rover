#!/bin/bash

# Mobile Beacon Screen GPS Display Launcher
# This script launches the GPS display on the physical Pi screen

echo "Starting GPS Display on Physical Screen..."

# Kill any existing GPS display processes
sudo pkill -f screen_gps_display.py 2>/dev/null
sudo pkill -f live_gps_display.py 2>/dev/null

# Wait a moment
sleep 2

# Change to the script directory
cd /home/jay/mobile_rtk_station

# Make the script executable
chmod +x screen_gps_display.py

# Run the GPS display on the main console (tty1)
# This will display on the physical screen attached to the Pi
sudo python3 screen_gps_display.py < /dev/tty1 > /dev/tty1 2>&1 &

echo "GPS Display started on physical screen!"
echo "Check the Pi's touchscreen for GPS status."
echo ""
echo "To stop the display, run:"
echo "sudo pkill -f screen_gps_display.py"