#!/usr/bin/env python3
"""
Mobile Beacon GPS Display for Physical Screen
Displays GPS status directly to console/framebuffer
"""

import serial
import time
import sys
import os
from datetime import datetime

class ScreenGPSDisplay:
    def __init__(self):
        self.gps_port = '/dev/ttyACM0'
        self.gps_baud = 115200
        self.running = True

        # GPS data storage
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.satellites = 0
        self.fix_type = 0
        self.hdop = 99.9
        self.last_update = "Never"

        # Clear screen and setup
        self.setup_display()

    def setup_display(self):
        """Setup console display"""
        # Force output to console
        sys.stdout = open('/dev/console', 'w') if os.path.exists('/dev/console') else sys.stdout
        # Clear screen
        print('\033[2J\033[H', end='', flush=True)
        # Hide cursor
        print('\033[?25l', end='', flush=True)

    def clear_screen(self):
        """Clear the screen"""
        print('\033[2J\033[H', end='', flush=True)

    def display_status(self):
        """Display GPS status on screen"""
        self.clear_screen()

        # Header
        print("=" * 60)
        print("        MOBILE BEACON GPS LIVE MONITOR")
        print("=" * 60)
        print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print()

        # GPS Status with color
        if self.fix_type == 0:
            status = 'NO FIX'
            status_color = '\033[91m'  # Red
        elif self.fix_type == 1:
            status = 'GPS FIX'
            status_color = '\033[93m'  # Yellow
        elif self.fix_type == 2:
            status = 'RTK FIXED'
            status_color = '\033[92m'  # Green
        elif self.fix_type == 3:
            status = 'RTK FLOAT'
            status_color = '\033[96m'  # Cyan
        else:
            status = 'UNKNOWN'
            status_color = '\033[90m'  # Gray

        print(f"GPS Status: {status_color}{status}\033[0m")
        print()

        # Position
        print("POSITION:")
        if self.fix_type > 0:
            print(f"  Latitude:   {self.lat:.8f}°")
            print(f"  Longitude: {self.lon:.8f}°")
            print(f"  Altitude:     {self.alt:.1f} m")
        else:
            print("  Latitude:   No Fix")
            print("  Longitude:  No Fix")
            print("  Altitude:   --")
        print()

        # Signal Quality
        print("SIGNAL QUALITY:")
        print(f"  Satellites: {self.satellites}")
        print(f"  HDOP:       {self.hdop:.1f}")

        if self.hdop < 1.0:
            quality = "EXCELLENT"
        elif self.hdop < 2.0:
            quality = "GOOD"
        elif self.hdop < 5.0:
            quality = "FAIR"
        else:
            quality = "POOR"
        print(f"  Quality:    {quality}")
        print()

        # RTK Status
        print("RTK STATUS:")
        if self.fix_type == 2:
            print("  RTK Active:  YES")
            print("  Accuracy:    Centimeter-level")
        elif self.fix_type == 3:
            print("  RTK Active:  FLOAT")
            print("  Accuracy:    Sub-meter")
        else:
            print("  RTK Active:  NO")
            print("  Accuracy:    Standard GPS")
        print()

        # System Info
        print("SYSTEM:")
        print(f"  Last Update: {self.last_update}")
        print(f"  GPS Port:    {self.gps_port}")
        print(f"  Baud Rate:   {self.gps_baud}")
        print()

        print("=" * 60)
        print("Press Ctrl+C to exit")
        print("=" * 60)

        # Flush output to ensure it appears
        sys.stdout.flush()

    def parse_gps_data(self):
        """Parse GPS NMEA data"""
        try:
            gps = serial.Serial(self.gps_port, self.gps_baud, timeout=1)
            print(f"Connected to GPS on {self.gps_port}")

            while self.running:
                try:
                    line = gps.readline().decode('ascii', errors='ignore').strip()

                    if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                        parts = line.split(',')
                        if len(parts) >= 15:
                            # Parse latitude
                            if parts[2] and parts[3]:
                                lat_deg = float(parts[2][:2])
                                lat_min = float(parts[2][2:])
                                self.lat = lat_deg + lat_min/60.0
                                if parts[3] == 'S':
                                    self.lat = -self.lat

                            # Parse longitude
                            if parts[4] and parts[5]:
                                lon_deg = float(parts[4][:3])
                                lon_min = float(parts[4][3:])
                                self.lon = lon_deg + lon_min/60.0
                                if parts[5] == 'W':
                                    self.lon = -self.lon

                            # Parse other data
                            self.fix_type = int(parts[6]) if parts[6] else 0
                            self.satellites = int(parts[7]) if parts[7] else 0
                            self.alt = float(parts[9]) if parts[9] else 0.0
                            self.hdop = float(parts[8]) if parts[8] else 99.9
                            self.last_update = datetime.now().strftime("%H:%M:%S")

                            # Update display
                            self.display_status()

                except Exception as e:
                    continue

                time.sleep(2)  # Update every 2 seconds

        except Exception as e:
            self.clear_screen()
            print(f"GPS connection error: {e}")
            print("Make sure GPS is connected to /dev/ttyACM0")
            sys.stdout.flush()

    def run(self):
        """Start the display"""
        try:
            self.parse_gps_data()
        except KeyboardInterrupt:
            self.cleanup()

    def cleanup(self):
        """Clean up display"""
        self.running = False
        # Show cursor
        print('\033[?25h', end='', flush=True)
        self.clear_screen()
        print("GPS Display stopped.")

if __name__ == "__main__":
    display = ScreenGPSDisplay()
    display.run()