#!/usr/bin/env python3
"""
Mobile Beacon GPS Display for Console
Simple GPS display that writes directly to console
"""

import serial
import time
import os
import subprocess
from datetime import datetime

def clear_console():
    """Clear the console screen"""
    os.system('clear')

def get_gps_data():
    """Get current GPS data"""
    try:
        gps = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        # Read a few lines to get current data
        for _ in range(10):
            line = gps.readline().decode('ascii', errors='ignore').strip()

            if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                parts = line.split(',')
                if len(parts) >= 15 and parts[6]:  # Has fix
                    data = {}

                    # Parse latitude
                    if parts[2] and parts[3]:
                        lat_deg = float(parts[2][:2])
                        lat_min = float(parts[2][2:])
                        data['lat'] = lat_deg + lat_min/60.0
                        if parts[3] == 'S':
                            data['lat'] = -data['lat']
                    else:
                        data['lat'] = None

                    # Parse longitude
                    if parts[4] and parts[5]:
                        lon_deg = float(parts[4][:3])
                        lon_min = float(parts[4][3:])
                        data['lon'] = lon_deg + lon_min/60.0
                        if parts[5] == 'W':
                            data['lon'] = -data['lon']
                    else:
                        data['lon'] = None

                    # Parse other data
                    data['fix_type'] = int(parts[6]) if parts[6] else 0
                    data['satellites'] = int(parts[7]) if parts[7] else 0
                    data['alt'] = float(parts[9]) if parts[9] else 0.0
                    data['hdop'] = float(parts[8]) if parts[8] else 99.9
                    data['time'] = datetime.now().strftime("%H:%M:%S")

                    gps.close()
                    return data

        gps.close()
        return None

    except Exception as e:
        return None

def display_gps_status():
    """Display GPS status"""
    while True:
        clear_console()

        print("=" * 50)
        print("     MOBILE BEACON GPS STATUS")
        print("=" * 50)
        print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print()

        # Get GPS data
        gps_data = get_gps_data()

        if gps_data:
            # Fix status
            if gps_data['fix_type'] == 0:
                status = 'NO FIX'
            elif gps_data['fix_type'] == 1:
                status = 'GPS FIX'
            elif gps_data['fix_type'] == 2:
                status = 'RTK FIXED'
            elif gps_data['fix_type'] == 3:
                status = 'RTK FLOAT'
            else:
                status = 'UNKNOWN'

            print(f"GPS Status: {status}")
            print()

            # Position
            if gps_data['lat'] and gps_data['lon']:
                print(f"Latitude:  {gps_data['lat']:.8f}°")
                print(f"Longitude: {gps_data['lon']:.8f}°")
                print(f"Altitude:  {gps_data['alt']:.1f} m")
            else:
                print("Latitude:  No Fix")
                print("Longitude: No Fix")
                print("Altitude:  --")
            print()

            # Signal quality
            print(f"Satellites: {gps_data['satellites']}")
            print(f"HDOP:       {gps_data['hdop']:.1f}")

            if gps_data['hdop'] < 1.0:
                quality = "EXCELLENT"
            elif gps_data['hdop'] < 2.0:
                quality = "GOOD"
            elif gps_data['hdop'] < 5.0:
                quality = "FAIR"
            else:
                quality = "POOR"
            print(f"Quality:    {quality}")
            print()

            # RTK status
            if gps_data['fix_type'] == 2:
                print("RTK: FIXED (Centimeter accuracy)")
            elif gps_data['fix_type'] == 3:
                print("RTK: FLOAT (Sub-meter accuracy)")
            else:
                print("RTK: Not active")

        else:
            print("GPS Status: DISCONNECTED")
            print()
            print("Check GPS connection on /dev/ttyACM0")

        print()
        print("=" * 50)
        print("Press Ctrl+C to exit")
        print("=" * 50)

        # Wait 3 seconds before next update
        time.sleep(3)

if __name__ == "__main__":
    try:
        display_gps_status()
    except KeyboardInterrupt:
        clear_console()
        print("GPS Display stopped.")