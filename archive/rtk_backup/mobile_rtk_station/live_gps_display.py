#!/usr/bin/env python3
import serial
import time
import os
from datetime import datetime

def clear_screen():
    os.system('clear')

def main():
    try:
        gps = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        
        # Initial values
        lat, lon, alt = 0.0, 0.0, 0.0
        fix_type, satellites, hdop = 0, 0, 99.9
        speed_knots = 0.0
        last_update = 'Never'
        
        print('Starting Mobile Beacon GPS Monitor...')
        time.sleep(2)
        
        while True:
            clear_screen()
            
            # Header
            print('=' * 60)
            print('        MOBILE BEACON GPS LIVE MONITOR')
            print('=' * 60)
            print(f'Time: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
            print()
            
            # Read GPS data
            try:
                for _ in range(20):  # Try to get fresh data
                    line = gps.readline().decode('ascii', errors='ignore').strip()
                    
                    if line.startswith('') or line.startswith(''):
                        parts = line.split(',')
                        if len(parts) >= 15 and parts[6]:
                            try:
                                # Parse latitude
                                if parts[2] and parts[3]:
                                    lat_deg = float(parts[2][:2])
                                    lat_min = float(parts[2][2:])
                                    lat = lat_deg + lat_min/60.0
                                    if parts[3] == 'S':
                                        lat = -lat
                                
                                # Parse longitude
                                if parts[4] and parts[5]:
                                    lon_deg = float(parts[4][:3])
                                    lon_min = float(parts[4][3:])
                                    lon = lon_deg + lon_min/60.0
                                    if parts[5] == 'W':
                                        lon = -lon
                                
                                fix_type = int(parts[6]) if parts[6] else 0
                                satellites = int(parts[7]) if parts[7] else 0
                                alt = float(parts[9]) if parts[9] else 0.0
                                hdop = float(parts[8]) if parts[8] else 99.9
                                last_update = datetime.now().strftime("%H:%M:%S")
                            except:
                                pass
                    
                    elif line.startswith('') or line.startswith(''):
                        parts = line.split(',')
                        if len(parts) >= 8 and parts[7]:
                            try:
                                speed_knots = float(parts[7]) if parts[7] else 0.0
                            except:
                                pass
                        break
            except:
                pass
            
            # Display GPS Status
            if fix_type == 0:
                status = 'NO FIX'
                status_color = '[RED]'
            elif fix_type == 1:
                status = 'GPS FIX'
                status_color = '[YELLOW]'
            elif fix_type == 2:
                status = 'RTK FIXED'
                status_color = '[GREEN]'
            elif fix_type == 3:
                status = 'RTK FLOAT'
                status_color = '[CYAN]'
            else:
                status = 'UNKNOWN'
                status_color = '[GRAY]'
            
            print(f'GPS Status: {status_color} {status}')
            print()
            
            # Position Information
            print('POSITION:')
            print(f'  Latitude:  {lat:12.8f}°')
            print(f'  Longitude: {lon:12.8f}°')
            print(f'  Altitude:  {alt:8.1f} m')
            print()
            
            # Quality Information  
            print('SIGNAL QUALITY:')
            print(f'  Satellites: {satellites:2d}')
            print(f'  HDOP:       {hdop:5.1f}')
            if hdop < 1.0:
                hdop_quality = 'EXCELLENT'
            elif hdop < 2.0:
                hdop_quality = 'GOOD'
            elif hdop < 5.0:
                hdop_quality = 'FAIR'
            else:
                hdop_quality = 'POOR'
            print(f'  Quality:    {hdop_quality}')
            print()
            
            # Movement Information
            speed_mph = speed_knots * 1.15078  # Convert knots to mph
            speed_kph = speed_knots * 1.852    # Convert knots to km/h
            print('MOVEMENT:')
            print(f'  Speed:      {speed_knots:5.1f} knots')
            print(f'              {speed_mph:5.1f} mph')
            print(f'              {speed_kph:5.1f} km/h')
            print()
            
            # System Information
            print('SYSTEM:')
            print(f'  Last Update: {last_update}')
            print(f'  GPS Port:    /dev/ttyACM0')
            print(f'  Baud Rate:   115200')
            print()
            
            # RTK Information
            print('RTK STATUS:')
            if fix_type >= 2:
                print('  RTK Active:  YES')
                print('  Accuracy:    Centimeter-level')
            elif fix_type == 1:
                print('  RTK Active:  NO')
                print('  Accuracy:    Meter-level')
            else:
                print('  RTK Active:  NO')
                print('  Accuracy:    No position fix')
            print()
            
            print('=' * 60)
            print('Press Ctrl+C to exit')
            
            # Update every 2 seconds
            time.sleep(2)
            
    except KeyboardInterrupt:
        print('\n\nGPS Monitor stopped.')
    except Exception as e:
        print(f'\nError: {e}')
        print('Check GPS connection and try again.')
    finally:
        if 'gps' in locals():
            gps.close()

if __name__ == '__main__':
    main()
