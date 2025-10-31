#!/usr/bin/env python3
import serial
import time
import os
from datetime import datetime

def clear_screen():
    os.system('clear')

def get_gps_data():
    try:
        gps = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        
        while True:
            clear_screen()
            print('=' * 60)
            print('          MOBILE BEACON GPS STATUS')
            print('=' * 60)
            
            # Read GPS data
            for i in range(10):
                line = gps.readline().decode('ascii', errors='ignore').strip()
                if 'GGA' in line and line.startswith('$'):
                    parts = line.split(',')
                    if len(parts) >= 15:
                        # Parse coordinates
                        if parts[2] and parts[3] and parts[4] and parts[5]:
                            try:
                                lat_deg = float(parts[2][:2])
                                lat_min = float(parts[2][2:])
                                lat = lat_deg + lat_min/60.0
                                if parts[3] == 'S':
                                    lat = -lat
                                
                                lon_deg = float(parts[4][:3])  
                                lon_min = float(parts[4][3:])
                                lon = lon_deg + lon_min/60.0
                                if parts[5] == 'W':
                                    lon = -lon
                                
                                fix_type = int(parts[6]) if parts[6] else 0
                                satellites = int(parts[7]) if parts[7] else 0
                                alt = float(parts[9]) if parts[9] else 0.0
                                hdop = float(parts[8]) if parts[8] else 99.9
                                
                                # Display status
                                if fix_type == 0:
                                    status = 'NO FIX'
                                elif fix_type == 1:
                                    status = 'GPS FIX'
                                elif fix_type == 2:
                                    status = 'RTK FIXED'
                                elif fix_type == 3:
                                    status = 'RTK FLOAT'
                                else:
                                    status = 'UNKNOWN'
                                
                                print(f'GPS Status: {status}')
                                print(f'Latitude:  {lat:.8f}°')
                                print(f'Longitude: {lon:.8f}°') 
                                print(f'Altitude:  {alt:.1f}m')
                                print(f'Satellites: {satellites}')
                                print(f'HDOP:      {hdop:.1f}')
                                print(f'Time:      {datetime.now().strftime("%H:%M:%S")}')
                                print('')
                                print('Press Ctrl+C to exit')
                                break
                            except:
                                print('Parsing GPS data...')
                        else:
                            print('Waiting for GPS fix...')
                        break
                time.sleep(0.1)
            
            time.sleep(2)
            
    except KeyboardInterrupt:
        print('\nExiting...')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'gps' in locals():
            gps.close()

if __name__ == '__main__':
    get_gps_data()
