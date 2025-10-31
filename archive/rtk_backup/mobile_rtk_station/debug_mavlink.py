#!/usr/bin/env python3
import serial
import time
import pynmea2

print("Starting Mobile RTK Station Debug...")

try:
    print("Opening GPS port /dev/ttyACM0...")
    with serial.Serial('/dev/ttyACM0', 9600, timeout=1) as gps:
        print("GPS connected successfully")
        
        # Read GPS data for a while
        for i in range(20):
            line = gps.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('$'):
                print(f"Raw NMEA: {line}")
                
                if 'GGA' in line:
                    try:
                        msg = pynmea2.parse(line)
                        print(f"  Parsed GGA: Lat={msg.latitude}, Lon={msg.longitude}, Fix={msg.gps_qual}, Sats={msg.num_sats}")
                        
                        if msg.latitude and msg.longitude:
                            print(f"  Valid position found! Ready for MAVLink")
                            
                    except Exception as e:
                        print(f"  Parse error: {e}")
                        
            time.sleep(0.5)
            
except Exception as e:
    print(f"Error: {e}")

print("Debug complete")
