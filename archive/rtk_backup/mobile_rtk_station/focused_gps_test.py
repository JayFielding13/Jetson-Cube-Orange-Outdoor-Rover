#!/usr/bin/env python3
import serial
import pynmea2
import time

print("Testing GPS with exact original parameters...")

try:
    gps_port = serial.Serial('/dev/ttyACM0', 38400, timeout=1)
    print("GPS connected at 38400 baud with 1s timeout")
    
    valid_messages = 0
    for i in range(30):  # Try for 30 iterations
        line = gps_port.readline().decode('ascii', errors='ignore').strip()
        
        if line.startswith('$'):
            print(f"Raw: {line[:80]}...")
            
            if 'GGA' in line:
                try:
                    msg = pynmea2.parse(line)
                    print(f"  ✅ Valid GGA: Lat={msg.latitude}, Lon={msg.longitude}, Fix={msg.gps_qual}, Sats={msg.num_sats}")
                    valid_messages += 1
                    
                    if valid_messages >= 2:
                        print("  🎯 Got valid GPS data - ready for MAVLink!")
                        break
                        
                except Exception as e:
                    print(f"  ❌ Parse error: {e}")
        else:
            if line:
                print(f"Non-NMEA: {line[:40]}...")
    
    gps_port.close()
    print(f"\nResult: {valid_messages} valid GPS messages received")
    
except Exception as e:
    print(f"Connection error: {e}")
