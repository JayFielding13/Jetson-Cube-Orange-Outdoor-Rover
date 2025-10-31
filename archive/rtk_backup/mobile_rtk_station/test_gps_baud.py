#!/usr/bin/env python3
import serial
import time

baud_rates = [9600, 38400, 115200, 57600]

for baud in baud_rates:
    print(f"\nTesting baud rate: {baud}")
    try:
        with serial.Serial('/dev/ttyACM0', baud, timeout=2) as gps:
            print(f"Connected at {baud} baud")
            for i in range(5):
                line = gps.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('$') and len(line) > 20:
                    print(f"  Clean NMEA: {line[:60]}...")
                    break
                elif line:
                    print(f"  Raw data: {line[:40]}...")
            time.sleep(0.5)
    except Exception as e:
        print(f"  Error at {baud}: {e}")
