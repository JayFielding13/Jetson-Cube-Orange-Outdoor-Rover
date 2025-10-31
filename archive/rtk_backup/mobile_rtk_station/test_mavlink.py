#!/usr/bin/env python3
import serial
import time

print("Testing MAVLink GPS broadcaster components...")

# Test GPS port
try:
    print("Testing GPS port /dev/ttyACM0...")
    with serial.Serial('/dev/ttyACM0', 9600, timeout=2) as gps:
        print("GPS port opened successfully")
        for i in range(5):
            line = gps.readline().decode('ascii', errors='ignore').strip()
            if line:
                print(f"GPS: {line[:50]}...")
            time.sleep(0.5)
except Exception as e:
    print(f"GPS error: {e}")

# Test SiK radio port  
try:
    print("\nTesting SiK radio port /dev/ttyUSB0...")
    with serial.Serial('/dev/ttyUSB0', 57600, timeout=2) as radio:
        print("SiK radio port opened successfully")
        radio.write(b"test message\n")
        print("Test message sent to radio")
except Exception as e:
    print(f"Radio error: {e}")

print("Test complete!")
