#!/usr/bin/env python3
"""
Quick Arduino communication test
"""
import serial
import time
import json

print("Testing Arduino communication...")

# Test both USB ports
ports_to_test = ['/dev/ttyUSB0', '/dev/ttyUSB1']

for port in ports_to_test:
    print(f"\n🔍 Testing {port}...")
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)
        print(f"✅ Connected to {port}")
        print("📡 Listening for data (5 seconds)...")
        print("🎛️ Try switching your RC mode switch now!")
        
        data_received = False
        for i in range(10):  # 5 seconds of listening
            if ser.in_waiting > 0:
                line = ser.readline().decode().strip()
                if line:
                    data_received = True
                    print(f"Raw: {line}")
                    
                    # Try to parse as JSON
                    if line.startswith('{'):
                        try:
                            data = json.loads(line)
                            mode = data.get('mode', 'unknown')
                            distance = data.get('distance', 'unknown')
                            valid = data.get('valid', 'unknown')
                            print(f"  → Mode: {mode}, Distance: {distance}cm, RC Valid: {valid}")
                        except:
                            print("  → JSON parse failed")
            
            time.sleep(0.5)
        
        ser.close()
        
        if data_received:
            print(f"✅ {port} - Arduino found with data!")
            break
        else:
            print(f"⚠️ {port} - Connected but no data received")
        
    except Exception as e:
        print(f"❌ {port} - Connection failed: {e}")

print("\n✅ Port test complete")