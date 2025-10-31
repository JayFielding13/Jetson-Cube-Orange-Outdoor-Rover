#!/usr/bin/env python3
"""
Simple callback-only test for BlueCharm detection
Isolate callback scanning issue
"""

import asyncio
import time
import sys

try:
    from bleak import BleakScanner
    print("✅ Bleak library imported")
except ImportError:
    print("❌ Bleak library not found")
    sys.exit(1)

BLUECHARM_MAC = "DD:34:02:09:CA:1E"

detection_count = 0
last_detection_time = 0

def detection_callback(device, advertisement_data):
    global detection_count, last_detection_time
    
    print(f"📡 BLE Device detected: {device.address} | {device.name}")
    
    if device.address.upper() == BLUECHARM_MAC.upper():
        detection_count += 1
        last_detection_time = time.time()
        
        rssi = advertisement_data.rssi
        distance = rssi_to_distance(rssi)
        
        print(f"🎯 BLUECHARM FOUND! Detection #{detection_count}")
        print(f"   Address: {device.address}")
        print(f"   Name: {device.name}")
        print(f"   RSSI: {rssi} dBm")
        print(f"   Distance: ~{distance:.1f}m")
        print(f"   Time: {time.strftime('%H:%M:%S')}")
        print()

def rssi_to_distance(rssi):
    """Convert RSSI to distance"""
    if rssi <= -100:
        return 30.0
    tx_power = -59
    ratio = (tx_power - rssi) / 20.0
    distance = pow(10, ratio)
    return max(0.3, min(distance, 30.0))

async def main():
    global detection_count
    
    print("🔍 Simple Callback Scanner Test")
    print("=" * 40)
    print(f"Target: {BLUECHARM_MAC}")
    print("Looking for ANY BLE devices and specifically BlueCharm...")
    print()
    
    try:
        print("📡 Starting callback scanner...")
        scanner = BleakScanner(detection_callback)
        
        await scanner.start()
        print("✅ Scanner started - listening for 30 seconds...")
        
        start_time = time.time()
        while time.time() - start_time < 30:
            await asyncio.sleep(1)
            
            # Progress update every 5 seconds
            elapsed = int(time.time() - start_time)
            if elapsed % 5 == 0 and elapsed > 0:
                print(f"⏱️ {elapsed}s elapsed | BlueCharm detections: {detection_count}")
        
        print("📡 Stopping scanner...")
        await scanner.stop()
        
        print(f"\n✅ Test complete!")
        print(f"   Total BlueCharm detections: {detection_count}")
        
        if detection_count > 0:
            print("🎉 SUCCESS: Callback scanning works!")
        else:
            print("❌ ISSUE: No BlueCharm detected")
            print("💡 Troubleshooting:")
            print("   1. Is BlueCharm powered on?")
            print("   2. Is BlueCharm within 10 meters?")
            print("   3. Try moving BlueCharm closer")
            print("   4. Check if other BLE devices were detected above")
        
    except Exception as e:
        print(f"❌ Scanner error: {e}")

if __name__ == "__main__":
    asyncio.run(main())