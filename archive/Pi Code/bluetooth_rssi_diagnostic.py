#!/usr/bin/env python3
"""
Bluetooth RSSI Diagnostic Tool
Diagnose and fix RSSI reading issues with BlueCharm
"""

import asyncio
import sys
import time

try:
    from bleak import BleakScanner
    print("✅ Bleak library imported")
except ImportError:
    print("❌ Bleak library not found")
    sys.exit(1)

BLUECHARM_MAC = "DD:34:02:09:CA:1E"

async def test_rssi_methods():
    """Test different methods to get RSSI values"""
    print("🔍 Testing RSSI reading methods...")
    print("=" * 50)
    
    try:
        # Longer scan to ensure good detection
        print("📡 Scanning for BlueCharm (10 second scan)...")
        devices = await BleakScanner.discover(timeout=10.0)
        
        bluecharm_device = None
        for device in devices:
            if device.address.upper() == BLUECHARM_MAC.upper():
                bluecharm_device = device
                break
        
        if not bluecharm_device:
            print("❌ BlueCharm not found in scan")
            return
        
        print(f"✅ BlueCharm found: {bluecharm_device.address}")
        print(f"   Name: {bluecharm_device.name}")
        
        print("\n🔬 Testing RSSI reading methods:")
        print("-" * 30)
        
        # Method 1: Direct rssi attribute
        try:
            rssi1 = bluecharm_device.rssi
            print(f"Method 1 (device.rssi): {rssi1} dBm")
        except AttributeError:
            print("Method 1 (device.rssi): Not available")
        
        # Method 2: Metadata
        try:
            rssi2 = bluecharm_device.metadata.get('rssi', 'Not found')
            print(f"Method 2 (metadata['rssi']): {rssi2}")
        except AttributeError:
            print("Method 2 (metadata): Not available")
        
        # Method 3: Details
        try:
            rssi3 = bluecharm_device.details.get('rssi', 'Not found')
            print(f"Method 3 (details['rssi']): {rssi3}")
        except AttributeError:
            print("Method 3 (details): Not available")
        
        # Method 4: Props
        try:
            rssi4 = bluecharm_device.props.get('RSSI', 'Not found')
            print(f"Method 4 (props['RSSI']): {rssi4}")
        except AttributeError:
            print("Method 4 (props): Not available")
        
        # Show all available attributes
        print(f"\n📋 Device attributes:")
        for attr in dir(bluecharm_device):
            if not attr.startswith('_'):
                try:
                    value = getattr(bluecharm_device, attr)
                    if not callable(value):
                        print(f"   {attr}: {value}")
                except:
                    print(f"   {attr}: <error reading>")
        
        # Show metadata if available
        if hasattr(bluecharm_device, 'metadata'):
            print(f"\n📋 Metadata contents:")
            try:
                for key, value in bluecharm_device.metadata.items():
                    print(f"   {key}: {value}")
            except:
                print(f"   metadata: {bluecharm_device.metadata}")
        
    except Exception as e:
        print(f"❌ RSSI test failed: {e}")

async def test_scanner_callback():
    """Test callback-based scanning for real-time RSSI"""
    print("\n🔄 Testing callback-based scanning...")
    print("=" * 40)
    
    detection_count = 0
    
    def detection_callback(device, advertisement_data):
        nonlocal detection_count
        if device.address.upper() == BLUECHARM_MAC.upper():
            detection_count += 1
            print(f"📡 Detection #{detection_count}:")
            print(f"   Address: {device.address}")
            print(f"   Name: {device.name}")
            
            # Try to get RSSI from advertisement data
            try:
                rssi = advertisement_data.rssi
                print(f"   RSSI (adv_data): {rssi} dBm")
            except AttributeError:
                print(f"   RSSI (adv_data): Not available")
            
            # Try direct device RSSI
            try:
                rssi = device.rssi  
                print(f"   RSSI (device): {rssi} dBm")
            except AttributeError:
                print(f"   RSSI (device): Not available")
            
            print(f"   Advertisement data: {advertisement_data}")
            print()
    
    try:
        print("Starting callback scanner for 15 seconds...")
        scanner = BleakScanner(detection_callback)
        await scanner.start()
        await asyncio.sleep(15.0)
        await scanner.stop()
        
        print(f"✅ Callback test complete. Detections: {detection_count}")
        
    except Exception as e:
        print(f"❌ Callback test failed: {e}")

async def test_multiple_scans():
    """Test multiple short scans to see RSSI consistency"""
    print("\n🔄 Testing multiple short scans...")
    print("=" * 35)
    
    rssi_readings = []
    
    for i in range(5):
        print(f"📡 Scan {i+1}/5...")
        try:
            devices = await BleakScanner.discover(timeout=3.0)
            
            for device in devices:
                if device.address.upper() == BLUECHARM_MAC.upper():
                    try:
                        rssi = device.rssi
                        rssi_readings.append(rssi)
                        print(f"   Found BlueCharm: {rssi} dBm")
                        break
                    except AttributeError:
                        print(f"   Found BlueCharm: RSSI not available")
                        break
            else:
                print("   BlueCharm not found")
            
            await asyncio.sleep(1.0)
            
        except Exception as e:
            print(f"   Scan failed: {e}")
    
    if rssi_readings:
        avg_rssi = sum(rssi_readings) / len(rssi_readings)
        min_rssi = min(rssi_readings)
        max_rssi = max(rssi_readings)
        
        print(f"\n📊 RSSI Summary:")
        print(f"   Readings: {rssi_readings}")
        print(f"   Average: {avg_rssi:.1f} dBm")
        print(f"   Range: {min_rssi} to {max_rssi} dBm")
        print(f"   Variation: {max_rssi - min_rssi} dB")
    else:
        print("❌ No valid RSSI readings obtained")

async def main():
    print("🔍 BlueCharm RSSI Diagnostic Tool")
    print("=" * 50)
    print(f"Target: {BLUECHARM_MAC}")
    print()
    
    # Test 1: RSSI reading methods
    await test_rssi_methods()
    
    # Test 2: Callback scanning
    await test_scanner_callback()
    
    # Test 3: Multiple scans
    await test_multiple_scans()
    
    print("\n✅ Diagnostic complete!")
    print("\n💡 Recommendations:")
    print("   - If all RSSI readings are -100, there's a bleak version issue")
    print("   - If callback method works, we should switch to that")
    print("   - If readings vary, the signal is working correctly")
    print("   - Try moving BlueCharm closer (1-2 meters) and retest")

if __name__ == "__main__":
    asyncio.run(main())