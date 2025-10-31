#!/usr/bin/env python3
"""
BlueCharm BLE Detection Test
Test script to verify BLE scanning and detection of specific BlueCharm device
MAC: DD:34:02:09:CA:1E
"""

import asyncio
import time
import sys

try:
    from bleak import BleakScanner
    print("✅ Bleak library imported successfully")
except ImportError:
    print("❌ Bleak library not found. Install with: pip install bleak")
    sys.exit(1)

# BlueCharm specific configuration
BLUECHARM_MAC = "DD:34:02:09:CA:1E"
BLUECHARM_NAME_KEYWORDS = ["bluecharm", "blue charm", "charm"]

class BlueCharmDetector:
    def __init__(self):
        self.target_mac = BLUECHARM_MAC.upper()
        self.detection_count = 0
        self.last_rssi = None
        self.last_detection = 0
        self.rssi_history = []
        
    def rssi_to_distance(self, rssi):
        """Convert RSSI to approximate distance for BLE beacon"""
        if rssi == 0:
            return 50.0
        
        # BLE beacon distance estimation
        # TX power for typical BLE beacon at 1 meter
        tx_power = -59
        ratio = (tx_power - rssi) / 20.0
        distance = pow(10, ratio)
        
        return max(0.5, min(distance, 50.0))
    
    def process_detection(self, device):
        """Process a BlueCharm detection"""
        self.detection_count += 1
        self.last_rssi = device.rssi
        self.last_detection = time.time()
        
        # Track RSSI history for stability analysis
        self.rssi_history.append(device.rssi)
        if len(self.rssi_history) > 10:
            self.rssi_history.pop(0)
        
        distance = self.rssi_to_distance(device.rssi)
        avg_rssi = sum(self.rssi_history) / len(self.rssi_history)
        rssi_stability = max(self.rssi_history) - min(self.rssi_history)
        
        print(f"📡 DETECTION #{self.detection_count}:")
        print(f"   MAC: {device.address}")
        print(f"   Name: {device.name or 'No name'}")
        print(f"   RSSI: {device.rssi} dBm")
        print(f"   Distance: ~{distance:.1f}m")
        print(f"   Avg RSSI: {avg_rssi:.1f} dBm (stability: ±{rssi_stability/2:.1f})")
        print(f"   Time: {time.strftime('%H:%M:%S')}")
        print()
    
    def is_bluecharm(self, device):
        """Check if device is our target BlueCharm"""
        # Primary check: MAC address match
        if device.address.upper() == self.target_mac:
            return True
        
        # Secondary check: Name contains BlueCharm keywords
        if device.name:
            name_lower = device.name.lower()
            for keyword in BLUECHARM_NAME_KEYWORDS:
                if keyword in name_lower:
                    print(f"⚠️ Found device with BlueCharm name but different MAC:")
                    print(f"   Expected: {self.target_mac}")
                    print(f"   Found: {device.address.upper()}")
                    return True
        
        return False

async def scan_for_bluecharm(detector, scan_duration=5.0):
    """Scan for BlueCharm device"""
    print(f"🔍 Scanning for BlueCharm BLE device...")
    print(f"   Target MAC: {BLUECHARM_MAC}")
    print(f"   Scan duration: {scan_duration}s")
    print()
    
    try:
        # Scan for BLE devices
        devices = await BleakScanner.discover(timeout=scan_duration)
        
        bluecharm_found = False
        all_devices = []
        
        for device in devices:
            all_devices.append({
                'name': device.name or 'Unknown',
                'address': device.address,
                'rssi': device.rssi
            })
            
            if detector.is_bluecharm(device):
                bluecharm_found = True
                detector.process_detection(device)
        
        if not bluecharm_found:
            print(f"❌ BlueCharm not found in scan")
            print(f"📋 Found {len(all_devices)} BLE devices:")
            for dev in sorted(all_devices, key=lambda x: x['rssi'], reverse=True):
                print(f"   {dev['name'][:20]:20} | {dev['address']} | {dev['rssi']:3d} dBm")
            print()
        
        return bluecharm_found
        
    except Exception as e:
        print(f"❌ Scan error: {e}")
        return False

async def continuous_detection_test():
    """Run continuous detection test"""
    detector = BlueCharmDetector()
    scan_count = 0
    successful_detections = 0
    
    print("🚀 Starting continuous BlueCharm detection test")
    print("   Press Ctrl+C to stop")
    print("=" * 60)
    print()
    
    try:
        while True:
            scan_count += 1
            print(f"📡 Scan #{scan_count} - {time.strftime('%H:%M:%S')}")
            
            found = await scan_for_bluecharm(detector, scan_duration=2.0)
            if found:
                successful_detections += 1
            
            # Statistics every 10 scans
            if scan_count % 10 == 0:
                success_rate = (successful_detections / scan_count) * 100
                print(f"📊 STATISTICS (after {scan_count} scans):")
                print(f"   Success rate: {success_rate:.1f}%")
                print(f"   Total detections: {detector.detection_count}")
                print(f"   Last RSSI: {detector.last_rssi} dBm")
                if detector.rssi_history:
                    stability = max(detector.rssi_history) - min(detector.rssi_history)
                    print(f"   RSSI stability: ±{stability/2:.1f} dBm")
                print()
            
            # Wait between scans
            await asyncio.sleep(1.0)
            
    except KeyboardInterrupt:
        print(f"\n🛑 Test stopped by user")
        print(f"📊 FINAL STATISTICS:")
        print(f"   Total scans: {scan_count}")
        print(f"   Successful detections: {successful_detections}")
        print(f"   Success rate: {(successful_detections/scan_count)*100:.1f}%")
        print(f"   Total detection events: {detector.detection_count}")

async def single_scan_test():
    """Run a single comprehensive scan"""
    detector = BlueCharmDetector()
    
    print("🔍 Single scan test for BlueCharm")
    print("=" * 40)
    print()
    
    found = await scan_for_bluecharm(detector, scan_duration=10.0)
    
    if found:
        print("✅ BlueCharm detection successful!")
        distance = detector.rssi_to_distance(detector.last_rssi)
        print(f"   Distance: ~{distance:.1f}m")
        print(f"   RSSI: {detector.last_rssi} dBm")
    else:
        print("❌ BlueCharm not detected")
        print("   Check that:")
        print("   1. BlueCharm is powered on")
        print("   2. BlueCharm is within range (< 10m)")
        print("   3. No interference sources nearby")
        print("   4. MAC address is correct")

def main():
    print("🔵 BlueCharm BLE Detection Test")
    print("=" * 50)
    print(f"Target MAC: {BLUECHARM_MAC}")
    print()
    
    if len(sys.argv) > 1 and sys.argv[1] == "continuous":
        asyncio.run(continuous_detection_test())
    else:
        print("Running single scan test...")
        print("Use 'python test_bluecharm_detection.py continuous' for continuous testing")
        print()
        asyncio.run(single_scan_test())

if __name__ == "__main__":
    main()