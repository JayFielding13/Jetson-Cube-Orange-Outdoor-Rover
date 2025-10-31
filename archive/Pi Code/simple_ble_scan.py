#!/usr/bin/env python3
"""
Simple BLE Scanner - Fixed version for BlueCharm detection
Handles different bleak library versions properly
"""

import asyncio
import sys

try:
    from bleak import BleakScanner
    print("✅ Bleak library imported")
except ImportError:
    print("❌ Bleak library not found")
    sys.exit(1)

BLUECHARM_MAC = "DD:34:02:09:CA:1E"

def get_rssi(device):
    """Get RSSI value handling different bleak versions"""
    # Try different ways to get RSSI
    if hasattr(device, 'rssi'):
        return device.rssi
    elif hasattr(device, 'metadata') and 'rssi' in device.metadata:
        return device.metadata['rssi']
    elif hasattr(device, 'details') and 'rssi' in device.details:
        return device.details['rssi']
    else:
        return -100  # Unknown RSSI

async def simple_scan():
    """Simple BLE scan with proper RSSI handling"""
    print(f"🔍 Scanning for BLE devices (looking for {BLUECHARM_MAC})...")
    print("=" * 60)
    
    try:
        devices = await BleakScanner.discover(timeout=10.0)
        print(f"📡 Found {len(devices)} BLE devices:")
        print()
        
        bluecharm_found = False
        
        for i, device in enumerate(devices, 1):
            rssi = get_rssi(device)
            name = device.name or "Unknown"
            
            # Check if this is our BlueCharm
            is_bluecharm = device.address.upper() == BLUECHARM_MAC.upper()
            marker = "🎯 BLUECHARM! " if is_bluecharm else "   "
            
            print(f"{marker}{i:2d}. {device.address} | {name:20} | {rssi:4d} dBm")
            
            if is_bluecharm:
                bluecharm_found = True
                distance = rssi_to_distance(rssi)
                print(f"     ✅ BlueCharm detected!")
                print(f"     📏 Estimated distance: ~{distance:.1f}m")
                print(f"     📶 Signal strength: {get_signal_strength(rssi)}")
        
        print()
        if bluecharm_found:
            print("✅ SUCCESS: BlueCharm found and detectable!")
        else:
            print("❌ BlueCharm not found in scan")
            print("💡 Try moving closer or checking BlueCharm power")
        
        return bluecharm_found
        
    except Exception as e:
        print(f"❌ Scan failed: {e}")
        return False

def rssi_to_distance(rssi):
    """Convert RSSI to distance estimate"""
    if rssi == 0 or rssi < -100:
        return 50.0
    
    tx_power = -59
    ratio = (tx_power - rssi) / 20.0
    distance = pow(10, ratio)
    return max(0.5, min(distance, 30.0))

def get_signal_strength(rssi):
    """Get human-readable signal strength"""
    if rssi > -30:
        return "Excellent"
    elif rssi > -50:
        return "Good" 
    elif rssi > -70:
        return "Fair"
    elif rssi > -85:
        return "Weak"
    else:
        return "Very Weak"

async def main():
    print("🔵 Simple BLE Scanner for BlueCharm")
    print("=" * 40)
    
    success = await simple_scan()
    
    if success:
        print("\n🚀 Ready to proceed with rover Bluetooth integration!")
    else:
        print("\n🔧 Troubleshooting needed before Bluetooth integration")

if __name__ == "__main__":
    asyncio.run(main())