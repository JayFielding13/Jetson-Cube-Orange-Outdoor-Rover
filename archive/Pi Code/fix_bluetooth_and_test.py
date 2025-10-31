#!/usr/bin/env python3
"""
Fix Bluetooth Issues and Test BlueCharm Detection
Handles common Linux Bluetooth scanning problems
"""

import asyncio
import subprocess
import time
import sys
import os

try:
    from bleak import BleakScanner
    print("✅ Bleak library imported successfully")
except ImportError:
    print("❌ Bleak library not found. Install with: pip install bleak")
    sys.exit(1)

# BlueCharm configuration
BLUECHARM_MAC = "DD:34:02:09:CA:1E"

def run_command(cmd, check_output=False):
    """Run a system command safely"""
    try:
        if check_output:
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
            return result.returncode == 0, result.stdout.strip()
        else:
            result = subprocess.run(cmd, shell=True, timeout=10)
            return result.returncode == 0, ""
    except subprocess.TimeoutExpired:
        print(f"⚠️ Command timed out: {cmd}")
        return False, ""
    except Exception as e:
        print(f"⚠️ Command failed: {cmd} - {e}")
        return False, ""

def check_bluetooth_status():
    """Check Bluetooth adapter status"""
    print("🔍 Checking Bluetooth status...")
    
    # Check if Bluetooth service is running
    success, output = run_command("systemctl is-active bluetooth", check_output=True)
    print(f"   Bluetooth service: {'✅ Active' if success else '❌ Inactive'}")
    
    # Check if adapter is up
    success, output = run_command("hciconfig hci0", check_output=True)
    if success and "UP RUNNING" in output:
        print("   Bluetooth adapter: ✅ UP and RUNNING")
    else:
        print("   Bluetooth adapter: ⚠️ Not ready")
        return False
    
    # Check for ongoing scans
    success, output = run_command("hcitool lescan &", check_output=False)
    time.sleep(1)
    run_command("sudo pkill hcitool", check_output=False)
    
    return True

def reset_bluetooth_adapter():
    """Reset Bluetooth adapter to clear stuck states"""
    print("🔄 Resetting Bluetooth adapter...")
    
    commands = [
        "sudo hciconfig hci0 down",
        "sudo hciconfig hci0 up", 
        "sudo systemctl restart bluetooth"
    ]
    
    for cmd in commands:
        print(f"   Running: {cmd}")
        success, _ = run_command(cmd)
        if not success:
            print(f"   ⚠️ Failed: {cmd}")
        time.sleep(2)
    
    # Wait for service to stabilize
    print("   Waiting for Bluetooth to stabilize...")
    time.sleep(5)

def kill_competing_processes():
    """Kill processes that might interfere with BLE scanning"""
    print("🧹 Cleaning up competing Bluetooth processes...")
    
    processes_to_kill = [
        "hcitool",
        "bluetoothctl", 
        "gatttool",
        "hcidump"
    ]
    
    for process in processes_to_kill:
        cmd = f"sudo pkill -f {process}"
        success, _ = run_command(cmd)
        if success:
            print(f"   Killed: {process}")
    
    time.sleep(2)

async def simple_ble_scan():
    """Simple BLE scan with error handling"""
    print("🔍 Attempting simple BLE scan...")
    
    try:
        # Very short scan to test basic functionality
        devices = await BleakScanner.discover(timeout=3.0)
        print(f"   Found {len(devices)} BLE devices")
        
        for device in devices:
            print(f"   {device.address} | {device.name or 'No name'} | {device.rssi} dBm")
            
            if device.address.upper() == BLUECHARM_MAC.upper():
                print(f"🎯 FOUND BLUECHARM! {device.address} | RSSI: {device.rssi} dBm")
                return True
        
        print(f"❌ BlueCharm not found in scan")
        return False
        
    except Exception as e:
        print(f"❌ BLE scan failed: {e}")
        return False

async def robust_bluecharm_detection():
    """Robust BlueCharm detection with multiple attempts"""
    print("🎯 Starting robust BlueCharm detection...")
    
    max_attempts = 3
    for attempt in range(1, max_attempts + 1):
        print(f"🔄 Attempt {attempt}/{max_attempts}")
        
        try:
            # Try progressively longer scans
            timeout = 2.0 + (attempt * 2.0)
            print(f"   Scanning for {timeout}s...")
            
            devices = await BleakScanner.discover(timeout=timeout)
            
            bluecharm_devices = []
            for device in devices:
                if device.address.upper() == BLUECHARM_MAC.upper():
                    bluecharm_devices.append(device)
            
            if bluecharm_devices:
                device = bluecharm_devices[0]
                distance = rssi_to_distance(device.rssi)
                print(f"✅ SUCCESS! BlueCharm detected:")
                print(f"   MAC: {device.address}")
                print(f"   Name: {device.name or 'No name'}")
                print(f"   RSSI: {device.rssi} dBm")
                print(f"   Estimated distance: ~{distance:.1f}m")
                return True
            else:
                print(f"   No BlueCharm found in {len(devices)} devices")
        
        except Exception as e:
            print(f"   Scan failed: {e}")
        
        if attempt < max_attempts:
            print("   Waiting before retry...")
            await asyncio.sleep(3)
    
    print("❌ BlueCharm not detected after all attempts")
    return False

def rssi_to_distance(rssi):
    """Convert RSSI to distance estimate"""
    if rssi == 0:
        return 50.0
    
    tx_power = -59
    ratio = (tx_power - rssi) / 20.0
    distance = pow(10, ratio)
    return max(0.5, min(distance, 30.0))

async def main():
    """Main function with comprehensive Bluetooth troubleshooting"""
    print("🔵 BlueCharm BLE Detection Troubleshooter")
    print("=" * 50)
    print(f"Target BlueCharm MAC: {BLUECHARM_MAC}")
    print()
    
    # Check if running as root (sometimes needed for BLE)
    if os.geteuid() != 0:
        print("ℹ️ Running as regular user (not root)")
    else:
        print("ℹ️ Running as root")
    
    # Step 1: Check Bluetooth status
    if not check_bluetooth_status():
        print("🔧 Bluetooth adapter issues detected, attempting reset...")
        reset_bluetooth_adapter()
        
        if not check_bluetooth_status():
            print("❌ Bluetooth adapter still not ready. Manual intervention needed:")
            print("   1. sudo systemctl restart bluetooth")
            print("   2. sudo hciconfig hci0 down && sudo hciconfig hci0 up")
            print("   3. Check 'dmesg | grep bluetooth' for hardware errors")
            return
    
    # Step 2: Clean up competing processes
    kill_competing_processes()
    
    # Step 3: Test basic BLE scanning
    print("\n📡 Testing basic BLE functionality...")
    basic_success = await simple_ble_scan()
    
    if not basic_success:
        print("\n🔧 Basic scan failed, trying adapter reset...")
        reset_bluetooth_adapter()
        await asyncio.sleep(3)
        basic_success = await simple_ble_scan()
    
    # Step 4: Robust BlueCharm detection
    print("\n🎯 Attempting BlueCharm detection...")
    success = await robust_bluecharm_detection()
    
    if success:
        print("\n✅ BlueCharm detection SUCCESSFUL!")
        print("   You can now run the rover navigator with Bluetooth enabled")
    else:
        print("\n❌ BlueCharm detection FAILED")
        print("   Troubleshooting suggestions:")
        print("   1. Check BlueCharm battery level")
        print("   2. Move BlueCharm closer (< 5 meters)")
        print("   3. Check for WiFi interference (try different channel)")
        print("   4. Verify MAC address: DD:34:02:09:CA:1E")
        print("   5. Try: sudo bluetoothctl -> scan on")

if __name__ == "__main__":
    asyncio.run(main())