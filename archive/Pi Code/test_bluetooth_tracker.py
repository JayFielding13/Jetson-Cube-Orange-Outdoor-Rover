#!/usr/bin/env python3
"""
Test script for the updated Bluetooth tracker
"""

import sys
import subprocess
import time

def test_bluetooth_adapters():
    """Test that Bluetooth adapters are working"""
    print("🔍 Testing Bluetooth adapters...")
    
    try:
        result = subprocess.run(['hciconfig'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print("✅ hciconfig working")
            adapters = []
            for line in result.stdout.splitlines():
                if 'hci' in line and ':' in line:
                    adapter = line.split(':')[0].strip()
                    adapters.append(adapter)
            
            print(f"📡 Found adapters: {adapters}")
            return adapters
        else:
            print(f"❌ hciconfig failed: {result.stderr}")
            return []
    except Exception as e:
        print(f"❌ hciconfig error: {e}")
        return []

def test_arduino_connection():
    """Test Arduino connection"""
    print("\n🔍 Testing Arduino connection...")
    
    ports = ['/dev/ttyUSB0', '/dev/ttyUSB1']
    
    for port in ports:
        try:
            import serial
            ser = serial.Serial(port, 115200, timeout=1)
            time.sleep(1)
            
            # Try to read some data
            data_found = False
            for i in range(5):
                if ser.in_waiting > 0:
                    line = ser.readline().decode().strip()
                    if line:
                        print(f"✅ {port}: {line}")
                        data_found = True
                        break
                time.sleep(0.2)
            
            ser.close()
            
            if data_found:
                print(f"✅ Arduino responding on {port}")
                return port
            else:
                print(f"⚠️ {port}: Connected but no data")
                
        except Exception as e:
            print(f"❌ {port}: {e}")
    
    return None

def test_bluetooth_scan():
    """Test a basic Bluetooth scan"""
    print("\n🔍 Testing basic Bluetooth scan...")
    
    try:
        # Try hcitool lescan
        print("📡 Testing hcitool lescan...")
        result = subprocess.run(['sudo', 'hcitool', 'lescan'], 
                              capture_output=True, text=True, timeout=8)
        
        if result.stdout.strip():
            print("✅ hcitool lescan working")
            lines = result.stdout.strip().split('\n')
            print(f"   Found {len(lines)} devices")
            # Show first few
            for i, line in enumerate(lines[:3]):
                print(f"   {i+1}: {line}")
        else:
            print("⚠️ hcitool lescan returned no output")
            
        # Try bluetoothctl
        print("\n📡 Testing bluetoothctl...")
        subprocess.run(['sudo', 'bluetoothctl', 'scan', 'on'], 
                     capture_output=True, timeout=2)
        time.sleep(3)
        subprocess.run(['sudo', 'bluetoothctl', 'scan', 'off'], 
                     capture_output=True, timeout=2)
        
        result = subprocess.run(['bluetoothctl', 'devices'], 
                              capture_output=True, text=True, timeout=5)
        
        if result.stdout.strip():
            print("✅ bluetoothctl working")
            lines = result.stdout.strip().split('\n')
            print(f"   Found {len(lines)} devices")
            for i, line in enumerate(lines[:3]):
                print(f"   {i+1}: {line}")
        else:
            print("⚠️ bluetoothctl returned no devices")
            
    except Exception as e:
        print(f"❌ Bluetooth scan error: {e}")

def main():
    print("=" * 50)
    print("🔧 BLUETOOTH TRACKER TEST")
    print("=" * 50)
    
    # Test 1: Bluetooth adapters
    adapters = test_bluetooth_adapters()
    if not adapters:
        print("❌ No Bluetooth adapters found!")
        return
    
    # Test 2: Arduino connection
    arduino_port = test_arduino_connection()
    if not arduino_port:
        print("❌ No Arduino connection found!")
        return
    
    # Test 3: Bluetooth scanning
    test_bluetooth_scan()
    
    print("\n" + "=" * 50)
    print("✅ TEST SUMMARY:")
    print(f"   Bluetooth adapters: {len(adapters)}")
    print(f"   Arduino port: {arduino_port}")
    print("   Ready to test full tracker!")
    print("=" * 50)

if __name__ == "__main__":
    main()