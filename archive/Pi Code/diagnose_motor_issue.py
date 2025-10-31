#!/usr/bin/env python3
"""
Diagnostic script to test Arduino communication and motor control
"""

import serial
import json
import time

def test_arduino_communication():
    """Test basic Arduino communication"""
    print("🔍 Testing Arduino communication...")
    
    try:
        arduino = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        time.sleep(2)
        print("✅ Arduino connected")
        
        # Listen for data
        print("📡 Listening for Arduino data (10 seconds)...")
        valid_data_count = 0
        invalid_data_count = 0
        
        for i in range(100):  # 10 seconds worth
            if arduino.in_waiting > 0:
                try:
                    line = arduino.readline().decode().strip()
                    if line:
                        print(f"📋 Raw: {line}")
                        
                        if line.startswith('{'):
                            try:
                                data = json.loads(line)
                                valid_data_count += 1
                                mode = data.get('mode', 'unknown')
                                distance = data.get('distance', 'unknown')
                                valid = data.get('valid', 'unknown')
                                emergency = data.get('emergency', 'unknown')
                                print(f"✅ Parsed: mode={mode}, distance={distance}, valid={valid}, emergency={emergency}")
                            except json.JSONDecodeError:
                                invalid_data_count += 1
                                print(f"❌ JSON parse failed: {line}")
                        else:
                            print(f"ℹ️ Non-JSON: {line}")
                            
                except UnicodeDecodeError as e:
                    invalid_data_count += 1
                    print(f"❌ Unicode decode error: {e}")
                except Exception as e:
                    invalid_data_count += 1
                    print(f"❌ Read error: {e}")
            
            time.sleep(0.1)
        
        print(f"\n📊 Results: Valid data: {valid_data_count}, Invalid data: {invalid_data_count}")
        
        # Test motor command
        print("\n🚗 Testing motor command...")
        command = {'motor': {'left': 50, 'right': 50}}
        cmd_str = json.dumps(command) + '\n'
        arduino.write(cmd_str.encode())
        print(f"📤 Sent: {cmd_str.strip()}")
        
        time.sleep(2)
        
        # Stop motors
        command = {'motor': {'left': 0, 'right': 0}}
        cmd_str = json.dumps(command) + '\n'
        arduino.write(cmd_str.encode())
        print(f"📤 Sent stop: {cmd_str.strip()}")
        
        arduino.close()
        print("✅ Test complete")
        
    except Exception as e:
        print(f"❌ Arduino test failed: {e}")

def test_bluetooth_basic():
    """Test basic Bluetooth functionality"""
    print("\n🔍 Testing Bluetooth adapters...")
    
    import subprocess
    
    try:
        # Check adapters
        result = subprocess.run(['hciconfig'], capture_output=True, text=True, timeout=5)
        print("📡 Bluetooth adapters:")
        for line in result.stdout.splitlines():
            if 'hci' in line:
                print(f"   {line}")
        
        # Quick scan test
        print("\n📡 Quick scan test...")
        result = subprocess.run(['sudo', 'hcitool', 'scan'], capture_output=True, text=True, timeout=10)
        if result.stdout.strip():
            print("✅ Basic scan working:")
            for line in result.stdout.splitlines()[:5]:
                print(f"   {line}")
        else:
            print("⚠️ No devices found in basic scan")
            
    except Exception as e:
        print(f"❌ Bluetooth test failed: {e}")

def main():
    print("=" * 60)
    print("🔧 ROVER DIAGNOSTIC TEST")
    print("=" * 60)
    
    test_arduino_communication()
    test_bluetooth_basic()
    
    print("\n" + "=" * 60)
    print("💡 RECOMMENDATIONS:")
    print("1. Check power supply voltage and connections")
    print("2. Verify RC transmitter batteries and range")
    print("3. Check for electromagnetic interference")
    print("4. Ensure proper grounding of all components")
    print("=" * 60)

if __name__ == "__main__":
    main()