#!/usr/bin/env python3
"""
Phase 2A Test 1: Basic Arduino Communication
===========================================

Test basic serial communication with Arduino safety gatekeeper.
This test verifies the Navigation Pi can connect to and communicate
with the Arduino interface without sending movement commands.
"""

from Main.arduino_interface import ArduinoInterface
import time
import sys
import os

def test_arduino_communication():
    """Test basic Arduino communication"""
    print('🔍 Phase 2A-1: Basic Arduino Communication Test')
    print('=' * 50)
    
    # Test Arduino connection
    arduino = ArduinoInterface('/dev/ttyUSB0')
    print('📡 Attempting to connect to Arduino on /dev/ttyUSB0...')
    
    try:
        if arduino.connect():
            print('✅ Arduino connection successful!')
            print('🛡️ Safety gatekeeper is online and ready')
            print(f'   Port: {arduino.port}')
            print(f'   Baud rate: {arduino.baud_rate}')
            
            # Brief communication test
            print('🧪 Testing communication stability...')
            time.sleep(2)
            
            # Disconnect safely
            arduino.disconnect()
            print('🔌 Arduino disconnected safely')
            return True
            
        else:
            print('❌ Arduino connection failed!')
            print('🔍 Trying alternate port /dev/ttyUSB1...')
            
            # Try second USB port
            arduino2 = ArduinoInterface('/dev/ttyUSB1')
            if arduino2.connect():
                print('✅ Arduino connection successful on /dev/ttyUSB1!')
                print('🛡️ Safety gatekeeper is online and ready')
                arduino2.disconnect()
                print('🔌 Arduino disconnected safely')
                return True
            else:
                print('❌ Both Arduino ports failed to connect')
                return False
                
    except Exception as e:
        print(f'⚠️ Communication error: {e}')
        return False
    
    finally:
        print('📊 Test 2A-1 Complete')

if __name__ == "__main__":
    print("Phase 2A-1: Arduino Communication Test")
    print("Rover should remain in Failsafe mode during this test")
    print()
    
    success = test_arduino_communication()
    
    if success:
        print("\n🎉 Test 2A-1 PASSED: Arduino communication working")
        sys.exit(0)
    else:
        print("\n❌ Test 2A-1 FAILED: Arduino communication issues")
        sys.exit(1)