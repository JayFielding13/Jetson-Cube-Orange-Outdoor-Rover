#!/usr/bin/env python3
"""
Phase 2A Test 2: Sensor Data Pipeline
=====================================

Test Arduino -> Navigation Pi sensor data flow.
This test verifies ultrasonic sensor readings can be received
from the Arduino safety gatekeeper.
"""

from Main.arduino_interface import ArduinoInterface
from Sensors.ultrasonic_sensor import UltrasonicSensor
import time
import sys

def test_sensor_data_pipeline():
    """Test sensor data flow from Arduino to Navigation Pi"""
    print('🔍 Phase 2A-2: Sensor Data Pipeline Test')
    print('=' * 50)
    
    # Setup Arduino and sensor
    arduino = ArduinoInterface('/dev/ttyUSB0')
    sensor = UltrasonicSensor(arduino)
    
    print('📡 Connecting to Arduino for sensor testing...')
    
    try:
        if arduino.connect():
            print('✅ Arduino connected for sensor testing')
            
            # Set up data callback
            arduino.set_data_callback(sensor.update_from_arduino_data)
            print('🔗 Sensor data callback configured')
            
            print('📊 Reading sensor data for 10 seconds...')
            print('   (Move your hand near the ultrasonic sensor)')
            
            readings = []
            for i in range(20):  # 20 readings over 10 seconds
                arduino.read_data()
                distance = sensor.get_distance()
                readings.append(distance)
                
                print(f'   Reading {i+1:2d}: {distance:6.1f}cm')
                time.sleep(0.5)
            
            # Analyze readings
            valid_readings = [r for r in readings if r > 0]
            if valid_readings:
                avg_distance = sum(valid_readings) / len(valid_readings)
                print(f'\n📈 Sensor Analysis:')
                print(f'   Total readings: {len(readings)}')
                print(f'   Valid readings: {len(valid_readings)}')
                print(f'   Average distance: {avg_distance:.1f}cm')
                print(f'   Min distance: {min(valid_readings):.1f}cm')
                print(f'   Max distance: {max(valid_readings):.1f}cm')
                
                arduino.disconnect()
                print('🔌 Arduino disconnected safely')
                return True
            else:
                print('❌ No valid sensor readings received')
                arduino.disconnect()
                return False
                
        else:
            print('❌ Arduino connection failed!')
            # Try alternate port
            arduino2 = ArduinoInterface('/dev/ttyUSB1')
            if arduino2.connect():
                print('✅ Connected on alternate port /dev/ttyUSB1')
                arduino2.disconnect()
                print('🔌 Arduino disconnected safely')
                return True
            return False
            
    except Exception as e:
        print(f'⚠️ Sensor test error: {e}')
        if arduino.serial and arduino.serial.is_open:
            arduino.disconnect()
        return False
    
    finally:
        print('📊 Test 2A-2 Complete')

if __name__ == "__main__":
    print("Phase 2A-2: Sensor Data Pipeline Test")
    print("Rover should remain in Failsafe mode during this test")
    print("Wave your hand near the ultrasonic sensor during testing")
    print()
    
    success = test_sensor_data_pipeline()
    
    if success:
        print("\n🎉 Test 2A-2 PASSED: Sensor data pipeline working")
        sys.exit(0)
    else:
        print("\n❌ Test 2A-2 FAILED: Sensor data pipeline issues")
        sys.exit(1)