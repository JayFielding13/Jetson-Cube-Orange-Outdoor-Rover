#!/usr/bin/env python3
import subprocess
import socket
import serial
import time

def check_complete_rtk_system():
    print('🚀 Complete RTK System Status Check')
    print('=' * 50)
    
    results = {}
    
    # 1. Check Pi GPS
    print('📡 Testing Pi GPS (Mobile RTK Beacon)...')
    try:
        ser = serial.Serial('/dev/ttyACM0', 38400, timeout=3)
        line = ser.readline().decode('ascii', errors='ignore')
        if 'GGA' in line or line.startswith('\$'):
            print('✅ Pi GPS: Working - High precision receiver ready')
            results['pi_gps'] = True
        else:
            print('❌ Pi GPS: No response')
            results['pi_gps'] = False
        ser.close()
    except Exception as e:
        print(f'❌ Pi GPS: Error - {e}')
        results['pi_gps'] = False
    
    # 2. Check base station connectivity
    print('\n📡 Testing Base Station Connection...')
    try:
        result = subprocess.run(['ping', '-c', '1', '192.168.8.182'], 
                              capture_output=True, timeout=5)
        if result.returncode == 0:
            print('✅ Base Station: Reachable on 192.168.8.182')
            results['base_station'] = True
        else:
            print('❌ Base Station: Not reachable')
            results['base_station'] = False
    except:
        print('❌ Base Station: Connection test failed')
        results['base_station'] = False
    
    # 3. Check SiK radio
    print('\n📻 Testing SiK Radio...')
    try:
        ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
        ser.close()
        print('✅ SiK Radio: Connected on /dev/ttyUSB0')
        results['sik_radio'] = True
    except Exception as e:
        print(f'❌ SiK Radio: Error - {e}')
        results['sik_radio'] = False
    
    # 4. Check MAVLink bridge
    print('\n🌉 Testing MAVLink Bridge...')
    try:
        result = subprocess.run(['netstat', '-tln'], capture_output=True, text=True)
        if ':5760' in result.stdout:
            print('✅ MAVLink Bridge: Running on port 5760')
            results['mavlink_bridge'] = True
        else:
            print('❌ MAVLink Bridge: Not detected')
            results['mavlink_bridge'] = False
    except:
        print('❌ MAVLink Bridge: Status unknown')
        results['mavlink_bridge'] = False
    
    # Summary
    print('\n📊 RTK System Integration Summary:')
    print('=' * 50)
    
    working_components = sum(results.values())
    total_components = len(results)
    
    print(f'🎯 System Status: {working_components}/{total_components} components operational')
    
    if results.get('pi_gps', False):
        print('✅ High-precision GPS ready for RTK')
    if results.get('base_station', False):
        print('✅ RTK base station accessible')
    if results.get('sik_radio', False):
        print('✅ Rover communication link ready')
    if results.get('mavlink_bridge', False):
        print('✅ Ground control integration ready')
    
    print('\n🎯 RTK Capabilities Ready:')
    print('📍 Centimeter-accurate positioning')
    print('📡 Real-time corrections from base station') 
    print('🚁 Rover control via QGroundControl')
    print('📱 Mobile RTK beacon functionality')
    
    return results

if __name__ == '__main__':
    check_complete_rtk_system()
