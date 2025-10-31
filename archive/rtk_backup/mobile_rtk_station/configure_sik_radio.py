#!/usr/bin/env python3
"""
Configure SiK Radio for Holybro Network Integration
Sets up the Pi's SiK radio to communicate with Holybro radios
"""

import serial
import time
import sys

def configure_sik_radio(port='/dev/ttyUSB0', baud=57600):
    """Configure SiK radio for Holybro network compatibility"""
    
    print('🔧 Configuring SiK radio for Holybro network...')
    
    try:
        # Connect to radio
        sik = serial.Serial(port, baud, timeout=3)
        print(f'📡 Connected to SiK radio on {port}')
        
        # Enter AT command mode
        print('⏳ Entering AT command mode...')
        time.sleep(1.2)
        sik.write(b'+++')
        time.sleep(1.2)
        
        response = sik.read(10).decode('ascii', errors='ignore')
        if 'OK' not in response:
            print('❌ Failed to enter AT command mode')
            return False
            
        print('✅ AT command mode active')
        sik.read(100)  # Clear buffer
        
        # Configuration for Holybro compatibility
        config_commands = [
            ('ATS3=26', 'Set Node ID to 26 (Mobile RTK Station)'),
            ('ATS0=25', 'Set Serial Format to 25 (MAVLink)'),
            ('ATS1=57', 'Set Serial Speed to 57 (57600 baud)'),
            ('ATS2=64', 'Set Air Speed to 64 kbps'),
            ('ATS4=20', 'Set TX Power to 20 dBm'),
            ('ATS5=1', 'Enable ECC'),
            ('AT&W', 'Write settings to EEPROM'),
            ('ATZ', 'Reset radio with new settings')
        ]
        
        for cmd, desc in config_commands:
            print(f'📝 {desc}...')
            sik.write(f'{cmd}\r\n'.encode())
            time.sleep(0.8)
            
            response = sik.read(50).decode('ascii', errors='ignore').strip()
            if 'OK' in response or cmd == 'ATZ':
                print(f'✅ {desc} - OK')
            else:
                print(f'⚠️  {desc} - Response: {response}')
        
        print('⏳ Radio resetting with new configuration...')
        time.sleep(3)
        
        sik.close()
        print('✅ SiK radio configuration complete!')
        
        return True
        
    except Exception as e:
        print(f'❌ Configuration error: {e}')
        return False

def verify_configuration(port='/dev/ttyUSB0', baud=57600):
    """Verify the radio configuration"""
    
    print('\n🔍 Verifying configuration...')
    
    try:
        sik = serial.Serial(port, baud, timeout=3)
        
        # Enter AT mode
        time.sleep(1.2)
        sik.write(b'+++')
        time.sleep(1.2)
        sik.read(10)
        
        # Check key settings
        checks = [
            ('ATS3?', 'Node ID'),
            ('ATS1?', 'Serial Speed'), 
            ('ATS2?', 'Air Speed')
        ]
        
        for cmd, desc in checks:
            sik.write(f'{cmd}\r\n'.encode())
            time.sleep(0.5)
            response = sik.read(50).decode('ascii', errors='ignore').strip()
            print(f'{desc}: {response}')
        
        # Exit AT mode
        sik.write(b'ATO\r\n')
        sik.close()
        
        print('✅ Verification complete')
        
    except Exception as e:
        print(f'❌ Verification error: {e}')

def main():
    print('🚀 SiK Radio Configuration for Holybro Network')
    print('📡 This will configure the Pi SiK radio to work with Holybro radios\n')
    
    # Show current network requirements
    print('📋 Target Configuration:')
    print('   Network ID: 25 (match Holybro radios)')
    print('   Node ID: 26 (unique for Mobile RTK Station)')
    print('   Serial Speed: 57600 baud')
    print('   Air Speed: 64 kbps')
    print('   Protocol: MAVLink compatible\n')
    
    if configure_sik_radio():
        verify_configuration()
        
        print('\n🎯 Next Steps:')
        print('1. Test connection to Holybro radios')
        print('2. Verify MAVLink communication')
        print('3. Test Mission Planner Follow Me feature')
        
        return 0
    else:
        return 1

if __name__ == '__main__':
    sys.exit(main())
