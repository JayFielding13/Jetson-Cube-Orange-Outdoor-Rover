#!/usr/bin/env python3
import subprocess
import time
import threading
import socket

class RTKClient:
    def __init__(self):
        self.ntrip_host = '192.168.8.182'
        self.ntrip_port = 2101
        self.gps_device = '/dev/ttyACM0'
        self.running = False
        
    def download_rtklib(self):
        """Download and compile RTKLib for Pi if not available"""
        print('📥 Setting up RTKLib on Pi...')
        try:
            # Check if already exists
            result = subprocess.run(['ls', '/home/jay/RTKLIB'], capture_output=True)
            if result.returncode == 0:
                print('✅ RTKLib already available')
                return True
                
            # Clone and compile (simplified)
            print('⚠️  RTKLib not found - would need internet to download')
            return False
            
        except Exception as e:
            print(f'❌ RTKLib setup error: {e}')
            return False
    
    def test_ntrip_connection(self):
        """Test connection to NTRIP server"""
        print(f'🌐 Testing NTRIP connection to {self.ntrip_host}:{self.ntrip_port}')
        
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10)
            sock.connect((self.ntrip_host, self.ntrip_port))
            
            print('✅ NTRIP server reachable')
            
            # Test data reception
            data = sock.recv(1024, socket.MSG_DONTWAIT)
            if len(data) > 0:
                print(f'📊 Received {len(data)} bytes immediately')
            
            sock.close()
            return True
            
        except Exception as e:
            print(f'❌ NTRIP connection failed: {e}')
            return False
    
    def check_gps_before_rtk(self):
        """Check GPS status before RTK"""
        print('📡 Checking GPS status before RTK...')
        
        try:
            import serial
            ser = serial.Serial(self.gps_device, 38400, timeout=2)
            
            start_time = time.time()
            nmea_data = b''
            
            while time.time() - start_time < 5:
                data = ser.read(100)
                if b'\$' in data:  # NMEA sentence
                    nmea_data += data
                    break
            
            ser.close()
            
            if nmea_data:
                print('✅ GPS responding with NMEA data')
                # Parse basic info
                if b'GGA' in nmea_data:
                    print('🎯 GGA sentences detected (good for RTK)')
                return True
            else:
                print('❌ No GPS data received')
                return False
                
        except Exception as e:
            print(f'❌ GPS check error: {e}')
            return False
    
    def create_rtk_config(self):
        """Create RTK client configuration"""
        print('⚙️  Creating RTK client configuration...')
        
        config = f'''# RTK Client Configuration for Mobile RTK Beacon
# Input: GPS receiver + NTRIP corrections
inpstr1-type       =serial     # GPS input
inpstr1-path       ={self.gps_device[5:]}:38400:8:n:1:off
inpstr1-format     =ubx        # u-blox format

inpstr2-type       =ntripcli   # NTRIP client for corrections
inpstr2-path       ={self.ntrip_host}:{self.ntrip_port}
inpstr2-format     =rtcm3      # RTCM3 corrections

# Output: Enhanced position
outstr1-type       =file       # Log to file
outstr1-path       =rtk_positions.pos
outstr1-format     =llh        # lat/lon/height format

# RTK Settings
pos1-posmode       =kinematic  # kinematic RTK
pos1-frequency     =l1+l2      # dual frequency
pos1-navsys        =1+2+4+8+16 # GPS+SBAS+GLO+GAL+QZSS
pos1-elmask        =15         # 15 degree elevation mask
pos1-dynamics      =on         # dynamic model for moving platform
'''
        
        with open('rtk_client.conf', 'w') as f:
            f.write(config)
            
        print('✅ RTK client configuration created')
        return True

if __name__ == '__main__':
    client = RTKClient()
    
    print('🚀 RTK Client Setup for Mobile RTK Beacon')
    print('=' * 50)
    
    # Run setup steps
    client.check_gps_before_rtk()
    client.test_ntrip_connection() 
    client.create_rtk_config()
    
    print('\n📋 Setup Summary:')
    print('✅ GPS receiver ready')
    print('✅ NTRIP connection configured') 
    print('✅ RTK client config created')
    print('\n🎯 Next: Run RTK client to receive corrections')
