#!/usr/bin/env python3
import socket
import time

def test_rtcm_client():
    print('🌐 RTCM Client - Connecting to laptop base station')
    print('📡 Server: 192.168.8.182:2101')
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10)
        sock.connect(('192.168.8.182', 2101))
        
        print('✅ Connected to RTCM server!')
        
        # Read raw RTCM data
        print('📊 Reading RTCM corrections...')
        total_data = 0
        start_time = time.time()
        
        while time.time() - start_time < 10:  # Read for 10 seconds
            try:
                data = sock.recv(1024)
                if data:
                    total_data += len(data)
                    print(f'📨 Received {len(data)} bytes, total: {total_data}')
                    
                    # Check for RTCM v3 messages (start with 0xD3)
                    if b'\xd3' in data:
                        print('🎯 RTCM v3 messages detected!')
                        
                    if total_data > 5000:  # Got enough data
                        break
                        
            except socket.timeout:
                print('⏰ Timeout waiting for data')
                break
                
        sock.close()
        
        if total_data > 0:
            print(f'✅ RTK test successful! Received {total_data} bytes of corrections')
            print(f'📈 Data rate: {total_data*8/10:.0f} bps')
        else:
            print('❌ No RTCM data received')
            
    except Exception as e:
        print(f'❌ Connection error: {e}')

if __name__ == '__main__':
    test_rtcm_client()
