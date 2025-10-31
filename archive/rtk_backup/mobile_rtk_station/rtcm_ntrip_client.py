#!/usr/bin/env python3
import socket
import time

def test_ntrip_client():
    print('🌐 NTRIP Client - Testing RTK corrections')
    print('📡 Server: 192.168.8.182:2101')
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(15)
        sock.connect(('192.168.8.182', 2101))
        
        print('✅ Connected to NTRIP server!')
        
        # Send proper NTRIP request (no mount point for raw stream)
        ntrip_request = 'GET / HTTP/1.1\r\nHost: 192.168.8.182:2101\r\nUser-Agent: RTK-Pi\r\n\r\n'
        sock.send(ntrip_request.encode())
        
        print('📤 Sent NTRIP request')
        print('📊 Waiting for RTCM corrections...')
        
        total_data = 0
        start_time = time.time()
        rtcm_detected = False
        
        while time.time() - start_time < 15:  # Wait 15 seconds
            try:
                data = sock.recv(1024)
                if data:
                    total_data += len(data)
                    
                    # Check for RTCM v3 magic byte
                    if b'\xd3' in data:
                        if not rtcm_detected:
                            print('🎯 RTCM v3 corrections detected!')
                            rtcm_detected = True
                    
                    print(f'📨 +{len(data)} bytes (total: {total_data})')
                    
                    if total_data > 10000:  # Got enough data
                        break
                        
            except socket.timeout:
                print('⏰ Still waiting for data...')
                continue
                
        sock.close()
        
        if total_data > 0:
            rate = total_data * 8 / 15  # bits per second
            print(f'\n✅ RTK corrections working!')
            print(f'📊 Received: {total_data} bytes ({rate:.0f} bps)')
            if rtcm_detected:
                print('🎯 RTCM v3 protocol confirmed')
            return True
        else:
            print('❌ No corrections received')
            return False
            
    except Exception as e:
        print(f'❌ Connection error: {e}')
        return False

if __name__ == '__main__':
    test_ntrip_client()
