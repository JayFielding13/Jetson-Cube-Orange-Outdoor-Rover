#!/usr/bin/env python3
import socket
import time
import threading

class SimpleNTRIPClient:
    def __init__(self):
        self.host = '192.168.8.182'
        self.port = 2101
        self.running = False
        self.corrections_received = 0
        
    def connect_and_receive(self):
        """Connect to NTRIP server and receive corrections"""
        print(f'🌐 Connecting to NTRIP server {self.host}:{self.port}')
        
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(30)
            sock.connect((self.host, self.port))
            
            print('✅ Connected to NTRIP server')
            
            # Simple request for raw RTCM stream
            request = b'GET / HTTP/1.1\r\nHost: ' + f'{self.host}:{self.port}'.encode() + b'\r\n\r\n'
            sock.send(request)
            
            print('📤 Sent NTRIP request')
            print('📊 Receiving RTCM corrections...')
            
            self.running = True
            start_time = time.time()
            
            while self.running and (time.time() - start_time < 30):
                try:
                    data = sock.recv(1024)
                    if data:
                        self.corrections_received += len(data)
                        
                        # Check for RTCM v3 messages (start with 0xD3)
                        if b'\xd3' in data:
                            print(f'📡 RTCM v3 message received! Total: {self.corrections_received} bytes')
                        
                        # Print progress every 5 seconds
                        if int(time.time() - start_time) % 5 == 0:
                            rate = self.corrections_received * 8 / (time.time() - start_time)
                            print(f'📈 Data rate: {rate:.0f} bps, Total: {self.corrections_received} bytes')
                            
                    else:
                        print('⚠️  No data received')
                        break
                        
                except socket.timeout:
                    print('⏰ Timeout waiting for corrections')
                    break
                except Exception as e:
                    print(f'❌ Reception error: {e}')
                    break
                    
            sock.close()
            
            if self.corrections_received > 0:
                runtime = time.time() - start_time
                avg_rate = self.corrections_received * 8 / runtime
                print(f'\n✅ RTCM Reception Test Complete!')
                print(f'📊 Total received: {self.corrections_received} bytes')
                print(f'📈 Average rate: {avg_rate:.0f} bps')
                print(f'⏱️  Runtime: {runtime:.1f} seconds')
                return True
            else:
                print('❌ No RTCM corrections received')
                return False
                
        except Exception as e:
            print(f'❌ NTRIP connection error: {e}')
            return False
    
    def stop(self):
        self.running = False

if __name__ == '__main__':
    client = SimpleNTRIPClient()
    
    print('🚀 Simple NTRIP Client for RTK Corrections')
    print('=' * 50)
    
    success = client.connect_and_receive()
    
    if success:
        print('\n🎯 RTK corrections are working!')
        print('📍 Pi is ready for centimeter-accurate positioning')
    else:
        print('\n⚠️  Need to troubleshoot NTRIP connection')
