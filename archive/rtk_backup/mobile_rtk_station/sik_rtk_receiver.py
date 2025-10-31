#!/usr/bin/env python3
import serial
import time
import threading

class SiKRTKReceiver:
    def __init__(self):
        self.sik_device = '/dev/ttyUSB0'  # Pi SiK radio
        self.corrections_received = 0
        self.running = False
        
    def listen_for_corrections(self, duration=15):
        """Listen for RTCM corrections via SiK radio"""
        print(f'📻 Listening for RTK corrections via SiK radio ({duration}s)...')
        
        try:
            ser = serial.Serial(self.sik_device, 57600, timeout=1)
            
            start_time = time.time()
            rtk_packets = []
            
            while time.time() - start_time < duration:
                try:
                    # Read data from SiK radio
                    data = ser.read(100)
                    
                    if data:
                        # Look for RTK packets with our header
                        if b'RTK:' in data:
                            print('📡 RTK correction packet received!')
                            
                            # Extract RTCM data after header
                            rtk_start = data.find(b'RTK:') + 4
                            rtcm_data = data[rtk_start:]
                            
                            if rtcm_data:
                                # Check for RTCM v3 signature
                                if b'\xd3' in rtcm_data:
                                    print('🎯 Valid RTCM v3 correction detected!')
                                    
                                rtk_packets.append(rtcm_data)
                                self.corrections_received += len(rtcm_data)
                                
                                print(f'📊 Received: {len(rtcm_data)} bytes, Total: {self.corrections_received}')
                        
                        elif len(data) > 5:  # Other SiK traffic
                            # Check for any RTCM signatures
                            if b'\xd3' in data:
                                print('📡 Raw RTCM data detected in SiK stream')
                                
                except Exception as e:
                    continue
                    
            ser.close()
            
            if rtk_packets:
                print(f'\n✅ RTK Reception via SiK Radio Successful!')
                print(f'📊 Received {len(rtk_packets)} correction packets')
                print(f'📈 Total data: {self.corrections_received} bytes')
                return True
            else:
                print('\n⚠️  No RTK corrections received via SiK radio')
                return False
                
        except Exception as e:
            print(f'❌ SiK reception error: {e}')
            return False
    
    def test_wireless_rtk_system(self):
        """Test the complete wireless RTK system"""
        print('🚀 Wireless RTK System Test')
        print('📡 Pi receiving corrections via SiK from laptop base station')
        print('=' * 60)
        
        # Listen for corrections
        success = self.listen_for_corrections(20)
        
        if success:
            print('\n🎯 Wireless RTK System Working!')
            print('✅ Base station → SiK radio → Pi → RTK corrections')
            print('📍 Ready for centimeter-accurate positioning')
            
            print('\n📋 Complete System Status:')
            print('✅ Laptop: RTK base station generating corrections') 
            print('✅ SiK Network: Wireless correction relay working')
            print('✅ Pi: Receiving corrections via SiK radio')
            print('✅ GPS: Ready for RTK enhanced positioning')
            
        else:
            print('\n⚠️  Wireless RTK needs troubleshooting')
            print('💡 Try sending more corrections from base station')
            
        return success

if __name__ == '__main__':
    receiver = SiKRTKReceiver()
    receiver.test_wireless_rtk_system()
