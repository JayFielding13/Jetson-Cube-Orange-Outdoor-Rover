#!/usr/bin/env python3
"""
Mobile RTK Station - MAVLink Broadcaster (Updated for New Hardware)
Broadcasts GPS position data from new u-blox receiver via SiK antenna
New Hardware: u-blox GPS on /dev/ttyACM0, SiK antenna on /dev/ttyUSB0
"""

import serial
import time
import struct
import sys
from datetime import datetime

# Hardware configuration - UPDATED for new components
GPS_PORT = '/dev/ttyACM0'      # New u-blox GPS receiver
SIK_PORT = '/dev/ttyUSB0'      # SiK antenna (same port)
GPS_BAUD = 38400               # Standard u-blox baud rate
SIK_BAUD = 57600               # SiK radio baud rate

# MAVLink configuration
SYSTEM_ID = 255                # Mobile RTK station ID
COMPONENT_ID = 1               # GPS component
HEARTBEAT_INTERVAL = 1.0       # Send heartbeat every second

class MAVLinkBroadcaster:
    def __init__(self):
        self.gps_serial = None
        self.sik_serial = None
        self.sequence = 0
        self.last_heartbeat = 0
        
    def connect_devices(self):
        """Connect to both GPS and SiK antenna"""
        try:
            print(f'📡 Connecting to new u-blox GPS on {GPS_PORT}...')
            self.gps_serial = serial.Serial(GPS_PORT, GPS_BAUD, timeout=1)
            print(f'✅ u-blox GPS connected')
            
            print(f'📡 Connecting to SiK antenna on {SIK_PORT}...')
            self.sik_serial = serial.Serial(SIK_PORT, SIK_BAUD, timeout=1)
            print(f'✅ SiK antenna connected')
            
            return True
            
        except Exception as e:
            print(f'❌ Connection error: {e}')
            return False
    
    def parse_gga(self, nmea_sentence):
        """Parse NMEA GGA sentence for position data"""
        try:
            parts = nmea_sentence.split(',')
            if len(parts) < 15 or parts[2] == '':
                return None
                
            # Extract position data
            lat_deg = float(parts[2][:2]) + float(parts[2][2:]) / 60.0
            if parts[3] == 'S':
                lat_deg = -lat_deg
                
            lon_deg = float(parts[4][:3]) + float(parts[4][3:]) / 60.0  
            if parts[5] == 'W':
                lon_deg = -lon_deg
                
            fix_quality = int(parts[6]) if parts[6] else 0
            num_sats = int(parts[7]) if parts[7] else 0
            hdop = float(parts[8]) if parts[8] else 99.0
            altitude = float(parts[9]) if parts[9] else 0.0
            
            return {
                'lat': lat_deg,
                'lon': lon_deg, 
                'alt': altitude,
                'fix_quality': fix_quality,
                'satellites': num_sats,
                'hdop': hdop
            }
            
        except Exception:
            return None
    
    def create_heartbeat(self):
        """Create MAVLink heartbeat message"""
        # MAVLink 2.0 heartbeat
        payload = struct.pack('<IBBBB', 0, 4, 0, 0, 5)  # Custom mode, MAV_TYPE_QUADROTOR, autopilot, base_mode, system_status
        return self.create_mavlink_message(0, payload)  # HEARTBEAT message ID = 0
    
    def create_gps_message(self, gps_data):
        """Create MAVLink GPS_RAW_INT message"""
        timestamp = int(time.time() * 1e6)  # Microseconds since epoch
        
        # Convert to required formats
        lat_int = int(gps_data['lat'] * 1e7)  # degrees * 1e7
        lon_int = int(gps_data['lon'] * 1e7)  # degrees * 1e7
        alt_int = int(gps_data['alt'] * 1000)  # mm above MSL
        
        # GPS fix type mapping
        fix_type = 3 if gps_data['fix_quality'] >= 1 else 1  # 3D fix or no fix
        
        payload = struct.pack('<QiiihHHHBB',
                            timestamp,           # time_usec
                            lat_int,             # lat (degrees * 1e7)
                            lon_int,             # lon (degrees * 1e7) 
                            alt_int,             # alt (mm)
                            int(gps_data['hdop'] * 100), # eph (cm)
                            65535,               # epv (cm) - unknown
                            65535,               # vel (cm/s) - unknown
                            65535,               # cog (cdeg) - unknown
                            fix_type,            # fix_type
                            gps_data['satellites']) # satellites_visible
        
        return self.create_mavlink_message(24, payload)  # GPS_RAW_INT message ID = 24
    
    def create_mavlink_message(self, msg_id, payload):
        """Create MAVLink 2.0 message"""
        payload_len = len(payload)
        
        # Calculate CRC
        crc = 0xFFFF
        header = struct.pack('<BBBBBBBB', 
                            0xFD,           # STX (MAVLink 2.0)
                            payload_len,    # Payload length
                            0,              # Incompatible flags
                            0,              # Compatible flags  
                            self.sequence,  # Sequence
                            SYSTEM_ID,      # System ID
                            COMPONENT_ID,   # Component ID
                            msg_id & 0xFF)  # Message ID (low byte)
        
        # Add high byte of message ID and low byte of message ID
        header += struct.pack('<BB', (msg_id >> 8) & 0xFF, msg_id & 0xFF)
        
        message = header + payload
        
        # Simple CRC calculation (simplified for demo)
        crc_bytes = struct.pack('<H', 0x1234)  # Placeholder CRC
        message += crc_bytes
        
        self.sequence = (self.sequence + 1) % 256
        return message
    
    def broadcast_loop(self):
        """Main broadcasting loop"""
        print(f'🚀 Starting Mobile RTK MAVLink Broadcaster (New Hardware)')
        print(f'📍 GPS: {GPS_PORT} (u-blox receiver)')
        print(f'📡 SiK: {SIK_PORT} (57600 baud)')
        print(f'🎯 System ID: {SYSTEM_ID}')
        print('Press Ctrl+C to stop\n')
        
        gps_count = 0
        
        try:
            while True:
                current_time = time.time()
                
                # Send heartbeat every second
                if current_time - self.last_heartbeat >= HEARTBEAT_INTERVAL:
                    heartbeat = self.create_heartbeat()
                    self.sik_serial.write(heartbeat)
                    self.sik_serial.flush()
                    self.last_heartbeat = current_time
                    print(f'💓 Heartbeat sent ({len(heartbeat)} bytes)')
                
                # Read and process GPS data
                if self.gps_serial.in_waiting > 0:
                    line = self.gps_serial.readline().decode('ascii', errors='ignore').strip()
                    
                    if line.startswith('') or line.startswith(''):
                        gps_data = self.parse_gga(line)
                        
                        if gps_data:
                            gps_count += 1
                            gps_msg = self.create_gps_message(gps_data)
                            self.sik_serial.write(gps_msg)
                            self.sik_serial.flush()
                            
                            print(f'📡 GPS #{gps_count}: {gps_data["lat"]:.6f}, {gps_data["lon"]:.6f} '
                                  f'(Fix:{gps_data["fix_quality"]}, Sats:{gps_data["satellites"]}) '
                                  f'({len(gps_msg)} bytes)')
                
                time.sleep(0.1)  # 10Hz loop
                
        except KeyboardInterrupt:
            print('\n⏹️  Broadcasting stopped by user')
        except Exception as e:
            print(f'❌ Error in broadcast loop: {e}')
    
    def cleanup(self):
        """Clean up serial connections"""
        if self.gps_serial:
            self.gps_serial.close()
        if self.sik_serial:
            self.sik_serial.close()
        print('🔌 Serial connections closed')

def main():
    broadcaster = MAVLinkBroadcaster()
    
    if not broadcaster.connect_devices():
        print('❌ Failed to connect to devices')
        return 1
    
    try:
        broadcaster.broadcast_loop()
    finally:
        broadcaster.cleanup()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
