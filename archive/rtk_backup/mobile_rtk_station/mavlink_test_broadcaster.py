#!/usr/bin/env python3
"""
MAVLink Test Broadcaster - Uses simulated GPS data for testing
Tests MAVLink transmission to Mission Planner via SiK radio
"""

import serial
import time
import struct
import math

class MAVLinkTestBroadcaster:
    def __init__(self, radio_port='/dev/ttyUSB0', radio_baud=57600):
        self.radio_port = radio_port
        self.radio_baud = radio_baud
        
        # MAVLink constants
        self.MAVLINK_MSG_ID_GPS_RAW_INT = 24
        self.system_id = 255  # Mobile RTK Station
        self.component_id = 190  # GPS component
        self.seq = 0
        
        # Test position (approximate location from earlier GPS data)
        self.base_lat = 45.43  # degrees
        self.base_lon = -122.84  # degrees  
        self.altitude = 50.0  # meters
        
    def calculate_checksum(self, data):
        """Calculate MAVLink CRC"""
        crcTmp = 0
        for byte in data:
            tmp = byte ^ (crcTmp & 0xff)
            tmp ^= (tmp << 4)
            crcTmp = (crcTmp >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
            crcTmp = crcTmp & 0xffff
        return crcTmp

    def create_gps_message(self, lat, lon, alt):
        """Create MAVLink GPS_RAW_INT message"""
        # Convert to MAVLink units
        lat_int = int(lat * 1e7)
        lon_int = int(lon * 1e7) 
        alt_int = int(alt * 1000)  # mm
        
        time_usec = int(time.time() * 1000000)
        
        # GPS_RAW_INT payload
        payload = struct.pack('<QIiiiHHHHiB',
            time_usec,  # time_usec
            lat_int,    # lat
            lon_int,    # lon  
            alt_int,    # alt
            100,        # eph (HDOP * 100) - good accuracy
            100,        # epv (VDOP * 100) - good accuracy
            0,          # vel (ground speed cm/s)
            0,          # cog (course over ground cdeg)
            alt_int,    # alt_ellipsoid
            3,          # fix_type (3 = 3D fix)
            12          # satellites_visible
        )
        
        return self.create_mavlink_packet(self.MAVLINK_MSG_ID_GPS_RAW_INT, payload)
    
    def create_mavlink_packet(self, msg_id, payload):
        """Create complete MAVLink v1 packet"""
        # Header
        stx = 0xFE
        length = len(payload)
        self.seq = (self.seq + 1) % 256
        
        header = struct.pack('<BBBBB',
            stx, length, self.seq, self.system_id, self.component_id
        ) + struct.pack('<B', msg_id)
        
        # Calculate checksum on header[1:] + payload  
        checksum_data = header[1:] + payload
        checksum = self.calculate_checksum(checksum_data)
        
        return header + payload + struct.pack('<H', checksum)
    
    def start_broadcasting(self):
        """Start broadcasting test GPS data"""
        print("🚀 Starting MAVLink Test Broadcaster")
        print(f"📡 Broadcasting on {self.radio_port} at {self.radio_baud} baud")
        print(f"📍 Test position: {self.base_lat:.6f}, {self.base_lon:.6f}")
        print("📟 Watch Mission Planner for 'Mobile RTK Station' to appear")
        print("Press Ctrl+C to stop\n")
        
        try:
            with serial.Serial(self.radio_port, self.radio_baud, timeout=1) as radio:
                print("✅ SiK radio connected")
                
                count = 0
                while True:
                    # Create slight movement in position for testing
                    offset_lat = self.base_lat + (math.sin(count * 0.1) * 0.0001) 
                    offset_lon = self.base_lon + (math.cos(count * 0.1) * 0.0001)
                    
                    # Create and send GPS message
                    gps_msg = self.create_gps_message(offset_lat, offset_lon, self.altitude)
                    radio.write(gps_msg)
                    radio.flush()
                    
                    count += 1
                    print(f"📡 Sent MAVLink GPS #{count}: {offset_lat:.6f}, {offset_lon:.6f} ({len(gps_msg)} bytes)")
                    
                    time.sleep(1.0)  # 1Hz update rate
                    
        except KeyboardInterrupt:
            print("\n⏹️  Broadcasting stopped")
        except Exception as e:
            print(f"❌ Error: {e}")

if __name__ == "__main__":
    broadcaster = MAVLinkTestBroadcaster()
    broadcaster.start_broadcasting()
