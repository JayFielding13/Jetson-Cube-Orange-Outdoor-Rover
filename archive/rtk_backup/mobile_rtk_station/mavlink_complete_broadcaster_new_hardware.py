#!/usr/bin/env python3
"""
Complete MAVLink Broadcaster with Heartbeat for Mission Planner
Sends heartbeat + GPS data via SiK radio
"""

import serial
import time
import struct
import threading
import math

class MAVLinkCompleteBroadcaster:
    def __init__(self, radio_port='/dev/ttyUSB0', radio_baud=57600):
        self.radio_port = radio_port
        self.radio_baud = radio_baud
        
        # MAVLink message IDs
        self.MAVLINK_MSG_ID_HEARTBEAT = 0
        self.MAVLINK_MSG_ID_GPS_RAW_INT = 24
        
        # System identification
        self.system_id = 255  # Mobile RTK Station
        self.component_id = 1  # MAV_COMP_ID_AUTOPILOT1 (main component)
        self.seq = 0
        
        # Test position
        self.base_lat = 45.43
        self.base_lon = -122.84
        self.altitude = 50.0
        
        self.running = False
        
    def calculate_checksum(self, data):
        """Calculate MAVLink CRC"""
        crcTmp = 0
        for byte in data:
            tmp = byte ^ (crcTmp & 0xff)
            tmp ^= (tmp << 4)
            crcTmp = (crcTmp >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
            crcTmp = crcTmp & 0xffff
        return crcTmp

    def create_heartbeat(self):
        """Create MAVLink HEARTBEAT message"""
        # HEARTBEAT payload
        payload = struct.pack('<IBBBBB',
            0,      # custom_mode
            6,      # type (MAV_TYPE_GCS - Ground Control Station)
            0,      # autopilot (MAV_AUTOPILOT_GENERIC)
            192,    # base_mode (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED)
            4,      # system_status (MAV_STATE_ACTIVE)
            3       # mavlink_version
        )
        return self.create_mavlink_packet(self.MAVLINK_MSG_ID_HEARTBEAT, payload)

    def create_gps_message(self, lat, lon, alt):
        """Create MAVLink GPS_RAW_INT message"""
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
            100,        # epv (VDOP * 100) 
            0,          # vel (ground speed cm/s)
            0,          # cog (course over ground cdeg)
            alt_int,    # alt_ellipsoid
            3,          # fix_type (3 = 3D GPS fix)
            12          # satellites_visible
        )
        
        return self.create_mavlink_packet(self.MAVLINK_MSG_ID_GPS_RAW_INT, payload)
    
    def create_mavlink_packet(self, msg_id, payload):
        """Create complete MAVLink v1 packet"""
        stx = 0xFE
        length = len(payload)
        self.seq = (self.seq + 1) % 256
        
        header = struct.pack('<BBBBB',
            stx, length, self.seq, self.system_id, self.component_id
        ) + struct.pack('<B', msg_id)
        
        # Calculate checksum
        checksum_data = header[1:] + payload
        checksum = self.calculate_checksum(checksum_data)
        
        return header + payload + struct.pack('<H', checksum)
    
    def start_broadcasting(self):
        """Start broadcasting with heartbeat + GPS"""
        print("🚀 Starting Complete MAVLink Broadcaster")
        print(f"📡 Broadcasting on {self.radio_port} at {self.radio_baud} baud")
        print(f"💓 Sending HEARTBEAT messages (required for Mission Planner)")
        print(f"📍 GPS position: {self.base_lat:.6f}, {self.base_lon:.6f}")
        print(f"🎯 System ID: {self.system_id} (Mobile RTK Station)")
        print("📟 Mission Planner should now connect successfully!")
        print("Press Ctrl+C to stop\n")
        
        try:
            with serial.Serial(self.radio_port, self.radio_baud, timeout=1) as radio:
                print("✅ SiK radio connected")
                self.running = True
                
                count = 0
                last_heartbeat = 0
                
                while self.running:
                    current_time = time.time()
                    
                    # Send heartbeat every 1 second (required!)
                    if current_time - last_heartbeat >= 1.0:
                        heartbeat = self.create_heartbeat()
                        radio.write(heartbeat)
                        radio.flush()
                        last_heartbeat = current_time
                        print(f"💓 Heartbeat sent ({len(heartbeat)} bytes)")
                    
                    # Send GPS data every 1 second
                    offset_lat = self.base_lat + (math.sin(count * 0.1) * 0.0001) 
                    offset_lon = self.base_lon + (math.cos(count * 0.1) * 0.0001)
                    
                    gps_msg = self.create_gps_message(offset_lat, offset_lon, self.altitude)
                    radio.write(gps_msg)
                    radio.flush()
                    
                    count += 1
                    print(f"📡 GPS #{count}: {offset_lat:.6f}, {offset_lon:.6f} ({len(gps_msg)} bytes)")
                    
                    time.sleep(1.0)
                    
        except KeyboardInterrupt:
            print("\n⏹️  Broadcasting stopped")
            self.running = False
        except Exception as e:
            print(f"❌ Error: {e}")

if __name__ == "__main__":
    broadcaster = MAVLinkCompleteBroadcaster()
    broadcaster.start_broadcasting()
