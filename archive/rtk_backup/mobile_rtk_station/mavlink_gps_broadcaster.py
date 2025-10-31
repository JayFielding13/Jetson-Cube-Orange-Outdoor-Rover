#!/usr/bin/env python3
"""
Mobile RTK Station - MAVLink GPS Position Broadcaster
Sends GPS position data via SiK radio to Mission Planner
"""

import serial
import time
import struct
import threading
import pynmea2
from datetime import datetime

class MAVLinkGPSBroadcaster:
    def __init__(self, gps_port='/dev/ttyACM0', radio_port='/dev/ttyUSB0', 
                 gps_baud=9600, radio_baud=57600):
        self.gps_port = gps_port
        self.radio_port = radio_port
        self.gps_baud = gps_baud
        self.radio_baud = radio_baud
        
        # MAVLink message IDs
        self.MAVLINK_MSG_ID_GPS_RAW_INT = 24
        self.MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33
        
        # System and component IDs
        self.system_id = 255  # Mobile RTK Station
        self.component_id = 190  # MAV_COMP_ID_GPS
        
        self.running = False
        self.latest_position = None
        
    def calculate_checksum(self, data):
        """Calculate MAVLink checksum"""
        crcTmp = 0
        for byte in data:
            tmp = byte ^ (crcTmp & 0xff)
            tmp ^= (tmp << 4)
            crcTmp = (crcTmp >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
            crcTmp = crcTmp & 0xffff
        return crcTmp

    def create_mavlink_gps_raw_message(self, lat, lon, alt, fix_type, satellites):
        """Create MAVLink GPS_RAW_INT message"""
        # Convert to required units
        lat_int = int(lat * 1e7)  # degrees * 1e7
        lon_int = int(lon * 1e7)  # degrees * 1e7
        alt_int = int(alt * 1000)  # mm above sea level
        
        # GPS_RAW_INT message payload
        time_usec = int(time.time() * 1000000)  # Timestamp (microseconds)
        fix_type = max(0, min(8, fix_type))  # GPS fix type (0-8)
        satellites_visible = max(0, min(255, satellites))
        
        # Pack payload (GPS_RAW_INT format)
        payload = struct.pack('<QIiiiHHHHHB',
            time_usec,      # time_usec (uint64_t)
            lat_int,        # lat (int32_t)
            lon_int,        # lon (int32_t) 
            alt_int,        # alt (int32_t)
            65535,          # eph (uint16_t) - GPS HDOP * 100 (unknown = 65535)
            65535,          # epv (uint16_t) - GPS VDOP * 100 (unknown = 65535)
            65535,          # vel (uint16_t) - GPS ground speed (unknown = 65535)
            65535,          # cog (uint16_t) - Course over ground (unknown = 65535)
            0,              # alt_ellipsoid (int32_t) - Altitude above ellipsoid
            fix_type,       # fix_type (uint8_t)
            satellites_visible  # satellites_visible (uint8_t)
        )
        
        return self.create_mavlink_packet(self.MAVLINK_MSG_ID_GPS_RAW_INT, payload)

    def create_mavlink_packet(self, msg_id, payload):
        """Create complete MAVLink packet"""
        # MAVLink header
        stx = 0xFE  # MAVLink 1.0 start byte
        length = len(payload)
        seq = 0  # Sequence number (can increment)
        
        # Create header + payload
        packet_data = struct.pack('<BBBBB', 
            stx,                # start byte
            length,             # payload length  
            seq,                # sequence
            self.system_id,     # system ID
            self.component_id   # component ID
        ) + struct.pack('<B', msg_id) + payload
        
        # Calculate checksum on everything except start byte and checksum itself
        checksum_data = packet_data[1:]  # Skip start byte
        checksum = self.calculate_checksum(checksum_data)
        
        # Complete packet
        packet = packet_data + struct.pack('<H', checksum)
        return packet

    def parse_gps_data(self, line):
        """Parse NMEA GPS data"""
        try:
            msg = pynmea2.parse(line)
            
            if msg.sentence_type == 'GGA':
                # Position and fix data
                if msg.latitude and msg.longitude:
                    position_data = {
                        'latitude': float(msg.latitude),
                        'longitude': float(msg.longitude),
                        'altitude': float(msg.altitude) if msg.altitude else 0.0,
                        'fix_type': int(msg.gps_qual) if msg.gps_qual else 0,
                        'satellites': int(msg.num_sats) if msg.num_sats else 0,
                        'timestamp': time.time()
                    }
                    return position_data
                    
        except (pynmea2.ParseError, ValueError, AttributeError) as e:
            pass  # Skip invalid messages
            
        return None

    def gps_reader_thread(self):
        """Thread to read GPS data"""
        try:
            with serial.Serial(self.gps_port, self.gps_baud, timeout=1) as gps:
                print(f"GPS connected on {self.gps_port} at {self.gps_baud} baud")
                
                while self.running:
                    try:
                        line = gps.readline().decode('ascii', errors='ignore').strip()
                        if line.startswith('$') and 'GGA' in line:
                            position = self.parse_gps_data(line)
                            if position:
                                self.latest_position = position
                                print(f"GPS: {position['latitude']:.6f}, {position['longitude']:.6f}, "
                                      f"Alt: {position['altitude']:.1f}m, Sats: {position['satellites']}, "
                                      f"Fix: {position['fix_type']}")
                                      
                    except Exception as e:
                        print(f"GPS read error: {e}")
                        time.sleep(0.1)
                        
        except Exception as e:
            print(f"GPS connection error: {e}")

    def mavlink_broadcaster_thread(self):
        """Thread to broadcast MAVLink messages"""
        try:
            with serial.Serial(self.radio_port, self.radio_baud, timeout=1) as radio:
                print(f"SiK radio connected on {self.radio_port} at {self.radio_baud} baud")
                
                while self.running:
                    if self.latest_position:
                        # Create and send GPS message
                        gps_msg = self.create_mavlink_gps_raw_message(
                            self.latest_position['latitude'],
                            self.latest_position['longitude'], 
                            self.latest_position['altitude'],
                            self.latest_position['fix_type'],
                            self.latest_position['satellites']
                        )
                        
                        radio.write(gps_msg)
                        radio.flush()
                        
                        print(f"MAVLink GPS message sent: {len(gps_msg)} bytes")
                    
                    time.sleep(1.0)  # Send at 1Hz
                    
        except Exception as e:
            print(f"Radio connection error: {e}")

    def start(self):
        """Start the GPS broadcaster"""
        self.running = True
        
        # Start GPS reader thread
        gps_thread = threading.Thread(target=self.gps_reader_thread)
        gps_thread.daemon = True
        gps_thread.start()
        
        # Start MAVLink broadcaster thread
        mavlink_thread = threading.Thread(target=self.mavlink_broadcaster_thread)
        mavlink_thread.daemon = True
        mavlink_thread.start()
        
        print("Mobile RTK Station started - Broadcasting GPS via MAVLink")
        print("Press Ctrl+C to stop")
        
        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nStopping Mobile RTK Station...")
            self.running = False

if __name__ == "__main__":
    broadcaster = MAVLinkGPSBroadcaster()
    broadcaster.start()
