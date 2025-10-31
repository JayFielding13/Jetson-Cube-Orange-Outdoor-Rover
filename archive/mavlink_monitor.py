#!/usr/bin/env python3
"""
Simple MAVLink Monitor - Receive and display MAVLink messages from Mobile RTK Station
"""

import serial
import time
import struct

class MAVLinkMonitor:
    def __init__(self, port='/dev/ttyUSB0', baud=57600):
        self.port = port
        self.baud = baud
        
    def parse_mavlink_packet(self, data, pos):
        """Parse a MAVLink packet starting at position pos"""
        if pos + 8 > len(data):
            return None, 0
            
        # MAVLink v1 packet format
        stx = data[pos]
        if stx != 0xFE:
            return None, 1
            
        length = data[pos + 1]
        seq = data[pos + 2]
        system_id = data[pos + 3]
        component_id = data[pos + 4]
        msg_id = data[pos + 5]
        
        packet_size = length + 8  # Header (6) + payload (length) + checksum (2)
        
        if pos + packet_size > len(data):
            return None, 1  # Not enough data
            
        payload = data[pos + 6:pos + 6 + length]
        checksum = struct.unpack('<H', data[pos + 6 + length:pos + 8 + length])[0]
        
        return {
            'system_id': system_id,
            'component_id': component_id,
            'msg_id': msg_id,
            'seq': seq,
            'payload': payload,
            'checksum': checksum,
            'size': packet_size
        }, packet_size
    
    def decode_gps_message(self, payload):
        """Decode GPS_RAW_INT message (ID 24)"""
        if len(payload) < 30:
            return None
            
        try:
            data = struct.unpack('<QIiiiHHHHiB', payload[:30])
            return {
                'time_usec': data[0],
                'lat': data[1] / 1e7,  # Convert to degrees
                'lon': data[2] / 1e7,  # Convert to degrees
                'alt': data[3] / 1000,  # Convert to meters
                'eph': data[4],
                'epv': data[5],
                'vel': data[6],
                'cog': data[7],
                'fix_type': data[9],
                'satellites': data[10]
            }
        except:
            return None
    
    def decode_heartbeat(self, payload):
        """Decode HEARTBEAT message (ID 0)"""
        if len(payload) < 9:
            return None
            
        try:
            data = struct.unpack('<IBBBBB', payload[:9])
            return {
                'custom_mode': data[0],
                'type': data[1],
                'autopilot': data[2],
                'base_mode': data[3],
                'system_status': data[4],
                'mavlink_version': data[5]
            }
        except:
            return None
    
    def monitor(self, duration=60):
        """Monitor MAVLink messages for specified duration"""
        print(f"🔍 MAVLink Monitor Starting")
        print(f"📻 Port: {self.port} at {self.baud} baud")
        print(f"⏱️  Duration: {duration} seconds")
        print(f"🎯 Looking for Mobile RTK Station (System ID 255)")
        print("=" * 60)
        
        stats = {
            'total_packets': 0,
            'heartbeats': 0,
            'gps_messages': 0,
            'other_messages': 0,
            'parse_errors': 0
        }
        
        try:
            with serial.Serial(self.port, self.baud, timeout=1) as radio:
                print("✅ SiK radio connected")
                
                start_time = time.time()
                buffer = b''
                
                while time.time() - start_time < duration:
                    # Read data
                    new_data = radio.read(1024)
                    if new_data:
                        buffer += new_data
                        
                        # Process complete packets
                        pos = 0
                        while pos < len(buffer):
                            packet, advance = self.parse_mavlink_packet(buffer, pos)
                            
                            if packet is None:
                                pos += advance
                                continue
                                
                            stats['total_packets'] += 1
                            
                            # Process specific message types
                            if packet['msg_id'] == 0:  # HEARTBEAT
                                stats['heartbeats'] += 1
                                hb = self.decode_heartbeat(packet['payload'])
                                if hb:
                                    print(f"💓 HEARTBEAT from System {packet['system_id']}: "
                                          f"Type={hb['type']}, Status={hb['system_status']}")
                                          
                            elif packet['msg_id'] == 24:  # GPS_RAW_INT
                                stats['gps_messages'] += 1
                                gps = self.decode_gps_message(packet['payload'])
                                if gps:
                                    print(f"📍 GPS from System {packet['system_id']}: "
                                          f"{gps['lat']:.6f}, {gps['lon']:.6f}, "
                                          f"Alt={gps['alt']:.1f}m, Fix={gps['fix_type']}, "
                                          f"Sats={gps['satellites']}")
                            else:
                                stats['other_messages'] += 1
                                print(f"📦 Message ID {packet['msg_id']} from System {packet['system_id']}")
                            
                            pos += advance
                        
                        # Keep only unprocessed data
                        buffer = buffer[pos:]
                        
                        # Limit buffer size
                        if len(buffer) > 4096:
                            buffer = buffer[-2048:]
                    
                    time.sleep(0.1)
                    
        except Exception as e:
            print(f"❌ Error: {e}")
            return
        
        print("=" * 60)
        print(f"📊 Statistics:")
        print(f"   Total packets: {stats['total_packets']}")
        print(f"   Heartbeats: {stats['heartbeats']}")
        print(f"   GPS messages: {stats['gps_messages']}")
        print(f"   Other messages: {stats['other_messages']}")
        
        if stats['heartbeats'] > 0 and stats['gps_messages'] > 0:
            print("🎯 SUCCESS: Mobile RTK Station working perfectly!")
        else:
            print("❌ No valid MAVLink data received")

if __name__ == "__main__":
    monitor = MAVLinkMonitor()
    monitor.monitor(30)  # Monitor for 30 seconds
