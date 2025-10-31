#!/usr/bin/env python3
"""
Basic Position Broadcaster for Mobile RTK Station
Sends GPS position data to rover via UDP
"""

import serial
import pynmea2
import json
import socket
import time
import threading

class MobileRTKBroadcaster:
    def __init__(self, rover_ip='192.168.8.70', rover_port=12345, gps_device='/dev/ttyACM0'):
        self.rover_ip = rover_ip
        self.rover_port = rover_port
        self.gps_device = gps_device
        
        # GPS connection
        self.gps_port = None
        
        # UDP socket for rover communication
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # State tracking
        self.running = False
        self.current_position = None
        self.message_count = 0
        self.position_count = 0
        
    def start(self):
        """Start the mobile RTK broadcaster"""
        print("🚀 Starting Mobile RTK Station...")
        
        try:
            # Connect to GPS
            self.gps_port = serial.Serial(self.gps_device, 38400, timeout=1)
            print(f"✅ GPS connected on {self.gps_device}")
            
            # Start broadcasting
            self.running = True
            print(f"📡 Broadcasting to rover at {self.rover_ip}:{self.rover_port}")
            print("📍 Waiting for GPS position fix...\n")
            
            self.broadcast_loop()
            
        except Exception as e:
            print(f"❌ Failed to start: {e}")
            self.stop()
            
    def broadcast_loop(self):
        """Main broadcasting loop"""
        last_position_time = 0
        
        while self.running:
            try:
                # Read GPS data
                line = self.gps_port.readline().decode('ascii', errors='ignore').strip()
                
                if line:
                    self.message_count += 1
                    
                    # Parse position messages
                    if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                        position_data = self.parse_gga_message(line)
                        
                        if position_data:
                            self.current_position = position_data
                            self.position_count += 1
                            
                            # Send to rover every 200ms (5Hz)
                            current_time = time.time()
                            if current_time - last_position_time >= 0.2:
                                self.send_position_to_rover(position_data)
                                last_position_time = current_time
                                
                                # Display status
                                fix_type = position_data['fix_type']
                                sats = position_data['satellites']
                                lat = position_data['latitude']
                                lon = position_data['longitude']
                                acc = position_data['accuracy_estimate']
                                
                                print(f"📍 #{self.position_count:3d} | {fix_type:10s} | {sats:2d} sats | "
                                      f"{lat:10.6f}, {lon:11.6f} | {acc} → Rover")
                
            except KeyboardInterrupt:
                print("\n⏹️  Stopping Mobile RTK Station...")
                break
            except Exception as e:
                print(f"❌ Error in broadcast loop: {e}")
                time.sleep(1)
                
        self.stop()
        
    def parse_gga_message(self, line):
        """Parse NMEA GGA message for position data"""
        try:
            msg = pynmea2.parse(line)
            
            if not (msg.latitude and msg.longitude):
                return None
                
            fix_quality = int(msg.gps_qual) if msg.gps_qual else 0
            satellites = int(msg.num_sats) if msg.num_sats else 0
            
            # Determine fix type and accuracy
            fix_types = {
                0: "NO_FIX",
                1: "GPS_FIX", 
                2: "DGPS_FIX",
                4: "RTK_FIXED",
                5: "RTK_FLOAT"
            }
            
            accuracy_estimates = {
                0: "Unknown",
                1: "±3m",
                2: "±1m", 
                4: "±2cm",
                5: "±10cm"
            }
            
            return {
                'latitude': float(msg.latitude),
                'longitude': float(msg.longitude),
                'altitude': float(msg.altitude) if msg.altitude else 0.0,
                'fix_quality': fix_quality,
                'fix_type': fix_types.get(fix_quality, "UNKNOWN"),
                'satellites': satellites,
                'hdop': float(msg.horizontal_dil) if msg.horizontal_dil else 0.0,
                'accuracy_estimate': accuracy_estimates.get(fix_quality, "Unknown"),
                'timestamp': time.time()
            }
            
        except Exception:
            return None
            
    def send_position_to_rover(self, position_data):
        """Send position data to rover via UDP"""
        try:
            message = {
                'message_type': 'mobile_position',
                'station_id': 'mobile_rtk_001',
                'timestamp': time.time(),
                'position': {
                    'latitude': position_data['latitude'],
                    'longitude': position_data['longitude'],
                    'altitude': position_data['altitude']
                },
                'gps_status': {
                    'fix_type': position_data['fix_type'],
                    'fix_quality': position_data['fix_quality'],
                    'satellites': position_data['satellites'],
                    'hdop': position_data['hdop'],
                    'accuracy': position_data['accuracy_estimate']
                }
            }
            
            # Send UDP message to rover
            message_json = json.dumps(message)
            self.udp_socket.sendto(
                message_json.encode('utf-8'),
                (self.rover_ip, self.rover_port)
            )
            
        except Exception as e:
            print(f"❌ Failed to send to rover: {e}")
            
    def stop(self):
        """Stop the broadcaster"""
        self.running = False
        
        if self.gps_port:
            self.gps_port.close()
            print("GPS connection closed")
            
        self.udp_socket.close()
        print("UDP socket closed")
        print(f"📊 Final stats: {self.message_count} GPS messages, {self.position_count} positions sent")

if __name__ == "__main__":
    # Configure for your network
    broadcaster = MobileRTKBroadcaster(
        rover_ip='192.168.8.70',  # Your outdoor rover IP
        rover_port=12345,         # UDP port for position data
        gps_device='/dev/ttyACM0'  # GPS USB device
    )
    
    try:
        broadcaster.start()
    except KeyboardInterrupt:
        print("\nExiting...")
        broadcaster.stop()
