#!/usr/bin/env python3
"""
MAVLink Position Broadcaster for Mobile RTK Station
Sends GPS position to Mission Planner via SiK radio
"""

import serial
import pynmea2
import time
import struct
# import pymavlink  # Will install this later

class MobileStationMAVLink:
    def __init__(self, gps_device='/dev/ttyACM0', radio_device='/dev/ttyUSB0'):
        self.gps_device = gps_device
        self.radio_device = radio_device
        
        # MAVLink configuration
        self.system_id = 200  # Mobile station system ID
        self.component_id = 1  # Ground control station component
        
        # Connections
        self.gps_port = None
        self.radio_port = None
        
        # State
        self.running = False
        self.current_position = None
        
    def start(self):
        """Start the MAVLink broadcaster"""
        print("🚀 Starting Mobile RTK Station MAVLink Broadcaster...")
        
        try:
            # Connect to GPS
            self.gps_port = serial.Serial(self.gps_device, 38400, timeout=1)
            print(f"✅ GPS connected on {self.gps_device}")
            
            # Connect to SiK radio
            self.radio_port = serial.Serial(self.radio_device, 57600, timeout=1)  # Common SiK baud rate
            print(f"✅ SiK radio connected on {self.radio_device}")
            
            print("📡 Broadcasting GPS position to Mission Planner...")
            print("🗺️  Mobile station should appear on Mission Planner map\n")
            
            self.main_loop()
            
        except Exception as e:
            print(f"❌ Failed to start: {e}")
            self.stop()
            
    def main_loop(self):
        """Main broadcasting loop"""
        last_heartbeat = 0
        last_position = 0
        position_count = 0
        
        self.running = True
        
        while self.running:
            try:
                current_time = time.time()
                
                # Send heartbeat every 1 second
                if current_time - last_heartbeat >= 1.0:
                    self.send_heartbeat()
                    last_heartbeat = current_time
                
                # Read GPS data
                line = self.gps_port.readline().decode('ascii', errors='ignore').strip()
                
                if line and (line.startswith('$GNGGA') or line.startswith('$GPGGA')):
                    position_data = self.parse_gga_message(line)
                    
                    if position_data and (current_time - last_position >= 0.2):  # 5Hz
                        self.send_position_update(position_data)
                        last_position = current_time
                        position_count += 1
                        
                        # Display status
                        fix_type = position_data['fix_type']
                        sats = position_data['satellites']
                        lat = position_data['latitude']
                        lon = position_data['longitude']
                        
                        print(f"📍 #{position_count:3d} | {fix_type:10s} | {sats:2d} sats | "
                              f"{lat:10.6f}, {lon:11.6f} → Mission Planner")
                
            except KeyboardInterrupt:
                print("\n⏹️  Stopping MAVLink broadcaster...")
                break
            except Exception as e:
                print(f"❌ Error: {e}")
                time.sleep(1)
                
        self.stop()
        
    def parse_gga_message(self, line):
        """Parse NMEA GGA message"""
        try:
            msg = pynmea2.parse(line)
            if not (msg.latitude and msg.longitude):
                return None
                
            fix_quality = int(msg.gps_qual) if msg.gps_qual else 0
            satellites = int(msg.num_sats) if msg.num_sats else 0
            
            fix_types = {0: "NO_FIX", 1: "GPS_FIX", 2: "DGPS_FIX", 4: "RTK_FIXED", 5: "RTK_FLOAT"}
            
            return {
                'latitude': float(msg.latitude),
                'longitude': float(msg.longitude), 
                'altitude': float(msg.altitude) if msg.altitude else 0.0,
                'fix_quality': fix_quality,
                'fix_type': fix_types.get(fix_quality, "UNKNOWN"),
                'satellites': satellites,
                'timestamp': time.time()
            }
        except:
            return None
            
    def send_heartbeat(self):
        """Send MAVLink heartbeat message"""
        # TODO: Implement with pymavlink
        # This will identify the mobile station to Mission Planner
        print("💓 Heartbeat sent", end='\r')
        pass
        
    def send_position_update(self, position_data):
        """Send GPS position as MAVLink GLOBAL_POSITION_INT message"""
        # TODO: Implement with pymavlink
        # This will show mobile station position on Mission Planner map
        pass
        
    def stop(self):
        """Stop the broadcaster"""
        self.running = False
        
        if self.gps_port:
            self.gps_port.close()
        if self.radio_port:
            self.radio_port.close()
            
        print("MAVLink broadcaster stopped")

if __name__ == "__main__":
    # Will auto-detect SiK radio when connected
    broadcaster = MobileStationMAVLink(
        gps_device='/dev/ttyACM0',
        radio_device='/dev/ttyUSB0'  # SiK radio will appear here
    )
    
    try:
        broadcaster.start()
    except KeyboardInterrupt:
        broadcaster.stop()
