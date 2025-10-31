#!/usr/bin/env python3
"""
Base Station Position Broadcaster
Broadcasts base station GPS position via MAVLink for QGroundControl display
"""

import serial
import time
import socket
from pymavlink import mavutil
import threading
import json

class BaseStationBroadcaster:
    def __init__(self):
        self.gps_port = '/dev/ttyACM0'
        self.gps_baud = 115200
        self.tcp_port = 5761  # Different port from mobile beacon
        self.system_id = 1    # Base station system ID
        self.component_id = 1

        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.satellites = 0
        self.fix_type = 0
        self.hdop = 99.9

        self.running = True

    def parse_gps_data(self):
        """Parse GPS NMEA data from base station GPS"""
        try:
            gps = serial.Serial(self.gps_port, self.gps_baud, timeout=1)
            print(f"Connected to base station GPS on {self.gps_port}")

            while self.running:
                try:
                    line = gps.readline().decode('ascii', errors='ignore').strip()

                    if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                        parts = line.split(',')
                        if len(parts) >= 15 and parts[6]:  # Valid fix
                            # Parse latitude
                            if parts[2] and parts[3]:
                                lat_deg = float(parts[2][:2])
                                lat_min = float(parts[2][2:])
                                self.lat = lat_deg + lat_min/60.0
                                if parts[3] == 'S':
                                    self.lat = -self.lat

                            # Parse longitude
                            if parts[4] and parts[5]:
                                lon_deg = float(parts[4][:3])
                                lon_min = float(parts[4][3:])
                                self.lon = lon_deg + lon_min/60.0
                                if parts[5] == 'W':
                                    self.lon = -self.lon

                            # Parse other data
                            self.fix_type = int(parts[6]) if parts[6] else 0
                            self.satellites = int(parts[7]) if parts[7] else 0
                            self.alt = float(parts[9]) if parts[9] else 0.0
                            self.hdop = float(parts[8]) if parts[8] else 99.9

                except Exception as e:
                    print(f"GPS parsing error: {e}")
                    continue

        except Exception as e:
            print(f"GPS connection error: {e}")

    def mavlink_server(self):
        """TCP server for QGroundControl connection"""
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind(('0.0.0.0', self.tcp_port))
            server_socket.listen(1)
            print(f"Base station MAVLink server listening on port {self.tcp_port}")

            while self.running:
                try:
                    client_socket, addr = server_socket.accept()
                    print(f"QGroundControl connected from {addr}")

                    # Create MAVLink connection
                    mav = mavutil.mavlink_connection('tcp:localhost:' + str(self.tcp_port),
                                                   source_system=self.system_id,
                                                   source_component=self.component_id)

                    while self.running:
                        # Send heartbeat
                        mav.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
                            mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                            0, 0, mavutil.mavlink.MAV_STATE_ACTIVE
                        )

                        # Send GPS position if we have a fix
                        if self.fix_type > 0 and self.lat != 0.0 and self.lon != 0.0:
                            # Convert to required units
                            lat_int = int(self.lat * 1e7)
                            lon_int = int(self.lon * 1e7)
                            alt_int = int(self.alt * 1000)  # mm

                            # Send GPS raw data
                            mav.mav.gps_raw_int_send(
                                int(time.time() * 1000),  # timestamp
                                self.fix_type,
                                lat_int, lon_int, alt_int,
                                int(self.hdop * 100),  # eph
                                65535,  # epv (unknown)
                                65535,  # vel (unknown)
                                65535,  # cog (unknown)
                                self.satellites,
                                0       # alt_ellipsoid
                            )

                            # Send global position
                            mav.mav.global_position_int_send(
                                int(time.time() * 1000),
                                lat_int, lon_int, alt_int, alt_int,
                                0, 0, 0,  # velocities
                                65535     # heading unknown
                            )

                        # Send data to client
                        try:
                            data = mav.recv_msg()
                            if data:
                                client_socket.send(data.get_msgbuf())
                        except:
                            pass

                        time.sleep(0.1)  # 10 Hz update rate

                except Exception as e:
                    print(f"Client connection error: {e}")
                    continue

        except Exception as e:
            print(f"MAVLink server error: {e}")

    def start(self):
        """Start GPS parsing and MAVLink server"""
        print("Starting Base Station Position Broadcaster...")
        print(f"Base Station will appear as System ID {self.system_id} on port {self.tcp_port}")

        # Start GPS parsing thread
        gps_thread = threading.Thread(target=self.parse_gps_data)
        gps_thread.daemon = True
        gps_thread.start()

        # Start MAVLink server thread
        mavlink_thread = threading.Thread(target=self.mavlink_server)
        mavlink_thread.daemon = True
        mavlink_thread.start()

        try:
            while True:
                print(f"Base Station Position: {self.lat:.7f}, {self.lon:.7f}, Alt: {self.alt:.1f}m")
                print(f"GPS Fix: {self.fix_type}, Satellites: {self.satellites}, HDOP: {self.hdop:.1f}")
                print("=" * 60)
                time.sleep(5)
        except KeyboardInterrupt:
            print("\nShutting down...")
            self.running = False

if __name__ == "__main__":
    broadcaster = BaseStationBroadcaster()
    broadcaster.start()