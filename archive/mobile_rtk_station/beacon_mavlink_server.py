#!/usr/bin/env python3
"""
Mobile Beacon MAVLink Server for QGroundControl
Shows mobile beacon as a vehicle with GPS position
"""

import socket
import time
import serial
import threading
from pymavlink import mavutil
import struct

class BeaconMAVLinkServer:
    def __init__(self):
        self.gps_port = '/dev/ttyACM0'
        self.gps_baud = 115200
        self.tcp_port = 5760
        self.system_id = 2  # Different from base station
        self.component_id = 1

        # GPS data
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.satellites = 0
        self.fix_type = 0
        self.hdop = 99.9
        self.running = True

        # MAVLink connection
        self.mav = None
        self.client_socket = None

    def parse_gps_data(self):
        """Parse GPS NMEA data"""
        try:
            gps = serial.Serial(self.gps_port, self.gps_baud, timeout=1)
            print(f"Connected to GPS on {self.gps_port}")

            while self.running:
                try:
                    line = gps.readline().decode('ascii', errors='ignore').strip()

                    if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                        parts = line.split(',')
                        if len(parts) >= 15 and parts[6]:
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
                    continue

        except Exception as e:
            print(f"GPS connection error: {e}")

    def send_mavlink_messages(self):
        """Send MAVLink messages to QGroundControl"""
        if not self.client_socket:
            return

        try:
            # Create MAVLink instance
            self.mav = mavutil.mavlink.MAVLink(
                file=None,
                srcSystem=self.system_id,
                srcComponent=self.component_id
            )

            last_heartbeat = 0
            last_gps = 0

            while self.running and self.client_socket:
                current_time = time.time()

                # Send heartbeat every 1 second
                if current_time - last_heartbeat > 1.0:
                    heartbeat = self.mav.heartbeat_encode(
                        mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
                        mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                        0,  # base_mode
                        0,  # custom_mode
                        mavutil.mavlink.MAV_STATE_ACTIVE
                    )
                    self.client_socket.send(heartbeat.pack(self.mav))
                    last_heartbeat = current_time

                # Send GPS data every 0.2 seconds (5Hz)
                if current_time - last_gps > 0.2 and self.fix_type > 0:
                    # Convert to required units
                    lat_int = int(self.lat * 1e7)
                    lon_int = int(self.lon * 1e7)
                    alt_int = int(self.alt * 1000)  # mm

                    # Send GPS raw data
                    gps_raw = self.mav.gps_raw_int_encode(
                        int(current_time * 1000000),  # timestamp microseconds
                        self.fix_type,
                        lat_int, lon_int, alt_int,
                        int(self.hdop * 100),  # eph
                        65535,  # epv
                        65535,  # vel
                        65535,  # cog
                        self.satellites,
                        0,      # alt_ellipsoid
                        0,      # h_acc
                        0,      # v_acc
                        0,      # vel_acc
                        0       # hdg_acc
                    )
                    self.client_socket.send(gps_raw.pack(self.mav))

                    # Send global position
                    global_pos = self.mav.global_position_int_encode(
                        int(current_time * 1000),
                        lat_int, lon_int, alt_int, alt_int,
                        0, 0, 0,  # velocities
                        65535     # heading
                    )
                    self.client_socket.send(global_pos.pack(self.mav))

                    last_gps = current_time

                time.sleep(0.1)

        except Exception as e:
            print(f"MAVLink error: {e}")

    def start_server(self):
        """Start TCP server for QGroundControl"""
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(('0.0.0.0', self.tcp_port))
        server_socket.listen(1)
        print(f"Mobile Beacon MAVLink server listening on port {self.tcp_port}")

        while self.running:
            try:
                self.client_socket, addr = server_socket.accept()
                print(f"QGroundControl connected from {addr}")

                # Start MAVLink message sending in thread
                mavlink_thread = threading.Thread(target=self.send_mavlink_messages)
                mavlink_thread.daemon = True
                mavlink_thread.start()

                # Keep connection alive
                while self.running and self.client_socket:
                    try:
                        # Check if connection is still alive
                        self.client_socket.settimeout(1.0)
                        data = self.client_socket.recv(1024)
                        if not data:
                            break
                    except socket.timeout:
                        continue
                    except:
                        break

            except Exception as e:
                print(f"Server error: {e}")
                continue
            finally:
                if self.client_socket:
                    self.client_socket.close()
                    self.client_socket = None

    def start(self):
        """Start GPS parsing and MAVLink server"""
        print("Starting Mobile Beacon MAVLink Server...")
        print(f"System ID: {self.system_id}, Component ID: {self.component_id}")

        # Start GPS parsing thread
        gps_thread = threading.Thread(target=self.parse_gps_data)
        gps_thread.daemon = True
        gps_thread.start()

        # Start MAVLink server
        self.start_server()

if __name__ == "__main__":
    server = BeaconMAVLinkServer()
    try:
        server.start()
    except KeyboardInterrupt:
        print("\nShutting down...")
        server.running = False