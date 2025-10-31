#!/usr/bin/env python3
"""
WiFi RTK Receiver
Connects to base station WiFi RTK transmitter and forwards corrections to GPS
Much more reliable and higher bandwidth than SiK radio
"""

import socket
import serial
import time
import threading
from datetime import datetime

class WiFiRTKReceiver:
    def __init__(self):
        # Base station connection
        self.base_station_ip = '192.168.254.128'  # Base station IP
        self.base_station_port = 8101
        self.wifi_socket = None

        # GPS module connection (for RTCM injection)
        self.gps_port = '/dev/ttyACM0'  # u-blox GPS
        self.gps_baudrate = 38400
        self.gps_serial = None

        # Statistics
        self.bytes_received = 0
        self.bytes_forwarded = 0
        self.packets_received = 0
        self.connection_attempts = 0

    def connect_to_base_station(self):
        """Connect to base station WiFi RTK transmitter"""
        try:
            self.connection_attempts += 1
            print(f"🔌 Connecting to base station at {self.base_station_ip}:{self.base_station_port}...")

            self.wifi_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.wifi_socket.settimeout(10)
            self.wifi_socket.connect((self.base_station_ip, self.base_station_port))

            print(f"✅ Connected to WiFi RTK transmitter (attempt #{self.connection_attempts})")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to base station: {e}")
            if self.wifi_socket:
                self.wifi_socket.close()
                self.wifi_socket = None
            return False

    def connect_gps_module(self):
        """Connect to GPS module for RTCM injection"""
        try:
            self.gps_serial = serial.Serial(
                self.gps_port,
                self.gps_baudrate,
                timeout=1
            )
            time.sleep(2)
            print(f"✅ Connected to GPS module at {self.gps_port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to GPS module: {e}")
            return False

    def receive_and_forward(self):
        """Main loop - receive corrections and forward to GPS"""
        print("🚀 Starting WiFi RTK correction reception...")
        print("📡 Receiving RTCM corrections from base station")
        print("📍 Forwarding to GPS module for RTK processing")
        print("=" * 60)

        last_stats_time = time.time()
        last_data_time = time.time()
        reconnect_interval = 5  # seconds

        try:
            while True:
                # Check if we need to reconnect
                if not self.wifi_socket:
                    if time.time() - last_data_time > reconnect_interval:
                        print("🔄 Attempting to reconnect...")
                        if self.connect_to_base_station():
                            last_data_time = time.time()
                        else:
                            time.sleep(2)
                            continue

                try:
                    # Receive data from base station
                    self.wifi_socket.settimeout(5.0)
                    data = self.wifi_socket.recv(4096)

                    if not data:
                        print("⚠️  Base station connection closed")
                        self.wifi_socket.close()
                        self.wifi_socket = None
                        continue

                    self.bytes_received += len(data)
                    self.packets_received += 1
                    last_data_time = time.time()

                    # Forward to GPS module
                    if self.gps_serial:
                        try:
                            self.gps_serial.write(data)
                            self.bytes_forwarded += len(data)
                        except Exception as e:
                            print(f"⚠️  GPS write error: {e}")
                            # Try to reconnect GPS
                            try:
                                self.gps_serial.close()
                            except:
                                pass
                            self.connect_gps_module()

                    # Print statistics every 5 seconds
                    current_time = time.time()
                    if current_time - last_stats_time >= 5.0:
                        self.print_statistics()
                        last_stats_time = current_time

                except socket.timeout:
                    # Check for stale connection
                    if time.time() - last_data_time > 15:
                        print("⚠️  No data received for 15 seconds, reconnecting...")
                        try:
                            self.wifi_socket.close()
                        except:
                            pass
                        self.wifi_socket = None
                    continue
                except Exception as e:
                    print(f"❌ Reception error: {e}")
                    try:
                        self.wifi_socket.close()
                    except:
                        pass
                    self.wifi_socket = None
                    time.sleep(2)

        except KeyboardInterrupt:
            print("\n🛑 Reception stopped by user")
        except Exception as e:
            print(f"❌ Unexpected error: {e}")
        finally:
            self.cleanup()

    def print_statistics(self):
        """Print reception statistics"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        rate_kbps = (self.bytes_received / 1024) / max(1, time.time() - getattr(self, 'start_time', time.time()))
        connection_status = "🟢 Connected" if self.wifi_socket else "🔴 Disconnected"

        print(f"[{timestamp}] 📊 {connection_status} | "
              f"RX: {self.bytes_received:,} bytes | "
              f"GPS: {self.bytes_forwarded:,} bytes | "
              f"Rate: {rate_kbps:.1f} KB/s | "
              f"Packets: {self.packets_received:,}")

    def test_connection(self):
        """Test basic connectivity to base station"""
        print("🧪 Testing connection to base station...")
        try:
            test_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            test_socket.settimeout(5)
            test_socket.connect((self.base_station_ip, self.base_station_port))
            test_socket.close()
            print("✅ Base station is reachable")
            return True
        except Exception as e:
            print(f"❌ Base station unreachable: {e}")
            return False

    def cleanup(self):
        """Clean up connections"""
        print("\n🧹 Cleaning up connections...")

        if self.wifi_socket:
            self.wifi_socket.close()
            print("📡 WiFi connection closed")

        if self.gps_serial:
            self.gps_serial.close()
            print("📍 GPS module connection closed")

    def run(self):
        """Run the WiFi RTK receiver"""
        print("🚀 WiFi RTK Receiver")
        print("📍 High-bandwidth RTK correction reception via WiFi")
        print("💡 Much more reliable than SiK radio!")
        print()

        self.start_time = time.time()

        # Test connectivity first
        if not self.test_connection():
            print("💡 Make sure base station WiFi RTK transmitter is running")
            return False

        # Connect to GPS module
        if not self.connect_gps_module():
            return False

        # Connect to base station
        if not self.connect_to_base_station():
            print("⚠️  Initial connection failed, will retry during operation")

        # Start receiving
        self.receive_and_forward()
        return True

def main():
    receiver = WiFiRTKReceiver()
    try:
        receiver.run()
    except KeyboardInterrupt:
        print("\n🛑 WiFi RTK Receiver stopped")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()