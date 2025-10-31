#!/usr/bin/env python3
"""
SiK Radio RTK Transmitter
Connects to RTCM server and broadcasts corrections via SiK radio
"""

import socket
import serial
import time
import threading
from datetime import datetime

class SiKRTKTransmitter:
    def __init__(self):
        # RTCM server connection
        self.rtcm_host = 'localhost'
        self.rtcm_port = 2101
        self.rtcm_socket = None

        # SiK radio connection
        self.sik_port = '/dev/ttyUSB0'
        self.sik_baudrate = 57600
        self.sik_radio = None

        # Statistics
        self.bytes_received = 0
        self.bytes_transmitted = 0
        self.packets_sent = 0

    def connect_rtcm_server(self):
        """Connect to RTCM correction server"""
        try:
            print(f"🔌 Attempting to connect to RTCM server at {self.rtcm_host}:{self.rtcm_port}...")
            self.rtcm_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.rtcm_socket.settimeout(10)  # Increased timeout
            self.rtcm_socket.connect((self.rtcm_host, self.rtcm_port))

            # Test receiving data
            self.rtcm_socket.settimeout(3)
            test_data = self.rtcm_socket.recv(32)
            if test_data:
                print(f"✅ Connected to RTCM server - received {len(test_data)} bytes")
                # Put the data back by reconnecting (simple approach)
                self.rtcm_socket.close()
                self.rtcm_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.rtcm_socket.settimeout(1)
                self.rtcm_socket.connect((self.rtcm_host, self.rtcm_port))
                return True
            else:
                print("⚠️  Connected but no data available")
                return False
        except Exception as e:
            print(f"❌ Failed to connect to RTCM server: {e}")
            if self.rtcm_socket:
                self.rtcm_socket.close()
                self.rtcm_socket = None
            return False

    def connect_sik_radio(self):
        """Connect to SiK radio"""
        try:
            self.sik_radio = serial.Serial(
                self.sik_port,
                self.sik_baudrate,
                timeout=1,
                write_timeout=1
            )
            time.sleep(2)  # Allow radio to initialize
            print(f"✅ Connected to SiK radio at {self.sik_port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to SiK radio: {e}")
            return False

    def transmit_corrections(self):
        """Main transmission loop"""
        print("🚀 Starting RTK correction transmission...")
        print("📡 Broadcasting RTCM corrections via SiK radio")
        print("=" * 50)

        last_stats_time = time.time()

        try:
            while True:
                # Receive data from RTCM server
                try:
                    data = self.rtcm_socket.recv(1024)
                    if not data:
                        print("⚠️  RTCM server connection closed")
                        break

                    self.bytes_received += len(data)

                    # Transmit via SiK radio
                    self.sik_radio.write(data)
                    self.bytes_transmitted += len(data)
                    self.packets_sent += 1

                    # Print statistics every 5 seconds
                    current_time = time.time()
                    if current_time - last_stats_time >= 5.0:
                        self.print_statistics()
                        last_stats_time = current_time

                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"❌ Transmission error: {e}")
                    break

        except KeyboardInterrupt:
            print("\n🛑 Transmission stopped by user")
        except Exception as e:
            print(f"❌ Unexpected error: {e}")
        finally:
            self.cleanup()

    def print_statistics(self):
        """Print transmission statistics"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f"[{timestamp}] 📊 RX: {self.bytes_received:,} bytes | "
              f"TX: {self.bytes_transmitted:,} bytes | "
              f"Packets: {self.packets_sent:,}")

    def cleanup(self):
        """Clean up connections"""
        if self.rtcm_socket:
            self.rtcm_socket.close()
            print("🔌 RTCM server connection closed")

        if self.sik_radio:
            self.sik_radio.close()
            print("📻 SiK radio connection closed")

    def run(self):
        """Run the RTK transmitter"""
        print("🚀 SiK Radio RTK Transmitter")
        print("📍 Connecting to RTCM server and SiK radio...")
        print()

        # Connect to RTCM server
        if not self.connect_rtcm_server():
            return False

        # Connect to SiK radio
        if not self.connect_sik_radio():
            return False

        # Start transmission
        self.transmit_corrections()
        return True

def main():
    transmitter = SiKRTKTransmitter()
    try:
        transmitter.run()
    except KeyboardInterrupt:
        print("\n🛑 SiK RTK Transmitter stopped")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()