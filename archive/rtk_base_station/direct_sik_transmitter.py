#!/usr/bin/env python3
"""
Direct GPS-to-SiK RTK Transmitter
Reads RTK corrections directly from GPS and transmits via SiK radio
"""

import serial
import time
import threading
from datetime import datetime

class DirectSiKTransmitter:
    def __init__(self):
        # Base station GPS (generating corrections)
        self.gps_port = '/dev/ttyACM0'
        self.gps_baudrate = 115200
        self.gps = None

        # SiK radio connection
        self.sik_port = '/dev/ttyUSB0'
        self.sik_baudrate = 57600
        self.sik_radio = None

        # Statistics
        self.bytes_received = 0
        self.bytes_transmitted = 0
        self.rtcm_packets = 0

    def connect_gps(self):
        """Connect to base station GPS"""
        try:
            self.gps = serial.Serial(
                self.gps_port,
                self.gps_baudrate,
                timeout=1
            )
            time.sleep(2)
            print(f"✅ Connected to base station GPS at {self.gps_port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to GPS: {e}")
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
            time.sleep(2)
            print(f"✅ Connected to SiK radio at {self.sik_port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to SiK radio: {e}")
            return False

    def is_rtcm_message(self, data):
        """Check if data contains RTCM message"""
        if len(data) < 3:
            return False
        # RTCM3 messages start with 0xD3
        return data[0] == 0xD3

    def transmit_gps_data(self):
        """Read GPS data and transmit RTCM corrections via SiK"""
        print("🚀 Starting direct GPS-to-SiK transmission...")
        print("📡 Broadcasting RTK corrections via SiK radio")
        print("=" * 50)

        last_stats_time = time.time()
        buffer = b''

        try:
            while True:
                # Read data from GPS
                try:
                    data = self.gps.read(256)
                    if not data:
                        continue

                    self.bytes_received += len(data)
                    buffer += data

                    # Look for RTCM messages or send all data
                    # For simplicity, we'll transmit all GPS data
                    # The rover can filter what it needs
                    if len(buffer) >= 100:  # Send in chunks
                        # Transmit via SiK radio
                        self.sik_radio.write(buffer)
                        self.bytes_transmitted += len(buffer)
                        self.rtcm_packets += 1
                        buffer = b''

                    # Print statistics every 5 seconds
                    current_time = time.time()
                    if current_time - last_stats_time >= 5.0:
                        self.print_statistics()
                        last_stats_time = current_time

                except Exception as e:
                    print(f"❌ Transmission error: {e}")
                    time.sleep(1)

        except KeyboardInterrupt:
            print("\n🛑 Transmission stopped by user")
        except Exception as e:
            print(f"❌ Unexpected error: {e}")
        finally:
            self.cleanup()

    def print_statistics(self):
        """Print transmission statistics"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        tx_rate = self.bytes_transmitted / (1024.0)  # KB
        print(f"[{timestamp}] 📊 RX: {self.bytes_received:,} bytes | "
              f"TX: {tx_rate:.1f} KB | "
              f"Packets: {self.rtcm_packets:,}")

    def cleanup(self):
        """Clean up connections"""
        if self.gps:
            self.gps.close()
            print("📍 GPS connection closed")

        if self.sik_radio:
            self.sik_radio.close()
            print("📻 SiK radio connection closed")

    def run(self):
        """Run the direct transmitter"""
        print("🚀 Direct GPS-to-SiK Transmitter")
        print("📍 Connecting to GPS and SiK radio...")
        print()

        # Connect to GPS
        if not self.connect_gps():
            return False

        # Connect to SiK radio
        if not self.connect_sik_radio():
            return False

        # Start transmission
        self.transmit_gps_data()
        return True

def main():
    transmitter = DirectSiKTransmitter()
    try:
        transmitter.run()
    except KeyboardInterrupt:
        print("\n🛑 Direct SiK Transmitter stopped")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()