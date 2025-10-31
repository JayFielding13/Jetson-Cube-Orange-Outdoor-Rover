#!/usr/bin/env python3
"""
SiK Radio Receiver Test
Receives test messages from laptop via SiK radio
"""

import serial
import time
import threading
from datetime import datetime

class SiKRadioReceiver:
    def __init__(self):
        # SiK radio connection
        self.sik_port = '/dev/ttyUSB0'
        self.sik_baudrate = 57600
        self.sik_radio = None

        # Test statistics
        self.messages_received = 0
        self.bytes_received = 0
        self.start_time = None

    def connect_sik_radio(self):
        """Connect to SiK radio"""
        try:
            self.sik_radio = serial.Serial(
                self.sik_port,
                self.sik_baudrate,
                timeout=1
            )
            time.sleep(2)
            print(f"✅ Connected to SiK radio at {self.sik_port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to SiK radio: {e}")
            return False

    def listen_for_messages(self):
        """Listen for test messages via SiK radio"""
        print("🚀 Starting SiK radio receiver test...")
        print("📡 Listening for messages from laptop")
        print("=" * 50)

        self.start_time = time.time()
        last_stats_time = time.time()
        buffer = ""

        try:
            while True:
                # Read data from SiK radio
                if self.sik_radio.in_waiting > 0:
                    data = self.sik_radio.read(self.sik_radio.in_waiting)
                    received_text = data.decode('ascii', errors='ignore')
                    buffer += received_text
                    self.bytes_received += len(data)

                    # Process complete messages (newline-separated)
                    while '\n' in buffer:
                        message, buffer = buffer.split('\n', 1)
                        if message.strip():
                            self.messages_received += 1
                            timestamp = datetime.now().strftime("%H:%M:%S")
                            print(f"[{timestamp}] 📥 RX #{self.messages_received:03d}: {message.strip()}")

                            # Send acknowledgment back
                            ack_msg = f"ACK_{self.messages_received:03d}_{timestamp}_FROM_BEACON\n"
                            self.sik_radio.write(ack_msg.encode())

                # Print statistics every 5 seconds
                current_time = time.time()
                if current_time - last_stats_time >= 5.0:
                    self.print_statistics()
                    last_stats_time = current_time

                time.sleep(0.1)  # Small delay

        except KeyboardInterrupt:
            print("\n🛑 Receiver test stopped by user")
        except Exception as e:
            print(f"❌ Reception error: {e}")
        finally:
            self.print_final_stats()
            self.cleanup()

    def print_statistics(self):
        """Print current statistics"""
        if self.start_time:
            elapsed = time.time() - self.start_time
            rate = self.messages_received / max(1, elapsed) * 60  # messages per minute
            timestamp = datetime.now().strftime("%H:%M:%S")
            print(f"[{timestamp}] 📊 Messages: {self.messages_received} | "
                  f"Bytes: {self.bytes_received} | "
                  f"Rate: {rate:.1f}/min")

    def print_final_stats(self):
        """Print final test statistics"""
        elapsed = time.time() - self.start_time if self.start_time else 1
        print("\n" + "=" * 50)
        print("📊 SiK Radio Receiver Test Results:")
        print(f"   Messages received: {self.messages_received}")
        print(f"   Total bytes:       {self.bytes_received}")
        print(f"   Test duration:     {elapsed:.1f} seconds")
        print(f"   Message rate:      {self.messages_received/max(1,elapsed)*60:.1f}/min")
        print("=" * 50)

    def cleanup(self):
        """Clean up connections"""
        if self.sik_radio:
            self.sik_radio.close()
            print("📻 SiK radio connection closed")

    def run(self):
        """Run the radio receiver test"""
        print("🚀 SiK Radio Receiver Test")
        print("📍 Mobile beacon listening for laptop transmission")
        print()

        # Connect to SiK radio
        if not self.connect_sik_radio():
            return False

        # Listen for messages
        self.listen_for_messages()
        return True

def main():
    receiver = SiKRadioReceiver()
    try:
        receiver.run()
    except KeyboardInterrupt:
        print("\n🛑 SiK Radio Receiver Test stopped")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()