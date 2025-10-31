#!/usr/bin/env python3
"""
SiK Radio Communication Test
Tests basic radio communication between laptop and mobile beacon
"""

import serial
import time
import threading
from datetime import datetime

class SiKRadioTest:
    def __init__(self):
        # SiK radio connection
        self.sik_port = '/dev/ttyUSB0'
        self.sik_baudrate = 57600
        self.sik_radio = None

        # Test statistics
        self.messages_sent = 0
        self.bytes_sent = 0

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

    def send_test_messages(self):
        """Send test messages via SiK radio"""
        print("🚀 Starting SiK radio test transmission...")
        print("📡 Sending test messages to mobile beacon")
        print("=" * 50)

        try:
            for i in range(60):  # Send for 1 minute
                # Create test message
                timestamp = datetime.now().strftime("%H:%M:%S")
                test_message = f"TEST_MSG_{i+1:03d}_{timestamp}_FROM_LAPTOP\n"

                # Send via SiK radio
                self.sik_radio.write(test_message.encode())
                self.messages_sent += 1
                self.bytes_sent += len(test_message)

                print(f"[{timestamp}] 📤 Sent message {i+1:03d} ({len(test_message)} bytes)")

                # Check for any received data
                if self.sik_radio.in_waiting > 0:
                    received = self.sik_radio.read(self.sik_radio.in_waiting)
                    print(f"📥 Received: {received.decode('ascii', errors='ignore').strip()}")

                time.sleep(1)  # Send every second

        except KeyboardInterrupt:
            print("\n🛑 Test stopped by user")
        except Exception as e:
            print(f"❌ Transmission error: {e}")
        finally:
            self.print_final_stats()
            self.cleanup()

    def print_final_stats(self):
        """Print final test statistics"""
        print("\n" + "=" * 50)
        print("📊 SiK Radio Test Results:")
        print(f"   Messages sent: {self.messages_sent}")
        print(f"   Total bytes:   {self.bytes_sent}")
        print(f"   Average size:  {self.bytes_sent/max(1,self.messages_sent):.1f} bytes/msg")
        print("=" * 50)

    def cleanup(self):
        """Clean up connections"""
        if self.sik_radio:
            self.sik_radio.close()
            print("📻 SiK radio connection closed")

    def run(self):
        """Run the radio test"""
        print("🚀 SiK Radio Communication Test")
        print("📍 Testing radio link between laptop and mobile beacon")
        print()

        # Connect to SiK radio
        if not self.connect_sik_radio():
            return False

        # Send test messages
        self.send_test_messages()
        return True

def main():
    tester = SiKRadioTest()
    try:
        tester.run()
    except KeyboardInterrupt:
        print("\n🛑 SiK Radio Test stopped")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()