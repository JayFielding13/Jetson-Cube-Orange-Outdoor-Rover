#!/usr/bin/env python3
"""
SiK Radio Configuration Tool
Configures SiK radios for RTK communication
"""

import serial
import time

class SiKConfig:
    def __init__(self, port='/dev/ttyUSB0', baudrate=57600):
        self.port = port
        self.baudrate = baudrate
        self.radio = None

    def connect(self):
        """Connect to SiK radio"""
        try:
            self.radio = serial.Serial(self.port, self.baudrate, timeout=2)
            time.sleep(1)
            print(f"✅ Connected to SiK radio at {self.port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect: {e}")
            return False

    def enter_at_mode(self):
        """Enter AT command mode"""
        print("🔧 Entering AT command mode...")
        try:
            # Send +++ to enter command mode
            time.sleep(1)
            self.radio.write(b'+++')
            time.sleep(2)

            # Look for OK response
            response = self.radio.read_all().decode('ascii', errors='ignore')
            if 'OK' in response:
                print("✅ Entered AT command mode")
                return True
            else:
                print(f"⚠️  AT mode response: {response}")
                return False
        except Exception as e:
            print(f"❌ Failed to enter AT mode: {e}")
            return False

    def send_at_command(self, command):
        """Send AT command and get response"""
        try:
            self.radio.write(f'AT{command}\r\n'.encode())
            time.sleep(0.5)
            response = self.radio.read_all().decode('ascii', errors='ignore')
            return response.strip()
        except Exception as e:
            print(f"❌ AT command error: {e}")
            return ""

    def configure_for_rtk(self):
        """Configure radio for RTK communication"""
        print("🚀 Configuring SiK radio for RTK...")

        if not self.enter_at_mode():
            return False

        # Standard RTK configuration
        configs = [
            ('I', 'Show radio info'),
            ('S1=40', 'Set air data rate to 64kbps'),  # Higher rate for RTK
            ('S2=64', 'Set mavlink stream rate'),
            ('S3=25', 'Set TX power to 25mW'),
            ('S4=915000', 'Set frequency to 915MHz'),
            ('S5=2', 'Set network ID to 2'),
            ('S6=0', 'Set ECC enabled'),
            ('S7=0', 'Set mavlink framing'),
            ('S8=2', 'Set minimum frequency'),
            ('S9=5', 'Set maximum frequency'),
            ('S10=3', 'Set number of channels'),
            ('&W', 'Write settings to EEPROM'),
            ('Z', 'Reboot radio')
        ]

        for cmd, desc in configs:
            print(f"📝 {desc}...")
            response = self.send_at_command(cmd)
            print(f"   Response: {response}")

            if cmd == 'Z':  # Reboot command
                print("🔄 Radio rebooting...")
                time.sleep(3)
                break

        print("✅ Radio configuration complete!")
        return True

    def show_config(self):
        """Show current radio configuration"""
        print("📋 Current SiK Radio Configuration:")
        print("=" * 40)

        if not self.enter_at_mode():
            return False

        # Show important settings
        settings = [
            ('I', 'Radio Information'),
            ('S1?', 'Air Data Rate'),
            ('S2?', 'Mavlink Rate'),
            ('S3?', 'TX Power'),
            ('S4?', 'Frequency'),
            ('S5?', 'Network ID'),
            ('S6?', 'ECC Enable'),
            ('S10?', 'Number of Channels')
        ]

        for cmd, desc in settings:
            response = self.send_at_command(cmd)
            print(f"{desc:20}: {response}")

        return True

    def exit_at_mode(self):
        """Exit AT command mode"""
        try:
            self.send_at_command('O')  # Return to normal mode
            print("✅ Exited AT command mode")
        except:
            pass

    def cleanup(self):
        """Clean up connection"""
        if self.radio:
            self.exit_at_mode()
            self.radio.close()
            print("🔌 Radio connection closed")

def main():
    import sys

    print("🚀 SiK Radio Configuration Tool")
    print("Choose an option:")
    print("1. Show current configuration")
    print("2. Configure for RTK communication")
    print("3. Exit")

    try:
        choice = input("Enter choice (1-3): ").strip()

        config = SiKConfig()
        if not config.connect():
            return

        if choice == '1':
            config.show_config()
        elif choice == '2':
            config.configure_for_rtk()
        elif choice == '3':
            print("Exiting...")
        else:
            print("Invalid choice")

    except KeyboardInterrupt:
        print("\n🛑 Configuration stopped")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        config.cleanup()

if __name__ == "__main__":
    main()