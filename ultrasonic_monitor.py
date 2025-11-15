#!/usr/bin/env python3
"""
Ultrasonic Sensor Monitor - Standalone Debugging Tool

This tool connects directly to the ESP32 via serial and displays sensor
readings in real-time. Use this for testing and debugging without ROS2.

Usage:
    python3 ultrasonic_monitor.py [serial_port]

    Default serial port: /dev/ttyUSB0

Examples:
    python3 ultrasonic_monitor.py
    python3 ultrasonic_monitor.py /dev/ttyUSB1
"""

import json
import sys
import time
import serial


class UltrasonicMonitor:
    def __init__(self, serial_port='/dev/ttyUSB0', baud_rate=115200):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.serial = None

        # Statistics
        self.message_count = 0
        self.error_count = 0
        self.start_time = time.time()

    def connect(self):
        """Connect to ESP32 via serial"""
        print(f"Connecting to {self.serial_port} @ {self.baud_rate} baud...")

        try:
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=2.0
            )
            time.sleep(2)  # Wait for Arduino to reset
            print(f"Connected to {self.serial_port}")

            # Read and display startup message
            if self.serial.in_waiting:
                startup_msg = self.serial.readline().decode('utf-8', errors='ignore').strip()
                print(f"ESP32: {startup_msg}")

            print("\n" + "=" * 80)
            print("Ultrasonic Sensor Monitor - Press Ctrl+C to exit")
            print("=" * 80 + "\n")

            return True

        except serial.SerialException as e:
            print(f"ERROR: Failed to connect to {self.serial_port}")
            print(f"       {e}")
            print("\nTroubleshooting:")
            print("  1. Check that ESP32 is connected via USB")
            print("  2. Verify serial port name (ls /dev/ttyUSB*)")
            print("  3. Ensure you have permissions (sudo usermod -a -G dialout $USER)")
            return False

    def run(self):
        """Main monitoring loop"""
        if not self.connect():
            return

        try:
            while True:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()

                    try:
                        # Try to parse as JSON
                        data = json.loads(line)
                        self.display_json(data)
                        self.message_count += 1

                    except json.JSONDecodeError:
                        # Not JSON - probably debug output or status message
                        if line:
                            print(f"[RAW] {line}")
                        self.error_count += 1

                time.sleep(0.01)  # Small delay to reduce CPU usage

        except KeyboardInterrupt:
            print("\n\nShutting down...")
            self.print_summary()

        finally:
            if self.serial and self.serial.is_open:
                self.serial.close()
                print("Serial connection closed")

    def display_json(self, data):
        """Display parsed JSON sensor data"""
        # Clear screen for live updating display
        print("\033[2J\033[H", end='')  # ANSI clear screen + home

        # Header
        print("=" * 80)
        print("Ultrasonic Sensor Array - Live Data")
        print("=" * 80)

        # Timestamp
        if 'timestamp' in data:
            timestamp_ms = data['timestamp']
            print(f"ESP32 Time: {timestamp_ms / 1000.0:.1f} s")

        runtime = time.time() - self.start_time
        print(f"Runtime:    {runtime:.1f} s")
        print()

        # Sensor readings
        if 'sensors' in data:
            print(f"{'Sensor':<15} {'Angle':<10} {'Distance':<12} {'Status':<15}")
            print("-" * 80)

            for sensor in data['sensors']:
                name = sensor.get('name', 'unknown')
                angle = sensor.get('angle', 0)
                distance = sensor.get('distance', 0.0)
                valid = sensor.get('valid', False)

                # Format angle with sign
                angle_str = f"{angle:+4d}°"

                # Format distance
                distance_str = f"{distance:.2f} m"

                # Status indicator
                if valid:
                    if distance < 0.5:
                        status = "✓ CLOSE"
                    elif distance < 2.0:
                        status = "✓ NEAR"
                    else:
                        status = "✓ FAR"
                else:
                    status = "✗ INVALID"

                # Visual distance bar (20 chars = 6m max)
                bar_length = int((distance / 6.0) * 20)
                bar = "█" * bar_length + "·" * (20 - bar_length)

                print(f"{name:<15} {angle_str:<10} {distance_str:<12} {status:<15} {bar}")

            print()

        # Statistics
        hz = self.message_count / runtime if runtime > 0 else 0
        print(f"Messages: {self.message_count} ({hz:.1f} Hz)")
        print(f"Errors:   {self.error_count}")
        print()
        print("Press Ctrl+C to exit")

    def print_summary(self):
        """Print session summary"""
        runtime = time.time() - self.start_time
        print("\n" + "=" * 80)
        print("Session Summary")
        print("=" * 80)
        print(f"Runtime:      {runtime:.1f} s")
        print(f"Messages:     {self.message_count}")
        print(f"Errors:       {self.error_count}")
        if runtime > 0:
            print(f"Average Rate: {self.message_count / runtime:.1f} Hz")
        if self.message_count > 0:
            print(f"Success Rate: {100.0 * self.message_count / (self.message_count + self.error_count):.1f}%")
        print("=" * 80)


def main():
    # Parse command line arguments
    serial_port = '/dev/ttyUSB0'

    if len(sys.argv) > 1:
        serial_port = sys.argv[1]

    # Check for help flag
    if '-h' in sys.argv or '--help' in sys.argv:
        print(__doc__)
        return

    # Create and run monitor
    monitor = UltrasonicMonitor(serial_port=serial_port)
    monitor.run()


if __name__ == '__main__':
    main()
