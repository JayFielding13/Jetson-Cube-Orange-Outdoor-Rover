#!/usr/bin/env python3
"""
Ultrasonic Sensor Visualizer - Real-time Sensor Testing

This tool creates a visual representation of all 6 ultrasonic sensors
in a top-down view, showing which sensors are detecting objects and
at what distance. Perfect for verifying sensor wiring and placement.

Usage:
    python3 ultrasonic_visualizer.py [serial_port]

    Default: /dev/ttyUSB0

Controls:
    Ctrl+C - Exit
"""

import json
import math
import sys
import time
import serial


class UltrasonicVisualizer:
    def __init__(self, serial_port='/dev/ttyUSB0', baud_rate=115200):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.serial = None

        # Sensor configuration (name, angle in degrees)
        self.sensors = {
            'front': {'angle': 0, 'distance': 6.0, 'valid': False},
            'corner_right': {'angle': 45, 'distance': 6.0, 'valid': False},
            'side_right': {'angle': 90, 'distance': 6.0, 'valid': False},
            'rear': {'angle': 180, 'distance': 6.0, 'valid': False},
            'side_left': {'angle': -90, 'distance': 6.0, 'valid': False},
            'corner_left': {'angle': -45, 'distance': 6.0, 'valid': False},
        }

        self.message_count = 0
        self.start_time = time.time()

    def connect(self):
        """Connect to ESP32"""
        try:
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            time.sleep(0.5)
            return True
        except Exception as e:
            print(f"ERROR: Could not connect to {self.serial_port}")
            print(f"       {e}")
            return False

    def draw_radar(self):
        """Draw top-down radar view of sensors"""
        # Clear screen
        print("\033[2J\033[H", end='')

        print("=" * 80)
        print("Ultrasonic Sensor Array - Live Visualization")
        print("=" * 80)
        print()

        # Draw top-down view
        width = 60
        height = 25
        center_x = width // 2
        center_y = height // 2

        # Create grid
        grid = [[' ' for _ in range(width)] for _ in range(height)]

        # Draw rover body (center)
        rover_size = 2
        for dy in range(-rover_size, rover_size + 1):
            for dx in range(-rover_size, rover_size + 1):
                y = center_y + dy
                x = center_x + dx
                if 0 <= y < height and 0 <= x < width:
                    grid[y][x] = '█'

        # Draw sensors and detections
        max_distance = 6.0  # meters
        scale = 3.5  # pixels per meter

        for name, data in self.sensors.items():
            angle_rad = math.radians(data['angle'])
            distance = data['distance'] if data['valid'] else max_distance

            # Sensor ray direction
            dx = math.sin(angle_rad)
            dy = -math.cos(angle_rad)  # Negative because y increases downward

            # Draw sensor ray
            ray_length = min(distance, max_distance) * scale
            for i in range(1, int(ray_length)):
                ray_x = int(center_x + dx * i)
                ray_y = int(center_y + dy * i)

                if 0 <= ray_y < height and 0 <= ray_x < width:
                    if data['valid']:
                        # Valid detection - show obstacle
                        if i < ray_length - 1:
                            grid[ray_y][ray_x] = '·'
                        else:
                            grid[ray_y][ray_x] = '●'  # Obstacle
                    else:
                        # No detection - show faint ray
                        grid[ray_y][ray_x] = '·'

        # Draw sensor labels at edges
        label_distance = max_distance * scale + 3
        for name, data in self.sensors.items():
            angle_rad = math.radians(data['angle'])
            dx = math.sin(angle_rad)
            dy = -math.cos(angle_rad)

            label_x = int(center_x + dx * label_distance)
            label_y = int(center_y + dy * label_distance)

            # Place label
            if 0 <= label_y < height and 0 <= label_x - 2 < width:
                label = name[:4].upper()
                for i, char in enumerate(label):
                    x = label_x - len(label)//2 + i
                    if 0 <= x < width:
                        grid[label_y][x] = char

        # Print grid
        print("    " + "─" * width)
        for y, row in enumerate(grid):
            print("    │" + ''.join(row) + "│")
        print("    " + "─" * width)
        print()

        # Print sensor data table
        print(f"{'Sensor':<15} {'Angle':<8} {'Distance':<12} {'Status':<15} {'Bar':<25}")
        print("─" * 80)

        for name, data in self.sensors.items():
            angle = data['angle']
            distance = data['distance']
            valid = data['valid']

            # Format angle
            angle_str = f"{angle:+4d}°"

            # Format distance
            distance_str = f"{distance:.2f} m" if valid else "---"

            # Status
            if valid:
                if distance < 0.5:
                    status = "⚠ VERY CLOSE"
                elif distance < 1.5:
                    status = "✓ NEAR"
                elif distance < 3.0:
                    status = "✓ MEDIUM"
                else:
                    status = "✓ FAR"
            else:
                status = "✗ NO OBJECT"

            # Visual bar (20 chars = 6m max)
            if valid:
                bar_length = int((distance / 6.0) * 20)
                bar = "█" * bar_length + "·" * (20 - bar_length)
            else:
                bar = "·" * 20

            print(f"{name:<15} {angle_str:<8} {distance_str:<12} {status:<15} {bar}")

        print()

        # Statistics
        runtime = time.time() - self.start_time
        hz = self.message_count / runtime if runtime > 0 else 0
        print(f"Messages: {self.message_count} | Rate: {hz:.1f} Hz | Runtime: {runtime:.1f}s")
        print()
        print("Press Ctrl+C to exit")

    def run(self):
        """Main visualization loop"""
        if not self.connect():
            return

        print("Starting visualization...")
        time.sleep(1)

        try:
            while True:
                # Read serial data
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()

                    try:
                        data = json.loads(line)

                        # Update sensor data
                        if 'sensors' in data:
                            for sensor in data['sensors']:
                                name = sensor['name']
                                if name in self.sensors:
                                    self.sensors[name]['distance'] = sensor['distance']
                                    self.sensors[name]['valid'] = sensor['valid']

                            self.message_count += 1

                            # Redraw every message
                            self.draw_radar()

                    except json.JSONDecodeError:
                        pass

                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\n\nShutting down visualizer...")

        finally:
            if self.serial and self.serial.is_open:
                self.serial.close()


def main():
    # Parse arguments
    serial_port = '/dev/ttyUSB0'
    if len(sys.argv) > 1:
        serial_port = sys.argv[1]

    # Help
    if '-h' in sys.argv or '--help' in sys.argv:
        print(__doc__)
        return

    # Run visualizer
    viz = UltrasonicVisualizer(serial_port=serial_port)
    viz.run()


if __name__ == '__main__':
    main()
