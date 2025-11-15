#!/usr/bin/env python3
"""
Ultrasonic Sensor Array - Graphical Debugging Interface

Professional GUI for testing and debugging 6 ultrasonic sensors on ESP32.
Works with both Arduino and CircuitPython firmware.

Features:
- Real-time 360° radar visualization
- Individual sensor status monitoring
- Distance graphs with history
- Performance metrics (Hz, latency)
- Data logging capability
- Sensor calibration tools

Usage:
    python3 ultrasonic_gui.py [serial_port]

    Default: /dev/ttyUSB0

Controls:
    ESC - Exit
    SPACE - Pause/Resume
    L - Toggle logging
    R - Reset statistics
    S - Screenshot
"""

import pygame
import serial
import json
import math
import sys
import time
from collections import deque
from datetime import datetime

# Initialize Pygame
pygame.init()

# Constants
WINDOW_WIDTH = 1400
WINDOW_HEIGHT = 800
FPS = 60

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
DARK_GREEN = (0, 128, 0)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
ORANGE = (255, 165, 0)
BLUE = (0, 150, 255)
GRAY = (128, 128, 128)
DARK_GRAY = (50, 50, 50)
LIGHT_GRAY = (200, 200, 200)

# Sensor configuration
SENSORS = {
    'front': {'angle': 0, 'color': GREEN},
    'corner_right': {'angle': 45, 'color': BLUE},
    'side_right': {'angle': 90, 'color': YELLOW},
    'rear': {'angle': 180, 'color': RED},
    'side_left': {'angle': -90, 'color': ORANGE},
    'corner_left': {'angle': -45, 'color': (128, 0, 255)},
}


class UltrasonicGUI:
    def __init__(self, serial_port='/dev/ttyUSB0', baud_rate=115200):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.serial = None

        # Window setup
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption('Ultrasonic Sensor Array - Debug Interface')
        self.clock = pygame.time.Clock()
        self.font_large = pygame.font.Font(None, 36)
        self.font_medium = pygame.font.Font(None, 24)
        self.font_small = pygame.font.Font(None, 18)

        # Sensor data
        self.sensor_data = {}
        for name in SENSORS:
            self.sensor_data[name] = {
                'distance': 6.0,
                'valid': False,
                'history': deque(maxlen=100),
                'last_update': 0
            }

        # Statistics
        self.message_count = 0
        self.start_time = time.time()
        self.last_message_time = 0
        self.fps_history = deque(maxlen=30)

        # UI state
        self.paused = False
        self.logging_enabled = False
        self.log_file = None

        # Radar settings
        self.radar_center_x = 350
        self.radar_center_y = 400
        self.radar_radius = 280
        self.max_distance = 6.0  # meters

    def connect(self):
        """Connect to ESP32"""
        try:
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.01
            )
            time.sleep(0.5)
            print(f"Connected to {self.serial_port}")
            return True
        except Exception as e:
            print(f"ERROR: Could not connect to {self.serial_port}")
            print(f"       {e}")
            return False

    def toggle_logging(self):
        """Toggle data logging"""
        self.logging_enabled = not self.logging_enabled

        if self.logging_enabled:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"ultrasonic_log_{timestamp}.csv"
            self.log_file = open(filename, 'w')
            self.log_file.write("timestamp,sensor,distance,valid\n")
            print(f"Logging started: {filename}")
        else:
            if self.log_file:
                self.log_file.close()
                self.log_file = None
            print("Logging stopped")

    def log_data(self, sensor_name, distance, valid):
        """Log sensor data to file"""
        if self.log_file:
            timestamp = time.time()
            self.log_file.write(f"{timestamp},{sensor_name},{distance},{valid}\n")

    def read_serial_data(self):
        """Read and parse serial data from ESP32"""
        if not self.serial or self.paused:
            return

        while self.serial.in_waiting:
            try:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()

                if not line:
                    continue

                # Parse JSON
                data = json.loads(line)

                if 'sensors' in data:
                    current_time = time.time()

                    for sensor in data['sensors']:
                        name = sensor['name']
                        if name in self.sensor_data:
                            distance = sensor['distance']
                            valid = sensor['valid']

                            # Update sensor data
                            self.sensor_data[name]['distance'] = distance
                            self.sensor_data[name]['valid'] = valid
                            self.sensor_data[name]['last_update'] = current_time
                            self.sensor_data[name]['history'].append(distance if valid else None)

                            # Log data
                            if self.logging_enabled:
                                self.log_data(name, distance, valid)

                    # Update statistics
                    self.message_count += 1
                    if self.last_message_time > 0:
                        hz = 1.0 / (current_time - self.last_message_time)
                        self.fps_history.append(hz)
                    self.last_message_time = current_time

            except json.JSONDecodeError:
                pass
            except Exception as e:
                print(f"Error reading data: {e}")

    def draw_radar(self):
        """Draw radar display"""
        cx, cy = self.radar_center_x, self.radar_center_y
        radius = self.radar_radius

        # Draw radar background circles
        for i in range(6, 0, -1):
            r = int(radius * i / 6)
            color = DARK_GRAY if i % 2 == 0 else (40, 40, 40)
            pygame.draw.circle(self.screen, color, (cx, cy), r)
            pygame.draw.circle(self.screen, GRAY, (cx, cy), r, 1)

            # Draw distance labels
            if i < 6:
                dist_text = self.font_small.render(f"{i}m", True, GRAY)
                self.screen.blit(dist_text, (cx + r + 5, cy - 10))

        # Draw angle lines
        for angle in [0, 45, 90, 135, 180, -45, -90, -135]:
            rad = math.radians(angle)
            dx = math.sin(rad) * radius
            dy = -math.cos(rad) * radius
            pygame.draw.line(self.screen, GRAY, (cx, cy),
                           (cx + dx, cy + dy), 1)

        # Draw rover body
        rover_size = 15
        pygame.draw.rect(self.screen, WHITE,
                        (cx - rover_size, cy - rover_size,
                         rover_size * 2, rover_size * 2))

        # Draw front indicator
        pygame.draw.polygon(self.screen, RED, [
            (cx, cy - rover_size),
            (cx - 8, cy - rover_size - 12),
            (cx + 8, cy - rover_size - 12)
        ])

        # Draw sensor rays and detections
        current_time = time.time()

        for name, config in SENSORS.items():
            angle_rad = math.radians(config['angle'])
            data = self.sensor_data[name]

            # Check if sensor data is recent (within 1 second)
            is_recent = (current_time - data['last_update']) < 1.0

            if not is_recent:
                continue

            distance = data['distance']
            valid = data['valid']
            color = config['color']

            # Calculate ray endpoint
            if valid and distance < self.max_distance:
                ray_length = (distance / self.max_distance) * radius
            else:
                ray_length = radius

            dx = math.sin(angle_rad) * ray_length
            dy = -math.cos(angle_rad) * ray_length

            # Draw sensor ray
            if valid:
                # Solid line with obstacle marker
                pygame.draw.line(self.screen, color, (cx, cy),
                               (cx + dx, cy + dy), 2)
                # Draw obstacle
                pygame.draw.circle(self.screen, color,
                                 (int(cx + dx), int(cy + dy)), 8)
                pygame.draw.circle(self.screen, WHITE,
                                 (int(cx + dx), int(cy + dy)), 8, 2)
            else:
                # Dashed line for no detection
                self.draw_dashed_line(self.screen, GRAY, (cx, cy),
                                    (cx + dx, cy + dy), 2, 10)

            # Draw sensor label
            label_dist = radius + 30
            label_dx = math.sin(angle_rad) * label_dist
            label_dy = -math.cos(angle_rad) * label_dist

            label_text = self.font_small.render(name.upper(), True, color)
            label_rect = label_text.get_rect(
                center=(int(cx + label_dx), int(cy + label_dy))
            )
            self.screen.blit(label_text, label_rect)

    def draw_dashed_line(self, surface, color, start, end, width, dash_length):
        """Draw a dashed line"""
        x1, y1 = start
        x2, y2 = end
        dl = dash_length

        dx = x2 - x1
        dy = y2 - y1
        distance = math.sqrt(dx**2 + dy**2)

        if distance == 0:
            return

        dashes = int(distance / dl)

        for i in range(0, dashes, 2):
            start_ratio = i / dashes
            end_ratio = min((i + 1) / dashes, 1.0)

            sx = x1 + dx * start_ratio
            sy = y1 + dy * start_ratio
            ex = x1 + dx * end_ratio
            ey = y1 + dy * end_ratio

            pygame.draw.line(surface, color, (sx, sy), (ex, ey), width)

    def draw_sensor_status(self):
        """Draw sensor status panel"""
        panel_x = 750
        panel_y = 50
        panel_width = 600

        # Title
        title = self.font_large.render("Sensor Status", True, WHITE)
        self.screen.blit(title, (panel_x, panel_y))

        # Headers
        y = panel_y + 50
        headers = ["Sensor", "Distance", "Status", "Updates"]
        x_positions = [panel_x, panel_x + 150, panel_x + 280, panel_x + 420]

        for i, header in enumerate(headers):
            text = self.font_medium.render(header, True, LIGHT_GRAY)
            self.screen.blit(text, (x_positions[i], y))

        # Draw separator
        y += 35
        pygame.draw.line(self.screen, GRAY, (panel_x, y),
                        (panel_x + panel_width, y), 2)
        y += 10

        # Sensor rows
        current_time = time.time()

        for name, config in SENSORS.items():
            data = self.sensor_data[name]
            color = config['color']

            # Sensor name
            name_text = self.font_medium.render(name.upper(), True, color)
            self.screen.blit(name_text, (x_positions[0], y))

            # Distance
            if data['valid']:
                dist_str = f"{data['distance']:.2f} m"
                dist_color = color
            else:
                dist_str = "---"
                dist_color = GRAY

            dist_text = self.font_medium.render(dist_str, True, dist_color)
            self.screen.blit(dist_text, (x_positions[1], y))

            # Status indicator
            if data['last_update'] > 0 and (current_time - data['last_update']) < 1.0:
                if data['valid']:
                    if data['distance'] < 0.5:
                        status = "⚠ VERY CLOSE"
                        status_color = RED
                    elif data['distance'] < 1.5:
                        status = "✓ NEAR"
                        status_color = YELLOW
                    else:
                        status = "✓ CLEAR"
                        status_color = GREEN
                else:
                    status = "○ NO OBJECT"
                    status_color = GRAY
            else:
                status = "✗ NO DATA"
                status_color = RED

            status_text = self.font_medium.render(status, True, status_color)
            self.screen.blit(status_text, (x_positions[2], y))

            # Update count (last second)
            updates = len([h for h in data['history'] if h is not None])
            update_text = self.font_small.render(f"{updates}", True, GRAY)
            self.screen.blit(update_text, (x_positions[3], y))

            y += 35

    def draw_distance_graphs(self):
        """Draw distance history graphs"""
        graph_x = 750
        graph_y = 400
        graph_width = 600
        graph_height = 120

        # Title
        title = self.font_large.render("Distance History", True, WHITE)
        self.screen.blit(title, (graph_x, graph_y - 40))

        # Draw graph background
        pygame.draw.rect(self.screen, DARK_GRAY,
                        (graph_x, graph_y, graph_width, graph_height))
        pygame.draw.rect(self.screen, GRAY,
                        (graph_x, graph_y, graph_width, graph_height), 1)

        # Draw grid
        for i in range(0, 7):
            y = graph_y + (i * graph_height / 6)
            pygame.draw.line(self.screen, (60, 60, 60),
                           (graph_x, y), (graph_x + graph_width, y), 1)

            dist_label = f"{6 - i}m"
            label = self.font_small.render(dist_label, True, GRAY)
            self.screen.blit(label, (graph_x - 30, y - 8))

        # Draw sensor histories
        for name, config in SENSORS.items():
            data = self.sensor_data[name]
            history = list(data['history'])

            if len(history) < 2:
                continue

            color = config['color']
            points = []

            for i, dist in enumerate(history):
                if dist is not None and dist < self.max_distance:
                    x = graph_x + (i / len(history)) * graph_width
                    y = graph_y + graph_height - (dist / self.max_distance) * graph_height
                    points.append((x, y))

            if len(points) > 1:
                pygame.draw.lines(self.screen, color, False, points, 2)

    def draw_statistics(self):
        """Draw performance statistics"""
        stats_y = 550

        # Performance stats
        runtime = time.time() - self.start_time

        if self.fps_history:
            avg_hz = sum(self.fps_history) / len(self.fps_history)
        else:
            avg_hz = 0

        stats = [
            f"Messages: {self.message_count}",
            f"Update Rate: {avg_hz:.1f} Hz",
            f"Runtime: {runtime:.0f}s",
            f"Logging: {'ON' if self.logging_enabled else 'OFF'}",
            f"Status: {'PAUSED' if self.paused else 'RUNNING'}"
        ]

        y = stats_y
        for stat in stats:
            text = self.font_medium.render(stat, True, LIGHT_GRAY)
            self.screen.blit(text, (750, y))
            y += 30

    def draw_controls(self):
        """Draw control instructions"""
        controls_y = WINDOW_HEIGHT - 80

        controls = [
            "ESC: Exit  |  SPACE: Pause/Resume  |  L: Toggle Logging  |  R: Reset Stats  |  S: Screenshot"
        ]

        for i, control in enumerate(controls):
            text = self.font_small.render(control, True, GRAY)
            rect = text.get_rect(center=(WINDOW_WIDTH // 2, controls_y + i * 20))
            self.screen.blit(text, rect)

    def reset_statistics(self):
        """Reset all statistics"""
        self.message_count = 0
        self.start_time = time.time()
        self.fps_history.clear()

        for name in self.sensor_data:
            self.sensor_data[name]['history'].clear()

        print("Statistics reset")

    def save_screenshot(self):
        """Save a screenshot"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"ultrasonic_screenshot_{timestamp}.png"
        pygame.image.save(self.screen, filename)
        print(f"Screenshot saved: {filename}")

    def run(self):
        """Main application loop"""
        if not self.connect():
            return

        running = True

        while running:
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        self.paused = not self.paused
                    elif event.key == pygame.K_l:
                        self.toggle_logging()
                    elif event.key == pygame.K_r:
                        self.reset_statistics()
                    elif event.key == pygame.K_s:
                        self.save_screenshot()

            # Read sensor data
            self.read_serial_data()

            # Clear screen
            self.screen.fill(BLACK)

            # Draw all components
            self.draw_radar()
            self.draw_sensor_status()
            self.draw_distance_graphs()
            self.draw_statistics()
            self.draw_controls()

            # Update display
            pygame.display.flip()
            self.clock.tick(FPS)

        # Cleanup
        if self.logging_enabled:
            self.toggle_logging()

        if self.serial:
            self.serial.close()

        pygame.quit()
        print("Shutting down...")


def main():
    # Parse arguments
    serial_port = '/dev/ttyUSB0'
    if len(sys.argv) > 1:
        serial_port = sys.argv[1]

    # Help
    if '-h' in sys.argv or '--help' in sys.argv:
        print(__doc__)
        return

    # Run GUI
    app = UltrasonicGUI(serial_port=serial_port)
    app.run()


if __name__ == '__main__':
    main()
