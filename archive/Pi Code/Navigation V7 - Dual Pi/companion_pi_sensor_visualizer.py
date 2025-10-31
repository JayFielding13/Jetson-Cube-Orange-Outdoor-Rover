#!/usr/bin/env python3
"""
Companion Pi - Real-time Dual Sensor Visualizer
Runs on 192.168.254.70 (Companion Pi)

Creates real-time radar-style visualization of dual ultrasonic sensors:
- Shows 30° left and right detection cones
- Plots detected obstacles with directional awareness
- Streams visualization to development PC
- Lightweight processing dedicated to visualization only
"""

import serial
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import socket
import io
import base64
import time
import threading
from datetime import datetime
import sys

# Configuration
ARDUINO_PORT = '/dev/ttyUSB0'
ARDUINO_BAUD = 115200
VISUALIZATION_PORT = 5555
DEV_PC_IP = '192.168.254.1'  # Adjust to your development PC IP

# Sensor parameters (matching Arduino code)
SENSOR_ANGLE_DEG = 30  # Each sensor covers 30°
SENSOR_SEPARATION_CM = 11.43  # 4.5 inches
MAX_RANGE_CM = 200
EMERGENCY_DISTANCE_CM = 10

# Visualization parameters
PLOT_RANGE = 100  # cm
UPDATE_RATE = 10  # Hz

class SensorVisualizer:
    def __init__(self):
        self.arduino = None
        self.fig = None
        self.ax = None
        self.socket = None
        self.current_data = {
            'left_distance': None,
            'right_distance': None,
            'left_valid': False,
            'right_valid': False,
            'obstacle_direction': 0,
            'emergency': False,
            'timestamp': None
        }
        self.obstacle_history = []  # Track recent obstacles for persistence
        self.setup_plot()
        self.setup_network()
        
    def setup_plot(self):
        """Initialize the radar-style plot"""
        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.fig.patch.set_facecolor('black')
        
        # Set up the plot area
        self.ax.set_xlim(-PLOT_RANGE, PLOT_RANGE)
        self.ax.set_ylim(0, PLOT_RANGE)
        self.ax.set_aspect('equal')
        self.ax.set_facecolor('black')
        
        # Add grid circles for distance reference
        for radius in [25, 50, 75, 100]:
            circle = plt.Circle((0, 0), radius, fill=False, color='gray', alpha=0.3, linestyle='--')
            self.ax.add_patch(circle)
            self.ax.text(radius * 0.7, radius * 0.7, f'{radius}cm', 
                        color='gray', fontsize=8, alpha=0.7)
        
        # Add center lines
        self.ax.axhline(y=0, color='gray', alpha=0.3, linestyle='-')
        self.ax.axvline(x=0, color='gray', alpha=0.3, linestyle='-')
        
        # Rover representation
        rover_body = patches.Rectangle((-5, -8), 10, 8, 
                                     facecolor='blue', alpha=0.7, edgecolor='cyan')
        self.ax.add_patch(rover_body)
        
        # Direction indicator
        self.ax.arrow(0, 0, 0, 15, head_width=3, head_length=3, 
                     fc='cyan', ec='cyan', alpha=0.8)
        
        self.ax.set_title('Dual Ultrasonic Sensor Visualization\nCompanion Pi - Real-time Obstacle Detection', 
                         color='white', fontsize=12, pad=20)
        self.ax.set_xlabel('Distance (cm) - Left/Right', color='white')
        self.ax.set_ylabel('Distance (cm) - Forward', color='white')
        
        # Create sensor cone patches (will be updated in animation)
        self.left_cone = None
        self.right_cone = None
        self.obstacle_markers = []
        
    def setup_network(self):
        """Setup network connection to stream to development PC"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind(('', VISUALIZATION_PORT))
            self.socket.listen(1)
            print(f"🌐 Visualization server listening on port {VISUALIZATION_PORT}")
            print(f"💡 Connect from development PC: {socket.gethostbyname(socket.gethostname())}:{VISUALIZATION_PORT}")
        except Exception as e:
            print(f"❌ Network setup failed: {e}")
            self.socket = None
    
    def connect_arduino(self):
        """Connect to Arduino for sensor data"""
        try:
            print(f"🔌 Connecting to Arduino on {ARDUINO_PORT}...")
            self.arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
            time.sleep(2)  # Arduino startup delay
            print("✅ Arduino connected successfully!")
            return True
        except Exception as e:
            print(f"❌ Arduino connection failed: {e}")
            return False
    
    def parse_sensor_data(self, data_str):
        """Parse sensor telemetry from Arduino"""
        try:
            data = json.loads(data_str)
            sensors = data.get('sensors', {})
            
            self.current_data.update({
                'left_distance': sensors.get('left_distance'),
                'right_distance': sensors.get('right_distance'),
                'left_valid': sensors.get('left_valid', False),
                'right_valid': sensors.get('right_valid', False),
                'obstacle_direction': sensors.get('obstacle_direction', 0),
                'emergency': data.get('emergency', False),
                'timestamp': datetime.now()
            })
            
            # Add to obstacle history for persistence
            if self.current_data['emergency']:
                self.add_obstacle_to_history()
                
            return True
        except Exception as e:
            return False
    
    def add_obstacle_to_history(self):
        """Add current obstacle detection to history for visualization persistence"""
        timestamp = time.time()
        
        # Calculate obstacle positions based on sensor readings and angles
        if self.current_data['left_valid'] and self.current_data['left_distance']:
            left_angle = np.radians(-SENSOR_ANGLE_DEG)  # 30° left
            left_x = self.current_data['left_distance'] * np.sin(left_angle)
            left_y = self.current_data['left_distance'] * np.cos(left_angle)
            
            if self.current_data['left_distance'] < EMERGENCY_DISTANCE_CM * 2:
                self.obstacle_history.append({
                    'x': left_x, 'y': left_y, 'timestamp': timestamp,
                    'sensor': 'left', 'distance': self.current_data['left_distance']
                })
        
        if self.current_data['right_valid'] and self.current_data['right_distance']:
            right_angle = np.radians(SENSOR_ANGLE_DEG)  # 30° right
            right_x = self.current_data['right_distance'] * np.sin(right_angle)
            right_y = self.current_data['right_distance'] * np.cos(right_angle)
            
            if self.current_data['right_distance'] < EMERGENCY_DISTANCE_CM * 2:
                self.obstacle_history.append({
                    'x': right_x, 'y': right_y, 'timestamp': timestamp,
                    'sensor': 'right', 'distance': self.current_data['right_distance']
                })
        
        # Clean old obstacles (keep last 5 seconds)
        current_time = time.time()
        self.obstacle_history = [obs for obs in self.obstacle_history 
                               if current_time - obs['timestamp'] < 5.0]
    
    def update_visualization(self, frame):
        """Update the visualization with current sensor data"""
        # Clear previous dynamic elements
        if self.left_cone:
            self.left_cone.remove()
        if self.right_cone:
            self.right_cone.remove()
        for marker in self.obstacle_markers:
            marker.remove()
        self.obstacle_markers.clear()
        
        # Draw sensor cones
        self.draw_sensor_cones()
        
        # Draw current obstacles
        self.draw_obstacles()
        
        # Update status text
        self.update_status_display()
        
        return []
    
    def draw_sensor_cones(self):
        """Draw the 30° sensor detection cones"""
        cone_length = PLOT_RANGE * 0.8
        
        # Left sensor cone (30° left from forward)
        left_color = 'red' if (self.current_data['left_valid'] and 
                             self.current_data['left_distance'] and 
                             self.current_data['left_distance'] < EMERGENCY_DISTANCE_CM) else 'green'
        left_alpha = 0.6 if self.current_data['left_valid'] else 0.2
        
        left_angle1 = np.radians(-SENSOR_ANGLE_DEG - 10)  # -40°
        left_angle2 = np.radians(-SENSOR_ANGLE_DEG + 10)  # -20°
        
        left_x1, left_y1 = cone_length * np.sin(left_angle1), cone_length * np.cos(left_angle1)
        left_x2, left_y2 = cone_length * np.sin(left_angle2), cone_length * np.cos(left_angle2)
        
        self.left_cone = patches.Polygon([[0, 0], [left_x1, left_y1], [left_x2, left_y2]], 
                                       facecolor=left_color, alpha=left_alpha, 
                                       edgecolor=left_color, linewidth=2)
        self.ax.add_patch(self.left_cone)
        
        # Right sensor cone (30° right from forward)
        right_color = 'red' if (self.current_data['right_valid'] and 
                              self.current_data['right_distance'] and 
                              self.current_data['right_distance'] < EMERGENCY_DISTANCE_CM) else 'green'
        right_alpha = 0.6 if self.current_data['right_valid'] else 0.2
        
        right_angle1 = np.radians(SENSOR_ANGLE_DEG - 10)  # 20°
        right_angle2 = np.radians(SENSOR_ANGLE_DEG + 10)  # 40°
        
        right_x1, right_y1 = cone_length * np.sin(right_angle1), cone_length * np.cos(right_angle1)
        right_x2, right_y2 = cone_length * np.sin(right_angle2), cone_length * np.cos(right_angle2)
        
        self.right_cone = patches.Polygon([[0, 0], [right_x1, right_y1], [right_x2, right_y2]], 
                                        facecolor=right_color, alpha=right_alpha,
                                        edgecolor=right_color, linewidth=2)
        self.ax.add_patch(self.right_cone)
        
        # Add sensor labels
        self.ax.text(-30, 70, 'LEFT\nSENSOR', ha='center', va='center', 
                    color=left_color, fontsize=10, fontweight='bold')
        self.ax.text(30, 70, 'RIGHT\nSENSOR', ha='center', va='center', 
                    color=right_color, fontsize=10, fontweight='bold')
    
    def draw_obstacles(self):
        """Draw detected obstacles from history"""
        current_time = time.time()
        
        for obstacle in self.obstacle_history:
            age = current_time - obstacle['timestamp']
            alpha = max(0.2, 1.0 - age / 5.0)  # Fade over 5 seconds
            
            color = 'red' if obstacle['distance'] < EMERGENCY_DISTANCE_CM else 'orange'
            size = max(100, 300 - obstacle['distance'] * 10)  # Larger for closer objects
            
            marker = self.ax.scatter(obstacle['x'], obstacle['y'], 
                                   s=size, c=color, alpha=alpha, 
                                   marker='o', edgecolors='white', linewidths=1)
            self.obstacle_markers.append(marker)
        
        # Draw current real-time readings
        if self.current_data['left_valid'] and self.current_data['left_distance']:
            left_angle = np.radians(-SENSOR_ANGLE_DEG)
            left_x = self.current_data['left_distance'] * np.sin(left_angle)
            left_y = self.current_data['left_distance'] * np.cos(left_angle)
            
            marker = self.ax.scatter(left_x, left_y, s=50, c='cyan', alpha=0.8, marker='^')
            self.obstacle_markers.append(marker)
        
        if self.current_data['right_valid'] and self.current_data['right_distance']:
            right_angle = np.radians(SENSOR_ANGLE_DEG)
            right_x = self.current_data['right_distance'] * np.sin(right_angle)
            right_y = self.current_data['right_distance'] * np.cos(right_angle)
            
            marker = self.ax.scatter(right_x, right_y, s=50, c='cyan', alpha=0.8, marker='^')
            self.obstacle_markers.append(marker)
    
    def update_status_display(self):
        """Update status information on the plot"""
        # Clear previous status text
        for txt in self.ax.texts[-10:]:  # Remove last few text elements
            if 'STATUS:' in txt.get_text():
                txt.remove()
        
        # Emergency status
        if self.current_data['emergency']:
            direction_text = {-1: 'LEFT', 0: 'CENTER', 1: 'RIGHT'}[self.current_data['obstacle_direction']]
            status_text = f"🚨 EMERGENCY - {direction_text} OBSTACLE!"
            color = 'red'
        else:
            status_text = "✅ Path Clear"
            color = 'green'
        
        self.ax.text(-95, 85, f"STATUS: {status_text}", 
                    color=color, fontsize=12, fontweight='bold',
                    bbox=dict(boxstyle="round,pad=0.3", facecolor='black', alpha=0.7))
        
        # Sensor readings
        left_text = f"L: {self.current_data['left_distance']:.1f}cm" if self.current_data['left_distance'] else "L: NULL"
        right_text = f"R: {self.current_data['right_distance']:.1f}cm" if self.current_data['right_distance'] else "R: NULL"
        
        self.ax.text(-95, 75, f"SENSORS: {left_text} | {right_text}", 
                    color='white', fontsize=10,
                    bbox=dict(boxstyle="round,pad=0.3", facecolor='black', alpha=0.7))
    
    def data_collection_thread(self):
        """Background thread to collect Arduino data"""
        if not self.arduino:
            return
            
        while True:
            try:
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode('utf-8').strip()
                    if line.startswith('{') and line.endswith('}'):
                        self.parse_sensor_data(line)
                time.sleep(0.05)  # 20Hz data collection
            except Exception as e:
                print(f"⚠️  Data collection error: {e}")
                time.sleep(1)
    
    def stream_to_pc(self, client_socket):
        """Stream visualization frames to development PC"""
        try:
            while True:
                # Capture current plot as image
                buf = io.BytesIO()
                self.fig.savefig(buf, format='png', dpi=100, bbox_inches='tight')
                buf.seek(0)
                
                # Encode as base64
                img_data = base64.b64encode(buf.read()).decode('utf-8')
                
                # Send with timestamp
                message = {
                    'timestamp': time.time(),
                    'image': img_data,
                    'sensor_data': self.current_data.copy()
                }
                
                data = json.dumps(message) + '\n'
                client_socket.send(data.encode('utf-8'))
                
                time.sleep(1.0 / UPDATE_RATE)  # Control frame rate
                
        except Exception as e:
            print(f"⚠️  Streaming error: {e}")
        finally:
            client_socket.close()
    
    def run(self):
        """Main execution loop"""
        print("🎯 Starting Companion Pi Sensor Visualizer...")
        
        # Connect to Arduino
        if not self.connect_arduino():
            print("❌ Cannot continue without Arduino connection")
            return
        
        # Start data collection thread
        data_thread = threading.Thread(target=self.data_collection_thread, daemon=True)
        data_thread.start()
        
        # Start network streaming thread if available
        if self.socket:
            def accept_connections():
                while True:
                    try:
                        client, addr = self.socket.accept()
                        print(f"📡 Development PC connected from {addr}")
                        stream_thread = threading.Thread(
                            target=self.stream_to_pc, 
                            args=(client,), 
                            daemon=True
                        )
                        stream_thread.start()
                    except Exception as e:
                        print(f"⚠️  Connection error: {e}")
            
            network_thread = threading.Thread(target=accept_connections, daemon=True)
            network_thread.start()
        
        # Start real-time visualization
        print("🖥️  Starting real-time visualization...")
        print("📡 Streaming available on port 5555")
        print("🛑 Press Ctrl+C to stop")
        
        ani = FuncAnimation(self.fig, self.update_visualization, 
                          interval=1000//UPDATE_RATE, blit=False, cache_frame_data=False)
        
        plt.show()

def main():
    visualizer = SensorVisualizer()
    try:
        visualizer.run()
    except KeyboardInterrupt:
        print("\n🛑 Visualization stopped by user")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
    finally:
        if visualizer.arduino:
            visualizer.arduino.close()
        if visualizer.socket:
            visualizer.socket.close()
        print("✅ Cleanup complete")

if __name__ == "__main__":
    main()