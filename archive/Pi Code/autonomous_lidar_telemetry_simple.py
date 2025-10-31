#!/usr/bin/env python3
"""
Simplified LiDAR Telemetry Controller for testing
Removes problematic navigation logic to focus on telemetry pipeline
"""

import serial
import json
import time
import threading
import random
import math
from datetime import datetime

# Try to import optional libraries
try:
    import psutil
    PSUTIL_AVAILABLE = True
    print("✅ psutil library found")
except ImportError:
    PSUTIL_AVAILABLE = False
    print("⚠️ psutil library not found - will simulate system data")

class SimpleLidarTelemetryController:
    def __init__(self, arduino_port='/dev/ttyUSB0', baud=115200):
        self.arduino_port = arduino_port
        self.baud = baud
        self.arduino = None
        self.running = False
        
        # Robot state from Arduino
        self.robot_state = {
            'mode': 0,
            'rc_valid': False,
            'emergency_stop': False,
            'distance': 150.0,  # Default safe distance
            'last_update': time.time()
        }
        
        # Navigation state - simplified
        self.nav_state = 'STRAIGHT'
        
        # Simple movement parameters
        self.cruise_speed = 80
        
        # Simulated LiDAR data
        self.lidar_data = {
            'points_count': 350,
            'min_distance': 150.0,  # Safe default
            'min_angle': 0.0,
            'quality_avg': 25.0,
            'front_sector_clear': True,
            'left_clear': True,
            'right_clear': True
        }
        
        # System telemetry
        self.system_telemetry = {
            'cpu_percent': 15.0,
            'memory_percent': 45.0,
            'temperature': 55.0,
            'disk_usage': 65.0,
            'uptime': 0.0
        }
        
        # Statistics
        self.commands_sent = 0
        self.start_time = time.time()
        
        print("🚀 Simple LiDAR Telemetry Controller Initialized")
        print("📡 Focus: Telemetry pipeline testing")
    
    def connect_arduino(self):
        """Connect to Arduino gatekeeper"""
        try:
            self.arduino = serial.Serial(self.arduino_port, self.baud, timeout=1)
            time.sleep(2)
            print(f"✅ Connected to Arduino gatekeeper on {self.arduino_port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to Arduino: {e}")
            return False
    
    def start(self):
        """Start the simplified controller"""
        if not self.connect_arduino():
            return False
        
        self.running = True
        
        # Start threads
        self.arduino_thread = threading.Thread(target=self.arduino_communication_loop, daemon=True)
        self.navigation_thread = threading.Thread(target=self.simple_navigation_loop, daemon=True)
        self.telemetry_thread = threading.Thread(target=self.system_telemetry_loop, daemon=True)
        
        self.arduino_thread.start()
        self.navigation_thread.start()
        self.telemetry_thread.start()
        
        print("🚀 Simple Controller started - telemetry pipeline active")
        return True
    
    def arduino_communication_loop(self):
        """Handle Arduino communication"""
        while self.running and self.arduino:
            try:
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode().strip()
                    if line and line.startswith('{'):
                        arduino_data = json.loads(line)
                        
                        # Update robot state safely
                        self.robot_state.update({
                            'mode': arduino_data.get('mode', 0),
                            'rc_valid': arduino_data.get('valid', False),
                            'emergency_stop': arduino_data.get('emergency', False),
                            'distance': arduino_data.get('distance', 150.0),  # Default safe
                            'last_update': time.time()
                        })
                        
                        if 'pi_ack' in arduino_data:
                            self.commands_sent += 1
                            
            except Exception as e:
                print(f"❌ Arduino communication error: {e}")
                time.sleep(0.1)
            
            time.sleep(0.02)  # 50Hz communication
    
    def system_telemetry_loop(self):
        """Collect system telemetry data"""
        while self.running:
            try:
                if PSUTIL_AVAILABLE:
                    # Real system data
                    self.system_telemetry['cpu_percent'] = psutil.cpu_percent(interval=None)
                    self.system_telemetry['memory_percent'] = psutil.virtual_memory().percent
                    disk = psutil.disk_usage('/')
                    self.system_telemetry['disk_usage'] = (disk.used / disk.total) * 100
                else:
                    # Simulated data
                    base_time = time.time()
                    self.system_telemetry['cpu_percent'] = 15 + 10 * math.sin(base_time * 0.1)
                    self.system_telemetry['memory_percent'] = 45 + 5 * math.sin(base_time * 0.05)
                    self.system_telemetry['disk_usage'] = 65 + 2 * math.sin(base_time * 0.02)
                
                # Uptime
                self.system_telemetry['uptime'] = time.time() - self.start_time
                
                # Simulated varying LiDAR data
                base_dist = 150 + 50 * math.sin(time.time() * 0.1)
                self.lidar_data.update({
                    'points_count': random.randint(300, 400),
                    'min_distance': max(30.0, base_dist),
                    'min_angle': random.uniform(0, 360),
                    'quality_avg': random.uniform(20, 30)
                })
                
            except Exception as e:
                print(f"⚠️ Telemetry collection error: {e}")
            
            time.sleep(2)  # Update every 2 seconds
    
    def simple_navigation_loop(self):
        """Simplified navigation - just forward motion with telemetry"""
        while self.running:
            try:
                # Simple straight movement
                left_speed = self.cruise_speed
                right_speed = self.cruise_speed
                
                # Send command with telemetry
                self.send_command_with_telemetry(left_speed, right_speed)
                
                # Log status periodically
                if time.time() % 5 < 0.1:
                    self.log_status()
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                self.send_command_with_telemetry(0, 0)  # Stop on error
            
            time.sleep(0.2)  # 5Hz navigation (slower for testing)
    
    def send_command_with_telemetry(self, left_speed, right_speed):
        """Send motor command with telemetry data to Arduino"""
        if not self.arduino:
            return
        
        # Constrain speeds
        left_speed = max(-255, min(255, int(left_speed)))
        right_speed = max(-255, min(255, int(right_speed)))
        
        # Prepare telemetry data - all values guaranteed non-None
        telemetry = {
            'nav_state': self.nav_state,
            'lidar_points': int(self.lidar_data['points_count']),
            'lidar_min_dist': round(float(self.lidar_data['min_distance']), 1),
            'lidar_min_angle': round(float(self.lidar_data['min_angle']), 1),
            'lidar_quality': round(float(self.lidar_data['quality_avg']), 1),
            'sectors': {
                'front': bool(self.lidar_data['front_sector_clear']),
                'left': bool(self.lidar_data['left_clear']),
                'right': bool(self.lidar_data['right_clear'])
            },
            'system': {
                'cpu': round(float(self.system_telemetry['cpu_percent']), 1),
                'memory': round(float(self.system_telemetry['memory_percent']), 1),
                'temp': round(float(self.system_telemetry['temperature']), 1),
                'uptime': round(float(self.system_telemetry['uptime']), 1),
                'disk': round(float(self.system_telemetry['disk_usage']), 1)
            }
        }
        
        # Extended command with telemetry
        command_data = {
            'motor': {
                'left': left_speed,
                'right': right_speed
            },
            'telemetry': telemetry
        }
        
        command = json.dumps(command_data, separators=(',', ':')) + '\n'
        
        try:
            self.arduino.write(command.encode())
            self.commands_sent += 1
        except Exception as e:
            print(f"❌ Command send failed: {e}")
    
    def log_status(self):
        """Log current status"""
        mode = self.robot_state.get('mode', 0)
        mode_names = ['FAILSAFE', 'MANUAL', 'AUTONOMOUS']
        mode_name = mode_names[mode] if 0 <= mode < len(mode_names) else 'UNKNOWN'
        
        lidar_points = int(self.lidar_data['points_count'])
        min_dist = self.lidar_data['min_distance']
        cpu = self.system_telemetry['cpu_percent']
        
        print(f"📊 Mode: {mode_name} | Nav: {self.nav_state} | LiDAR: {lidar_points}pts, {min_dist:.1f}cm | CPU: {cpu:.1f}%")
        print(f"📡 Commands sent: {self.commands_sent}")
    
    def disconnect(self):
        """Clean shutdown"""
        self.running = False
        
        if self.arduino:
            # Send stop command
            self.send_command_with_telemetry(0, 0)
            time.sleep(0.1)
            self.arduino.close()
        
        print("🔌 Simple Controller disconnected")

def main():
    print("=" * 60)
    print("🚀 SIMPLE LIDAR TELEMETRY CONTROLLER")
    print("=" * 60)
    print()
    print("Purpose: Test telemetry pipeline without complex navigation")
    print("📡 Pi → Arduino → Dashboard data flow testing")
    print()
    
    controller = SimpleLidarTelemetryController()
    
    try:
        if controller.start():
            print("✅ Simple controller started successfully")
            print("Press Ctrl+C to stop...")
            
            while True:
                time.sleep(1)
        else:
            print("❌ Failed to start controller")
    
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested...")
    
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
    
    finally:
        controller.disconnect()
        print("👋 Simple controller stopped")

if __name__ == "__main__":
    main()