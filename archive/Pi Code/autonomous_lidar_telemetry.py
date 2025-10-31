#!/usr/bin/env python3
"""
LiDAR-Integrated Autonomous Controller with Telemetry
Demonstrates the new telemetry data pipeline from Pi → Arduino → Dashboard
"""

import serial
import json
import time
import threading
import random
import math
import subprocess
from datetime import datetime

# Try to import optional libraries (will fail gracefully if not available)
try:
    import psutil
    PSUTIL_AVAILABLE = True
    print("✅ psutil library found")
except ImportError:
    PSUTIL_AVAILABLE = False
    print("⚠️ psutil library not found - will simulate system data")

try:
    import rplidar
    LIDAR_AVAILABLE = True
    print("✅ RPLidar library found")
except ImportError:
    LIDAR_AVAILABLE = False
    print("⚠️ RPLidar library not found - will simulate LiDAR data")

class LidarTelemetryController:
    def __init__(self, arduino_port='/dev/ttyUSB0', lidar_port='/dev/ttyUSB1', baud=115200):
        self.arduino_port = arduino_port
        self.lidar_port = lidar_port
        self.baud = baud
        self.arduino = None
        self.lidar = None
        self.running = False
        
        # Robot state from Arduino
        self.robot_state = {
            'mode': 0,  # 0=Failsafe, 1=Manual, 2=Autonomous
            'rc_valid': False,
            'emergency_stop': False,
            'distance': None,
            'pi_command_valid': False,
            'gatekeeper_mode': 'FAILSAFE',
            'last_update': time.time()
        }
        
        # Navigation state
        self.nav_state = 'INITIALIZING'
        self.nav_start = time.time()
        self.turn_direction = 'left'
        
        # Movement parameters
        self.cruise_speed = 120
        self.slow_speed = 60
        self.turn_speed = 100
        
        # LiDAR data
        self.lidar_data = {
            'points_count': 0,
            'min_distance': None,
            'min_angle': None,
            'scan_time': None,
            'quality_avg': 0,
            'front_sector_clear': True,
            'left_clear': True,
            'right_clear': True
        }
        
        # System telemetry
        self.system_telemetry = {
            'cpu_percent': 0,
            'memory_percent': 0,
            'temperature': None,
            'disk_usage': 0,
            'uptime': 0,
            'network_strength': None
        }
        
        # Navigation parameters
        self.obstacle_threshold = 60.0  # cm
        self.front_sector_angle = 45    # degrees on each side of front
        self.safe_distance = 100.0      # cm
        
        # Statistics
        self.commands_sent = 0
        self.telemetry_sent = 0
        self.start_time = time.time()
        
        print("🚀 LiDAR Telemetry Controller Initialized")
        print("📡 Enhanced telemetry pipeline: Pi → Arduino → Dashboard")
    
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
    
    def connect_lidar(self):
        """Connect to LiDAR sensor"""
        if not LIDAR_AVAILABLE:
            print("📊 LiDAR simulation mode - generating synthetic data")
            return True
            
        try:
            self.lidar = rplidar.RPLidar(self.lidar_port)
            self.lidar.start_motor()
            print(f"✅ LiDAR connected on {self.lidar_port}")
            return True
        except Exception as e:
            print(f"⚠️ LiDAR connection failed: {e}")
            print("📊 Falling back to simulation mode")
            return True  # Continue with simulation
    
    def start(self):
        """Start the controller with LiDAR integration"""
        if not self.connect_arduino():
            return False
        
        if not self.connect_lidar():
            print("⚠️ Continuing without LiDAR")
        
        self.running = True
        
        # Start threads
        self.arduino_thread = threading.Thread(target=self.arduino_communication_loop, daemon=True)
        self.navigation_thread = threading.Thread(target=self.navigation_loop, daemon=True)
        self.lidar_thread = threading.Thread(target=self.lidar_loop, daemon=True)
        self.telemetry_thread = threading.Thread(target=self.system_telemetry_loop, daemon=True)
        
        self.arduino_thread.start()
        self.navigation_thread.start()
        self.lidar_thread.start()
        self.telemetry_thread.start()
        
        self.nav_state = 'STRAIGHT'
        print("🚀 LiDAR Telemetry Controller started")
        print("🧠 Pi navigation with LiDAR integration")
        print("📡 Full telemetry pipeline active")
        return True
    
    def arduino_communication_loop(self):
        """Handle Arduino communication"""
        while self.running and self.arduino:
            try:
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode().strip()
                    if line and line.startswith('{'):
                        arduino_data = json.loads(line)
                        
                        # Update robot state
                        self.robot_state.update({
                            'mode': arduino_data.get('mode', 0),
                            'rc_valid': arduino_data.get('valid', False),
                            'emergency_stop': arduino_data.get('emergency', False),
                            'distance': arduino_data.get('distance'),
                            'pi_command_valid': arduino_data.get('pi_command_valid', False),
                            'gatekeeper_mode': arduino_data.get('gatekeeper_mode', 'FAILSAFE'),
                            'last_update': time.time()
                        })
                        
                        # Check for acknowledgments
                        if 'pi_ack' in arduino_data:
                            self.commands_sent += 1
                            if arduino_data['pi_ack'].get('applied'):
                                print(f"✅ Command applied by gatekeeper")
                
            except Exception as e:
                print(f"❌ Arduino communication error: {e}")
                time.sleep(0.1)
            
            time.sleep(0.02)  # 50Hz communication
    
    def lidar_loop(self):
        """Process LiDAR data continuously"""
        while self.running:
            try:
                if LIDAR_AVAILABLE and self.lidar:
                    self.process_real_lidar_data()
                else:
                    self.generate_simulated_lidar_data()
                    
            except Exception as e:
                print(f"❌ LiDAR processing error: {e}")
                # Fall back to simulation
                self.generate_simulated_lidar_data()
            
            time.sleep(0.1)  # 10Hz LiDAR processing
    
    def process_real_lidar_data(self):
        """Process real LiDAR scan data"""
        try:
            scan_data = list(self.lidar.iter_scans(max_buf_meas=500))
            if not scan_data:
                return
            
            latest_scan = scan_data[-1]  # Get most recent scan
        except Exception as e:
            # LiDAR communication error - fall back to simulation
            print(f"⚠️ LiDAR communication error: {e}")
            self.generate_simulated_lidar_data()
            return
        
        front_distances = []
        left_distances = []
        right_distances = []
        all_distances = []
        quality_sum = 0
        
        for quality, angle, distance in latest_scan:
            if distance > 0:  # Valid measurement
                all_distances.append(distance/10)  # Convert mm to cm
                quality_sum += quality
                
                # Categorize by angle (front, left, right)
                if angle <= self.front_sector_angle or angle >= (360 - self.front_sector_angle):
                    front_distances.append(distance/10)
                elif 45 <= angle <= 135:
                    left_distances.append(distance/10)
                elif 225 <= angle <= 315:
                    right_distances.append(distance/10)
        
        # Update LiDAR data
        if all_distances:
            self.lidar_data.update({
                'points_count': len(latest_scan),
                'min_distance': min(all_distances),
                'min_angle': self.find_min_angle(latest_scan),
                'scan_time': time.time(),
                'quality_avg': quality_sum / len(latest_scan) if latest_scan else 0,
                'front_sector_clear': min(front_distances) > self.safe_distance if front_distances else True,
                'left_clear': min(left_distances) > self.safe_distance if left_distances else True,
                'right_clear': min(right_distances) > self.safe_distance if right_distances else True
            })
    
    def generate_simulated_lidar_data(self):
        """Generate realistic simulated LiDAR data"""
        # Simulate varying obstacle patterns
        current_time = time.time()
        base_distance = 150 + 50 * math.sin(current_time * 0.1)  # Vary between 100-200cm
        
        # Add some randomness for obstacles
        min_distance = base_distance + random.uniform(-30, 30)
        min_distance = max(20, min_distance)  # Never less than 20cm
        
        self.lidar_data.update({
            'points_count': random.randint(300, 400),
            'min_distance': min_distance,
            'min_angle': random.uniform(0, 360),
            'scan_time': current_time,
            'quality_avg': random.randint(10, 47),
            'front_sector_clear': min_distance > self.safe_distance,
            'left_clear': random.choice([True, True, False]),  # Mostly clear
            'right_clear': random.choice([True, True, False])  # Mostly clear
        })
    
    def find_min_angle(self, scan_data):
        """Find angle of minimum distance measurement"""
        min_dist = float('inf')
        min_angle = 0
        
        for quality, angle, distance in scan_data:
            if 0 < distance < min_dist:
                min_dist = distance
                min_angle = angle
        
        return min_angle
    
    def system_telemetry_loop(self):
        """Collect system telemetry data"""
        while self.running:
            try:
                if PSUTIL_AVAILABLE:
                    # Real system data using psutil
                    self.system_telemetry['cpu_percent'] = psutil.cpu_percent(interval=None)
                    self.system_telemetry['memory_percent'] = psutil.virtual_memory().percent
                    
                    # Disk usage
                    disk = psutil.disk_usage('/')
                    self.system_telemetry['disk_usage'] = (disk.used / disk.total) * 100
                else:
                    # Simulated system data
                    base_time = time.time()
                    self.system_telemetry['cpu_percent'] = 15 + 10 * math.sin(base_time * 0.1)  # 5-25%
                    self.system_telemetry['memory_percent'] = 45 + 5 * math.sin(base_time * 0.05)  # 40-50%
                    self.system_telemetry['disk_usage'] = 65 + 2 * math.sin(base_time * 0.02)  # 63-67%
                
                # Uptime
                self.system_telemetry['uptime'] = time.time() - self.start_time
                
                # Temperature (try to read from Pi)
                try:
                    temp_result = subprocess.run(['vcgencmd', 'measure_temp'], 
                                               capture_output=True, text=True, timeout=2)
                    if temp_result.returncode == 0:
                        temp_str = temp_result.stdout.strip()
                        temp_val = float(temp_str.split('=')[1].split("'")[0])
                        self.system_telemetry['temperature'] = temp_val
                except:
                    # Simulate temperature if vcgencmd fails
                    self.system_telemetry['temperature'] = 45 + 5 * math.sin(time.time() * 0.01)
                
                # Network strength (simulate for now)
                self.system_telemetry['network_strength'] = random.randint(-70, -30)
                
            except Exception as e:
                print(f"⚠️ Telemetry collection error: {e}")
            
            time.sleep(2)  # Update every 2 seconds
    
    def navigation_loop(self):
        """Main navigation logic with LiDAR integration"""
        while self.running:
            try:
                # Compute navigation based on LiDAR + ultrasonic
                left_speed, right_speed = self.compute_lidar_navigation()
                
                # Send command with telemetry
                self.send_command_with_telemetry(left_speed, right_speed)
                
                # Log status periodically
                if time.time() % 5 < 0.1:  # Every ~5 seconds
                    self.log_status()
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                self.send_command_with_telemetry(0, 0)  # Stop on error
            
            time.sleep(0.1)  # 10Hz navigation
    
    def compute_lidar_navigation(self):
        """Compute navigation using LiDAR + ultrasonic data"""
        current_time = time.time()
        time_in_state = current_time - self.nav_start
        
        # Get distances with None handling
        lidar_min = self.lidar_data.get('min_distance') or 200
        ultrasonic_dist = self.robot_state.get('distance') or 200
        
        # Use the more conservative (shorter) distance
        closest_obstacle = min(lidar_min, ultrasonic_dist)
        
        # Default speeds
        left_speed = self.cruise_speed
        right_speed = self.cruise_speed
        
        if self.nav_state == 'STRAIGHT':
            # Check for obstacles using both sensors
            if closest_obstacle < self.obstacle_threshold:
                # Obstacle detected - decide action based on LiDAR sector data
                front_clear = self.lidar_data.get('front_sector_clear', False)
                left_clear = self.lidar_data.get('left_clear', True)
                right_clear = self.lidar_data.get('right_clear', True)
                
                if closest_obstacle < 30:
                    # Very close - must turn
                    self.nav_state = 'AVOIDING'
                    self.nav_start = current_time
                    
                    # Choose turn direction based on LiDAR sector data
                    if left_clear and not right_clear:
                        self.turn_direction = 'left'
                    elif right_clear and not left_clear:
                        self.turn_direction = 'right'
                    else:
                        self.turn_direction = random.choice(['left', 'right'])
                    
                    print(f"🚨 Obstacle at {closest_obstacle:.1f}cm - avoiding {self.turn_direction}")
                    print(f"   LiDAR sectors: front={front_clear}, left={left_clear}, right={right_clear}")
                else:
                    # Moderate distance - slow down
                    left_speed = self.slow_speed
                    right_speed = self.slow_speed
            
            # Periodic exploration turns
            elif time_in_state > 8.0:
                self.nav_state = 'EXPLORING'
                self.nav_start = current_time
                self.turn_direction = random.choice(['left', 'right'])
                print(f"🔍 Exploration turn {self.turn_direction} after {time_in_state:.1f}s")
        
        elif self.nav_state == 'AVOIDING':
            # Execute avoidance turn
            if self.turn_direction == 'left':
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
            else:
                left_speed = self.turn_speed
                right_speed = -self.turn_speed
            
            # Return to straight when clear
            if time_in_state > 2.0 and closest_obstacle > self.safe_distance:
                self.nav_state = 'STRAIGHT'
                self.nav_start = current_time
                print(f"✅ Avoidance complete - resuming straight")
        
        elif self.nav_state == 'EXPLORING':
            # Execute exploration turn
            if self.turn_direction == 'left':
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
            else:
                left_speed = self.turn_speed
                right_speed = -self.turn_speed
            
            # Return to straight after turn
            if time_in_state > 1.5:
                self.nav_state = 'STRAIGHT'
                self.nav_start = current_time
                print("🔍 Exploration complete - resuming straight")
        
        return left_speed, right_speed
    
    def send_command_with_telemetry(self, left_speed, right_speed):
        """Send motor command with telemetry data to Arduino"""
        if not self.arduino:
            return
        
        # Constrain speeds
        left_speed = max(-255, min(255, int(left_speed)))
        right_speed = max(-255, min(255, int(right_speed)))
        
        # Prepare telemetry data
        telemetry = {
            'nav_state': self.nav_state,
            'lidar_points': self.lidar_data.get('points_count', 0),
            'lidar_min_dist': round(self.lidar_data.get('min_distance') or 0, 1),
            'lidar_min_angle': round(self.lidar_data.get('min_angle') or 0, 1),
            'lidar_quality': round(self.lidar_data.get('quality_avg') or 0, 1),
            'sectors': {
                'front': self.lidar_data.get('front_sector_clear', True),
                'left': self.lidar_data.get('left_clear', True),
                'right': self.lidar_data.get('right_clear', True)
            },
            'system': {
                'cpu': round(self.system_telemetry.get('cpu_percent') or 0, 1),
                'memory': round(self.system_telemetry.get('memory_percent') or 0, 1),
                'temp': round(self.system_telemetry.get('temperature') or 0, 1) if self.system_telemetry.get('temperature') else None,
                'uptime': round(self.system_telemetry.get('uptime') or 0, 1),
                'disk': round(self.system_telemetry.get('disk_usage') or 0, 1)
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
            self.telemetry_sent += 1
        except Exception as e:
            print(f"❌ Command send failed: {e}")
    
    def log_status(self):
        """Log current status"""
        mode = self.robot_state.get('gatekeeper_mode', 'UNKNOWN')
        lidar_points = self.lidar_data.get('points_count', 0)
        min_dist = self.lidar_data.get('min_distance', 0)
        cpu = self.system_telemetry.get('cpu_percent', 0)
        
        print(f"📊 Mode: {mode} | Nav: {self.nav_state} | LiDAR: {lidar_points}pts, {min_dist:.1f}cm | CPU: {cpu:.1f}%")
        print(f"📡 Telemetry sent: {self.telemetry_sent} commands")
    
    def disconnect(self):
        """Clean shutdown"""
        self.running = False
        
        if self.lidar and LIDAR_AVAILABLE:
            try:
                self.lidar.stop_motor()
                self.lidar.disconnect()
            except:
                pass
        
        if self.arduino:
            # Send stop command
            self.send_command_with_telemetry(0, 0)
            time.sleep(0.1)
            self.arduino.close()
        
        print("🔌 LiDAR Telemetry Controller disconnected")

def main():
    print("=" * 80)
    print("🚀 LIDAR-INTEGRATED AUTONOMOUS CONTROLLER WITH TELEMETRY")
    print("=" * 80)
    print()
    print("Features:")
    print("📡 Extended telemetry pipeline: Pi → Arduino → Dashboard")
    print("🔍 LiDAR integration for enhanced obstacle detection")
    print("📊 System monitoring: CPU, memory, temperature, disk usage")
    print("🧠 Intelligent navigation with multi-sensor fusion")
    print("🛡️ Arduino gatekeeper maintains safety authority")
    print()
    
    controller = LidarTelemetryController()
    
    try:
        if controller.start():
            print("✅ Controller started successfully")
            print("Press Ctrl+C to stop...")
            
            # Keep running
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
        print("👋 Controller stopped")

if __name__ == "__main__":
    main()