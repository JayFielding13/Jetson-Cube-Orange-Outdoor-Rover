#!/usr/bin/env python3
"""
Enhanced LiDAR Navigator with 4-Quadrant Analysis
Improved corner handling and stuck detection
"""

import serial
import json
import time
import threading
import random
import math

# Try to import LiDAR library
try:
    import rplidar
    LIDAR_AVAILABLE = True
    print("✅ RPLidar library found")
except ImportError:
    LIDAR_AVAILABLE = False
    print("⚠️ RPLidar library not found - will simulate LiDAR data")

class Enhanced4QuadrantNavigator:
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
            'ultrasonic_distance': 200.0,
            'last_update': time.time()
        }
        
        # Navigation state machine
        self.nav_state = 'STRAIGHT'
        self.nav_start_time = time.time()
        self.turn_direction = 'left'
        self.stuck_counter = 0
        self.last_position_time = time.time()
        
        # Movement parameters
        self.cruise_speed = 100
        self.slow_speed = 60
        self.turn_speed = 80
        self.reverse_speed = 60
        
        # Enhanced 4-quadrant LiDAR processing
        self.lidar_data = {
            # Four quadrants
            'front_distance': 200.0,    # 315° to 45° (0° ± 45°)
            'left_distance': 200.0,     # 45° to 135°
            'rear_distance': 200.0,     # 135° to 225°
            'right_distance': 200.0,    # 225° to 315°
            
            # Detailed quadrant data
            'front_avg': 200.0,
            'left_avg': 200.0,
            'rear_avg': 200.0,
            'right_avg': 200.0,
            
            'front_count': 0,
            'left_count': 0,
            'rear_count': 0,
            'right_count': 0,
            
            'overall_min': 200.0,
            'points_processed': 0,
            'last_scan_time': time.time(),
            'scan_counter': 0
        }
        
        # Navigation parameters
        self.danger_threshold = 40.0      # cm - immediate action
        self.caution_threshold = 80.0     # cm - slow down  
        self.safe_distance = 120.0        # cm - considered safe
        self.stuck_threshold = 5          # cycles before considering stuck
        
        print("🚀 Enhanced 4-Quadrant LiDAR Navigator Initialized")
        print("🎯 Mission: Advanced autonomous navigation with corner handling")
    
    def connect_arduino(self):
        """Connect to Arduino gatekeeper"""
        try:
            self.arduino = serial.Serial(self.arduino_port, self.baud, timeout=1)
            time.sleep(2)
            print(f"✅ Arduino connected on {self.arduino_port}")
            return True
        except Exception as e:
            print(f"❌ Arduino connection failed: {e}")
            return False
    
    def connect_lidar(self):
        """Connect to LiDAR sensor"""
        if not LIDAR_AVAILABLE:
            print("📊 Using simulated LiDAR data")
            return True
            
        try:
            self.lidar = rplidar.RPLidar(self.lidar_port)
            self.lidar.start_motor()
            print(f"✅ LiDAR connected on {self.lidar_port}")
            return True
        except Exception as e:
            print(f"⚠️ LiDAR connection failed: {e} - using simulation")
            return True  # Continue with simulation
    
    def start(self):
        """Start the navigator"""
        if not self.connect_arduino():
            return False
        
        self.connect_lidar()  # Continue even if LiDAR fails
        
        self.running = True
        
        # Start communication threads
        self.arduino_thread = threading.Thread(target=self.arduino_loop, daemon=True)
        self.lidar_thread = threading.Thread(target=self.lidar_loop, daemon=True)
        self.navigation_thread = threading.Thread(target=self.navigation_loop, daemon=True)
        
        self.arduino_thread.start()
        self.lidar_thread.start()
        self.navigation_thread.start()
        
        print("🚀 Enhanced Navigator started")
        print("🧭 4-Quadrant autonomous navigation active")
        return True
    
    def arduino_loop(self):
        """Handle Arduino communication"""
        while self.running and self.arduino:
            try:
                # Read Arduino data
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode().strip()
                    if line and line.startswith('{'):
                        try:
                            data = json.loads(line)
                            # Ensure ultrasonic distance is never None
                            distance = data.get('distance')
                            if distance is None:
                                distance = 200.0  # Safe default
                            
                            self.robot_state.update({
                                'mode': data.get('mode', 0),
                                'rc_valid': data.get('valid', False),
                                'emergency_stop': data.get('emergency', False),
                                'ultrasonic_distance': float(distance),
                                'last_update': time.time()
                            })
                        except json.JSONDecodeError:
                            pass  # Ignore malformed JSON
                            
            except Exception as e:
                print(f"❌ Arduino error: {e}")
                time.sleep(0.1)
            
            time.sleep(0.02)  # 50Hz
    
    def lidar_loop(self):
        """Process LiDAR data using proven working method"""
        while self.running:
            try:
                if LIDAR_AVAILABLE and self.lidar:
                    success = self.process_4quadrant_lidar()
                    if not success:
                        # Fallback to simulation for this cycle
                        self.simulate_4quadrant_data()
                else:
                    # Use simulated data if no LiDAR
                    self.simulate_4quadrant_data()
                    
            except Exception as e:
                print(f"⚠️ LiDAR error: {e}")
                self.simulate_4quadrant_data()  # Fallback
            
            time.sleep(0.2)  # 5Hz LiDAR processing
    
    def process_4quadrant_lidar(self):
        """Process LiDAR scan data into 4 quadrants with detailed analysis"""
        try:
            # Use proven working protocol
            scan_generator = self.lidar.iter_scans()
            scan = next(scan_generator)
            
            # Initialize quadrant arrays
            front_distances = []  # 315° to 45° (0° ± 45°)
            left_distances = []   # 45° to 135°
            rear_distances = []   # 135° to 225°
            right_distances = []  # 225° to 315°
            all_distances = []
            
            points_count = 0
            
            # Process each LiDAR point
            for quality, angle, distance in scan:
                if distance > 0 and quality > 5:  # Valid measurement
                    dist_cm = distance / 10.0  # Convert mm to cm
                    if 10 <= dist_cm <= 500:  # Reasonable range
                        all_distances.append(dist_cm)
                        points_count += 1
                        
                        # Classify into quadrants based on angle
                        if angle <= 45 or angle >= 315:  # Front quadrant
                            front_distances.append(dist_cm)
                        elif 45 < angle <= 135:  # Left quadrant
                            left_distances.append(dist_cm)
                        elif 135 < angle <= 225:  # Rear quadrant  
                            rear_distances.append(dist_cm)
                        elif 225 < angle < 315:  # Right quadrant
                            right_distances.append(dist_cm)
            
            # Calculate quadrant statistics
            self.lidar_data.update({
                # Minimum distances (closest obstacle in each quadrant)
                'front_distance': min(front_distances) if front_distances else 200.0,
                'left_distance': min(left_distances) if left_distances else 200.0,
                'rear_distance': min(rear_distances) if rear_distances else 200.0,
                'right_distance': min(right_distances) if right_distances else 200.0,
                
                # Average distances (general clearance in each quadrant)
                'front_avg': sum(front_distances) / len(front_distances) if front_distances else 200.0,
                'left_avg': sum(left_distances) / len(left_distances) if left_distances else 200.0,
                'rear_avg': sum(rear_distances) / len(rear_distances) if rear_distances else 200.0,
                'right_avg': sum(right_distances) / len(right_distances) if right_distances else 200.0,
                
                # Point counts per quadrant
                'front_count': len(front_distances),
                'left_count': len(left_distances),
                'rear_count': len(rear_distances),
                'right_count': len(right_distances),
                
                # Overall statistics
                'overall_min': min(all_distances) if all_distances else 200.0,
                'points_processed': points_count,
                'last_scan_time': time.time(),
                'scan_counter': self.lidar_data.get('scan_counter', 0) + 1
            })
            
            # Debug: Log detailed quadrant data occasionally
            if time.time() % 4 < 0.3:  # Every 4 seconds
                print(f"📊 4-Quadrant Analysis (Scan #{self.lidar_data['scan_counter']}):")
                print(f"   Front: min={self.lidar_data['front_distance']:.1f}cm, avg={self.lidar_data['front_avg']:.1f}cm ({self.lidar_data['front_count']} pts)")
                print(f"   Left:  min={self.lidar_data['left_distance']:.1f}cm, avg={self.lidar_data['left_avg']:.1f}cm ({self.lidar_data['left_count']} pts)")
                print(f"   Right: min={self.lidar_data['right_distance']:.1f}cm, avg={self.lidar_data['right_avg']:.1f}cm ({self.lidar_data['right_count']} pts)")
                print(f"   Rear:  min={self.lidar_data['rear_distance']:.1f}cm, avg={self.lidar_data['rear_avg']:.1f}cm ({self.lidar_data['rear_count']} pts)")
            
            return True  # Success
            
        except StopIteration:
            return False
        except Exception as e:
            print(f"⚠️ LiDAR processing error: {e}")
            return False
    
    def simulate_4quadrant_data(self):
        """Generate simulated 4-quadrant LiDAR data"""
        current_time = time.time()
        
        # Simulate different obstacles in different quadrants
        base_front = 150 + 50 * math.sin(current_time * 0.1)
        base_left = 120 + 30 * math.sin(current_time * 0.15)
        base_right = 140 + 40 * math.sin(current_time * 0.12)
        base_rear = 200  # Usually more open behind
        
        self.lidar_data.update({
            'front_distance': max(30.0, base_front),
            'left_distance': max(40.0, base_left),
            'rear_distance': max(50.0, base_rear),
            'right_distance': max(35.0, base_right),
            
            'front_avg': max(50.0, base_front + 20),
            'left_avg': max(60.0, base_left + 25),
            'rear_avg': max(80.0, base_rear),
            'right_avg': max(55.0, base_right + 15),
            
            'front_count': random.randint(20, 40),
            'left_count': random.randint(15, 35),
            'rear_count': random.randint(10, 25),
            'right_count': random.randint(15, 35),
            
            'overall_min': min(base_front, base_left, base_right, base_rear),
            'points_processed': 0,  # Indicate simulation
            'last_scan_time': current_time,
            'scan_counter': self.lidar_data.get('scan_counter', 0) + 1
        })
    
    def navigation_loop(self):
        """Enhanced navigation with 4-quadrant analysis"""
        while self.running:
            try:
                # Only navigate if in autonomous mode
                mode = self.robot_state.get('mode', 0)
                if mode != 2:  # Not autonomous
                    time.sleep(0.5)
                    continue
                
                # Get 4-quadrant sensor data
                front_min = self.lidar_data['front_distance']
                left_min = self.lidar_data['left_distance']
                right_min = self.lidar_data['right_distance']
                rear_min = self.lidar_data['rear_distance']
                
                front_avg = self.lidar_data['front_avg']
                left_avg = self.lidar_data['left_avg']
                right_avg = self.lidar_data['right_avg']
                rear_avg = self.lidar_data['rear_avg']
                
                ultrasonic_dist = self.robot_state.get('ultrasonic_distance') or 200.0
                
                # Use most conservative front distance reading
                front_obstacle = min(front_min, ultrasonic_dist)
                
                # Compute enhanced navigation
                left_speed, right_speed = self.compute_4quadrant_navigation(
                    front_obstacle, left_min, right_min, rear_min,
                    front_avg, left_avg, right_avg, rear_avg
                )
                
                # Send motor commands
                self.send_motor_command(left_speed, right_speed)
                
                # Status logging
                if time.time() % 3 < 0.1:  # Every 3 seconds
                    self.log_4quadrant_status(front_obstacle)
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                self.send_motor_command(0, 0)  # Stop on error
            
            time.sleep(0.3)  # 3Hz navigation - smooth Arduino processing
    
    def compute_4quadrant_navigation(self, front_obstacle, left_min, right_min, rear_min,
                                   front_avg, left_avg, right_avg, rear_avg):
        """Enhanced navigation using 4-quadrant analysis"""
        current_time = time.time()
        time_in_state = current_time - self.nav_start_time
        
        left_speed = self.cruise_speed
        right_speed = self.cruise_speed
        
        # Detect if stuck (same distances for too long)
        if abs(front_obstacle - getattr(self, 'last_front_distance', 0)) < 2.0:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
        self.last_front_distance = front_obstacle
        
        if self.nav_state == 'STRAIGHT':
            if front_obstacle < self.danger_threshold:
                # Immediate danger - choose best escape direction
                self.nav_state = 'AVOIDING'
                self.nav_start_time = current_time
                
                # Smart direction selection using 4-quadrant data
                if rear_avg > 100 and front_obstacle < 25:
                    # If there's room behind and very close obstacle in front - reverse
                    self.turn_direction = 'reverse'
                    print(f"🚨 TRAPPED! Front={front_obstacle:.1f}cm - REVERSING (rear clear: {rear_avg:.1f}cm)")
                elif left_avg > right_avg and left_min > 60:
                    # Left side has more space
                    self.turn_direction = 'left'
                    print(f"🚨 DANGER! Front={front_obstacle:.1f}cm - turning LEFT (left: {left_avg:.1f}cm)")
                elif right_avg > left_avg and right_min > 60:
                    # Right side has more space
                    self.turn_direction = 'right'
                    print(f"🚨 DANGER! Front={front_obstacle:.1f}cm - turning RIGHT (right: {right_avg:.1f}cm)")
                else:
                    # Choose side with minimum obstacles
                    self.turn_direction = 'left' if left_min > right_min else 'right'
                    print(f"🚨 DANGER! Front={front_obstacle:.1f}cm - turning {self.turn_direction.upper()}")
                    
            elif front_obstacle < self.caution_threshold:
                # Caution zone - slow down
                left_speed = self.slow_speed
                right_speed = self.slow_speed
                print(f"⚠️ Slowing down - obstacle at {front_obstacle:.1f}cm")
                
            elif time_in_state > 8.0 or self.stuck_counter > self.stuck_threshold:
                # Periodic exploration or stuck detection
                self.nav_state = 'EXPLORING'
                self.nav_start_time = current_time
                self.stuck_counter = 0
                
                # Choose exploration direction based on space available
                if left_avg > right_avg:
                    self.turn_direction = 'left'
                else:
                    self.turn_direction = 'right'
                print(f"🔍 Exploration turn {self.turn_direction} (stuck_count: {self.stuck_counter})")
        
        elif self.nav_state == 'AVOIDING':
            if self.turn_direction == 'reverse':
                # Reverse away from obstacle
                left_speed = -self.reverse_speed
                right_speed = -self.reverse_speed
            elif self.turn_direction == 'left':
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
            else:  # right
                left_speed = self.turn_speed
                right_speed = -self.turn_speed
            
            # Return to straight when path is clear
            if time_in_state > 2.0 and front_obstacle > self.safe_distance:
                self.nav_state = 'STRAIGHT'
                self.nav_start_time = current_time
                print("✅ Path cleared - resuming forward")
        
        elif self.nav_state == 'EXPLORING':
            # Execute exploration turn
            if self.turn_direction == 'left':
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
            else:
                left_speed = self.turn_speed
                right_speed = -self.turn_speed
            
            # Return to straight after turn
            if time_in_state > 1.2:
                self.nav_state = 'STRAIGHT'
                self.nav_start_time = current_time
                print("🔍 Exploration complete")
        
        return left_speed, right_speed
    
    def send_motor_command(self, left_speed, right_speed):
        """Send basic motor command to Arduino"""
        if not self.arduino:
            return
        
        # Constrain speeds
        left_speed = max(-255, min(255, int(left_speed)))
        right_speed = max(-255, min(255, int(right_speed)))
        
        # Simple motor command
        command = {
            'motor': {
                'left': left_speed,
                'right': right_speed
            }
        }
        
        try:
            cmd_str = json.dumps(command) + '\n'
            self.arduino.write(cmd_str.encode())
        except Exception as e:
            print(f"❌ Command send failed: {e}")
    
    def log_4quadrant_status(self, front_obstacle):
        """Log current 4-quadrant navigation status"""
        mode = self.robot_state.get('mode', 0)
        mode_names = ['FAILSAFE', 'MANUAL', 'AUTONOMOUS']
        mode_name = mode_names[mode] if 0 <= mode < len(mode_names) else 'UNKNOWN'
        
        points = self.lidar_data['points_processed']
        sensor_mode = "LiDAR" if points > 0 else "BACKUP"
        
        print(f"🧭 Mode: {mode_name} | State: {self.nav_state} | "
              f"Front: {front_obstacle:.1f}cm | {sensor_mode}: {points}pts")
        print(f"   Quadrants - L:{self.lidar_data['left_distance']:.0f}cm "
              f"R:{self.lidar_data['right_distance']:.0f}cm "
              f"Rear:{self.lidar_data['rear_distance']:.0f}cm")
    
    def stop(self):
        """Stop the navigator"""
        self.running = False
        
        # Stop motors
        if self.arduino:
            self.send_motor_command(0, 0)
            time.sleep(0.1)
            self.arduino.close()
        
        # Stop LiDAR
        if self.lidar and LIDAR_AVAILABLE:
            try:
                self.lidar.stop_motor()
                self.lidar.disconnect()
            except:
                pass
        
        print("🛑 Enhanced Navigator stopped")

def main():
    print("=" * 60)
    print("🚀 ENHANCED 4-QUADRANT LIDAR NAVIGATOR")
    print("=" * 60)
    print()
    print("Features:")
    print("📊 4-Quadrant obstacle analysis (Front/Left/Right/Rear)")
    print("🧠 Smart escape route selection")
    print("🔄 Reverse capability for tight corners")
    print("📡 Stuck detection and recovery")
    print("📈 Average and minimum distance tracking")
    print()
    
    navigator = Enhanced4QuadrantNavigator()
    
    try:
        if navigator.start():
            print("✅ Enhanced Navigator ready - switch to AUTONOMOUS mode")
            print("Press Ctrl+C to stop...")
            
            while True:
                time.sleep(1)
        else:
            print("❌ Failed to start navigator")
    
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested...")
    
    finally:
        navigator.stop()
        print("👋 Enhanced Navigator shutdown complete")

if __name__ == "__main__":
    main()