#!/usr/bin/env python3
"""
Simple LiDAR Autonomous Navigator
Focus: Basic obstacle avoidance using LiDAR data
No telemetry - just navigation
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

class LidarNavigator:
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
        
        # Movement parameters
        self.cruise_speed = 100
        self.slow_speed = 60
        self.turn_speed = 80
        
        # LiDAR processing
        self.lidar_data = {
            'front_distance': 200.0,
            'left_distance': 200.0,
            'right_distance': 200.0,
            'min_distance': 200.0,
            'points_processed': 0,
            'last_scan_time': time.time()
        }
        
        # Navigation parameters
        self.obstacle_threshold = 80.0    # cm - start avoiding
        self.danger_threshold = 40.0      # cm - immediate turn
        self.safe_distance = 120.0        # cm - considered safe
        self.front_angle_range = 30       # degrees each side of front
        
        print("🚀 LiDAR Navigator Initialized")
        print("🎯 Mission: Autonomous obstacle avoidance")
    
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
        
        print("🚀 LiDAR Navigator started")
        print("🧭 Autonomous navigation active")
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
                    success = self.process_real_lidar()
                    if not success:
                        # Fallback to simulation for this cycle
                        self.simulate_lidar_data()
                else:
                    # Use simulated data if no LiDAR
                    self.simulate_lidar_data()
                    
            except Exception as e:
                print(f"⚠️ LiDAR error: {e}")
                self.simulate_lidar_data()  # Fallback
            
            time.sleep(0.2)  # 5Hz LiDAR processing
    
    def process_real_lidar(self):
        """Process real LiDAR scan data with improved buffering"""
        try:
            # Use Method 1: Basic iter_scans() - PROVEN TO WORK (120 points per scan)
            scan_generator = self.lidar.iter_scans()
            scan = next(scan_generator)
            
            front_distances = []
            left_distances = []
            right_distances = []
            all_distances = []
            points_count = 0
            
            for quality, angle, distance in scan:
                if distance > 0 and quality > 5:  # Lower quality threshold
                    dist_cm = distance / 10.0  # Convert mm to cm
                    if dist_cm < 500:  # Ignore very far readings
                        all_distances.append(dist_cm)
                        points_count += 1
                        
                        # Fixed angle sectors for forward-facing LiDAR (0° = forward)
                        if angle <= 45 or angle >= 315:  # Front sector: -45° to +45°
                            front_distances.append(dist_cm)
                        elif 45 < angle <= 135:  # Left sector: 45° to 135°
                            left_distances.append(dist_cm)
                        elif 225 <= angle < 315:  # Right sector: 225° to 315°
                            right_distances.append(dist_cm)
            
            # Only update if we got valid data
            if points_count > 0:
                self.lidar_data.update({
                    'front_distance': min(front_distances) if front_distances else 200.0,
                    'left_distance': min(left_distances) if left_distances else 200.0,
                    'right_distance': min(right_distances) if right_distances else 200.0,
                    'min_distance': min(all_distances) if all_distances else 200.0,
                    'points_processed': points_count,
                    'last_scan_time': time.time()
                })
                
                # Debug: Log sector data occasionally
                if time.time() % 5 < 0.2:  # Every 5 seconds
                    print(f"📊 LiDAR: Front={min(front_distances) if front_distances else 'N/A'}cm, "
                          f"Left={min(left_distances) if left_distances else 'N/A'}cm, "
                          f"Right={min(right_distances) if right_distances else 'N/A'}cm, "
                          f"Points={points_count}")
                return True  # Success
            else:
                return False  # No valid data
            
        except StopIteration:
            # No scan data available
            return False
        except Exception as e:
            print(f"⚠️ LiDAR processing error: {e}")
            return False  # Failed
    
    def use_ultrasonic_navigation(self):
        """Use only ultrasonic sensor for navigation when LiDAR fails"""
        ultrasonic_dist = self.robot_state.get('ultrasonic_distance') or 200.0
        
        # Use ultrasonic distance for all directions (conservative approach)
        self.lidar_data.update({
            'front_distance': ultrasonic_dist,
            'left_distance': 200.0,  # Assume sides are clear
            'right_distance': 200.0,  # Assume sides are clear
            'min_distance': ultrasonic_dist,
            'points_processed': 0,  # Indicate using ultrasonic only
            'last_scan_time': time.time()
        })
    
    def simulate_lidar_data(self):
        """Generate simulated LiDAR data for testing"""
        current_time = time.time()
        
        # Simulate obstacles with some variation
        base_distance = 150 + 50 * math.sin(current_time * 0.1)
        noise = random.uniform(-20, 20)
        
        simulated_distance = max(30.0, base_distance + noise)
        
        self.lidar_data.update({
            'front_distance': simulated_distance,
            'left_distance': simulated_distance + random.uniform(-30, 30),
            'right_distance': simulated_distance + random.uniform(-30, 30),
            'min_distance': simulated_distance,
            'points_processed': random.randint(200, 400),
            'last_scan_time': current_time
        })
    
    def navigation_loop(self):
        """Main navigation logic"""
        while self.running:
            try:
                # Only navigate if in autonomous mode
                mode = self.robot_state.get('mode', 0)
                if mode != 2:  # Not autonomous
                    time.sleep(0.5)
                    continue
                
                # Get sensor data with None protection
                front_dist = self.lidar_data['front_distance']
                left_dist = self.lidar_data['left_distance']
                right_dist = self.lidar_data['right_distance']
                ultrasonic_dist = self.robot_state.get('ultrasonic_distance') or 200.0
                
                # Use most conservative distance reading (both guaranteed to be numbers)
                closest_obstacle = min(front_dist, ultrasonic_dist)
                
                # Compute navigation
                left_speed, right_speed = self.compute_navigation(
                    closest_obstacle, front_dist, left_dist, right_dist
                )
                
                # Send motor commands
                self.send_motor_command(left_speed, right_speed)
                
                # Status logging
                if time.time() % 3 < 0.1:  # Every 3 seconds
                    self.log_navigation_status(closest_obstacle)
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                self.send_motor_command(0, 0)  # Stop on error
            
            time.sleep(0.3)  # 3Hz navigation - slower for smoother Arduino processing
    
    def compute_navigation(self, closest_obstacle, front_dist, left_dist, right_dist):
        """Compute motor speeds based on sensor data"""
        current_time = time.time()
        time_in_state = current_time - self.nav_start_time
        
        left_speed = self.cruise_speed
        right_speed = self.cruise_speed
        
        if self.nav_state == 'STRAIGHT':
            if closest_obstacle < self.danger_threshold:
                # Immediate danger - turn away
                self.nav_state = 'AVOIDING'
                self.nav_start_time = current_time
                
                # Choose turn direction based on side clearances
                if left_dist > right_dist:
                    self.turn_direction = 'left'
                elif right_dist > left_dist:
                    self.turn_direction = 'right'
                else:
                    self.turn_direction = random.choice(['left', 'right'])
                
                print(f"🚨 DANGER! Obstacle at {closest_obstacle:.1f}cm - turning {self.turn_direction}")
                
            elif closest_obstacle < self.obstacle_threshold:
                # Caution zone - slow down
                left_speed = self.slow_speed
                right_speed = self.slow_speed
                print(f"⚠️ Slowing down - obstacle at {closest_obstacle:.1f}cm")
                
            elif time_in_state > 10.0:
                # Periodic exploration turn
                self.nav_state = 'EXPLORING'
                self.nav_start_time = current_time
                self.turn_direction = random.choice(['left', 'right'])
                print(f"🔍 Exploration turn {self.turn_direction}")
        
        elif self.nav_state == 'AVOIDING':
            # Execute avoidance turn
            if self.turn_direction == 'left':
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
            else:
                left_speed = self.turn_speed
                right_speed = -self.turn_speed
            
            # Return to straight when safe
            if time_in_state > 1.5 and closest_obstacle > self.safe_distance:
                self.nav_state = 'STRAIGHT'
                self.nav_start_time = current_time
                print("✅ Obstacle cleared - resuming straight")
        
        elif self.nav_state == 'EXPLORING':
            # Execute exploration turn
            if self.turn_direction == 'left':
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
            else:
                left_speed = self.turn_speed
                right_speed = -self.turn_speed
            
            # Return to straight after brief turn
            if time_in_state > 1.0:
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
        
        # Simple motor command (no telemetry)
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
    
    def log_navigation_status(self, closest_obstacle):
        """Log current navigation status"""
        mode = self.robot_state.get('mode', 0)
        mode_names = ['FAILSAFE', 'MANUAL', 'AUTONOMOUS']
        mode_name = mode_names[mode] if 0 <= mode < len(mode_names) else 'UNKNOWN'
        
        front_dist = self.lidar_data['front_distance']
        points = self.lidar_data['points_processed']
        
        # Indicate sensor mode
        sensor_mode = "LiDAR" if points > 0 else "BACKUP"
        
        print(f"🧭 Mode: {mode_name} | State: {self.nav_state} | "
              f"Closest: {closest_obstacle:.1f}cm | Front: {front_dist:.1f}cm | "
              f"{sensor_mode}: {points}pts")
    
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
        
        print("🛑 LiDAR Navigator stopped")

def main():
    print("=" * 60)
    print("🚀 LIDAR AUTONOMOUS NAVIGATOR")
    print("=" * 60)
    print()
    print("Mission: Navigate autonomously using LiDAR obstacle avoidance")
    print("Controls: Switch to AUTONOMOUS mode (RC CH9) to start navigation")
    print()
    
    navigator = LidarNavigator()
    
    try:
        if navigator.start():
            print("✅ Navigator ready - switch to AUTONOMOUS mode to begin")
            print("Press Ctrl+C to stop...")
            
            while True:
                time.sleep(1)
        else:
            print("❌ Failed to start navigator")
    
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested...")
    
    finally:
        navigator.stop()
        print("👋 Navigator shutdown complete")

if __name__ == "__main__":
    main()