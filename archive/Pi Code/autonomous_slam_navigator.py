#!/usr/bin/env python3
"""
SLAM-Based Rover Navigator
Uses rplidar-roboticia + BreezySLAM for intelligent navigation
"""

import serial
import json
import time
import threading
import math
import numpy as np
from collections import deque

try:
    # Try different possible import names for the roboticia library
    try:
        import rplidar_roboticia as rplidar
    except ImportError:
        import rplidar
    LIDAR_AVAILABLE = True
    print("✅ RPLidar library found")
except ImportError:
    LIDAR_AVAILABLE = False
    print("⚠️ RPLidar library not found - will simulate LiDAR data")

try:
    from breezyslam.algorithms import RMHC_SLAM
    from breezyslam.sensors import RPLidarA1 as Laser
    SLAM_AVAILABLE = True
    print("✅ BreezySLAM library found")
except ImportError:
    SLAM_AVAILABLE = False
    print("⚠️ BreezySLAM library not found - will use basic navigation")

class SLAMNavigator:
    def __init__(self, arduino_port='/dev/ttyUSB0', lidar_port='/dev/ttyUSB1', baud=115200):
        self.arduino_port = arduino_port
        self.lidar_port = lidar_port
        self.baud = baud
        self.arduino = None
        self.lidar = None
        self.running = False
        
        # Robot state from Arduino
        self.robot_state = {
            'mode': 0,
            'rc_valid': False,
            'emergency_stop': False,
            'ultrasonic_distance': 200.0,
            'last_update': time.time()
        }
        
        # Navigation state machine
        self.nav_state = 'EXPLORING'
        self.nav_start_time = time.time()
        self.target_heading = 0.0
        self.current_heading = 0.0
        
        # Movement parameters
        self.cruise_speed = 80
        self.slow_speed = 50
        self.turn_speed = 70
        
        # SLAM Configuration
        self.map_size_meters = 10  # 10x10 meter map
        self.map_size_pixels = 800  # 800x800 pixel resolution (1.25cm per pixel)
        
        # Store SLAM availability as instance variable
        self.slam_available = SLAM_AVAILABLE
        
        if self.slam_available:
            # Initialize SLAM components - RPLidarA1 uses no parameters
            try:
                self.laser = Laser()  # RPLidarA1 constructor takes no arguments
                self.slam = RMHC_SLAM(self.laser, self.map_size_pixels, self.map_size_meters)
                self.mapbytes = bytearray(self.map_size_pixels * self.map_size_pixels)
                print(f"🗺️ SLAM initialized: {self.map_size_pixels}x{self.map_size_pixels} map")
            except Exception as e:
                print(f"⚠️ SLAM initialization failed: {e}")
                print("🗺️ SLAM disabled - using basic navigation only")
                self.laser = None
                self.slam = None
                self.slam_available = False
        else:
            self.laser = None
            self.slam = None
            print("🗺️ SLAM disabled - using basic navigation")
        
        # Robot position tracking
        self.robot_x = self.map_size_meters / 2.0  # Start at center
        self.robot_y = self.map_size_meters / 2.0
        self.robot_theta = 0.0  # Heading in radians
        
        # Scan data storage
        self.scan_data = deque(maxlen=10)  # Store last 10 scans
        self.last_scan_time = time.time()
        
        # Navigation parameters
        self.obstacle_threshold = 50.0  # cm
        self.safe_distance = 100.0      # cm
        self.goal_threshold = 30.0      # cm to reach goal
        
        # Exploration targets
        self.exploration_targets = self.generate_exploration_targets()
        self.current_target_idx = 0
        
        print("🚀 SLAM Navigator Initialized")
    
    def generate_exploration_targets(self):
        """Generate exploration waypoints around the map"""
        targets = []
        center_x, center_y = self.map_size_meters / 2.0, self.map_size_meters / 2.0
        radius = 3.0  # 3 meter exploration radius
        
        # Generate 8 points around center
        for i in range(8):
            angle = (i * 2 * math.pi) / 8
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            targets.append((x, y))
        
        return targets
    
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
        """Connect to LiDAR sensor with new library"""
        if not LIDAR_AVAILABLE:
            print("📊 Using simulated LiDAR data")
            return True
            
        try:
            self.lidar = rplidar.RPLidar(self.lidar_port)
            
            # Get device info
            try:
                info = self.lidar.get_info()
                print(f"📊 LiDAR Info: {info}")
            except:
                print("📊 Could not get LiDAR device info")
            
            # Start motor and scanning
            self.lidar.start_motor()
            time.sleep(2)  # Let motor spin up
            print(f"✅ LiDAR connected on {self.lidar_port}")
            return True
        except Exception as e:
            print(f"⚠️ LiDAR connection failed: {e} - using simulation")
            self.lidar = None
            return True
    
    def start(self):
        """Start the navigator"""
        if not self.connect_arduino():
            return False
        
        self.connect_lidar()
        self.running = True
        
        # Start communication threads
        self.arduino_thread = threading.Thread(target=self.arduino_loop, daemon=True)
        self.lidar_thread = threading.Thread(target=self.lidar_loop, daemon=True)
        self.navigation_thread = threading.Thread(target=self.navigation_loop, daemon=True)
        
        self.arduino_thread.start()
        self.lidar_thread.start()
        self.navigation_thread.start()
        
        print("🚀 SLAM Navigator started")
        return True
    
    def arduino_loop(self):
        """Handle Arduino communication"""
        while self.running and self.arduino:
            try:
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode().strip()
                    if line and line.startswith('{'):
                        try:
                            data = json.loads(line)
                            distance = data.get('distance')
                            if distance is None:
                                distance = 200.0
                            
                            self.robot_state.update({
                                'mode': data.get('mode', 0),
                                'rc_valid': data.get('valid', False),
                                'emergency_stop': data.get('emergency', False),
                                'ultrasonic_distance': float(distance),
                                'last_update': time.time()
                            })
                        except json.JSONDecodeError:
                            pass
                            
            except Exception as e:
                print(f"❌ Arduino error: {e}")
                time.sleep(0.1)
            
            time.sleep(0.02)  # 50Hz
    
    def lidar_loop(self):
        """SLAM-based LiDAR processing with buffer control"""
        scan_count = 0
        consecutive_failures = 0
        
        while self.running:
            try:
                if LIDAR_AVAILABLE and self.lidar and consecutive_failures < 5:
                    success = self.process_slam_scan()
                    if success:
                        scan_count += 1
                        consecutive_failures = 0
                    else:
                        consecutive_failures += 1
                        if consecutive_failures >= 5:
                            print("⚠️ Too many LiDAR failures - switching to simulation")
                        self.simulate_scan_data()
                else:
                    self.simulate_scan_data()
                    
            except Exception as e:
                print(f"⚠️ LiDAR SLAM error: {e}")
                consecutive_failures += 1
                self.simulate_scan_data()
            
            time.sleep(0.5)  # 2Hz processing to reduce buffer pressure
    
    def reset_lidar_motor(self):
        """Stop and restart LiDAR motor to clear buffer buildup"""
        try:
            print("🔄 Resetting LiDAR motor to clear buffer...")
            self.lidar.stop_motor()
            time.sleep(0.5)  # Let motor stop completely
            
            # Clear any remaining buffer
            if hasattr(self.lidar, '_serial'):
                self.lidar._serial.reset_input_buffer()
            
            self.lidar.start_motor()
            time.sleep(1.5)  # Let motor spin up
            print("✅ LiDAR motor reset complete")
            return True
        except Exception as e:
            print(f"❌ Motor reset failed: {e}")
            return False
    
    def process_slam_scan(self):
        """Process LiDAR scan using stop/start motor approach"""
        try:
            # Check buffer size before processing
            buffer_size = 0
            if hasattr(self.lidar, '_serial'):
                buffer_size = self.lidar._serial.in_waiting
            
            # If buffer is getting large, reset the motor
            if buffer_size > 2000:
                print(f"🧹 Large buffer detected: {buffer_size} bytes - resetting motor")
                if not self.reset_lidar_motor():
                    return False
                time.sleep(0.2)  # Brief pause after reset
            
            # Stop motor, grab data quickly, restart motor
            self.lidar.stop_motor()
            time.sleep(0.1)  # Brief stop
            
            # Clear buffer and restart
            if hasattr(self.lidar, '_serial'):
                self.lidar._serial.reset_input_buffer()
            
            self.lidar.start_motor()
            time.sleep(0.3)  # Quick spin-up
            
            # Now try to get a scan with minimal buffer
            scan_generator = self.lidar.iter_scans(max_buf_meas=30)  # Very small buffer
            
            start_time = time.time()
            try:
                scan = next(scan_generator)
                if time.time() - start_time > 0.3:  # Quick timeout
                    print("⏰ Quick scan timeout")
                    return False
            except StopIteration:
                return False
            
            if not scan or len(scan) < 5:
                return False
            
            # Build 360-degree scan from partial data
            distances_mm = [2000] * 360  # Initialize with 2m default
            
            # Fill in actual measurements
            valid_measurements = 0
            for quality, angle, distance in scan:
                if quality > 5 and 100 <= distance <= 8000:
                    angle_idx = int(angle) % 360
                    distances_mm[angle_idx] = int(distance)
                    valid_measurements += 1
            
            # Only process if we got reasonable data
            if valid_measurements < 10:
                print(f"⚠️ Insufficient data: {valid_measurements} measurements")
                return False
            
            # Smooth interpolation for missing angles
            for i in range(360):
                if distances_mm[i] == 2000:  # No measurement
                    # Find nearest valid measurements
                    prev_val = distances_mm[(i-1) % 360]
                    next_val = distances_mm[(i+1) % 360]
                    if prev_val != 2000 and next_val != 2000:
                        distances_mm[i] = (prev_val + next_val) // 2
                    elif prev_val != 2000:
                        distances_mm[i] = prev_val
                    elif next_val != 2000:
                        distances_mm[i] = next_val
                    else:
                        distances_mm[i] = 1500  # Default safe distance
            
            # Store scan data for navigation
            self.scan_data.append(distances_mm)
            self.last_scan_time = time.time()
            
            # Process with SLAM if available (every 4th scan to reduce load)
            if self.slam_available and self.slam and len(self.scan_data) % 4 == 0:
                try:
                    # Update SLAM with scan
                    self.slam.update(distances_mm)
                    
                    # Get updated robot position from SLAM
                    x_mm, y_mm, theta_deg = self.slam.getpos()
                    self.robot_x = x_mm / 1000.0  # Convert to meters
                    self.robot_y = y_mm / 1000.0
                    self.robot_theta = math.radians(theta_deg)
                    self.current_heading = theta_deg
                    
                    # Update map
                    self.slam.getmap(self.mapbytes)
                except Exception as slam_error:
                    print(f"⚠️ SLAM update failed: {slam_error}")
            
            print(f"📡 Scan processed: {valid_measurements} valid measurements")
            return True
            
        except Exception as e:
            print(f"⚠️ SLAM processing failed: {e}")
            # Try to restart motor on failure
            try:
                self.reset_lidar_motor()
            except:
                pass
            return False
    
    def simulate_scan_data(self):
        """Generate simulated scan data"""
        current_time = time.time()
        
        # Create simulated 360-degree scan
        distances_mm = []
        for angle in range(360):
            # Simulate room with walls and obstacles
            base_distance = 2000  # 2 meters base
            
            # Add walls at certain angles
            if 80 <= angle <= 100 or 260 <= angle <= 280:  # Side walls
                distance = 1000 + 200 * math.sin(current_time * 0.1)
            elif 170 <= angle <= 190:  # Back wall
                distance = 1500
            elif 350 <= angle <= 360 or 0 <= angle <= 10:  # Front obstacle
                distance = 800 + 300 * math.sin(current_time * 0.2)
            else:
                distance = base_distance + 300 * math.sin(angle * 0.1 + current_time * 0.05)
            
            distances_mm.append(max(200, min(8000, int(distance))))
        
        # Store simulated scan
        self.scan_data.append(distances_mm)
        self.last_scan_time = current_time
    
    def get_front_obstacle_distance(self):
        """Get closest obstacle distance in front arc (±30°)"""
        if not self.scan_data:
            return 200.0
        
        latest_scan = self.scan_data[-1]
        front_angles = list(range(330, 360)) + list(range(0, 31))  # ±30° front arc
        
        min_distance = float('inf')
        for angle in front_angles:
            if angle < len(latest_scan) and latest_scan[angle] > 0:
                min_distance = min(min_distance, latest_scan[angle])
        
        return min_distance / 10.0 if min_distance != float('inf') else 200.0  # Convert to cm
    
    def get_side_clearances(self):
        """Get left and right clearance distances"""
        if not self.scan_data:
            return 200.0, 200.0
        
        latest_scan = self.scan_data[-1]
        
        # Left side (45° to 135°)
        left_distances = [latest_scan[i] for i in range(45, 136) if i < len(latest_scan) and latest_scan[i] > 0]
        left_clearance = min(left_distances) / 10.0 if left_distances else 200.0
        
        # Right side (225° to 315°)
        right_distances = [latest_scan[i] for i in range(225, 316) if i < len(latest_scan) and latest_scan[i] > 0]
        right_clearance = min(right_distances) / 10.0 if right_distances else 200.0
        
        return left_clearance, right_clearance
    
    def navigation_loop(self):
        """Main SLAM-based navigation loop"""
        while self.running:
            try:
                # Only navigate if in autonomous mode
                mode = self.robot_state.get('mode', 0)
                if mode != 2:
                    time.sleep(0.5)
                    continue
                
                # Get current sensor data
                front_obstacle = self.get_front_obstacle_distance()
                left_clearance, right_clearance = self.get_side_clearances()
                
                # Include ultrasonic sensor for close obstacles
                ultrasonic_dist = self.robot_state.get('ultrasonic_distance', 200.0)
                effective_front = min(front_obstacle, ultrasonic_dist)
                
                # Compute SLAM-based navigation
                left_speed, right_speed = self.compute_slam_navigation(
                    effective_front, left_clearance, right_clearance
                )
                
                # Send motor commands
                self.send_motor_command(left_speed, right_speed)
                
                # Status logging
                if time.time() % 3 < 0.3:
                    scan_points = len(self.scan_data[-1]) if self.scan_data else 0
                    slam_mode = "SLAM" if SLAM_AVAILABLE else "SIM"
                    
                    print(f"🗺️ {slam_mode} | Pos: ({self.robot_x:.1f}, {self.robot_y:.1f}) | Heading: {self.current_heading:.0f}°")
                    print(f"   Front: {effective_front:.1f}cm | L: {left_clearance:.0f}cm | R: {right_clearance:.0f}cm | Points: {scan_points}")
                    print(f"   State: {self.nav_state} | Target: {self.current_target_idx + 1}/{len(self.exploration_targets)}")
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                self.send_motor_command(0, 0)
            
            time.sleep(0.5)  # 2Hz navigation
    
    def compute_slam_navigation(self, front_obstacle, left_clearance, right_clearance):
        """SLAM-enhanced navigation with exploration"""
        current_time = time.time()
        time_in_state = current_time - self.nav_start_time
        
        left_speed = self.cruise_speed
        right_speed = self.cruise_speed
        
        # Emergency obstacle avoidance always takes priority
        if front_obstacle < self.obstacle_threshold:
            self.nav_state = 'AVOIDING'
            self.nav_start_time = current_time
            
            # Choose avoidance direction based on clearances
            if left_clearance > right_clearance and left_clearance > 80:
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
                print(f"🚨 AVOIDING - Turn LEFT (L:{left_clearance:.0f} > R:{right_clearance:.0f})")
            elif right_clearance > left_clearance and right_clearance > 80:
                left_speed = self.turn_speed
                right_speed = -self.turn_speed
                print(f"🚨 AVOIDING - Turn RIGHT (R:{right_clearance:.0f} > L:{left_clearance:.0f})")
            else:
                # Both sides blocked, turn around
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
                print(f"🚨 BLOCKED - Turning around")
        
        elif self.nav_state == 'AVOIDING':
            # Continue avoiding until clear
            if time_in_state > 1.0 and front_obstacle > self.safe_distance:
                self.nav_state = 'EXPLORING'
                self.nav_start_time = current_time
                print("✅ Clear - resuming exploration")
            else:
                # Continue current avoidance maneuver
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
        
        elif self.nav_state == 'EXPLORING':
            # SLAM-based exploration toward targets
            if self.exploration_targets:
                target_x, target_y = self.exploration_targets[self.current_target_idx]
                
                # Calculate direction to target
                dx = target_x - self.robot_x
                dy = target_y - self.robot_y
                distance_to_target = math.sqrt(dx*dx + dy*dy)
                target_heading = math.degrees(math.atan2(dy, dx))
                
                # Check if we've reached current target
                if distance_to_target < self.goal_threshold / 100.0:  # Convert cm to meters
                    self.current_target_idx = (self.current_target_idx + 1) % len(self.exploration_targets)
                    print(f"🎯 Target reached! Moving to target {self.current_target_idx + 1}")
                    target_x, target_y = self.exploration_targets[self.current_target_idx]
                    dx = target_x - self.robot_x
                    dy = target_y - self.robot_y
                    target_heading = math.degrees(math.atan2(dy, dx))
                
                # Calculate heading error
                heading_error = target_heading - self.current_heading
                
                # Normalize heading error to [-180, 180]
                while heading_error > 180:
                    heading_error -= 360
                while heading_error < -180:
                    heading_error += 360
                
                # Navigate toward target
                if abs(heading_error) > 15:  # Need to turn
                    turn_intensity = min(1.0, abs(heading_error) / 45.0)
                    if heading_error > 0:  # Turn left
                        left_speed = self.cruise_speed * (1 - turn_intensity * 0.8)
                        right_speed = self.cruise_speed
                    else:  # Turn right
                        left_speed = self.cruise_speed
                        right_speed = self.cruise_speed * (1 - turn_intensity * 0.8)
                else:
                    # Move forward toward target
                    left_speed = self.cruise_speed
                    right_speed = self.cruise_speed
            
            # Random exploration if no targets or long time in state
            elif time_in_state > 8.0:
                self.nav_state = 'RANDOM_TURN'
                self.nav_start_time = current_time
                print("🔄 Random exploration turn")
        
        elif self.nav_state == 'RANDOM_TURN':
            left_speed = -self.turn_speed
            right_speed = self.turn_speed
            
            if time_in_state > 1.5:
                self.nav_state = 'EXPLORING'
                self.nav_start_time = current_time
                print("🔍 Random turn complete")
        
        return left_speed, right_speed
    
    def send_motor_command(self, left_speed, right_speed):
        """Send motor command to Arduino"""
        if not self.arduino:
            return
        
        left_speed = max(-255, min(255, int(left_speed)))
        right_speed = max(-255, min(255, int(right_speed)))
        
        command = {'motor': {'left': left_speed, 'right': right_speed}}
        
        try:
            cmd_str = json.dumps(command) + '\n'
            self.arduino.write(cmd_str.encode())
        except Exception as e:
            print(f"❌ Command send failed: {e}")
    
    def stop(self):
        """Stop the navigator"""
        self.running = False
        
        if self.arduino:
            self.send_motor_command(0, 0)
            time.sleep(0.1)
            self.arduino.close()
        
        if self.lidar and LIDAR_AVAILABLE:
            try:
                self.lidar.stop_motor()
                self.lidar.disconnect()
            except:
                pass
        
        print("🛑 SLAM Navigator stopped")

def main():
    print("=" * 60)
    print("🗺️ SLAM-BASED ROVER NAVIGATOR")
    print("=" * 60)
    print()
    print("Strategy: Real-time SLAM with intelligent exploration")
    print("Features: Map building, localization, path planning")
    print("Libraries: rplidar-roboticia + BreezySLAM")
    print()
    
    navigator = SLAMNavigator()
    
    try:
        if navigator.start():
            print("✅ SLAM Navigator ready")
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