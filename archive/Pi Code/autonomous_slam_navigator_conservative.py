#!/usr/bin/env python3
"""
Conservative SLAM Navigator
Uses on-demand scanning to completely avoid buffer issues
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

class ConservativeNavigator:
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
        self.map_size_pixels = 800  # 800x800 pixel resolution
        
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
        
        # Simplified range data (8 sectors)
        self.range_data = {
            'front': 200.0,       # 0° ±22.5°
            'front_right': 200.0, # 45° ±22.5°
            'right': 200.0,       # 90° ±22.5°
            'back_right': 200.0,  # 135° ±22.5°
            'back': 200.0,        # 180° ±22.5°
            'back_left': 200.0,   # 225° ±22.5°
            'left': 200.0,        # 270° ±22.5°
            'front_left': 200.0,  # 315° ±22.5°
            'last_update': time.time()
        }
        
        # LiDAR scanning parameters
        self.scan_interval = 2.0  # Scan every 2 seconds
        self.last_scan_time = 0
        self.lidar_motor_running = False
        
        # Navigation parameters
        self.obstacle_threshold = 50.0  # cm
        self.safe_distance = 100.0      # cm
        
        print("🚀 Conservative Navigator Initialized")
        print("📊 Using on-demand scanning to prevent buffer overruns")
    
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
        """Connect to LiDAR sensor but keep motor off initially"""
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
            
            # Do NOT start motor here - we'll start it only when scanning
            print(f"✅ LiDAR connected on {self.lidar_port} (motor off)")
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
        
        print("🚀 Conservative Navigator started")
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
        """On-demand LiDAR scanning"""
        while self.running:
            try:
                current_time = time.time()
                
                # Only scan periodically to avoid buffer buildup
                if current_time - self.last_scan_time > self.scan_interval:
                    if LIDAR_AVAILABLE and self.lidar:
                        self.perform_on_demand_scan()
                    else:
                        self.simulate_range_data()
                    
                    self.last_scan_time = current_time
                    
            except Exception as e:
                print(f"⚠️ LiDAR error: {e}")
                self.simulate_range_data()
            
            time.sleep(0.1)  # Check every 100ms
    
    def perform_on_demand_scan(self):
        """Perform a single scan and immediately stop motor"""
        try:
            print("📡 Starting on-demand scan...")
            
            # Start motor
            self.lidar.start_motor()
            self.lidar_motor_running = True
            time.sleep(1.0)  # Let motor spin up
            
            # Clear any buffer buildup
            if hasattr(self.lidar, '_serial'):
                self.lidar._serial.reset_input_buffer()
            
            # Get ONE scan with timeout
            scan_data = []
            timeout_start = time.time()
            
            try:
                for scan in self.lidar.iter_scans(max_buf_meas=100):
                    scan_data = list(scan)
                    break  # Take only the first scan
                    
                    # Timeout protection
                    if time.time() - timeout_start > 3.0:
                        print("⏰ Scan timeout after 3 seconds")
                        break
            except Exception as scan_error:
                print(f"⚠️ Scan error: {scan_error}")
                scan_data = []
            
            # IMMEDIATELY stop motor to prevent buffer buildup
            try:
                self.lidar.stop_motor()
                self.lidar_motor_running = False
                print("🛑 Motor stopped after scan")
            except:
                pass
            
            # Process scan data if we got any
            if scan_data and len(scan_data) > 50:
                self.process_scan_to_sectors(scan_data)
                print(f"✅ Processed {len(scan_data)} measurements")
            else:
                print(f"⚠️ Insufficient scan data: {len(scan_data)} measurements")
                self.simulate_range_data()
                
        except Exception as e:
            print(f"❌ On-demand scan failed: {e}")
            # Ensure motor is stopped on any error
            try:
                if self.lidar_motor_running:
                    self.lidar.stop_motor()
                    self.lidar_motor_running = False
            except:
                pass
            self.simulate_range_data()
    
    def process_scan_to_sectors(self, scan_data):
        """Convert scan data to 8-sector range data"""
        # Initialize sector minimums
        sectors = {
            'front': [],       # 337.5° to 22.5°
            'front_right': [], # 22.5° to 67.5°
            'right': [],       # 67.5° to 112.5°
            'back_right': [],  # 112.5° to 157.5°
            'back': [],        # 157.5° to 202.5°
            'back_left': [],   # 202.5° to 247.5°
            'left': [],        # 247.5° to 292.5°
            'front_left': [],  # 292.5° to 337.5°
        }
        
        # Sort measurements into sectors
        for quality, angle, distance in scan_data:
            if quality > 5 and 100 <= distance <= 8000:  # Valid measurement
                distance_cm = distance / 10.0
                
                # Determine sector
                if angle <= 22.5 or angle >= 337.5:
                    sectors['front'].append(distance_cm)
                elif 22.5 < angle <= 67.5:
                    sectors['front_right'].append(distance_cm)
                elif 67.5 < angle <= 112.5:
                    sectors['right'].append(distance_cm)
                elif 112.5 < angle <= 157.5:
                    sectors['back_right'].append(distance_cm)
                elif 157.5 < angle <= 202.5:
                    sectors['back'].append(distance_cm)
                elif 202.5 < angle <= 247.5:
                    sectors['back_left'].append(distance_cm)
                elif 247.5 < angle <= 292.5:
                    sectors['left'].append(distance_cm)
                elif 292.5 < angle <= 337.5:
                    sectors['front_left'].append(distance_cm)
        
        # Update range data with minimum distances
        current_time = time.time()
        for sector_name, distances in sectors.items():
            if distances:
                self.range_data[sector_name] = min(distances)
            else:
                self.range_data[sector_name] = 200.0  # No detection
        
        self.range_data['last_update'] = current_time
        
        # Process with SLAM if available
        if self.slam_available and self.slam:
            try:
                # Convert to 360-degree array for SLAM
                distances_mm = [2000] * 360  # 2m default
                for quality, angle, distance in scan_data:
                    if quality > 5 and 100 <= distance <= 8000:
                        angle_idx = int(angle) % 360
                        distances_mm[angle_idx] = int(distance)
                
                # Fill missing angles
                for i in range(360):
                    if distances_mm[i] == 2000:
                        distances_mm[i] = 1500  # 1.5m safe default
                
                # Update SLAM
                self.slam.update(distances_mm)
                
                # Get position update
                x_mm, y_mm, theta_deg = self.slam.getpos()
                self.robot_x = x_mm / 1000.0
                self.robot_y = y_mm / 1000.0
                self.robot_theta = math.radians(theta_deg)
                self.current_heading = theta_deg
                
                self.slam.getmap(self.mapbytes)
                print(f"🗺️ SLAM updated - Position: ({self.robot_x:.1f}, {self.robot_y:.1f})")
                
            except Exception as slam_error:
                print(f"⚠️ SLAM update failed: {slam_error}")
    
    def simulate_range_data(self):
        """Generate simulated range data"""
        current_time = time.time()
        
        # Create realistic room simulation
        self.range_data.update({
            'front': 150 + 50 * math.sin(current_time * 0.1),
            'front_right': 120 + 30 * math.sin(current_time * 0.15),
            'right': 200,
            'back_right': 180,
            'back': 200,
            'back_left': 170,
            'left': 140 + 40 * math.sin(current_time * 0.12),
            'front_left': 130 + 35 * math.sin(current_time * 0.08),
            'last_update': current_time
        })
    
    def navigation_loop(self):
        """Main navigation loop using sector data"""
        while self.running:
            try:
                # Only navigate if in autonomous mode
                mode = self.robot_state.get('mode', 0)
                if mode != 2:
                    time.sleep(0.5)
                    continue
                
                # Get current sensor data
                front_obstacle = self.range_data['front']
                left_clearance = self.range_data['left']
                right_clearance = self.range_data['right']
                front_left = self.range_data['front_left']
                front_right = self.range_data['front_right']
                
                # Include ultrasonic sensor for close obstacles
                ultrasonic_dist = self.robot_state.get('ultrasonic_distance', 200.0)
                effective_front = min(front_obstacle, ultrasonic_dist)
                
                # Compute navigation
                left_speed, right_speed = self.compute_navigation(
                    effective_front, left_clearance, right_clearance, front_left, front_right
                )
                
                # Send motor commands
                self.send_motor_command(left_speed, right_speed)
                
                # Status logging
                if time.time() % 3 < 0.3:
                    data_age = time.time() - self.range_data['last_update']
                    slam_mode = "SLAM" if self.slam_available else "BASIC"
                    
                    print(f"🧭 {slam_mode} | Front: {effective_front:.1f}cm | Age: {data_age:.1f}s")
                    print(f"   FL:{front_left:.0f} F:{front_obstacle:.0f} FR:{front_right:.0f}")
                    print(f"   L:{left_clearance:.0f}                    R:{right_clearance:.0f}")
                    print(f"   State: {self.nav_state}")
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                self.send_motor_command(0, 0)
            
            time.sleep(0.5)  # 2Hz navigation
    
    def compute_navigation(self, front_obstacle, left_clearance, right_clearance, front_left, front_right):
        """Basic navigation using sector data"""
        current_time = time.time()
        time_in_state = current_time - self.nav_start_time
        
        left_speed = self.cruise_speed
        right_speed = self.cruise_speed
        
        # Emergency obstacle avoidance
        if front_obstacle < self.obstacle_threshold:
            self.nav_state = 'AVOIDING'
            self.nav_start_time = current_time
            
            # Choose best escape direction
            if left_clearance > right_clearance and left_clearance > 80:
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
                print(f"🚨 AVOIDING LEFT - Front:{front_obstacle:.1f} (L:{left_clearance:.0f} > R:{right_clearance:.0f})")
            elif right_clearance > left_clearance and right_clearance > 80:
                left_speed = self.turn_speed
                right_speed = -self.turn_speed
                print(f"🚨 AVOIDING RIGHT - Front:{front_obstacle:.1f} (R:{right_clearance:.0f} > L:{left_clearance:.0f})")
            else:
                # Both sides blocked, turn around
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
                print(f"🚨 BLOCKED - Turning around")
        
        elif self.nav_state == 'AVOIDING':
            # Continue avoiding until clear
            if time_in_state > 1.5 and front_obstacle > self.safe_distance:
                self.nav_state = 'EXPLORING'
                self.nav_start_time = current_time
                print("✅ Clear - resuming exploration")
            else:
                # Continue avoidance
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
        
        elif self.nav_state == 'EXPLORING':
            # Basic exploration - turn occasionally
            if time_in_state > 8.0:
                self.nav_state = 'RANDOM_TURN'
                self.nav_start_time = current_time
                print("🔄 Random exploration turn")
            else:
                # Adjust for side obstacles
                if front_left < front_right and front_left < 100:
                    # Obstacle on front-left, turn slightly right
                    left_speed = self.cruise_speed
                    right_speed = self.cruise_speed * 0.7
                elif front_right < front_left and front_right < 100:
                    # Obstacle on front-right, turn slightly left
                    left_speed = self.cruise_speed * 0.7
                    right_speed = self.cruise_speed
        
        elif self.nav_state == 'RANDOM_TURN':
            left_speed = -self.turn_speed
            right_speed = self.turn_speed
            
            if time_in_state > 1.5:
                self.nav_state = 'EXPLORING'
                self.nav_start_time = current_time
                print("🔍 Turn complete - exploring")
        
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
                if self.lidar_motor_running:
                    self.lidar.stop_motor()
                self.lidar.disconnect()
            except:
                pass
        
        print("🛑 Conservative Navigator stopped")

def main():
    print("=" * 60)
    print("🛡️ CONSERVATIVE SLAM NAVIGATOR")
    print("=" * 60)
    print()
    print("Strategy: On-demand scanning to prevent buffer overruns")
    print("Features: Motor start/stop, 8-sector navigation, SLAM integration")
    print("Libraries: rplidar-roboticia + BreezySLAM")
    print()
    
    navigator = ConservativeNavigator()
    
    try:
        if navigator.start():
            print("✅ Conservative Navigator ready")
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