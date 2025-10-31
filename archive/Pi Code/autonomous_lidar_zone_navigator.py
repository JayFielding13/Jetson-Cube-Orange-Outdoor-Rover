#!/usr/bin/env python3
"""
Zone-Based LiDAR Navigator
Uses 8-zone processing to prevent buffer overruns and improve performance
"""

import serial
import json
import time
import threading
import random
import math

try:
    import rplidar
    LIDAR_AVAILABLE = True
    print("✅ RPLidar library found")
except ImportError:
    LIDAR_AVAILABLE = False
    print("⚠️ RPLidar library not found - will simulate LiDAR data")

class ZoneNavigator:
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
        self.nav_state = 'STRAIGHT'
        self.nav_start_time = time.time()
        self.turn_direction = 'left'
        
        # Movement parameters
        self.cruise_speed = 100
        self.slow_speed = 60
        self.turn_speed = 80
        
        # 8-Zone LiDAR data (45° sectors)
        self.zone_angles = [
            (337.5, 22.5),    # Zone 0: Front (0°)
            (22.5, 67.5),     # Zone 1: Front-Right (45°)
            (67.5, 112.5),    # Zone 2: Right (90°)
            (112.5, 157.5),   # Zone 3: Rear-Right (135°)
            (157.5, 202.5),   # Zone 4: Rear (180°)
            (202.5, 247.5),   # Zone 5: Rear-Left (225°)
            (247.5, 292.5),   # Zone 6: Left (270°)
            (292.5, 337.5),   # Zone 7: Front-Left (315°)
        ]
        
        self.zone_names = [
            "FRONT", "FRONT-RIGHT", "RIGHT", "REAR-RIGHT",
            "REAR", "REAR-LEFT", "LEFT", "FRONT-LEFT"
        ]
        
        # Zone distance data
        self.zone_distances = [200.0] * 8  # Initialize all zones to 200cm
        self.zone_point_counts = [0] * 8   # Track points per zone
        self.zone_last_update = [time.time()] * 8
        
        # Processing parameters
        self.measurement_timeout = 0.1  # 100ms timeout per measurement
        self.zone_update_interval = 2.0  # Reset zones every 2 seconds
        self.max_distance = 400.0  # Maximum valid distance (cm)
        self.min_distance = 5.0    # Minimum valid distance (cm)
        
        # Navigation parameters
        self.danger_threshold = 40.0
        self.caution_threshold = 80.0
        self.safe_distance = 120.0
        
        print("🚀 Zone Navigator Initialized")
        print(f"📊 Using 8 zones with {45}° sectors each")
    
    def get_zone_for_angle(self, angle):
        """Determine which zone an angle belongs to"""
        # Normalize angle to 0-360
        angle = angle % 360
        
        for zone_idx, (start, end) in enumerate(self.zone_angles):
            if start > end:  # Handle wraparound (like 337.5 to 22.5)
                if angle >= start or angle <= end:
                    return zone_idx
            else:
                if start <= angle <= end:
                    return zone_idx
        
        return 0  # Default to front zone
    
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
        """Connect to LiDAR sensor with minimal buffer usage"""
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
            
            # Start motor but don't start scanning yet
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
        
        print("🚀 Zone Navigator started")
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
        """Zone-based LiDAR processing"""
        measurement_count = 0
        last_zone_reset = time.time()
        
        while self.running:
            try:
                if LIDAR_AVAILABLE and self.lidar:
                    success = self.process_zone_measurements()
                    if success:
                        measurement_count += 1
                    else:
                        # If LiDAR fails, use simulation
                        self.simulate_zone_data()
                else:
                    self.simulate_zone_data()
                
                # Reset zones periodically to clear stale data
                current_time = time.time()
                if current_time - last_zone_reset > self.zone_update_interval:
                    self.reset_stale_zones()
                    last_zone_reset = current_time
                    
            except Exception as e:
                print(f"⚠️ LiDAR error: {e}")
                self.simulate_zone_data()
            
            time.sleep(0.05)  # 20Hz processing for responsive navigation
    
    def process_zone_measurements(self):
        """Process LiDAR measurements using micro-batch approach"""
        try:
            # Clear any buffer buildup
            if hasattr(self.lidar, '_serial') and self.lidar._serial.in_waiting > 2000:
                print(f"🧹 Clearing buffer: {self.lidar._serial.in_waiting} bytes")
                self.lidar._serial.reset_input_buffer()
            
            # Use iter_scans with very small batches to prevent buffer overflow
            scan_generator = self.lidar.iter_scans(max_buf_meas=50)  # Tiny buffer
            measurements_processed = 0
            start_time = time.time()
            
            # Process just one small scan per cycle
            try:
                scan = next(scan_generator)
                current_time = time.time()
                
                # Process each measurement in this micro-batch
                for quality, angle, distance in scan:
                    # Timeout protection
                    if time.time() - start_time > self.measurement_timeout:
                        break
                    
                    # Validate measurement
                    if quality > 5 and distance > 0 and self.min_distance <= distance/10.0 <= self.max_distance:
                        zone_idx = self.get_zone_for_angle(angle)
                        distance_cm = distance / 10.0
                        
                        # Update zone with closest distance
                        if distance_cm < self.zone_distances[zone_idx]:
                            self.zone_distances[zone_idx] = distance_cm
                            self.zone_last_update[zone_idx] = current_time
                        
                        self.zone_point_counts[zone_idx] += 1
                        measurements_processed += 1
                        
                        # Limit processing per cycle to prevent lag
                        if measurements_processed >= 30:
                            break
                
                return measurements_processed > 0
                
            except StopIteration:
                # No scan available yet
                return False
            
        except Exception as e:
            print(f"⚠️ Measurement processing failed: {e}")
            return False
    
    def reset_stale_zones(self):
        """Reset zones that haven't been updated recently"""
        current_time = time.time()
        for i in range(8):
            if current_time - self.zone_last_update[i] > self.zone_update_interval:
                self.zone_distances[i] = 200.0  # Reset to safe distance
                self.zone_point_counts[i] = 0
    
    def simulate_zone_data(self):
        """Generate simulated zone data"""
        current_time = time.time()
        
        for i in range(8):
            # Create dynamic obstacles in different zones
            base_distance = 120 + 50 * math.sin(current_time * 0.1 + i * 0.5)
            self.zone_distances[i] = max(30.0, base_distance)
            self.zone_point_counts[i] = 0  # Mark as simulated
            self.zone_last_update[i] = current_time
    
    def navigation_loop(self):
        """Main navigation loop using zone data"""
        while self.running:
            try:
                # Only navigate if in autonomous mode
                mode = self.robot_state.get('mode', 0)
                if mode != 2:
                    time.sleep(0.5)
                    continue
                
                # Get zone distances for navigation
                front_dist = self.zone_distances[0]  # Front zone
                front_right_dist = self.zone_distances[1]
                front_left_dist = self.zone_distances[7]
                left_dist = self.zone_distances[6]
                right_dist = self.zone_distances[2]
                
                # Combine with ultrasonic for close obstacles
                ultrasonic_dist = self.robot_state.get('ultrasonic_distance', 200.0)
                effective_front = min(front_dist, ultrasonic_dist)
                
                # Compute navigation based on zones
                left_speed, right_speed = self.compute_zone_navigation(
                    effective_front, front_left_dist, front_right_dist, 
                    left_dist, right_dist
                )
                
                # Send motor commands
                self.send_motor_command(left_speed, right_speed)
                
                # Status logging
                if time.time() % 3 < 0.3:
                    total_points = sum(self.zone_point_counts)
                    sensor_mode = "LiDAR" if total_points > 0 else "SIM"
                    
                    print(f"🧭 Front: {effective_front:.1f}cm | {sensor_mode} | Points: {total_points}")
                    print(f"   Zones: FL:{front_left_dist:.0f} F:{front_dist:.0f} FR:{front_right_dist:.0f}")
                    print(f"          L:{left_dist:.0f}          R:{right_dist:.0f}")
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                self.send_motor_command(0, 0)
            
            time.sleep(0.5)  # 2Hz navigation
    
    def compute_zone_navigation(self, front, front_left, front_right, left, right):
        """Enhanced navigation using zone data"""
        current_time = time.time()
        time_in_state = current_time - self.nav_start_time
        
        left_speed = self.cruise_speed
        right_speed = self.cruise_speed
        
        if self.nav_state == 'STRAIGHT':
            if front < self.danger_threshold:
                # Choose turn direction based on side clearances
                if left > right and left > 80:
                    self.turn_direction = 'left'
                    print(f"🚨 DANGER! Front={front:.1f}cm - turning LEFT (L:{left:.0f} > R:{right:.0f})")
                elif right > left and right > 80:
                    self.turn_direction = 'right'
                    print(f"🚨 DANGER! Front={front:.1f}cm - turning RIGHT (R:{right:.0f} > L:{left:.0f})")
                else:
                    self.turn_direction = 'left'  # Default
                    print(f"🚨 DANGER! Front={front:.1f}cm - default LEFT turn")
                
                self.nav_state = 'AVOIDING'
                self.nav_start_time = current_time
                    
            elif front < self.caution_threshold:
                # Slow down and nudge away from closer side obstacle
                speed_reduction = (self.caution_threshold - front) / self.caution_threshold
                base_speed = self.cruise_speed * (1 - speed_reduction * 0.5)
                
                if front_left < front_right:
                    left_speed = base_speed * 0.8  # Turn slightly right
                    right_speed = base_speed
                elif front_right < front_left:
                    left_speed = base_speed
                    right_speed = base_speed * 0.8  # Turn slightly left
                else:
                    left_speed = right_speed = base_speed
                
                print(f"⚠️ Caution - Front:{front:.1f} FL:{front_left:.0f} FR:{front_right:.0f}")
                
            elif time_in_state > 8.0:
                self.nav_state = 'EXPLORING'
                self.nav_start_time = current_time
                print("🔍 Exploration turn")
        
        elif self.nav_state == 'AVOIDING':
            if self.turn_direction == 'left':
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
            else:
                left_speed = self.turn_speed
                right_speed = -self.turn_speed
            
            if time_in_state > 1.5 and front > self.safe_distance:
                self.nav_state = 'STRAIGHT'
                self.nav_start_time = current_time
                print("✅ Path cleared - resuming forward")
        
        elif self.nav_state == 'EXPLORING':
            left_speed = -self.turn_speed
            right_speed = self.turn_speed
            
            if time_in_state > 1.0:
                self.nav_state = 'STRAIGHT'
                self.nav_start_time = current_time
                print("🔍 Exploration complete")
        
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
        
        print("🛑 Zone Navigator stopped")

def main():
    print("=" * 60)
    print("🎯 ZONE-BASED LIDAR NAVIGATOR")
    print("=" * 60)
    print()
    print("Strategy: 8-zone processing to prevent buffer overruns")
    print("Features: Individual measurements, minimal buffer usage")
    print("Zones: 45° sectors with closest distance tracking")
    print()
    
    navigator = ZoneNavigator()
    
    try:
        if navigator.start():
            print("✅ Zone Navigator ready")
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