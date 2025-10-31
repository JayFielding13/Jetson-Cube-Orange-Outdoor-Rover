#!/usr/bin/env python3
"""
Single-Scan LiDAR Navigator
Solves buffer overflow by getting fresh scan each cycle
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

class SingleScanNavigator:
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
        self.stuck_counter = 0
        
        # Movement parameters
        self.cruise_speed = 100
        self.slow_speed = 60
        self.turn_speed = 80
        self.reverse_speed = 60
        
        # 4-quadrant LiDAR data
        self.lidar_data = {
            'front_distance': 200.0,
            'left_distance': 200.0,
            'rear_distance': 200.0,
            'right_distance': 200.0,
            'front_avg': 200.0,
            'left_avg': 200.0,
            'rear_avg': 200.0,
            'right_avg': 200.0,
            'points_processed': 0,
            'scan_number': 0,
            'last_scan_time': time.time()
        }
        
        # Navigation parameters
        self.danger_threshold = 40.0
        self.caution_threshold = 80.0
        self.safe_distance = 120.0
        self.stuck_threshold = 5
        
        print("🚀 Single-Scan Navigator Initialized")
        print("🎯 Solves buffer overflow with fresh scans each cycle")
    
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
            return True
    
    def start(self):
        """Start the navigator"""
        if not self.connect_arduino():
            return False
        
        self.connect_lidar()
        
        self.running = True
        
        # Start communication threads
        self.arduino_thread = threading.Thread(target=self.arduino_loop, daemon=True)
        self.navigation_thread = threading.Thread(target=self.navigation_loop, daemon=True)
        
        self.arduino_thread.start()
        self.navigation_thread.start()
        
        print("🚀 Single-Scan Navigator started")
        print("🧭 Fresh scan navigation active")
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
    
    def get_single_fresh_scan(self):
        """Get ONE fresh LiDAR scan, avoiding buffer issues"""
        if not LIDAR_AVAILABLE or not self.lidar:
            return self.simulate_scan_data()
        
        try:
            # Create fresh iterator for each scan
            scan_generator = self.lidar.iter_scans()
            
            # Get exactly ONE scan and process immediately
            scan = next(scan_generator)
            
            # Process scan data immediately
            return self.process_scan_to_quadrants(scan)
            
        except Exception as e:
            print(f"⚠️ Fresh scan failed: {e}")
            return self.simulate_scan_data()
    
    def process_scan_to_quadrants(self, scan):
        """Process scan data into 4 quadrants immediately"""
        front_distances = []  # 315° to 45°
        left_distances = []   # 45° to 135°
        rear_distances = []   # 135° to 225°
        right_distances = []  # 225° to 315°
        
        points_count = 0
        
        for quality, angle, distance in scan:
            if distance > 0 and quality > 5:
                dist_cm = distance / 10.0
                if 10 <= dist_cm <= 500:
                    points_count += 1
                    
                    # Classify into quadrants
                    if angle <= 45 or angle >= 315:
                        front_distances.append(dist_cm)
                    elif 45 < angle <= 135:
                        left_distances.append(dist_cm)
                    elif 135 < angle <= 225:
                        rear_distances.append(dist_cm)
                    elif 225 < angle < 315:
                        right_distances.append(dist_cm)
        
        # Calculate results immediately
        result = {
            'front_distance': min(front_distances) if front_distances else 200.0,
            'left_distance': min(left_distances) if left_distances else 200.0,
            'rear_distance': min(rear_distances) if rear_distances else 200.0,
            'right_distance': min(right_distances) if right_distances else 200.0,
            'front_avg': sum(front_distances) / len(front_distances) if front_distances else 200.0,
            'left_avg': sum(left_distances) / len(left_distances) if left_distances else 200.0,
            'rear_avg': sum(rear_distances) / len(rear_distances) if rear_distances else 200.0,
            'right_avg': sum(right_distances) / len(right_distances) if right_distances else 200.0,
            'points_processed': points_count,
            'scan_number': self.lidar_data.get('scan_number', 0) + 1,
            'last_scan_time': time.time()
        }
        
        return result
    
    def simulate_scan_data(self):
        """Generate simulated scan data"""
        current_time = time.time()
        
        # Simulate dynamic obstacles
        base_front = 150 + 50 * math.sin(current_time * 0.1)
        base_left = 120 + 30 * math.sin(current_time * 0.15)
        base_right = 140 + 40 * math.sin(current_time * 0.12)
        base_rear = 200
        
        return {
            'front_distance': max(30.0, base_front),
            'left_distance': max(40.0, base_left),
            'rear_distance': max(50.0, base_rear),
            'right_distance': max(35.0, base_right),
            'front_avg': max(50.0, base_front + 20),
            'left_avg': max(60.0, base_left + 25),
            'rear_avg': max(80.0, base_rear),
            'right_avg': max(55.0, base_right + 15),
            'points_processed': 0,
            'scan_number': self.lidar_data.get('scan_number', 0) + 1,
            'last_scan_time': current_time
        }
    
    def navigation_loop(self):
        """Main navigation with fresh scan each cycle"""
        while self.running:
            try:
                # Only navigate if in autonomous mode
                mode = self.robot_state.get('mode', 0)
                if mode != 2:
                    time.sleep(0.5)
                    continue
                
                # Get ONE fresh scan per navigation cycle
                scan_start = time.time()
                fresh_data = self.get_single_fresh_scan()
                scan_time = time.time() - scan_start
                
                # Update lidar data with fresh scan
                self.lidar_data.update(fresh_data)
                
                # Get quadrant data
                front_min = self.lidar_data['front_distance']
                left_min = self.lidar_data['left_distance']
                right_min = self.lidar_data['right_distance']
                rear_min = self.lidar_data['rear_distance']
                
                front_avg = self.lidar_data['front_avg']
                left_avg = self.lidar_data['left_avg']
                right_avg = self.lidar_data['right_avg']
                rear_avg = self.lidar_data['rear_avg']
                
                ultrasonic_dist = self.robot_state.get('ultrasonic_distance') or 200.0
                front_obstacle = min(front_min, ultrasonic_dist)
                
                # Compute navigation
                left_speed, right_speed = self.compute_4quadrant_navigation(
                    front_obstacle, left_min, right_min, rear_min,
                    front_avg, left_avg, right_avg, rear_avg
                )
                
                # Send motor commands
                self.send_motor_command(left_speed, right_speed)
                
                # Detailed logging every 3 seconds
                if time.time() % 3 < 0.4:
                    scan_num = self.lidar_data['scan_number']
                    points = self.lidar_data['points_processed']
                    sensor_mode = "LiDAR" if points > 0 else "SIM"
                    
                    print(f"🧭 Scan #{scan_num} ({scan_time:.3f}s) | Front: {front_obstacle:.1f}cm | {sensor_mode}: {points}pts")
                    print(f"   Quadrants: L:{left_min:.0f}cm R:{right_min:.0f}cm Rear:{rear_min:.0f}cm")
                    print(f"   Averages:  L:{left_avg:.0f}cm R:{right_avg:.0f}cm Rear:{rear_avg:.0f}cm")
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                self.send_motor_command(0, 0)
            
            time.sleep(1.0)  # 1Hz navigation - gives time for fresh scans
    
    def compute_4quadrant_navigation(self, front_obstacle, left_min, right_min, rear_min,
                                   front_avg, left_avg, right_avg, rear_avg):
        """Enhanced navigation using 4-quadrant analysis"""
        current_time = time.time()
        time_in_state = current_time - self.nav_start_time
        
        left_speed = self.cruise_speed
        right_speed = self.cruise_speed
        
        # Detect if stuck
        if abs(front_obstacle - getattr(self, 'last_front_distance', 0)) < 2.0:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
        self.last_front_distance = front_obstacle
        
        if self.nav_state == 'STRAIGHT':
            if front_obstacle < self.danger_threshold:
                self.nav_state = 'AVOIDING'
                self.nav_start_time = current_time
                
                # Smart direction selection
                if rear_avg > 100 and front_obstacle < 25:
                    self.turn_direction = 'reverse'
                    print(f"🚨 TRAPPED! Front={front_obstacle:.1f}cm - REVERSING (rear: {rear_avg:.1f}cm)")
                elif left_avg > right_avg and left_min > 60:
                    self.turn_direction = 'left'
                    print(f"🚨 DANGER! Front={front_obstacle:.1f}cm - turning LEFT (left: {left_avg:.1f}cm)")
                elif right_avg > left_avg and right_min > 60:
                    self.turn_direction = 'right'
                    print(f"🚨 DANGER! Front={front_obstacle:.1f}cm - turning RIGHT (right: {right_avg:.1f}cm)")
                else:
                    self.turn_direction = 'left' if left_min > right_min else 'right'
                    print(f"🚨 DANGER! Front={front_obstacle:.1f}cm - turning {self.turn_direction.upper()}")
                    
            elif front_obstacle < self.caution_threshold:
                left_speed = self.slow_speed
                right_speed = self.slow_speed
                print(f"⚠️ Slowing down - obstacle at {front_obstacle:.1f}cm")
                
            elif time_in_state > 8.0 or self.stuck_counter > self.stuck_threshold:
                self.nav_state = 'EXPLORING'
                self.nav_start_time = current_time
                self.stuck_counter = 0
                self.turn_direction = 'left' if left_avg > right_avg else 'right'
                print(f"🔍 Exploration turn {self.turn_direction}")
        
        elif self.nav_state == 'AVOIDING':
            if self.turn_direction == 'reverse':
                left_speed = -self.reverse_speed
                right_speed = -self.reverse_speed
            elif self.turn_direction == 'left':
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
            else:
                left_speed = self.turn_speed
                right_speed = -self.turn_speed
            
            if time_in_state > 2.0 and front_obstacle > self.safe_distance:
                self.nav_state = 'STRAIGHT'
                self.nav_start_time = current_time
                print("✅ Path cleared - resuming forward")
        
        elif self.nav_state == 'EXPLORING':
            if self.turn_direction == 'left':
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
            else:
                left_speed = self.turn_speed
                right_speed = -self.turn_speed
            
            if time_in_state > 1.2:
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
        
        print("🛑 Single-Scan Navigator stopped")

def main():
    print("=" * 60)
    print("🚀 SINGLE-SCAN LIDAR NAVIGATOR")
    print("=" * 60)
    print()
    print("Solution: Fresh scan each cycle to avoid buffer overflow")
    print("Features: 4-quadrant analysis with dynamic readings")
    print()
    
    navigator = SingleScanNavigator()
    
    try:
        if navigator.start():
            print("✅ Single-Scan Navigator ready")
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