#!/usr/bin/env python3
"""
LiDAR-Enhanced Autonomous Controller
Uses RPLIDAR for 360° navigation with ultrasonic emergency backup
"""

import serial
import json
import time
import threading
import queue
from datetime import datetime
import statistics
import random
import math

class LiDARAutonomousController:
    def __init__(self, arduino_port='/dev/ttyUSB0', lidar_port='/dev/ttyUSB1', baud=115200):
        self.arduino_port = arduino_port
        self.lidar_port = lidar_port
        self.baud = baud
        self.arduino = None
        self.lidar = None
        self.running = False
        
        # Robot state
        self.robot_state = {
            'rc_ch1': 0, 'rc_ch2': 0, 'rc_ch9': 0,
            'rc_valid': False, 'mode': 0,
            'emergency_stop': False,
            'last_update': time.time()
        }
        
        # LiDAR data
        self.scan_data = {}  # angle -> distance mapping
        self.scan_lock = threading.Lock()
        
        # Navigation state machine
        self.nav_state = 'STRAIGHT'
        self.nav_state_start = time.time()
        self.last_direction_change = time.time()
        
        # Navigation parameters
        self.cruise_speed = 180
        self.slow_speed = 100
        self.turn_speed = 140
        
        # LiDAR-based obstacle detection
        self.front_sector_width = 60  # degrees (±30° from front)
        self.side_sector_width = 30   # degrees for side detection
        self.detection_distance = 80.0  # cm - much further than ultrasonic
        self.slow_distance = 120.0      # cm - start slowing
        self.critical_distance = 40.0   # cm - immediate stop
        
        # Timing parameters
        self.min_straight_time = 8.0    # Long confident straight runs
        self.confirm_time = 1.0         # Quick confirmation with LiDAR
        self.turn_time = 2.5            # Turn duration
        self.explore_interval = (45, 90) # Random exploration
        
        # Path finding
        self.preferred_clearance = 150.0  # cm - preferred open space
        
    def connect_arduino(self):
        """Connect to Arduino bridge"""
        try:
            self.arduino = serial.Serial(self.arduino_port, self.baud, timeout=1)
            time.sleep(2)
            print(f"✅ Connected to Arduino bridge on {self.arduino_port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to Arduino: {e}")
            return False
    
    def connect_lidar(self):
        """Connect to RPLIDAR"""
        try:
            from rplidar import RPLidar
            self.lidar = RPLidar(self.lidar_port)
            
            # Get device info
            info = self.lidar.get_info()
            print(f"✅ Connected to RPLIDAR: {info}")
            
            # Start motor
            self.lidar.start_motor()
            time.sleep(1)
            
            print(f"📡 LiDAR initialized on {self.lidar_port}")
            return True
            
        except ImportError:
            print("❌ RPLidar library not installed. Install with: pip install rplidar")
            return False
        except Exception as e:
            print(f"❌ Failed to connect to LiDAR: {e}")
            return False
    
    def start(self):
        """Start the LiDAR autonomous controller"""
        if not self.connect_arduino():
            return False
        
        if not self.connect_lidar():
            print("⚠️ Running without LiDAR - ultrasonic only")
        
        self.running = True
        
        # Start threads
        self.arduino_thread = threading.Thread(target=self.arduino_communication_loop, daemon=True)
        self.lidar_thread = threading.Thread(target=self.lidar_reading_loop, daemon=True)
        self.navigation_thread = threading.Thread(target=self.navigation_loop, daemon=True)
        
        self.arduino_thread.start()
        if self.lidar:
            self.lidar_thread.start()
        self.navigation_thread.start()
        
        print("🚀 LiDAR autonomous controller started")
        print("🎮 Switch RC to AUTONOMOUS mode for 360° intelligent navigation")
        return True
    
    def arduino_communication_loop(self):
        """Handle Arduino communication"""
        while self.running and self.arduino:
            try:
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode().strip()
                    if line and line.startswith('{'):
                        arduino_data = json.loads(line)
                        self.robot_state.update({
                            'rc_ch1': arduino_data.get('ch1', 0),
                            'rc_ch2': arduino_data.get('ch2', 0),
                            'rc_ch9': arduino_data.get('ch9', 0),
                            'rc_valid': arduino_data.get('valid', False),
                            'mode': arduino_data.get('mode', 0),
                            'emergency_stop': arduino_data.get('emergency', False),
                            'last_update': time.time()
                        })
            except Exception as e:
                print(f"❌ Arduino error: {e}")
                time.sleep(0.1)
            time.sleep(0.01)
    
    def lidar_reading_loop(self):
        """LiDAR data acquisition loop with error recovery"""
        if not self.lidar:
            return
            
        print("📡 Starting LiDAR scan acquisition")
        
        retry_count = 0
        max_retries = 3
        
        while self.running and retry_count < max_retries:
            try:
                # Reset and restart scanning
                self.lidar.stop_motor()
                time.sleep(0.5)
                self.lidar.start_motor()
                time.sleep(0.5)
                
                print(f"📡 LiDAR scan attempt {retry_count + 1}")
                
                for scan in self.lidar.iter_scans():
                    if not self.running:
                        break
                    
                    # Process scan data
                    scan_count = 0
                    with self.scan_lock:
                        self.scan_data.clear()
                        for (quality, angle, distance) in scan:
                            if quality > 10:  # Filter low quality readings
                                # Convert to cm and store
                                self.scan_data[angle] = distance / 10.0
                                scan_count += 1
                    
                    if scan_count > 0:
                        print(f"📊 LiDAR: {scan_count} points acquired")
                    
                    time.sleep(0.2)  # 5Hz scan processing
                
                # If we get here, scanning worked
                break
                
            except Exception as e:
                retry_count += 1
                print(f"❌ LiDAR error (attempt {retry_count}): {e}")
                if retry_count < max_retries:
                    print(f"🔄 Retrying LiDAR in 2 seconds...")
                    time.sleep(2)
                else:
                    print("❌ LiDAR failed after 3 attempts - continuing without scan data")
                    break
    
    def navigation_loop(self):
        """Main navigation loop"""
        print("🧭 LiDAR navigation started")
        
        while self.running:
            try:
                if (self.robot_state.get('mode') == 2 and 
                    self.robot_state.get('rc_valid') and 
                    not self.robot_state.get('emergency_stop')):
                    
                    self.lidar_navigation()
                else:
                    # Not in autonomous mode
                    pass
                
                time.sleep(0.1)  # 10Hz navigation updates
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                time.sleep(1)
    
    def lidar_navigation(self):
        """LiDAR-based intelligent navigation with fallback"""
        current_time = time.time()
        time_in_state = current_time - self.nav_state_start
        
        # Get obstacle analysis
        front_clear, front_distance = self.analyze_front_sector()
        best_direction = self.find_best_direction()
        
        # If no LiDAR data, use simple navigation
        if len(self.scan_data) == 0:
            self.simple_navigation(current_time, time_in_state)
            return
        
        if self.nav_state == 'STRAIGHT':
            # Confident straight movement
            self.send_motor_command(self.cruise_speed, self.cruise_speed)
            
            # Minimum straight time for confidence
            if time_in_state < self.min_straight_time:
                return
            
            # Check for obstacles ahead
            if not front_clear or front_distance < self.slow_distance:
                self.change_nav_state('SLOWING')
                print(f"🟡 LiDAR: Obstacle ahead at {front_distance:.0f}cm - slowing")
                return
            
            # Random exploration
            if current_time - self.last_direction_change > self.get_random_interval():
                self.change_nav_state('TURNING')
                self.last_direction_change = current_time
                print("🔄 LiDAR: Random exploration turn")
        
        elif self.nav_state == 'SLOWING':
            # Approach cautiously
            self.send_motor_command(self.slow_speed, self.slow_speed)
            
            # If path clears, resume
            if front_clear and front_distance > self.slow_distance:
                self.change_nav_state('STRAIGHT')
                print("✅ LiDAR: Path cleared - resuming cruise")
                return
            
            # If too close, stop and turn
            if front_distance < self.detection_distance:
                self.change_nav_state('TURNING')
                print(f"🔴 LiDAR: Too close ({front_distance:.0f}cm) - turning")
        
        elif self.nav_state == 'TURNING':
            # Turn toward best direction
            if best_direction == 'left':
                self.send_motor_command(-self.turn_speed, self.turn_speed)
                print("↺ LiDAR: Turning left toward open space")
            elif best_direction == 'right':
                self.send_motor_command(self.turn_speed, -self.turn_speed)
                print("↻ LiDAR: Turning right toward open space")
            else:
                # No clear direction - turn randomly
                if random.choice([True, False]):
                    self.send_motor_command(-self.turn_speed, self.turn_speed)
                else:
                    self.send_motor_command(self.turn_speed, -self.turn_speed)
                print("🔄 LiDAR: Random turn - no clear path")
            
            # Turn for specified duration
            if time_in_state > self.turn_time:
                self.change_nav_state('STRAIGHT')
                print("➡️ LiDAR: Turn complete - resuming straight")
    
    def analyze_front_sector(self):
        """Analyze front sector for obstacles"""
        with self.scan_lock:
            if not self.scan_data:
                return True, 999.0  # No data = assume clear
            
            front_distances = []
            
            # Check front sector (±30° from front)
            for angle in range(-30, 31):
                normalized_angle = angle % 360
                if normalized_angle in self.scan_data:
                    front_distances.append(self.scan_data[normalized_angle])
            
            if not front_distances:
                return True, 999.0
            
            # Use minimum distance in front sector
            min_distance = min(front_distances)
            avg_distance = statistics.mean(front_distances)
            
            # Consider clear if average distance is good and no close obstacles
            is_clear = (avg_distance > self.detection_distance and 
                       min_distance > self.critical_distance)
            
            return is_clear, min_distance
    
    def find_best_direction(self):
        """Find the direction with most open space"""
        with self.scan_lock:
            if not self.scan_data:
                return 'straight'
            
            # Analyze left and right sectors
            left_clearance = self.analyze_sector(270, 330)  # Left side
            right_clearance = self.analyze_sector(30, 90)   # Right side
            
            print(f"🔍 LiDAR clearance - Left: {left_clearance:.0f}cm, Right: {right_clearance:.0f}cm")
            
            # Choose direction with more clearance
            if left_clearance > right_clearance + 20:  # 20cm bias to avoid oscillation
                return 'left'
            elif right_clearance > left_clearance + 20:
                return 'right'
            else:
                return 'straight'
    
    def analyze_sector(self, start_angle, end_angle):
        """Analyze clearance in angular sector"""
        distances = []
        
        for angle in range(start_angle, end_angle + 1):
            normalized_angle = angle % 360
            if normalized_angle in self.scan_data:
                distances.append(self.scan_data[normalized_angle])
        
        if not distances:
            return 0.0
        
        # Return average clearance in sector
        return statistics.mean(distances)
    
    def change_nav_state(self, new_state):
        """Change navigation state"""
        self.nav_state = new_state
        self.nav_state_start = time.time()
    
    def get_random_interval(self):
        """Get random exploration interval"""
        return random.uniform(self.explore_interval[0], self.explore_interval[1])
    
    def send_motor_command(self, left_speed, right_speed):
        """Send motor command to Arduino"""
        if not self.arduino:
            return
        
        left_speed = max(-255, min(255, int(left_speed)))
        right_speed = max(-255, min(255, int(right_speed)))
        
        command = f'{{"left":{left_speed},"right":{right_speed}}}\n'
        
        try:
            self.arduino.write(command.encode())
        except Exception as e:
            print(f"❌ Motor command failed: {e}")
    
    def get_status(self):
        """Get detailed status"""
        mode_names = {0: 'FAILSAFE', 1: 'MANUAL', 2: 'AUTONOMOUS'}
        
        front_clear, front_distance = self.analyze_front_sector()
        scan_points = len(self.scan_data) if self.scan_data else 0
        
        return {
            'mode': mode_names.get(self.robot_state.get('mode'), 'UNKNOWN'),
            'nav_state': self.nav_state,
            'front_distance': front_distance,
            'front_clear': front_clear,
            'scan_points': scan_points,
            'rc_valid': self.robot_state.get('rc_valid'),
            'emergency_stop': self.robot_state.get('emergency_stop'),
            'time_in_state': time.time() - self.nav_state_start
        }
    
    def disconnect(self):
        """Clean shutdown"""
        self.running = False
        
        if self.lidar:
            try:
                self.lidar.stop_motor()
                self.lidar.disconnect()
                print("📡 LiDAR disconnected")
            except:
                pass
        
        if self.arduino:
            self.arduino.close()
            print("🔌 Arduino disconnected")
    
    def simple_navigation(self, current_time, time_in_state):
        """Simple navigation fallback when no LiDAR data"""
        if self.nav_state == 'STRAIGHT':
            # Move straight confidently
            self.send_motor_command(self.cruise_speed, self.cruise_speed)
            
            # Turn every 8-12 seconds for exploration
            if time_in_state > random.uniform(8, 12):
                self.change_nav_state('TURNING')
                self.turn_direction = random.choice(['left', 'right'])
                print(f"🔄 Simple navigation: turning {self.turn_direction}")
        
        elif self.nav_state == 'TURNING':
            # Execute turn
            if self.turn_direction == 'left':
                self.send_motor_command(-self.turn_speed, self.turn_speed)
            else:
                self.send_motor_command(self.turn_speed, -self.turn_speed)
            
            # Turn for 2-3 seconds
            if time_in_state > 2.5:
                self.change_nav_state('STRAIGHT')
                print("➡️ Simple navigation: resuming straight")

def main():
    print("=" * 60)
    print("🌟 LIDAR-ENHANCED AUTONOMOUS CONTROLLER")
    print("=" * 60)
    print()
    print("Features:")
    print("📡 RPLIDAR: 360° obstacle detection and path planning")
    print("🔧 Arduino: Real-time motor control and emergency stop")
    print("🧠 Pi: Intelligent navigation with spatial awareness")
    print("⚡ Smart: Long confident runs with optimal path selection")
    print()
    print("Requirements:")
    print("📦 pip install rplidar")
    print("🔌 LiDAR on /dev/ttyUSB1, Arduino on /dev/ttyUSB0")
    print()
    
    controller = LiDARAutonomousController()
    
    if not controller.start():
        print("❌ Failed to start LiDAR controller")
        return
    
    try:
        print("🔄 LiDAR controller running.")
        print("🎮 Switch to AUTONOMOUS mode for 360° navigation.")
        print("📊 Detailed status every 3 seconds. Press Ctrl+C to stop.")
        print()
        
        while True:
            time.sleep(3)
            status = controller.get_status()
            timestamp = datetime.now().strftime("%H:%M:%S")
            
            print(f"[{timestamp}] {status['mode']:<12} | "
                  f"{status['nav_state']:<12} | "
                  f"Front: {status['front_distance']:.0f}cm {'✅' if status['front_clear'] else '🚫'} | "
                  f"Scan: {status['scan_points']:3d}pts | "
                  f"RC: {'✅' if status['rc_valid'] else '❌'} | "
                  f"Emerg: {'🛑' if status['emergency_stop'] else '✅'}")
    
    except KeyboardInterrupt:
        print("\n⏹️ Stopping LiDAR controller...")
    
    finally:
        controller.disconnect()
        print("🏁 LiDAR controller stopped")

if __name__ == "__main__":
    main()