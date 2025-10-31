#!/usr/bin/env python3
"""
Navigation Pi - Dual Sensor Autonomous Controller
Runs on 192.168.254.65 (Primary Navigation Pi)

Focused navigation system using dual ultrasonic sensors:
- Lightweight processing dedicated to navigation only
- Uses sensor fusion data from Arduino dual sensor system
- Implements intelligent obstacle avoidance with directional awareness
- No visualization overhead - pure navigation performance
"""

import serial
import json
import time
import threading
import signal
import sys
from datetime import datetime
from enum import Enum

# Configuration
ARDUINO_PORT = '/dev/ttyUSB0'
ARDUINO_BAUD = 115200

# Navigation parameters
class NavigationMode(Enum):
    EXPLORE = "explore"
    AVOID_LEFT = "avoid_left"
    AVOID_RIGHT = "avoid_right"
    EMERGENCY_STOP = "emergency_stop"
    BACKING_UP = "backing_up"

# Speed settings
NORMAL_SPEED = 120      # Normal forward speed
TURN_SPEED = 80         # Speed for turning maneuvers
BACKUP_SPEED = -60      # Speed for backing up
STOP_SPEED = 0          # Complete stop

# Timing parameters
BACKUP_DURATION = 1.5   # Seconds to back up when stuck
TURN_DURATION = 1.0     # Seconds to turn after backing up
SENSOR_TIMEOUT = 2.0    # Max time without valid sensor data

class DualSensorNavigator:
    def __init__(self):
        self.arduino = None
        self.running = False
        
        # Navigation state
        self.mode = NavigationMode.EXPLORE
        self.mode_start_time = time.time()
        
        # Sensor data
        self.sensor_data = {
            'left_distance': None,
            'right_distance': None,
            'min_distance': None,
            'obstacle_direction': 0,
            'left_valid': False,
            'right_valid': False,
            'emergency': False,
            'last_update': None
        }
        
        # Navigation statistics
        self.stats = {
            'start_time': time.time(),
            'commands_sent': 0,
            'obstacles_avoided': 0,
            'mode_changes': 0
        }
        
        # Setup signal handlers for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
    def signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        print(f"\n🛑 Received signal {signum}, shutting down...")
        self.stop_navigation()
        sys.exit(0)
    
    def connect_arduino(self):
        """Connect to Arduino for sensor data and motor control"""
        try:
            print(f"🔌 Connecting to Arduino on {ARDUINO_PORT}...")
            self.arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
            time.sleep(2)  # Arduino startup delay
            print("✅ Arduino connected successfully!")
            
            # Send initial stop command
            self.send_motor_command(0, 0)
            return True
            
        except Exception as e:
            print(f"❌ Arduino connection failed: {e}")
            return False
    
    def parse_sensor_data(self, data_str):
        """Parse sensor telemetry from Arduino"""
        try:
            data = json.loads(data_str)
            sensors = data.get('sensors', {})
            
            # Update sensor data
            old_emergency = self.sensor_data.get('emergency', False)
            
            self.sensor_data.update({
                'left_distance': sensors.get('left_distance'),
                'right_distance': sensors.get('right_distance'),
                'min_distance': sensors.get('min_distance'),
                'obstacle_direction': sensors.get('obstacle_direction', 0),
                'left_valid': sensors.get('left_valid', False),
                'right_valid': sensors.get('right_valid', False),
                'emergency': data.get('emergency', False),
                'last_update': time.time()
            })
            
            # Track obstacle detection events
            if self.sensor_data['emergency'] and not old_emergency:
                self.stats['obstacles_avoided'] += 1
                direction_text = {-1: "LEFT", 0: "CENTER", 1: "RIGHT"}[self.sensor_data['obstacle_direction']]
                print(f"🚨 Obstacle detected: {direction_text}")
            
            return True
            
        except Exception as e:
            return False
    
    def send_motor_command(self, left_speed, right_speed):
        """Send motor command to Arduino"""
        if not self.arduino:
            return False
            
        try:
            # Use the simple command format for reliability
            command = {"left": left_speed, "right": right_speed}
            command_str = json.dumps(command) + '\n'
            
            self.arduino.write(command_str.encode('utf-8'))
            self.stats['commands_sent'] += 1
            
            return True
            
        except Exception as e:
            print(f"⚠️  Failed to send motor command: {e}")
            return False
    
    def is_sensor_data_fresh(self):
        """Check if sensor data is recent and valid"""
        if not self.sensor_data['last_update']:
            return False
            
        age = time.time() - self.sensor_data['last_update']
        return age < SENSOR_TIMEOUT
    
    def get_navigation_decision(self):
        """Determine navigation action based on sensor data"""
        # Check for fresh sensor data
        if not self.is_sensor_data_fresh():
            print("⚠️  Sensor data timeout - stopping for safety")
            return NavigationMode.EMERGENCY_STOP
        
        # Emergency stop takes highest priority
        if self.sensor_data['emergency']:
            obstacle_direction = self.sensor_data['obstacle_direction']
            
            if obstacle_direction == -1:
                return NavigationMode.AVOID_LEFT
            elif obstacle_direction == 1:
                return NavigationMode.AVOID_RIGHT
            else:  # Center or both sides
                return NavigationMode.BACKING_UP
        
        # Clear path - continue exploring
        return NavigationMode.EXPLORE
    
    def execute_navigation_mode(self):
        """Execute the current navigation mode"""
        mode_duration = time.time() - self.mode_start_time
        
        if self.mode == NavigationMode.EXPLORE:
            # Normal forward movement
            self.send_motor_command(NORMAL_SPEED, NORMAL_SPEED)
            
        elif self.mode == NavigationMode.AVOID_LEFT:
            # Obstacle on left - turn right
            self.send_motor_command(TURN_SPEED, -TURN_SPEED)
            print(f"🔄 Avoiding left obstacle - turning right")
            
        elif self.mode == NavigationMode.AVOID_RIGHT:
            # Obstacle on right - turn left  
            self.send_motor_command(-TURN_SPEED, TURN_SPEED)
            print(f"🔄 Avoiding right obstacle - turning left")
            
        elif self.mode == NavigationMode.BACKING_UP:
            # Back away from obstacle
            self.send_motor_command(BACKUP_SPEED, BACKUP_SPEED)
            print(f"🔙 Backing up from center obstacle")
            
            # Switch to turning after backup duration
            if mode_duration > BACKUP_DURATION:
                # Choose turn direction based on sensor readings
                if (self.sensor_data['left_valid'] and self.sensor_data['right_valid'] and
                    self.sensor_data['left_distance'] and self.sensor_data['right_distance']):
                    
                    if self.sensor_data['left_distance'] > self.sensor_data['right_distance']:
                        self.change_mode(NavigationMode.AVOID_RIGHT)  # Turn left (more space on left)
                    else:
                        self.change_mode(NavigationMode.AVOID_LEFT)   # Turn right (more space on right)
                else:
                    # Default turn right if sensor data unavailable
                    self.change_mode(NavigationMode.AVOID_LEFT)
            
        elif self.mode == NavigationMode.EMERGENCY_STOP:
            # Complete stop
            self.send_motor_command(STOP_SPEED, STOP_SPEED)
    
    def change_mode(self, new_mode):
        """Change navigation mode with logging"""
        if new_mode != self.mode:
            old_mode = self.mode
            self.mode = new_mode
            self.mode_start_time = time.time()
            self.stats['mode_changes'] += 1
            
            print(f"🎯 Mode change: {old_mode.value} → {new_mode.value}")
    
    def navigation_loop(self):
        """Main navigation decision loop"""
        print("🧭 Starting navigation loop...")
        
        while self.running:
            try:
                # Get navigation decision based on current sensor data
                desired_mode = self.get_navigation_decision()
                
                # Change mode if needed
                if desired_mode != self.mode:
                    self.change_mode(desired_mode)
                
                # Execute current mode
                self.execute_navigation_mode()
                
                # Check for mode timeouts
                self.check_mode_timeouts()
                
                # Control loop frequency
                time.sleep(0.1)  # 10Hz navigation loop
                
            except Exception as e:
                print(f"⚠️  Navigation loop error: {e}")
                self.send_motor_command(0, 0)  # Safety stop
                time.sleep(1)
    
    def check_mode_timeouts(self):
        """Check for mode timeouts and force transitions if needed"""
        mode_duration = time.time() - self.mode_start_time
        
        # Timeout for avoidance maneuvers
        if self.mode in [NavigationMode.AVOID_LEFT, NavigationMode.AVOID_RIGHT]:
            if mode_duration > TURN_DURATION:
                # Return to exploration if no longer detecting obstacle
                if not self.sensor_data['emergency']:
                    self.change_mode(NavigationMode.EXPLORE)
        
        # Timeout for emergency stop (try to resume after sensor recovery)
        elif self.mode == NavigationMode.EMERGENCY_STOP:
            if mode_duration > 2.0 and self.is_sensor_data_fresh():
                self.change_mode(NavigationMode.EXPLORE)
    
    def data_collection_thread(self):
        """Background thread to collect Arduino sensor data"""
        print("📡 Starting sensor data collection...")
        
        while self.running:
            try:
                if self.arduino and self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode('utf-8').strip()
                    if line.startswith('{') and line.endswith('}'):
                        self.parse_sensor_data(line)
                
                time.sleep(0.05)  # 20Hz data collection
                
            except Exception as e:
                print(f"⚠️  Data collection error: {e}")
                time.sleep(1)
    
    def print_status_update(self):
        """Print periodic status updates"""
        while self.running:
            try:
                elapsed = time.time() - self.stats['start_time']
                
                left_dist = f"{self.sensor_data['left_distance']:.1f}" if self.sensor_data['left_distance'] else "NULL"
                right_dist = f"{self.sensor_data['right_distance']:.1f}" if self.sensor_data['right_distance'] else "NULL"
                
                print(f"\n📊 STATUS UPDATE - {datetime.now().strftime('%H:%M:%S')}")
                print(f"   Mode: {self.mode.value.upper()}")
                print(f"   Sensors: L={left_dist}cm R={right_dist}cm")
                print(f"   Emergency: {'YES' if self.sensor_data['emergency'] else 'NO'}")
                print(f"   Runtime: {elapsed:.1f}s | Commands: {self.stats['commands_sent']} | Obstacles: {self.stats['obstacles_avoided']}")
                
                time.sleep(5)  # Status update every 5 seconds
                
            except Exception as e:
                print(f"⚠️  Status update error: {e}")
                time.sleep(5)
    
    def stop_navigation(self):
        """Stop navigation and clean up"""
        print("🛑 Stopping navigation...")
        self.running = False
        
        # Send stop command
        if self.arduino:
            self.send_motor_command(0, 0)
            time.sleep(0.5)  # Give time for command to be sent
            self.arduino.close()
        
        # Print final statistics
        elapsed = time.time() - self.stats['start_time']
        print(f"\n📈 FINAL STATISTICS:")
        print(f"   Total Runtime: {elapsed:.1f} seconds")
        print(f"   Commands Sent: {self.stats['commands_sent']}")
        print(f"   Obstacles Avoided: {self.stats['obstacles_avoided']}")
        print(f"   Mode Changes: {self.stats['mode_changes']}")
        print(f"✅ Navigation stopped cleanly")
    
    def run(self):
        """Main execution function"""
        print("🤖 Navigation Pi - Dual Sensor Autonomous Controller")
        print("=" * 55)
        print("🎯 Focused on navigation performance - no visualization overhead")
        print("🔍 Using dual 30° ultrasonic sensors with directional avoidance")
        
        # Connect to Arduino
        if not self.connect_arduino():
            print("❌ Cannot start navigation without Arduino connection")
            return False
        
        # Start execution
        self.running = True
        
        # Start background threads
        data_thread = threading.Thread(target=self.data_collection_thread, daemon=True)
        data_thread.start()
        
        status_thread = threading.Thread(target=self.print_status_update, daemon=True) 
        status_thread.start()
        
        print("🚀 All systems ready - starting autonomous navigation!")
        print("🛑 Press Ctrl+C to stop\n")
        
        # Run main navigation loop
        try:
            self.navigation_loop()
        except KeyboardInterrupt:
            print("\n🛑 Navigation stopped by user")
        except Exception as e:
            print(f"\n❌ Navigation error: {e}")
        finally:
            self.stop_navigation()
        
        return True

def main():
    navigator = DualSensorNavigator()
    success = navigator.run()
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())