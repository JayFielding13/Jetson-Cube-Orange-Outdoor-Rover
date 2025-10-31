#!/usr/bin/env python3
"""
Autonomous Ultrasonic Navigator
Stable rover navigation using ultrasonic sensor only
Part of the Mini Rover Development Project
"""

import serial
import json
import time
import threading
import math
import random

class UltrasonicNavigator:
    """
    Autonomous rover navigator using ultrasonic sensor for obstacle detection.
    Integrates with Arduino gatekeeper for safety-critical motor control.
    """
    
    def __init__(self, arduino_port='/dev/ttyUSB0', baud=115200):
        self.arduino_port = arduino_port
        self.baud = baud
        self.arduino = None
        self.running = False
        
        # Robot state from Arduino gatekeeper
        self.robot_state = {
            'mode': 0,                    # 0=Manual, 1=Assisted, 2=Autonomous
            'rc_valid': False,            # RC signal validity
            'emergency_stop': False,      # Emergency stop status
            'ultrasonic_distance': 200.0, # Distance in cm
            'last_update': time.time()    # Last data timestamp
        }
        
        # Navigation state machine
        self.nav_state = 'STRAIGHT'
        self.nav_start_time = time.time()
        self.turn_direction = 'left'
        self.stuck_counter = 0
        
        # Movement parameters (tunable)
        self.cruise_speed = 100       # Normal forward speed
        self.slow_speed = 60          # Cautious forward speed
        self.turn_speed = 80          # Turning speed
        self.reverse_speed = 70       # Backing up speed
        
        # Navigation thresholds (tunable)
        self.danger_threshold = 30.0   # Emergency stop distance (cm)
        self.caution_threshold = 60.0  # Slow down distance (cm)
        self.safe_distance = 100.0     # Safe following distance (cm)
        self.stuck_threshold = 5       # Consecutive stuck detections
        
        # Navigation timing (tunable)
        self.exploration_interval = 8.0  # Seconds before random exploration
        self.turn_duration = 1.5         # Seconds for avoidance turns
        self.backup_duration = 1.0       # Seconds for backing up
        self.stuck_reset_time = 10.0     # Reset stuck counter interval
        
        # Statistics tracking
        self.stats = {
            'total_runtime': 0,
            'distance_measurements': 0,
            'obstacle_avoidances': 0,
            'exploration_turns': 0,
            'emergency_stops': 0,
            'stuck_recoveries': 0
        }
        
        print("🚀 Ultrasonic Navigator Initialized")
        print(f"📊 Danger: {self.danger_threshold}cm | Caution: {self.caution_threshold}cm | Safe: {self.safe_distance}cm")
    
    def connect_arduino(self):
        """Establish connection to Arduino gatekeeper"""
        try:
            self.arduino = serial.Serial(self.arduino_port, self.baud, timeout=1)
            time.sleep(2)  # Allow Arduino to reset
            print(f"✅ Arduino connected on {self.arduino_port}")
            return True
        except Exception as e:
            print(f"❌ Arduino connection failed: {e}")
            return False
    
    def start(self):
        """Start the autonomous navigator"""
        if not self.connect_arduino():
            return False
        
        self.running = True
        self.stats['start_time'] = time.time()
        
        # Start communication threads
        self.arduino_thread = threading.Thread(target=self.arduino_loop, daemon=True)
        self.navigation_thread = threading.Thread(target=self.navigation_loop, daemon=True)
        
        self.arduino_thread.start()
        self.navigation_thread.start()
        
        print("🚀 Ultrasonic Navigator started")
        print("🎯 Ready for autonomous navigation")
        return True
    
    def arduino_loop(self):
        """Handle continuous Arduino communication"""
        consecutive_failures = 0
        max_failures = 10
        
        while self.running and self.arduino:
            try:
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode().strip()
                    if line and line.startswith('{'):
                        try:
                            data = json.loads(line)
                            self.process_arduino_data(data)
                            consecutive_failures = 0
                        except json.JSONDecodeError as e:
                            print(f"⚠️ JSON decode error: {e}")
                            consecutive_failures += 1
                
                # Check for communication timeout
                if time.time() - self.robot_state['last_update'] > 5.0:
                    print("⚠️ Arduino communication timeout")
                    consecutive_failures += 1
                
                # Handle persistent failures
                if consecutive_failures >= max_failures:
                    print("❌ Too many Arduino communication failures")
                    self.emergency_stop()
                    break
                    
            except Exception as e:
                print(f"❌ Arduino communication error: {e}")
                consecutive_failures += 1
                time.sleep(0.1)
            
            time.sleep(0.02)  # 50Hz communication loop
    
    def process_arduino_data(self, data):
        """Process incoming data from Arduino"""
        # Extract distance with validation
        distance = data.get('distance')
        if distance is None or distance < 0:
            distance = 200.0  # Safe default
        
        # Update robot state
        self.robot_state.update({
            'mode': data.get('mode', 0),
            'rc_valid': data.get('valid', False),
            'emergency_stop': data.get('emergency', False),
            'ultrasonic_distance': float(distance),
            'last_update': time.time()
        })
        
        # Track statistics
        self.stats['distance_measurements'] += 1
        
        # Log emergency conditions
        if self.robot_state['emergency_stop']:
            self.stats['emergency_stops'] += 1
    
    def navigation_loop(self):
        """Main autonomous navigation logic"""
        last_stuck_check = time.time()
        
        while self.running:
            try:
                # Only navigate in autonomous mode
                if self.robot_state.get('mode', 0) != 2:
                    time.sleep(0.5)
                    continue
                
                # Get current distance reading
                distance = self.robot_state.get('ultrasonic_distance', 200.0)
                current_time = time.time()
                
                # Check for stuck condition periodically
                if current_time - last_stuck_check > self.stuck_reset_time:
                    if self.stuck_counter > 0:
                        self.stuck_counter = max(0, self.stuck_counter - 1)
                    last_stuck_check = current_time
                
                # Compute navigation based on current state
                left_speed, right_speed = self.compute_navigation(distance, current_time)
                
                # Send motor commands to Arduino
                self.send_motor_command(left_speed, right_speed)
                
                # Status logging (every 3 seconds)
                if current_time % 3 < 0.5:
                    self.log_status(distance)
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                self.send_motor_command(0, 0)
                time.sleep(1.0)
            
            time.sleep(0.5)  # 2Hz navigation loop for smooth control
    
    def compute_navigation(self, distance, current_time):
        """Compute motor speeds based on ultrasonic distance and navigation state"""
        time_in_state = current_time - self.nav_start_time
        left_speed = 0
        right_speed = 0
        
        # State machine for navigation behavior
        if self.nav_state == 'STRAIGHT':
            if distance < self.danger_threshold:
                # Emergency avoidance
                self.nav_state = 'AVOIDING'
                self.nav_start_time = current_time
                self.turn_direction = self.choose_turn_direction()
                self.stats['obstacle_avoidances'] += 1
                print(f"🚨 EMERGENCY! Obstacle at {distance:.1f}cm - turning {self.turn_direction}")
                
            elif distance < self.caution_threshold:
                # Cautious forward movement
                speed_factor = (distance - self.danger_threshold) / (self.caution_threshold - self.danger_threshold)
                speed = self.slow_speed * max(0.3, speed_factor)
                left_speed = speed
                right_speed = speed
                print(f"⚠️ CAUTION: Obstacle at {distance:.1f}cm - slowing to {speed:.0f}")
                
            elif time_in_state > self.exploration_interval:
                # Periodic exploration
                self.nav_state = 'EXPLORING'
                self.nav_start_time = current_time
                self.stats['exploration_turns'] += 1
                print(f"🔍 EXPLORING: Random turn after {time_in_state:.1f}s")
                
            else:
                # Normal forward movement
                left_speed = self.cruise_speed
                right_speed = self.cruise_speed
        
        elif self.nav_state == 'AVOIDING':
            # Execute avoidance maneuver
            if self.turn_direction == 'left':
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
            else:
                left_speed = self.turn_speed
                right_speed = -self.turn_speed
            
            # Check if avoidance is complete
            if time_in_state > self.turn_duration and distance > self.safe_distance:
                self.nav_state = 'STRAIGHT'
                self.nav_start_time = current_time
                print(f"✅ CLEAR: Path ahead at {distance:.1f}cm - resuming forward")
            elif time_in_state > self.turn_duration * 2:
                # Stuck in avoidance - try backing up
                self.nav_state = 'BACKING'
                self.nav_start_time = current_time
                self.stuck_counter += 1
                print(f"🔄 STUCK: Backing up (count: {self.stuck_counter})")
        
        elif self.nav_state == 'BACKING':
            # Back up to create space
            left_speed = -self.reverse_speed
            right_speed = -self.reverse_speed
            
            if time_in_state > self.backup_duration:
                if self.stuck_counter >= self.stuck_threshold:
                    # Multiple stuck detections - try different strategy
                    self.nav_state = 'STUCK_RECOVERY'
                    self.nav_start_time = current_time
                    self.stats['stuck_recoveries'] += 1
                    print(f"🆘 STUCK RECOVERY: Multiple stuck detections")
                else:
                    # Return to avoidance
                    self.nav_state = 'AVOIDING'
                    self.nav_start_time = current_time
                    self.turn_direction = 'left' if self.turn_direction == 'right' else 'right'
                    print(f"🔄 RETRY: Switching to {self.turn_direction} turn")
        
        elif self.nav_state == 'STUCK_RECOVERY':
            # Aggressive stuck recovery - longer backing and turning
            if time_in_state < self.backup_duration * 2:
                # Extended backing
                left_speed = -self.reverse_speed
                right_speed = -self.reverse_speed
            elif time_in_state < self.backup_duration * 2 + self.turn_duration * 2:
                # Extended turning
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
            else:
                # Reset and try again
                self.nav_state = 'STRAIGHT'
                self.nav_start_time = current_time
                self.stuck_counter = 0
                print("🔄 RECOVERY COMPLETE: Resetting navigation")
        
        elif self.nav_state == 'EXPLORING':
            # Random exploration turn
            left_speed = -self.turn_speed
            right_speed = self.turn_speed
            
            if time_in_state > self.turn_duration:
                self.nav_state = 'STRAIGHT'
                self.nav_start_time = current_time
                print("🔍 EXPLORATION COMPLETE: Resuming forward")
        
        return left_speed, right_speed
    
    def choose_turn_direction(self):
        """Choose turn direction (can be enhanced with additional sensors)"""
        # For now, alternate directions to avoid getting stuck
        if hasattr(self, 'last_turn_direction'):
            return 'right' if self.last_turn_direction == 'left' else 'left'
        else:
            self.last_turn_direction = 'left'
            return 'left'
    
    def send_motor_command(self, left_speed, right_speed):
        """Send motor command to Arduino gatekeeper"""
        if not self.arduino or not self.running:
            return
        
        # Clamp speeds to valid range
        left_speed = max(-255, min(255, int(left_speed)))
        right_speed = max(-255, min(255, int(right_speed)))
        
        # Create command structure
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
            print(f"❌ Motor command failed: {e}")
    
    def emergency_stop(self):
        """Emergency stop - send zero speeds"""
        self.send_motor_command(0, 0)
        print("🛑 EMERGENCY STOP EXECUTED")
    
    def log_status(self, distance):
        """Log current navigation status"""
        runtime = time.time() - self.stats['start_time']
        mode_text = ['MANUAL', 'ASSISTED', 'AUTONOMOUS'][self.robot_state.get('mode', 0)]
        
        print(f"🧭 {mode_text} | {self.nav_state} | Distance: {distance:.1f}cm | Runtime: {runtime:.0f}s")
        
        if self.stuck_counter > 0:
            print(f"   Stuck counter: {self.stuck_counter}/{self.stuck_threshold}")
        
        # Periodic statistics
        if int(runtime) % 30 == 0 and runtime > 0:
            self.print_statistics()
    
    def print_statistics(self):
        """Print navigation statistics"""
        runtime = time.time() - self.stats['start_time']
        
        print("📊 === Navigation Statistics ===")
        print(f"   Runtime: {runtime:.0f} seconds")
        print(f"   Distance measurements: {self.stats['distance_measurements']}")
        print(f"   Obstacle avoidances: {self.stats['obstacle_avoidances']}")
        print(f"   Exploration turns: {self.stats['exploration_turns']}")
        print(f"   Emergency stops: {self.stats['emergency_stops']}")
        print(f"   Stuck recoveries: {self.stats['stuck_recoveries']}")
        
        if runtime > 0:
            print(f"   Avg measurements/sec: {self.stats['distance_measurements']/runtime:.1f}")
    
    def stop(self):
        """Stop the navigator and clean up resources"""
        print("🛑 Stopping Ultrasonic Navigator...")
        self.running = False
        
        # Send stop command
        if self.arduino:
            self.send_motor_command(0, 0)
            time.sleep(0.2)
            
            try:
                self.arduino.close()
            except:
                pass
        
        # Print final statistics
        if 'start_time' in self.stats:
            runtime = time.time() - self.stats['start_time']
            print(f"📊 Final runtime: {runtime:.1f} seconds")
            self.print_statistics()
        
        print("👋 Navigator shutdown complete")

def main():
    """Main entry point for the autonomous navigator"""
    print("=" * 60)
    print("🤖 AUTONOMOUS ULTRASONIC NAVIGATOR")
    print("=" * 60)
    print()
    print("Features:")
    print("  - Ultrasonic obstacle detection and avoidance")
    print("  - Arduino gatekeeper integration")
    print("  - State-based navigation with stuck recovery")
    print("  - Comprehensive error handling and statistics")
    print("  - Safe, reliable operation")
    print()
    
    navigator = UltrasonicNavigator()
    
    try:
        if navigator.start():
            print("✅ Navigator ready - switch to autonomous mode")
            print("Press Ctrl+C to stop...")
            
            while True:
                time.sleep(1)
        else:
            print("❌ Failed to start navigator")
    
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested by user...")
    
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
    
    finally:
        navigator.stop()

if __name__ == "__main__":
    main()