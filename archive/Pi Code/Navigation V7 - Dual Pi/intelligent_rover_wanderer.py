#!/usr/bin/env python3
"""
Intelligent Rover Wanderer - Advanced Object Avoidance
Runs on Navigation Pi (192.168.254.65)

Features:
- Smart wandering behavior with curiosity-driven exploration
- Advanced object avoidance using dual 30° ultrasonic sensors
- Intelligent steering to navigate around obstacles
- Path memory to avoid getting stuck in corners
- Adaptive behavior based on environment complexity
- Dynamic speed adjustment based on obstacle proximity
"""

import serial
import json
import time
import threading
import signal
import sys
import math
import random
from datetime import datetime
from enum import Enum
from collections import deque

# Configuration
ARDUINO_PORT = '/dev/ttyUSB0'
ARDUINO_BAUD = 115200

class WanderMode(Enum):
    EXPLORING = "exploring"           # Normal wandering
    APPROACHING_OBSTACLE = "approaching"  # Object detected, slowing down
    STEERING_LEFT = "steering_left"   # Actively steering left around obstacle
    STEERING_RIGHT = "steering_right" # Actively steering right around obstacle
    BACKING_UP = "backing_up"        # Backing away from obstacle
    CIRCLING_OBJECT = "circling"     # Circling around interesting object
    ESCAPING_CORNER = "escaping"     # Getting unstuck from corners
    CURIOSITY_SEEKING = "seeking"    # Seeking new areas to explore

# Speed profiles for different behaviors
SPEED_PROFILES = {
    WanderMode.EXPLORING: {'base': 100, 'variation': 30},      # Normal wandering
    WanderMode.APPROACHING_OBSTACLE: {'base': 60, 'variation': 20},  # Cautious approach
    WanderMode.STEERING_LEFT: {'base': 80, 'variation': 15},   # Steering maneuver
    WanderMode.STEERING_RIGHT: {'base': 80, 'variation': 15},  # Steering maneuver
    WanderMode.BACKING_UP: {'base': -70, 'variation': 20},     # Reverse movement
    WanderMode.CIRCLING_OBJECT: {'base': 90, 'variation': 25}, # Object investigation
    WanderMode.ESCAPING_CORNER: {'base': 120, 'variation': 30}, # Escape behavior
    WanderMode.CURIOSITY_SEEKING: {'base': 110, 'variation': 40} # Exploration drive
}

# Behavior parameters
OBSTACLE_ZONES = {
    'immediate': 15.0,    # Emergency zone - immediate action required
    'close': 30.0,        # Close zone - start steering around
    'approach': 50.0,     # Approach zone - slow down and evaluate
    'distant': 100.0      # Distant zone - note but continue
}

class IntelligentWanderer:
    def __init__(self):
        self.arduino = None
        self.running = False
        
        # Core navigation state
        self.current_mode = WanderMode.EXPLORING
        self.mode_start_time = time.time()
        self.last_mode_change = time.time()
        
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
        
        # Intelligent behavior state
        self.behavior_state = {
            'obstacle_history': deque(maxlen=20),  # Recent obstacle encounters
            'stuck_counter': 0,                   # Counter for stuck detection
            'last_significant_turn': time.time(), # Anti-stuck mechanism
            'preferred_direction': 0,             # Learned directional preference
            'exploration_bias': random.choice([-1, 1]),  # Random exploration bias
            'curiosity_level': 0.5,               # Curiosity drive (0-1)
            'object_following': None,             # Object being investigated
            'path_memory': deque(maxlen=50),      # Recent path for avoiding loops
        }
        
        # Performance tracking
        self.stats = {
            'start_time': time.time(),
            'obstacles_navigated': 0,
            'mode_switches': 0,
            'distance_traveled': 0.0,
            'interesting_objects': 0,
            'stuck_escapes': 0
        }
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        print(f"\n🛑 Received signal {signum}, stopping wanderer...")
        self.stop_wandering()
        sys.exit(0)
    
    def connect_arduino(self):
        """Connect to Arduino for sensor data and motor control"""
        try:
            print(f"🔌 Connecting to Arduino on {ARDUINO_PORT}...")
            self.arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
            time.sleep(2)
            print("✅ Arduino connected successfully!")
            
            # Send initial stop command
            self.send_motor_command(0, 0)
            return True
            
        except Exception as e:
            print(f"❌ Arduino connection failed: {e}")
            return False
    
    def parse_sensor_data(self, data_str):
        """Parse and analyze sensor telemetry"""
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
            
            # Track obstacle encounters for learning
            if self.sensor_data['emergency'] and not old_emergency:
                self.record_obstacle_encounter()
            
            # Update path memory
            self.update_path_memory()
            
            return True
            
        except Exception as e:
            return False
    
    def record_obstacle_encounter(self):
        """Record obstacle encounter for behavioral learning"""
        encounter = {
            'timestamp': time.time(),
            'direction': self.sensor_data['obstacle_direction'],
            'left_distance': self.sensor_data['left_distance'],
            'right_distance': self.sensor_data['right_distance'],
            'mode': self.current_mode
        }
        
        self.behavior_state['obstacle_history'].append(encounter)
        self.stats['obstacles_navigated'] += 1
        
        # Analyze for interesting objects (consistent readings)
        if len(self.behavior_state['obstacle_history']) >= 3:
            recent = list(self.behavior_state['obstacle_history'])[-3:]
            if self.is_interesting_object(recent):
                self.stats['interesting_objects'] += 1
                self.behavior_state['object_following'] = encounter
                print(f"🎯 Interesting object detected! Engaging curiosity mode...")
    
    def is_interesting_object(self, recent_encounters):
        """Determine if recent encounters indicate an interesting object"""
        if len(recent_encounters) < 3:
            return False
            
        # Check for consistent obstacle in similar position
        directions = [enc['direction'] for enc in recent_encounters]
        if len(set(directions)) == 1 and directions[0] != 0:  # Same side, not center
            # Check if distances are relatively stable (not moving obstacle)
            distances = [enc.get('left_distance') or enc.get('right_distance') for enc in recent_encounters]
            distances = [d for d in distances if d is not None]
            if len(distances) >= 2:
                variation = max(distances) - min(distances)
                return variation < 10.0  # Less than 10cm variation = stationary object
        
        return False
    
    def update_path_memory(self):
        """Update path memory to avoid loops"""
        if self.sensor_data['min_distance']:
            position_estimate = {
                'timestamp': time.time(),
                'left_distance': self.sensor_data['left_distance'],
                'right_distance': self.sensor_data['right_distance'],
                'mode': self.current_mode
            }
            self.behavior_state['path_memory'].append(position_estimate)
    
    def send_motor_command(self, left_speed, right_speed):
        """Send motor command with speed tracking"""
        if not self.arduino:
            return False
            
        try:
            command = {"left": left_speed, "right": right_speed}
            command_str = json.dumps(command) + '\n'
            
            self.arduino.write(command_str.encode('utf-8'))
            
            # Estimate distance traveled for stats
            avg_speed = abs(left_speed + right_speed) / 2
            time_delta = 0.1  # Approximate loop time
            distance_increment = (avg_speed / 255.0) * 0.5 * time_delta  # Rough estimate
            self.stats['distance_traveled'] += distance_increment
            
            return True
            
        except Exception as e:
            print(f"⚠️  Failed to send motor command: {e}")
            return False
    
    def analyze_environment(self):
        """Analyze current environment and choose intelligent behavior"""
        if not self.sensor_data['last_update']:
            return WanderMode.EXPLORING
        
        # Check sensor data freshness
        if time.time() - self.sensor_data['last_update'] > 2.0:
            print("⚠️  Sensor timeout - exploring cautiously")
            return WanderMode.EXPLORING
        
        left_dist = self.sensor_data['left_distance'] or 999
        right_dist = self.sensor_data['right_distance'] or 999
        min_dist = min(left_dist, right_dist)
        
        # Immediate danger - back up
        if min_dist < OBSTACLE_ZONES['immediate']:
            return WanderMode.BACKING_UP
        
        # Close obstacle - intelligent steering
        elif min_dist < OBSTACLE_ZONES['close']:
            return self.choose_steering_direction()
        
        # Approaching obstacle - slow down and evaluate
        elif min_dist < OBSTACLE_ZONES['approach']:
            # Check if this might be an interesting object to investigate
            if self.behavior_state['curiosity_level'] > 0.6 and self.should_investigate():
                return WanderMode.CIRCLING_OBJECT
            else:
                return WanderMode.APPROACHING_OBSTACLE
        
        # Check if we're stuck in a pattern
        elif self.detect_stuck_pattern():
            return WanderMode.ESCAPING_CORNER
        
        # Check for curiosity-driven exploration
        elif self.should_seek_new_areas():
            return WanderMode.CURIOSITY_SEEKING
        
        # Default exploring behavior
        else:
            return WanderMode.EXPLORING
    
    def choose_steering_direction(self):
        """Intelligently choose which direction to steer"""
        left_dist = self.sensor_data['left_distance'] or 0
        right_dist = self.sensor_data['right_distance'] or 0
        
        # Basic logic: steer toward the side with more space
        if left_dist > right_dist + 10:  # Left has significantly more space
            self.behavior_state['preferred_direction'] = 1  # Remember preference
            return WanderMode.STEERING_LEFT
        elif right_dist > left_dist + 10:  # Right has significantly more space
            self.behavior_state['preferred_direction'] = -1
            return WanderMode.STEERING_RIGHT
        else:
            # Similar distances - use learned preference or exploration bias
            if self.behavior_state['preferred_direction'] != 0:
                return (WanderMode.STEERING_LEFT if self.behavior_state['preferred_direction'] > 0 
                       else WanderMode.STEERING_RIGHT)
            else:
                return (WanderMode.STEERING_LEFT if self.behavior_state['exploration_bias'] > 0 
                       else WanderMode.STEERING_RIGHT)
    
    def should_investigate(self):
        """Determine if rover should investigate an object"""
        # Higher curiosity = more likely to investigate
        curiosity_threshold = 0.7
        
        # Don't investigate if we just investigated something recently
        if self.behavior_state['object_following']:
            time_since = time.time() - self.behavior_state['object_following']['timestamp']
            if time_since < 10.0:  # Wait 10 seconds between investigations
                return False
        
        # Random curiosity factor
        curiosity_roll = random.random()
        return curiosity_roll < self.behavior_state['curiosity_level']
    
    def detect_stuck_pattern(self):
        """Detect if rover is stuck in repetitive behavior"""
        mode_duration = time.time() - self.mode_start_time
        
        # Check for rapid mode switching (indicates confusion)
        recent_switches = time.time() - self.last_mode_change
        if recent_switches < 2.0:
            self.behavior_state['stuck_counter'] += 1
        else:
            self.behavior_state['stuck_counter'] = max(0, self.behavior_state['stuck_counter'] - 1)
        
        # Stuck if too many rapid switches
        if self.behavior_state['stuck_counter'] > 5:
            print("🔄 Stuck pattern detected - engaging escape behavior")
            self.stats['stuck_escapes'] += 1
            return True
        
        # Also check for being in same mode too long
        if mode_duration > 10.0 and self.current_mode in [WanderMode.STEERING_LEFT, WanderMode.STEERING_RIGHT]:
            return True
        
        return False
    
    def should_seek_new_areas(self):
        """Determine if rover should actively seek new exploration areas"""
        # Increase curiosity over time
        runtime = time.time() - self.stats['start_time']
        base_curiosity = 0.3 + min(0.4, runtime / 300.0)  # Increase over 5 minutes
        
        # Random exploration urges
        exploration_roll = random.random()
        return exploration_roll < base_curiosity * self.behavior_state['curiosity_level']
    
    def execute_behavior(self):
        """Execute the current wandering behavior"""
        mode_duration = time.time() - self.mode_start_time
        profile = SPEED_PROFILES[self.current_mode]
        base_speed = profile['base']
        variation = profile['variation']
        
        if self.current_mode == WanderMode.EXPLORING:
            # Normal wandering with slight random variations
            speed_var = random.randint(-variation, variation)
            left_speed = base_speed + speed_var + random.randint(-10, 10)
            right_speed = base_speed + speed_var + random.randint(-10, 10)
            
            # Add exploration bias for more interesting movement
            bias_factor = self.behavior_state['exploration_bias'] * 20
            left_speed += bias_factor
            right_speed -= bias_factor
            
        elif self.current_mode == WanderMode.APPROACHING_OBSTACLE:
            # Slow, cautious approach
            distance_factor = min(1.0, (self.sensor_data['min_distance'] or 50) / 50.0)
            adjusted_speed = int(base_speed * distance_factor)
            left_speed = right_speed = adjusted_speed
            
        elif self.current_mode == WanderMode.STEERING_LEFT:
            # Intelligent left steering
            turn_intensity = self.calculate_turn_intensity()
            left_speed = base_speed - turn_intensity
            right_speed = base_speed + turn_intensity
            
        elif self.current_mode == WanderMode.STEERING_RIGHT:
            # Intelligent right steering  
            turn_intensity = self.calculate_turn_intensity()
            left_speed = base_speed + turn_intensity
            right_speed = base_speed - turn_intensity
            
        elif self.current_mode == WanderMode.BACKING_UP:
            # Back up with slight turning to avoid backing into same obstacle
            turn_bias = self.behavior_state['exploration_bias'] * 30
            left_speed = base_speed - turn_bias
            right_speed = base_speed + turn_bias
            
        elif self.current_mode == WanderMode.CIRCLING_OBJECT:
            # Circle around interesting object
            if self.behavior_state['object_following']:
                direction = self.behavior_state['object_following']['direction']
                if direction == -1:  # Object on left, circle right
                    left_speed = base_speed + 40
                    right_speed = base_speed - 20
                elif direction == 1:  # Object on right, circle left
                    left_speed = base_speed - 20
                    right_speed = base_speed + 40
                else:  # Center object, random circle
                    circle_dir = random.choice([-1, 1])
                    left_speed = base_speed + (circle_dir * 30)
                    right_speed = base_speed - (circle_dir * 30)
            else:
                left_speed = right_speed = base_speed
                
        elif self.current_mode == WanderMode.ESCAPING_CORNER:
            # Aggressive escape maneuver
            escape_pattern = int(mode_duration * 2) % 4
            if escape_pattern == 0:  # Back up
                left_speed = right_speed = -120
            elif escape_pattern == 1:  # Sharp turn
                left_speed, right_speed = 150, -150
            elif escape_pattern == 2:  # Forward burst
                left_speed = right_speed = 150
            else:  # Opposite turn
                left_speed, right_speed = -150, 150
                
        elif self.current_mode == WanderMode.CURIOSITY_SEEKING:
            # Seeking new areas - faster movement with random direction changes
            if int(mode_duration * 3) % 6 < 4:  # 2/3 time going forward
                direction_change = random.randint(-variation, variation)
                left_speed = base_speed - direction_change
                right_speed = base_speed + direction_change
            else:  # 1/3 time turning to find new direction
                turn_dir = random.choice([-1, 1])
                left_speed = base_speed * turn_dir
                right_speed = -base_speed * turn_dir
        
        else:
            # Fallback
            left_speed = right_speed = 60
        
        # Apply safety limits and send command
        left_speed = max(-255, min(255, left_speed))
        right_speed = max(-255, min(255, right_speed))
        self.send_motor_command(left_speed, right_speed)
    
    def calculate_turn_intensity(self):
        """Calculate how aggressively to turn based on obstacle proximity"""
        min_dist = self.sensor_data['min_distance'] or 100
        
        if min_dist < 15:
            return 60  # Sharp turn
        elif min_dist < 25:
            return 40  # Medium turn
        else:
            return 25  # Gentle turn
    
    def change_mode(self, new_mode):
        """Change behavior mode with intelligent transitions"""
        if new_mode != self.current_mode:
            old_mode = self.current_mode
            self.current_mode = new_mode
            self.last_mode_change = self.mode_start_time
            self.mode_start_time = time.time()
            self.stats['mode_switches'] += 1
            
            # Update behavioral learning
            self.update_behavioral_learning(old_mode, new_mode)
            
            print(f"🧠 Behavior change: {old_mode.value} → {new_mode.value}")
    
    def update_behavioral_learning(self, old_mode, new_mode):
        """Update behavioral parameters based on mode transitions"""
        # Adjust curiosity based on successful obstacle navigation
        if old_mode in [WanderMode.STEERING_LEFT, WanderMode.STEERING_RIGHT] and new_mode == WanderMode.EXPLORING:
            self.behavior_state['curiosity_level'] = min(1.0, self.behavior_state['curiosity_level'] + 0.05)
        
        # Reduce curiosity if getting stuck frequently  
        if new_mode == WanderMode.ESCAPING_CORNER:
            self.behavior_state['curiosity_level'] = max(0.2, self.behavior_state['curiosity_level'] - 0.1)
            self.behavior_state['stuck_counter'] = 0
        
        # Reset object following after investigation
        if old_mode == WanderMode.CIRCLING_OBJECT:
            self.behavior_state['object_following'] = None
    
    def check_mode_transitions(self):
        """Check if current mode should transition"""
        mode_duration = time.time() - self.mode_start_time
        
        # Automatic transitions based on time and conditions
        if self.current_mode == WanderMode.BACKING_UP and mode_duration > 2.0:
            # After backing up, choose direction intelligently
            if self.sensor_data['min_distance'] and self.sensor_data['min_distance'] > OBSTACLE_ZONES['close']:
                self.change_mode(WanderMode.EXPLORING)
                
        elif self.current_mode == WanderMode.CIRCLING_OBJECT and mode_duration > 8.0:
            # Finished investigating object
            print("🎯 Object investigation complete")
            self.change_mode(WanderMode.EXPLORING)
            
        elif self.current_mode == WanderMode.ESCAPING_CORNER and mode_duration > 6.0:
            # Escape maneuver complete
            print("🔓 Corner escape complete")
            self.change_mode(WanderMode.EXPLORING)
            
        elif self.current_mode == WanderMode.CURIOSITY_SEEKING and mode_duration > 10.0:
            # Exploration phase complete
            self.change_mode(WanderMode.EXPLORING)
    
    def data_collection_thread(self):
        """Background thread for sensor data collection"""
        print("📡 Starting intelligent sensor analysis...")
        
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
    
    def wandering_loop(self):
        """Main intelligent wandering loop"""
        print("🎯 Starting intelligent wandering behavior...")
        
        while self.running:
            try:
                # Analyze environment and choose behavior
                desired_mode = self.analyze_environment()
                
                # Change mode if needed
                if desired_mode != self.current_mode:
                    self.change_mode(desired_mode)
                
                # Execute current behavior
                self.execute_behavior()
                
                # Check for automatic transitions
                self.check_mode_transitions()
                
                # Control loop frequency
                time.sleep(0.1)  # 10Hz behavior loop
                
            except Exception as e:
                print(f"⚠️  Wandering loop error: {e}")
                self.send_motor_command(0, 0)
                time.sleep(1)
    
    def status_update_thread(self):
        """Background thread for status updates"""
        while self.running:
            try:
                runtime = time.time() - self.stats['start_time']
                
                left_dist = f"{self.sensor_data['left_distance']:.1f}" if self.sensor_data['left_distance'] else "NULL"
                right_dist = f"{self.sensor_data['right_distance']:.1f}" if self.sensor_data['right_distance'] else "NULL"
                
                print(f"\n🤖 INTELLIGENT WANDERER STATUS - {datetime.now().strftime('%H:%M:%S')}")
                print(f"   Behavior: {self.current_mode.value.upper()}")
                print(f"   Sensors: L={left_dist}cm R={right_dist}cm")
                print(f"   Curiosity: {self.behavior_state['curiosity_level']:.2f} | Bias: {self.behavior_state['exploration_bias']:+d}")
                print(f"   Stats: {runtime:.0f}s | Obstacles: {self.stats['obstacles_navigated']} | Objects: {self.stats['interesting_objects']} | Escapes: {self.stats['stuck_escapes']}")
                
                time.sleep(5)  # Status every 5 seconds
                
            except Exception as e:
                print(f"⚠️  Status update error: {e}")
                time.sleep(5)
    
    def stop_wandering(self):
        """Stop wandering and clean up"""
        print("🛑 Stopping intelligent wanderer...")
        self.running = False
        
        if self.arduino:
            self.send_motor_command(0, 0)
            time.sleep(0.5)
            self.arduino.close()
        
        # Final statistics
        runtime = time.time() - self.stats['start_time']
        print(f"\n🧠 WANDERING SESSION COMPLETE:")
        print(f"   Duration: {runtime:.1f} seconds")
        print(f"   Obstacles Navigated: {self.stats['obstacles_navigated']}")
        print(f"   Interesting Objects Found: {self.stats['interesting_objects']}")
        print(f"   Behavior Switches: {self.stats['mode_switches']}")
        print(f"   Stuck Escapes: {self.stats['stuck_escapes']}")
        print(f"   Final Curiosity Level: {self.behavior_state['curiosity_level']:.2f}")
        print("✅ Wanderer stopped cleanly")
    
    def run(self):
        """Main execution function"""
        print("🤖 Intelligent Rover Wanderer - Advanced Object Avoidance")
        print("=" * 60)
        print("🧠 Features: Smart steering, curiosity-driven exploration, stuck detection")
        print("🎯 Using dual 30° ultrasonic sensors for environmental analysis")
        
        if not self.connect_arduino():
            print("❌ Cannot start without Arduino connection")
            return False
        
        self.running = True
        
        # Start background threads
        data_thread = threading.Thread(target=self.data_collection_thread, daemon=True)
        data_thread.start()
        
        status_thread = threading.Thread(target=self.status_update_thread, daemon=True)
        status_thread.start()
        
        print("🚀 Intelligent wandering activated!")
        print("🧭 Watch as the rover learns and adapts to its environment")
        print("🛑 Press Ctrl+C to stop\n")
        
        try:
            self.wandering_loop()
        except KeyboardInterrupt:
            print("\n🛑 Wandering stopped by user")
        except Exception as e:
            print(f"\n❌ Wandering error: {e}")
        finally:
            self.stop_wandering()
        
        return True

def main():
    wanderer = IntelligentWanderer()
    success = wanderer.run()
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())