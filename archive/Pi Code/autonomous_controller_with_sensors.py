#!/usr/bin/env python3
"""
Advanced Autonomous Controller with Sensor Integration
Handles intelligent navigation with ultrasonic obstacle avoidance
"""

import serial
import json
import time
import threading
import queue
from datetime import datetime
import statistics
import random

class AdvancedAutonomousController:
    def __init__(self, arduino_port='/dev/ttyUSB0', baud=115200):
        self.arduino_port = arduino_port
        self.baud = baud
        self.arduino = None
        self.running = False
        
        # Robot state
        self.robot_state = {
            'rc_ch1': 0, 'rc_ch2': 0, 'rc_ch9': 0,
            'rc_valid': False, 'mode': 0,
            'emergency_stop': False,
            'last_update': time.time()
        }
        
        # Sensor integration (using Arduino sensor data)
        self.distance_history = []
        self.current_distance = None
        self.arduino_distance = None
        
        # Navigation state machine
        self.nav_state = 'STRAIGHT'
        self.nav_state_start = time.time()
        self.last_direction_change = time.time()
        
        # Navigation parameters
        self.cruise_speed = 180
        self.slow_speed = 80
        self.turn_speed = 140
        
        # Obstacle detection thresholds (more conservative)
        self.slow_down_distance = 35.0
        self.confirm_distance = 20.0
        self.critical_distance = 12.0
        
        # Timing parameters (longer for confidence)
        self.min_straight_time = 6.0    # Minimum 6 seconds straight movement
        self.confirm_time = 2.0         # 2 seconds to confirm obstacle
        self.turn_time = 3.0            # Turn duration
        self.explore_interval = (30, 60) # Random exploration every 30-60 seconds
        
        # Enhanced confidence tracking
        self.obstacle_detections = 0
        self.required_detections = 5    # Need 5 consecutive detections
        self.clear_detections = 0       # Count of "clear" readings
        self.required_clear = 3         # Need 3 clear readings to resume
        
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
    
    def start(self):
        """Start the advanced autonomous controller"""
        if not self.connect_arduino():
            return False
        
        self.running = True
        
        # Start threads
        self.arduino_thread = threading.Thread(target=self.arduino_communication_loop, daemon=True)
        self.sensor_thread = threading.Thread(target=self.sensor_reading_loop, daemon=True)
        self.navigation_thread = threading.Thread(target=self.navigation_loop, daemon=True)
        
        self.arduino_thread.start()
        self.sensor_thread.start()
        self.navigation_thread.start()
        
        print("🚀 Advanced autonomous controller started")
        print("🎮 Switch RC to AUTONOMOUS mode for intelligent navigation")
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
                        
                        # Extract ultrasonic data if available
                        if 'distance' in arduino_data:
                            self.arduino_distance = arduino_data['distance']
                            if self.arduino_distance is not None:
                                self.current_distance = self.arduino_distance
                                self.update_distance_history(self.arduino_distance)
                                self.update_obstacle_confidence(self.arduino_distance)
                        
                        # Debug: Print raw Arduino data occasionally
                        if hasattr(self, '_debug_counter'):
                            self._debug_counter += 1
                        else:
                            self._debug_counter = 0
                        
                        if self._debug_counter % 10 == 0:  # Every 10th message
                            print(f"🔍 Arduino data: {line}")
            except Exception as e:
                print(f"❌ Arduino error: {e}")
                time.sleep(0.1)
            time.sleep(0.01)
    
    def sensor_reading_loop(self):
        """Monitor sensor data from Arduino"""
        print("📡 Using Arduino ultrasonic sensor data")
        while self.running:
            try:
                # Sensor data is now received via Arduino communication
                # This thread monitors for missing sensor updates
                if self.current_distance is None:
                    time.sleep(1)
                    continue
                    
                time.sleep(0.1)
                
            except Exception as e:
                print(f"❌ Sensor monitoring error: {e}")
                time.sleep(1)
    
    def update_distance_history(self, distance):
        """Maintain history of distance readings with validation"""
        # Only accept reasonable distance readings
        if distance is not None and 2 < distance < 400:
            self.distance_history.append(distance)
            if len(self.distance_history) > 15:  # Keep more history for better filtering
                self.distance_history.pop(0)
    
    def update_obstacle_confidence(self, distance):
        """Enhanced obstacle confidence with hysteresis"""
        if distance is None or distance <= 0:
            return
            
        if distance < self.confirm_distance:
            self.obstacle_detections += 1
            self.clear_detections = 0  # Reset clear counter
        else:
            self.clear_detections += 1
            if self.clear_detections >= self.required_clear:
                self.obstacle_detections = max(0, self.obstacle_detections - 2)  # Faster clear
        
        # Cap the detection counter
        self.obstacle_detections = min(self.obstacle_detections, self.required_detections + 2)
    
    def get_confident_distance(self):
        """Get heavily filtered distance reading"""
        if len(self.distance_history) >= 7:
            # Use median of last 7 readings for very stable output
            return statistics.median(self.distance_history[-7:])
        elif len(self.distance_history) >= 3:
            return statistics.median(self.distance_history[-3:])
        return self.current_distance
    
    def obstacle_confirmed(self):
        """Check if obstacle is confirmed"""
        return self.obstacle_detections >= self.required_detections
    
    def navigation_loop(self):
        """Intelligent navigation with sensor integration"""
        print("🧭 Advanced navigation started")
        
        while self.running:
            try:
                # More robust autonomous mode detection
                mode = self.robot_state.get('mode')
                rc_valid = self.robot_state.get('rc_valid')
                emergency = self.robot_state.get('emergency_stop')
                
                # More aggressive autonomous mode detection
                if mode == 2 and not emergency:
                    # Run autonomous regardless of RC valid flag
                    self.last_good_autonomous = time.time()
                    self.intelligent_navigation()
                    if not rc_valid:
                        print("🟡 Running autonomous with invalid RC flag")
                elif hasattr(self, 'last_good_autonomous') and (time.time() - self.last_good_autonomous < 2.0):
                    # Recently was autonomous, continue navigation for longer
                    self.intelligent_navigation()
                    print("🟠 Continuing autonomous despite mode change")
                else:
                    # Not in autonomous mode
                    if mode == 2:
                        print(f"⚠️ Mode 2 detected but emergency stop: {emergency}")
                    pass
                
                time.sleep(0.1)  # 10Hz navigation updates
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                time.sleep(1)
    
    def intelligent_navigation(self):
        """Advanced navigation state machine"""
        current_time = time.time()
        time_in_state = current_time - self.nav_state_start
        distance = self.get_confident_distance()
        
        if self.nav_state == 'STRAIGHT':
            # Confident straight movement - ignore sensors during minimum time
            self.send_motor_command(self.cruise_speed, self.cruise_speed)
            
            # STRICT minimum straight time - ignore ALL sensor readings
            if time_in_state < self.min_straight_time:
                # Reset confidence counters during minimum straight time
                self.obstacle_detections = 0
                self.clear_detections = 0
                return
            
            # Only react to CONFIRMED obstacles after minimum time
            if distance and distance < self.slow_down_distance and self.obstacle_confirmed():
                self.change_nav_state('SLOWING')
                print(f"🟡 CONFIRMED obstacle at {distance:.1f}cm - slowing down")
                return
            
            # Random exploration (much less frequent)
            if current_time - self.last_direction_change > self.get_random_interval():
                self.change_nav_state('EXPLORING')
                self.last_direction_change = current_time
                print("🔄 Random exploration turn")
        
        elif self.nav_state == 'SLOWING':
            # Approach obstacle cautiously
            self.send_motor_command(self.slow_speed, self.slow_speed)
            
            # Need clear confirmation to resume - not just one reading
            if not distance or (distance > self.slow_down_distance and self.clear_detections >= self.required_clear):
                self.change_nav_state('STRAIGHT')
                print("✅ Obstacle CONFIRMED cleared - resuming cruise speed")
                return
            
            # If we get close AND obstacle is confirmed, stop to confirm
            if distance < self.confirm_distance and self.obstacle_confirmed():
                self.change_nav_state('CONFIRMING')
                print(f"🟠 Close to CONFIRMED obstacle ({distance:.1f}cm) - confirming")
        
        elif self.nav_state == 'CONFIRMING':
            # Stop and confirm obstacle
            self.send_motor_command(0, 0)
            
            if time_in_state > self.confirm_time:
                if self.obstacle_confirmed() and distance and distance < self.confirm_distance:
                    self.change_nav_state('AVOIDING')
                    print("🔴 Obstacle confirmed - avoiding")
                else:
                    self.change_nav_state('STRAIGHT')
                    print("✅ False alarm - resuming straight movement")
        
        elif self.nav_state == 'AVOIDING':
            # Turn to avoid obstacle
            if not hasattr(self, 'avoid_direction'):
                self.avoid_direction = random.choice(['left', 'right'])
                print(f"↩️ Avoiding obstacle - turning {self.avoid_direction}")
            
            if self.avoid_direction == 'left':
                self.send_motor_command(-self.turn_speed, self.turn_speed)
            else:
                self.send_motor_command(self.turn_speed, -self.turn_speed)
            
            if time_in_state > self.turn_time:
                self.change_nav_state('STRAIGHT')
                delattr(self, 'avoid_direction')
                self.obstacle_detections = 0  # Reset confidence
                print("➡️ Avoidance complete - resuming straight movement")
        
        elif self.nav_state == 'EXPLORING':
            # Random exploration turn
            if not hasattr(self, 'explore_direction'):
                self.explore_direction = random.choice(['left', 'right'])
                print(f"🔍 Exploring - turning {self.explore_direction}")
            
            if self.explore_direction == 'left':
                self.send_motor_command(-self.turn_speed, self.turn_speed)
            else:
                self.send_motor_command(self.turn_speed, -self.turn_speed)
            
            if time_in_state > self.turn_time:
                self.change_nav_state('STRAIGHT')
                delattr(self, 'explore_direction')
                print("➡️ Exploration complete - resuming straight movement")
    
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
        
        # Debug mode detection
        raw_mode = self.robot_state.get('mode')
        print(f"🔍 Debug - Raw mode: {raw_mode}, RC valid: {self.robot_state.get('rc_valid')}")
        
        return {
            'mode': mode_names.get(raw_mode, f'UNKNOWN({raw_mode})'),
            'nav_state': self.nav_state,
            'distance': self.current_distance,
            'obstacle_confidence': self.obstacle_detections,
            'rc_valid': self.robot_state.get('rc_valid'),
            'emergency_stop': self.robot_state.get('emergency_stop'),
            'time_in_state': time.time() - self.nav_state_start
        }
    
    def disconnect(self):
        """Clean shutdown"""
        self.running = False
        if self.arduino:
            self.arduino.close()
        print("🔌 Arduino sensor connection closed")

class UltrasonicSensor:
    """Pi-based ultrasonic sensor"""
    
    def __init__(self, trig_pin=18, echo_pin=24):
        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
            self.trig_pin = trig_pin
            self.echo_pin = echo_pin
            
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(trig_pin, GPIO.OUT)
            GPIO.setup(echo_pin, GPIO.IN)
            
            self.available = True
            print(f"📡 Pi ultrasonic sensor on GPIO {trig_pin}/{echo_pin}")
            
        except ImportError:
            print("⚠️ RPi.GPIO not available - sensor disabled")
            self.available = False
        except Exception as e:
            print(f"⚠️ Sensor setup failed: {e}")
            self.available = False
    
    def read_distance(self):
        """Read distance with timeout protection"""
        if not self.available:
            return None
        
        try:
            # Send trigger pulse
            self.GPIO.output(self.trig_pin, False)
            time.sleep(0.000002)
            self.GPIO.output(self.trig_pin, True)
            time.sleep(0.00001)
            self.GPIO.output(self.trig_pin, False)
            
            # Measure echo with timeout
            timeout = time.time() + 0.02  # 20ms timeout
            
            pulse_start = time.time()
            while self.GPIO.input(self.echo_pin) == 0:
                pulse_start = time.time()
                if time.time() > timeout:
                    return None
            
            pulse_end = time.time()
            while self.GPIO.input(self.echo_pin) == 1:
                pulse_end = time.time()
                if time.time() > timeout:
                    return None
            
            # Calculate distance
            duration = pulse_end - pulse_start
            distance = duration * 17150
            
            return distance if 2 < distance < 400 else None
            
        except Exception:
            return None
    
    def cleanup(self):
        """Cleanup GPIO"""
        if self.available:
            try:
                self.GPIO.cleanup()
            except:
                pass

def main():
    print("=" * 60)
    print("🧠 ADVANCED AUTONOMOUS CONTROLLER")
    print("=" * 60)
    print()
    print("Features:")
    print("📊 Pi: Intelligent navigation with sensor fusion")
    print("🔧 Arduino: Real-time motor control bridge")
    print("📡 Ultrasonic: Pi-based obstacle detection")
    print("🤖 Smart: Confident movement with obstacle avoidance")
    print()
    
    controller = AdvancedAutonomousController()
    
    if not controller.start():
        print("❌ Failed to start controller")
        return
    
    try:
        print("🔄 Advanced controller running.")
        print("🎮 Switch to AUTONOMOUS mode for intelligent navigation.")
        print("📊 Detailed status every 3 seconds. Press Ctrl+C to stop.")
        print()
        
        while True:
            time.sleep(3)
            status = controller.get_status()
            timestamp = datetime.now().strftime("%H:%M:%S")
            
            distance_str = f"{status['distance']:.1f}cm" if status['distance'] else "N/A"
            confidence_str = f"{status['obstacle_confidence']}/{controller.required_detections}"
            
            print(f"[{timestamp}] {status['mode']:<12} | "
                  f"{status['nav_state']:<12} | "
                  f"Dist: {distance_str:<8} | "
                  f"Conf: {confidence_str:<5} | "
                  f"RC: {'✅' if status['rc_valid'] else '❌'} | "
                  f"Emerg: {'🛑' if status['emergency_stop'] else '✅'}")
    
    except KeyboardInterrupt:
        print("\n⏹️ Stopping advanced controller...")
    
    finally:
        controller.disconnect()
        print("🏁 Advanced controller stopped")

if __name__ == "__main__":
    main()