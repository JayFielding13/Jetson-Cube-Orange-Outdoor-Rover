#!/usr/bin/env python3
"""
Autonomous Controller for Raspberry Pi
Handles high-level autonomous navigation logic
Communicates with Arduino bridge for motor control
"""

import serial
import json
import time
import threading
import queue
from datetime import datetime
import statistics

class AutonomousController:
    def __init__(self, arduino_port='/dev/ttyUSB0', baud=115200):
        self.arduino_port = arduino_port
        self.baud = baud
        self.arduino = None
        self.running = False
        
        # Robot state
        self.robot_state = {
            'rc_ch1': 0,
            'rc_ch2': 0,
            'rc_ch9': 0,
            'rc_valid': False,
            'mode': 0,  # 0=Failsafe, 1=Manual, 2=Autonomous
            'emergency_stop': False,
            'last_update': time.time()
        }
        
        # Autonomous navigation state
        self.nav_state = 'EXPLORING'  # EXPLORING, AVOIDING, TURNING
        self.nav_state_start = time.time()
        self.last_direction_change = time.time()
        
        # Movement parameters
        self.cruise_speed = 180
        self.turn_speed = 140
        self.explore_interval = (15, 30)  # Random exploration every 15-30 seconds
        
        # Communication queues
        self.data_queue = queue.Queue()
        
        # Navigation timers
        self.min_straight_time = 5.0  # Minimum 5 seconds straight
        self.turn_duration = 3.0      # 3 second turns
        
    def connect_arduino(self):
        """Connect to Arduino bridge"""
        try:
            self.arduino = serial.Serial(self.arduino_port, self.baud, timeout=1)
            time.sleep(2)  # Wait for Arduino initialization
            print(f"✅ Connected to Arduino bridge on {self.arduino_port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to Arduino: {e}")
            return False
    
    def disconnect_arduino(self):
        """Disconnect from Arduino"""
        self.running = False
        if self.arduino:
            self.arduino.close()
            print("📱 Disconnected from Arduino")
    
    def start(self):
        """Start the autonomous controller"""
        if not self.connect_arduino():
            return False
        
        self.running = True
        
        # Start background threads
        self.arduino_thread = threading.Thread(target=self.arduino_communication_loop, daemon=True)
        self.arduino_thread.start()
        
        self.navigation_thread = threading.Thread(target=self.navigation_loop, daemon=True)
        self.navigation_thread.start()
        
        print("🚀 Autonomous controller started")
        print("🎮 Switch RC to AUTONOMOUS mode to begin navigation")
        return True
    
    def arduino_communication_loop(self):
        """Handle communication with Arduino"""
        while self.running and self.arduino:
            try:
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode().strip()
                    if line:
                        self.process_arduino_data(line)
            except Exception as e:
                print(f"❌ Arduino communication error: {e}")
                time.sleep(0.1)
            time.sleep(0.01)
    
    def process_arduino_data(self, data):
        """Process data received from Arduino"""
        try:
            if data.startswith('{') and data.endswith('}'):
                arduino_data = json.loads(data)
                
                # Update robot state
                self.robot_state.update({
                    'rc_ch1': arduino_data.get('ch1', 0),
                    'rc_ch2': arduino_data.get('ch2', 0),
                    'rc_ch9': arduino_data.get('ch9', 0),
                    'rc_valid': arduino_data.get('valid', False),
                    'mode': arduino_data.get('mode', 0),
                    'emergency_stop': arduino_data.get('emergency', False),
                    'last_update': time.time()
                })
                
                # Log mode changes
                mode_names = {0: 'FAILSAFE', 1: 'MANUAL', 2: 'AUTONOMOUS'}
                if hasattr(self, 'last_mode') and self.last_mode != self.robot_state['mode']:
                    mode_name = mode_names.get(self.robot_state['mode'], 'UNKNOWN')
                    print(f"🔄 Mode changed to: {mode_name}")
                
                self.last_mode = self.robot_state['mode']
                
        except json.JSONDecodeError:
            # Non-JSON messages (status, etc.)
            if "initialized" in data.lower():
                print(f"🔧 Arduino: {data}")
    
    def navigation_loop(self):
        """Main autonomous navigation loop"""
        print("🧭 Navigation loop started")
        
        while self.running:
            try:
                # Only navigate when in autonomous mode
                if self.robot_state.get('mode') == 2 and self.robot_state.get('rc_valid'):
                    if not self.robot_state.get('emergency_stop'):
                        self.autonomous_navigation()
                    else:
                        # Emergency stop - halt navigation
                        self.send_motor_command(0, 0)
                        print("🛑 Emergency stop activated")
                else:
                    # Not in autonomous mode - let Arduino handle control
                    pass
                
                time.sleep(0.1)  # 10Hz navigation updates
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                time.sleep(1)
    
    def autonomous_navigation(self):
        """Simple but confident autonomous navigation"""
        current_time = time.time()
        time_in_state = current_time - self.nav_state_start
        
        if self.nav_state == 'EXPLORING':
            # Default state: move straight confidently
            self.send_motor_command(self.cruise_speed, self.cruise_speed)
            
            # Minimum straight time before considering direction changes
            if time_in_state < self.min_straight_time:
                return
            
            # Occasional random direction change for exploration
            if current_time - self.last_direction_change > self.get_random_interval():
                self.change_nav_state('TURNING')
                self.last_direction_change = current_time
                print("🔄 Random exploration turn")
        
        elif self.nav_state == 'TURNING':
            # Execute exploration turn
            import random
            if not hasattr(self, 'turn_direction'):
                self.turn_direction = random.choice(['left', 'right'])
                print(f"↩️ Turning {self.turn_direction}")
            
            if self.turn_direction == 'left':
                self.send_motor_command(-self.turn_speed, self.turn_speed)
            else:
                self.send_motor_command(self.turn_speed, -self.turn_speed)
            
            # Turn for specified duration
            if time_in_state > self.turn_duration:
                self.change_nav_state('EXPLORING')
                delattr(self, 'turn_direction')
                print("➡️ Resuming straight exploration")
    
    def change_nav_state(self, new_state):
        """Change navigation state"""
        self.nav_state = new_state
        self.nav_state_start = time.time()
    
    def get_random_interval(self):
        """Get random exploration interval"""
        import random
        return random.uniform(self.explore_interval[0], self.explore_interval[1])
    
    def send_motor_command(self, left_speed, right_speed):
        """Send motor command to Arduino"""
        if not self.arduino:
            return
        
        # Constrain speeds
        left_speed = max(-255, min(255, int(left_speed)))
        right_speed = max(-255, min(255, int(right_speed)))
        
        command = f'{{"left":{left_speed},"right":{right_speed}}}\n'
        
        try:
            self.arduino.write(command.encode())
        except Exception as e:
            print(f"❌ Failed to send motor command: {e}")
    
    def get_status(self):
        """Get current status for monitoring"""
        mode_names = {0: 'FAILSAFE', 1: 'MANUAL', 2: 'AUTONOMOUS'}
        return {
            'mode': mode_names.get(self.robot_state.get('mode'), 'UNKNOWN'),
            'nav_state': self.nav_state,
            'rc_valid': self.robot_state.get('rc_valid'),
            'emergency_stop': self.robot_state.get('emergency_stop'),
            'time_in_state': time.time() - self.nav_state_start
        }

class UltrasonicSensor:
    """Ultrasonic sensor integration for Pi-based obstacle avoidance"""
    
    def __init__(self, trig_pin=18, echo_pin=24):
        """Initialize ultrasonic sensor on Pi GPIO pins"""
        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
            self.trig_pin = trig_pin
            self.echo_pin = echo_pin
            
            # Setup GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(trig_pin, GPIO.OUT)
            GPIO.setup(echo_pin, GPIO.IN)
            
            self.available = True
            print(f"📡 Ultrasonic sensor initialized on GPIO {trig_pin}/{echo_pin}")
            
        except ImportError:
            print("⚠️ RPi.GPIO not available - ultrasonic sensor disabled")
            self.available = False
        except Exception as e:
            print(f"⚠️ Ultrasonic sensor setup failed: {e}")
            self.available = False
    
    def read_distance(self):
        """Read distance from ultrasonic sensor"""
        if not self.available:
            return None
        
        try:
            # Send trigger pulse
            self.GPIO.output(self.trig_pin, False)
            time.sleep(0.000002)
            self.GPIO.output(self.trig_pin, True)
            time.sleep(0.00001)
            self.GPIO.output(self.trig_pin, False)
            
            # Measure echo
            pulse_start = time.time()
            timeout = pulse_start + 0.03  # 30ms timeout
            
            while self.GPIO.input(self.echo_pin) == 0:
                pulse_start = time.time()
                if pulse_start > timeout:
                    return None
            
            while self.GPIO.input(self.echo_pin) == 1:
                pulse_end = time.time()
                if pulse_end > timeout:
                    return None
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150  # Speed of sound calculation
            
            return distance if 2 < distance < 400 else None
            
        except Exception as e:
            print(f"❌ Ultrasonic sensor error: {e}")
            return None

def main():
    print("=" * 60)
    print("🤖 RASPBERRY PI AUTONOMOUS CONTROLLER")
    print("=" * 60)
    print()
    print("Architecture:")
    print("📊 Pi: High-level navigation logic")
    print("🔧 Arduino: RC processing & motor control bridge")
    print()
    
    # Create controller
    controller = AutonomousController()
    
    # Start controller
    if not controller.start():
        print("❌ Failed to start autonomous controller")
        return
    
    try:
        print("🔄 Controller running. Switch to AUTONOMOUS mode to begin navigation.")
        print("📊 Status updates every 5 seconds. Press Ctrl+C to stop.")
        print()
        
        # Status monitoring loop
        while True:
            time.sleep(5)
            status = controller.get_status()
            timestamp = datetime.now().strftime("%H:%M:%S")
            
            print(f"[{timestamp}] Mode: {status['mode']:<12} | "
                  f"Nav: {status['nav_state']:<12} | "
                  f"RC: {'✅' if status['rc_valid'] else '❌'} | "
                  f"Emergency: {'🛑' if status['emergency_stop'] else '✅'}")
    
    except KeyboardInterrupt:
        print("\n⏹️ Stopping autonomous controller...")
    
    finally:
        controller.disconnect_arduino()
        print("🏁 Autonomous controller stopped")

if __name__ == "__main__":
    main()