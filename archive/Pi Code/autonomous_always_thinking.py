#!/usr/bin/env python3
"""
Always Thinking Autonomous Controller
Pi runs navigation logic continuously, Arduino gatekeeper decides when to apply commands
"""

import serial
import json
import time
import threading
import random
from datetime import datetime

class AlwaysThinkingController:
    def __init__(self, arduino_port='/dev/ttyUSB0', baud=115200):
        self.arduino_port = arduino_port
        self.baud = baud
        self.arduino = None
        self.running = False
        
        # Robot state from Arduino
        self.robot_state = {
            'mode': 0,  # 0=Failsafe, 1=Manual, 2=Autonomous
            'rc_valid': False,
            'emergency_stop': False,
            'distance': None,
            'pi_command_valid': False,
            'gatekeeper_mode': 'FAILSAFE',
            'last_update': time.time()
        }
        
        # Navigation state - ALWAYS RUNNING
        self.nav_state = 'STRAIGHT'
        self.nav_start = time.time()
        self.turn_direction = 'left'
        
        # Movement parameters
        self.cruise_speed = 150
        self.slow_speed = 80
        self.turn_speed = 120
        
        # Navigation logic parameters
        self.straight_time = 6.0        # Confident straight runs
        self.turn_time = 2.5            # Turn duration
        self.obstacle_threshold = 30.0  # Slow down distance
        
        # Statistics tracking
        self.commands_sent = 0
        self.commands_applied = 0
        self.last_applied_check = time.time()
        
        print("🧠 Pi Controller: Always thinking, ready to control")
        print("🛡️ Arduino Gatekeeper: Decides when to apply Pi commands")
    
    def connect_arduino(self):
        """Connect to Arduino gatekeeper"""
        try:
            self.arduino = serial.Serial(self.arduino_port, self.baud, timeout=1)
            time.sleep(2)
            print(f"✅ Connected to Arduino gatekeeper on {self.arduino_port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to Arduino: {e}")
            return False
    
    def start(self):
        """Start the always-thinking controller"""
        if not self.connect_arduino():
            return False
        
        self.running = True
        
        # Start threads
        self.arduino_thread = threading.Thread(target=self.arduino_communication_loop, daemon=True)
        self.navigation_thread = threading.Thread(target=self.navigation_loop, daemon=True)
        
        self.arduino_thread.start()
        self.navigation_thread.start()
        
        print("🚀 Always-thinking controller started")
        print("🧠 Pi navigation logic running continuously")
        print("🎮 Arduino applies commands only in AUTONOMOUS mode")
        return True
    
    def arduino_communication_loop(self):
        """Handle Arduino communication and acknowledgments"""
        while self.running and self.arduino:
            try:
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode().strip()
                    if line and line.startswith('{'):
                        arduino_data = json.loads(line)
                        
                        # Update robot state
                        self.robot_state.update({
                            'mode': arduino_data.get('mode', 0),
                            'rc_valid': arduino_data.get('valid', False),
                            'emergency_stop': arduino_data.get('emergency', False),
                            'distance': arduino_data.get('distance'),
                            'pi_command_valid': arduino_data.get('pi_command_valid', False),
                            'gatekeeper_mode': arduino_data.get('gatekeeper_mode', 'UNKNOWN'),
                            'last_update': time.time()
                        })
                        
                        # Check for Pi command acknowledgments
                        if 'pi_ack' in arduino_data:
                            ack = arduino_data['pi_ack']
                            if ack.get('applied', False):
                                self.commands_applied += 1
                            
            except Exception as e:
                print(f"❌ Arduino communication error: {e}")
                time.sleep(0.1)
            time.sleep(0.01)
    
    def navigation_loop(self):
        """ALWAYS RUNNING navigation logic"""
        print("🧭 Navigation logic started - thinking continuously")
        
        while self.running:
            try:
                current_time = time.time()
                
                # ALWAYS compute navigation, regardless of mode
                left_speed, right_speed = self.compute_navigation(current_time)
                
                # ALWAYS send commands to Arduino (gatekeeper decides if to apply)
                self.send_motor_command(left_speed, right_speed)
                
                # Log thinking vs controlling
                if current_time - self.last_applied_check >= 5.0:
                    self.log_thinking_status()
                    self.last_applied_check = current_time
                
                time.sleep(0.2)  # 5Hz navigation updates
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                time.sleep(1)
    
    def compute_navigation(self, current_time):
        """Core navigation logic - always running"""
        time_in_state = current_time - self.nav_start
        distance = self.robot_state.get('distance')
        
        if self.nav_state == 'STRAIGHT':
            # Default: move straight confidently
            left_speed = self.cruise_speed
            right_speed = self.cruise_speed
            
            # Check for obstacles
            if distance and distance < self.obstacle_threshold:
                left_speed = self.slow_speed
                right_speed = self.slow_speed
                
                # If very close, prepare to turn
                if distance < 15.0:
                    self.nav_state = 'TURNING'
                    self.nav_start = current_time
                    self.turn_direction = random.choice(['left', 'right'])
                    print(f"🤔 Pi thinking: Obstacle at {distance:.1f}cm, planning {self.turn_direction} turn")
            
            # Periodic exploration turns
            elif time_in_state > self.straight_time:
                self.nav_state = 'TURNING'
                self.nav_start = current_time
                self.turn_direction = random.choice(['left', 'right'])
                print(f"🤔 Pi thinking: Exploration turn {self.turn_direction} after {time_in_state:.1f}s straight")
        
        elif self.nav_state == 'TURNING':
            # Execute turn
            if self.turn_direction == 'left':
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
            else:
                left_speed = self.turn_speed
                right_speed = -self.turn_speed
            
            # Return to straight after turn duration
            if time_in_state > self.turn_time:
                self.nav_state = 'STRAIGHT'
                self.nav_start = current_time
                print("🤔 Pi thinking: Turn complete, resuming straight")
        
        else:
            # Default to stopped
            left_speed = 0
            right_speed = 0
        
        return left_speed, right_speed
    
    def send_motor_command(self, left_speed, right_speed):
        """Send command to Arduino gatekeeper (always)"""
        if not self.arduino:
            return
        
        # Constrain speeds
        left_speed = max(-255, min(255, int(left_speed)))
        right_speed = max(-255, min(255, int(right_speed)))
        
        # Send JSON command
        command = f'{{"left":{left_speed},"right":{right_speed}}}\n'
        
        try:
            self.arduino.write(command.encode())
            self.commands_sent += 1
        except Exception as e:
            print(f"❌ Command send failed: {e}")
    
    def log_thinking_status(self):
        """Log what Pi is thinking vs what's actually happening"""
        mode = self.robot_state.get('gatekeeper_mode', 'UNKNOWN')
        applied_rate = (self.commands_applied / max(1, self.commands_sent)) * 100
        
        if mode == 'AUTONOMOUS':
            print(f"🧠→🤖 Pi CONTROLLING: {applied_rate:.0f}% commands applied")
        else:
            print(f"🧠💭 Pi THINKING: Mode={mode}, ready to control when switched")
        
        # Reset counters
        self.commands_sent = 0
        self.commands_applied = 0
    
    def get_status(self):
        """Get detailed status"""
        return {
            'gatekeeper_mode': self.robot_state.get('gatekeeper_mode', 'UNKNOWN'),
            'nav_state': self.nav_state,
            'distance': self.robot_state.get('distance'),
            'pi_commands_valid': self.robot_state.get('pi_command_valid', False),
            'rc_valid': self.robot_state.get('rc_valid'),
            'emergency_stop': self.robot_state.get('emergency_stop'),
            'thinking_vs_controlling': 'CONTROLLING' if self.robot_state.get('gatekeeper_mode') == 'AUTONOMOUS' else 'THINKING',
            'time_in_state': time.time() - self.nav_start
        }
    
    def disconnect(self):
        """Clean shutdown"""
        self.running = False
        if self.arduino:
            self.arduino.close()
        print("🔌 Disconnected from Arduino gatekeeper")

def main():
    print("=" * 70)
    print("🧠 ALWAYS THINKING AUTONOMOUS CONTROLLER")
    print("=" * 70)
    print()
    print("Revolutionary Architecture:")
    print("🧠 Pi: Always thinking, computing navigation continuously")
    print("🛡️ Arduino: Safety gatekeeper, applies commands only when safe")
    print("🎮 RC Switch: Instantly enables/disables Pi control")
    print("⚡ Benefits: Zero startup delay, seamless transitions, ultimate safety")
    print()
    print("Operation:")
    print("📊 MANUAL mode: Pi thinks, Arduino ignores Pi (FULL operator control)")
    print("🤖 AUTONOMOUS mode: Pi thinks, Arduino applies Pi (SAFETY systems active)")
    print("🛑 EMERGENCY: Safety systems override only in AUTONOMOUS mode")
    print()
    
    controller = AlwaysThinkingController()
    
    if not controller.start():
        print("❌ Failed to start controller")
        return
    
    try:
        print("🔄 Always-thinking controller running.")
        print("🧠 Pi navigation logic active - watch thinking vs controlling status")
        print("🎮 Switch RC to see instant mode transitions")
        print("📊 Status every 3 seconds. Press Ctrl+C to stop.")
        print()
        
        while True:
            time.sleep(3)
            status = controller.get_status()
            timestamp = datetime.now().strftime("%H:%M:%S")
            
            # Status indicators
            thinking_icon = "🧠💭" if status['thinking_vs_controlling'] == 'THINKING' else "🧠→🤖"
            distance_str = f"{status['distance']:.1f}cm" if status['distance'] else "N/A"
            
            print(f"[{timestamp}] {status['gatekeeper_mode']:<12} | "
                  f"{thinking_icon} {status['thinking_vs_controlling']:<12} | "
                  f"{status['nav_state']:<12} | "
                  f"Dist: {distance_str:<8} | "
                  f"Pi Valid: {'✅' if status['pi_commands_valid'] else '❌'} | "
                  f"RC: {'✅' if status['rc_valid'] else '❌'} | "
                  f"Emerg: {'🛑' if status['emergency_stop'] else '✅'}")
    
    except KeyboardInterrupt:
        print("\n⏹️ Stopping always-thinking controller...")
    
    finally:
        controller.disconnect()
        print("🏁 Controller stopped")

if __name__ == "__main__":
    main()