#!/usr/bin/env python3
"""
Smooth Autonomous Controller
Based on the successful simple test, adds RC mode detection
"""

import serial
import json
import time
import threading
import random
from datetime import datetime

class SmoothAutonomousController:
    def __init__(self, arduino_port='/dev/ttyUSB0', baud=115200):
        self.arduino_port = arduino_port
        self.baud = baud
        self.arduino = None
        self.running = False
        
        # Robot state
        self.robot_state = {
            'mode': 0,  # 0=Failsafe, 1=Manual, 2=Autonomous
            'rc_valid': False,
            'emergency_stop': False,
            'last_update': time.time()
        }
        
        # Navigation state (same as successful simple test)
        self.nav_state = 'STRAIGHT'
        self.nav_start = time.time()
        
        # Movement parameters (same as successful simple test)
        self.cruise_speed = 150
        self.turn_speed = 120
        
        # Autonomous mode tracking
        self.autonomous_active = False
        self.last_mode_2_time = 0
        
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
        """Start the controller"""
        if not self.connect_arduino():
            return False
        
        self.running = True
        
        # Start threads
        self.arduino_thread = threading.Thread(target=self.arduino_communication_loop, daemon=True)
        self.navigation_thread = threading.Thread(target=self.navigation_loop, daemon=True)
        
        self.arduino_thread.start()
        self.navigation_thread.start()
        
        print("🚀 Smooth autonomous controller started")
        print("🎮 Switch RC to AUTONOMOUS mode for smooth navigation")
        return True
    
    def arduino_communication_loop(self):
        """Handle Arduino communication"""
        while self.running and self.arduino:
            try:
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode().strip()
                    if line and line.startswith('{'):
                        arduino_data = json.loads(line)
                        
                        # Simple mode tracking
                        mode = arduino_data.get('mode', 0)
                        self.robot_state.update({
                            'mode': mode,
                            'rc_valid': arduino_data.get('valid', False),
                            'emergency_stop': arduino_data.get('emergency', False),
                            'last_update': time.time()
                        })
                        
                        # Track when we see mode 2 (autonomous)
                        if mode == 2:
                            self.last_mode_2_time = time.time()
                            
            except Exception as e:
                print(f"❌ Arduino error: {e}")
                time.sleep(0.1)
            time.sleep(0.01)
    
    def navigation_loop(self):
        """Main navigation loop - uses same logic as successful simple test"""
        print("🧭 Smooth navigation started")
        
        while self.running:
            try:
                current_time = time.time()
                
                # Determine if we should run autonomous
                mode = self.robot_state.get('mode')
                emergency = self.robot_state.get('emergency_stop')
                
                # Run autonomous if:
                # 1. Currently in mode 2 (autonomous), OR
                # 2. We saw mode 2 within the last 2 seconds (for stability)
                should_run_autonomous = (
                    mode == 2 or 
                    (current_time - self.last_mode_2_time < 2.0)
                ) and not emergency
                
                if should_run_autonomous:
                    if not self.autonomous_active:
                        print("🚀 Starting autonomous navigation")
                        self.autonomous_active = True
                        self.nav_state = 'STRAIGHT'
                        self.nav_start = current_time
                    
                    # Run the same smooth navigation that worked in simple test
                    self.smooth_navigation(current_time)
                else:
                    if self.autonomous_active:
                        print("⏹️ Stopping autonomous navigation")
                        self.autonomous_active = False
                        # Don't send stop command - let Arduino handle manual/failsafe
                
                time.sleep(0.2)  # 5Hz navigation updates (same as simple test)
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                time.sleep(1)
    
    def smooth_navigation(self, current_time):
        """Smooth navigation - exact same logic as successful simple test"""
        time_in_state = current_time - self.nav_start
        
        if self.nav_state == 'STRAIGHT':
            # Move straight for 5 seconds (same as simple test)
            self.send_motor_command(self.cruise_speed, self.cruise_speed)
            
            if time_in_state > 5.0:
                self.nav_state = 'TURNING'
                self.nav_start = current_time
                self.turn_direction = random.choice(['left', 'right'])
                print(f"🔄 Starting {self.turn_direction} turn")
        
        elif self.nav_state == 'TURNING':
            # Turn for 2 seconds (same as simple test)
            if self.turn_direction == 'left':
                self.send_motor_command(-self.turn_speed, self.turn_speed)
            else:
                self.send_motor_command(self.turn_speed, -self.turn_speed)
            
            if time_in_state > 2.0:
                self.nav_state = 'STRAIGHT'
                self.nav_start = current_time
                print("➡️ Resuming straight movement")
    
    def send_motor_command(self, left_speed, right_speed):
        """Send motor command to Arduino - same as simple test"""
        if not self.arduino:
            return
        
        # Constrain speeds
        left_speed = max(-255, min(255, int(left_speed)))
        right_speed = max(-255, min(255, int(right_speed)))
        
        # Send JSON command (same format as simple test)
        command = f'{{"left":{left_speed},"right":{right_speed}}}\n'
        
        try:
            self.arduino.write(command.encode())
        except Exception as e:
            print(f"❌ Motor command failed: {e}")
    
    def get_status(self):
        """Get current status"""
        mode_names = {0: 'FAILSAFE', 1: 'MANUAL', 2: 'AUTONOMOUS'}
        
        return {
            'mode': mode_names.get(self.robot_state.get('mode'), 'UNKNOWN'),
            'nav_state': self.nav_state if self.autonomous_active else 'IDLE',
            'autonomous_active': self.autonomous_active,
            'rc_valid': self.robot_state.get('rc_valid'),
            'emergency_stop': self.robot_state.get('emergency_stop'),
            'time_since_mode_2': time.time() - self.last_mode_2_time if self.last_mode_2_time > 0 else 999
        }
    
    def disconnect(self):
        """Clean shutdown"""
        self.running = False
        if self.arduino:
            self.arduino.close()
        print("🔌 Arduino disconnected")

def main():
    print("=" * 60)
    print("🎯 SMOOTH AUTONOMOUS CONTROLLER")
    print("=" * 60)
    print()
    print("Based on successful simple test architecture:")
    print("✅ Proven smooth motor control")
    print("🎮 RC mode switching")
    print("⚡ 5-second straight runs, 2-second turns")
    print("🛡️ Stable mode detection with hysteresis")
    print()
    
    controller = SmoothAutonomousController()
    
    if not controller.start():
        print("❌ Failed to start controller")
        return
    
    try:
        print("🔄 Controller running. Switch to AUTONOMOUS mode for smooth navigation.")
        print("📊 Status updates every 3 seconds. Press Ctrl+C to stop.")
        print()
        
        while True:
            time.sleep(3)
            status = controller.get_status()
            timestamp = datetime.now().strftime("%H:%M:%S")
            
            auto_status = "🟢 ACTIVE" if status['autonomous_active'] else "⚪ IDLE"
            time_since = f"{status['time_since_mode_2']:.1f}s" if status['time_since_mode_2'] < 999 else "N/A"
            
            print(f"[{timestamp}] {status['mode']:<12} | "
                  f"{status['nav_state']:<12} | "
                  f"Auto: {auto_status} | "
                  f"Since Mode2: {time_since:<6} | "
                  f"RC: {'✅' if status['rc_valid'] else '❌'} | "
                  f"Emerg: {'🛑' if status['emergency_stop'] else '✅'}")
    
    except KeyboardInterrupt:
        print("\n⏹️ Stopping controller...")
    
    finally:
        controller.disconnect()
        print("🏁 Controller stopped")

if __name__ == "__main__":
    main()