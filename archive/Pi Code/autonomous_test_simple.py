#!/usr/bin/env python3
"""
Simple Autonomous Test - Bypasses RC complexity
Just sends motor commands directly to test the distributed architecture
"""

import serial
import time
import threading
import random

class SimpleAutonomousTest:
    def __init__(self, arduino_port='/dev/ttyUSB0', baud=115200):
        self.arduino_port = arduino_port
        self.baud = baud
        self.arduino = None
        self.running = False
        
        # Simple navigation
        self.nav_state = 'STRAIGHT'
        self.nav_start = time.time()
        
        # Movement parameters
        self.cruise_speed = 150
        self.turn_speed = 120
        
    def connect_arduino(self):
        """Connect to Arduino"""
        try:
            self.arduino = serial.Serial(self.arduino_port, self.baud, timeout=1)
            time.sleep(2)
            print(f"✅ Connected to Arduino on {self.arduino_port}")
            return True
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def start(self):
        """Start test"""
        if not self.connect_arduino():
            return False
        
        self.running = True
        
        # Start navigation thread
        self.nav_thread = threading.Thread(target=self.navigation_loop, daemon=True)
        self.nav_thread.start()
        
        print("🚀 Simple autonomous test started")
        print("🤖 Sending motor commands directly to Arduino")
        return True
    
    def navigation_loop(self):
        """Simple navigation test"""
        print("🧭 Navigation test started")
        
        while self.running:
            try:
                current_time = time.time()
                time_in_state = current_time - self.nav_start
                
                if self.nav_state == 'STRAIGHT':
                    # Move straight for 5 seconds
                    self.send_motor_command(self.cruise_speed, self.cruise_speed)
                    print(f"➡️ Moving straight: {time_in_state:.1f}s")
                    
                    if time_in_state > 5.0:
                        self.nav_state = 'TURNING'
                        self.nav_start = current_time
                        self.turn_direction = random.choice(['left', 'right'])
                        print(f"🔄 Starting {self.turn_direction} turn")
                
                elif self.nav_state == 'TURNING':
                    # Turn for 2 seconds
                    if self.turn_direction == 'left':
                        self.send_motor_command(-self.turn_speed, self.turn_speed)
                    else:
                        self.send_motor_command(self.turn_speed, -self.turn_speed)
                    
                    print(f"↻ Turning {self.turn_direction}: {time_in_state:.1f}s")
                    
                    if time_in_state > 2.0:
                        self.nav_state = 'STRAIGHT'
                        self.nav_start = current_time
                        print("➡️ Resuming straight movement")
                
                time.sleep(0.2)  # 5Hz updates
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                time.sleep(1)
    
    def send_motor_command(self, left_speed, right_speed):
        """Send motor command to Arduino"""
        if not self.arduino:
            return
        
        # Constrain speeds
        left_speed = max(-255, min(255, int(left_speed)))
        right_speed = max(-255, min(255, int(right_speed)))
        
        # Send JSON command
        command = f'{{"left":{left_speed},"right":{right_speed}}}\n'
        
        try:
            self.arduino.write(command.encode())
            print(f"📤 Motor: L={left_speed}, R={right_speed}")
        except Exception as e:
            print(f"❌ Motor command failed: {e}")
    
    def stop(self):
        """Stop test"""
        self.running = False
        
        # Stop motors
        self.send_motor_command(0, 0)
        
        if self.arduino:
            self.arduino.close()
        
        print("🛑 Test stopped")

def main():
    print("=" * 50)
    print("🔧 SIMPLE AUTONOMOUS TEST")
    print("=" * 50)
    print()
    print("This test bypasses RC complexity and sends")
    print("motor commands directly to test the basic")
    print("Pi → Arduino → Motor communication.")
    print()
    print("Expected behavior:")
    print("- 5 seconds straight movement")
    print("- 2 seconds turning (random direction)")
    print("- Repeat cycle")
    print()
    
    tester = SimpleAutonomousTest()
    
    if not tester.start():
        print("❌ Test failed to start")
        return
    
    try:
        print("🔄 Test running. Press Ctrl+C to stop.")
        print()
        
        while True:
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("\n⏹️ Stopping test...")
    
    finally:
        tester.stop()
        print("🏁 Test complete")

if __name__ == "__main__":
    main()