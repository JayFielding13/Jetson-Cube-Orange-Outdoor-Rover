#!/usr/bin/env python3
"""
Arduino Motor Control Test
Sends motor commands to Arduino via serial and monitors RC data
Tests bidirectional communication and motor control
"""

import serial
import json
import time
import threading
from datetime import datetime

class ArduinoMotorController:
    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        self.port = port
        self.baud = baud
        self.serial_conn = None
        self.running = False
        self.latest_rc_data = None
        self.motor_ack_received = False
        
    def connect(self):
        """Connect to Arduino"""
        try:
            self.serial_conn = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            print(f"✅ Connected to Arduino on {self.port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to Arduino: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Arduino"""
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()
            print("📱 Disconnected from Arduino")
    
    def send_motor_command(self, left_speed, right_speed):
        """Send motor command to Arduino"""
        if not self.serial_conn:
            return False
            
        # Constrain speeds to valid range
        left_speed = max(-255, min(255, int(left_speed)))
        right_speed = max(-255, min(255, int(right_speed)))
        
        # Create JSON command
        command = f'{{"left":{left_speed},"right":{right_speed}}}\n'
        
        try:
            self.serial_conn.write(command.encode())
            print(f"📤 Sent: L={left_speed:+4d}, R={right_speed:+4d}")
            return True
        except Exception as e:
            print(f"❌ Failed to send command: {e}")
            return False
    
    def read_serial_data(self):
        """Read data from Arduino in background thread"""
        while self.running and self.serial_conn:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode().strip()
                    if line:
                        self.process_arduino_message(line)
            except Exception as e:
                print(f"❌ Serial read error: {e}")
                time.sleep(0.1)
            time.sleep(0.01)
    
    def process_arduino_message(self, message):
        """Process incoming message from Arduino"""
        try:
            # Try to parse as JSON
            if message.startswith('{') and message.endswith('}'):
                data = json.loads(message)
                
                # Handle RC data
                if 'ch1' in data:
                    self.latest_rc_data = data
                    mode_str = self.get_mode_string(data.get('ch9', 0))
                    print(f"📡 RC: CH1={data.get('ch1', 0):+4d}, CH2={data.get('ch2', 0):+4d}, CH9={data.get('ch9', 0):+4d} ({mode_str}), Valid={data.get('valid', False)}")
                
                # Handle motor acknowledgment
                elif 'motor_ack' in data:
                    ack_data = data['motor_ack']
                    print(f"✅ ACK: L={ack_data.get('left', 0):+4d}, R={ack_data.get('right', 0):+4d}")
                    self.motor_ack_received = True
            else:
                # Handle non-JSON messages (status/debug)
                print(f"🔧 Arduino: {message}")
                
        except json.JSONDecodeError:
            # Handle non-JSON messages
            print(f"🔧 Arduino: {message}")
    
    def get_mode_string(self, ch9_value):
        """Convert CH9 value to mode string"""
        if ch9_value < -500:
            return "FAILSAFE"
        elif ch9_value > 500:
            return "AUTONOMOUS"
        else:
            return "MANUAL"
    
    def start_monitoring(self):
        """Start background thread for serial monitoring"""
        self.running = True
        self.monitor_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.monitor_thread.start()
        print("🔄 Started serial monitoring")

def cycle_motor_speeds(controller, duration=3):
    """Cycle through motor speeds for testing"""
    print(f"\n=== Motor Speed Cycle Test (Duration: {duration}s each) ===")
    
    tests = [
        (100, 100, "Both Forward 100%"),
        (50, 50, "Both Forward 50%"),
        (-50, -50, "Both Reverse 50%"),
        (-100, -100, "Both Reverse 100%"),
        (100, -100, "Left Forward, Right Reverse"),
        (-100, 100, "Left Reverse, Right Forward"),
        (100, 0, "Left Forward, Right Stop"),
        (0, 100, "Left Stop, Right Forward"),
        (0, 0, "Both Stop")
    ]
    
    for left_speed, right_speed, description in tests:
        print(f"\n📋 Test: {description}")
        controller.send_motor_command(left_speed, right_speed)
        time.sleep(duration)
    
    print("\n✅ Motor cycle test complete")

def ramp_speed_test(controller, max_speed=255, steps=10, delay=0.5):
    """Ramp motor speeds up and down"""
    print(f"\n=== Motor Speed Ramp Test (Max: {max_speed}, Steps: {steps}) ===")
    
    # Ramp up forward
    print("🔼 Ramping up (forward)...")
    for i in range(steps + 1):
        speed = int((i / steps) * max_speed)
        controller.send_motor_command(speed, speed)
        time.sleep(delay)
    
    # Ramp down
    print("🔽 Ramping down...")
    for i in range(steps, -1, -1):
        speed = int((i / steps) * max_speed)
        controller.send_motor_command(speed, speed)
        time.sleep(delay)
    
    # Ramp reverse
    print("🔼 Ramping up (reverse)...")
    for i in range(steps + 1):
        speed = -int((i / steps) * max_speed)
        controller.send_motor_command(speed, speed)
        time.sleep(delay)
    
    # Ramp back to stop
    print("🔽 Ramping to stop...")
    for i in range(steps, -1, -1):
        speed = -int((i / steps) * max_speed)
        controller.send_motor_command(speed, speed)
        time.sleep(delay)
    
    print("✅ Ramp test complete")

def main():
    print("=== Arduino Motor Control Test ===")
    print("This test will:")
    print("1. Connect to Arduino via serial")
    print("2. Monitor RC data from Arduino")
    print("3. Send motor commands to Arduino")
    print("4. Test bidirectional communication")
    print()
    
    # Create controller
    controller = ArduinoMotorController()
    
    # Try to connect
    if not controller.connect():
        print("❌ Failed to connect to Arduino. Check connection and try again.")
        return
    
    try:
        # Start monitoring
        controller.start_monitoring()
        time.sleep(1)
        
        print("\n📊 Monitoring RC data for 5 seconds...")
        time.sleep(5)
        
        if controller.latest_rc_data:
            print(f"✅ RC data received: {controller.latest_rc_data}")
        else:
            print("⚠️ No RC data received")
        
        # Test motor commands
        print("\n🎮 Testing motor commands...")
        
        # Simple test
        print("\n--- Basic Motor Test ---")
        controller.send_motor_command(100, 100)
        time.sleep(2)
        controller.send_motor_command(0, 0)
        time.sleep(1)
        
        # Full cycle test
        cycle_motor_speeds(controller, duration=2)
        
        # Ramp test
        ramp_speed_test(controller, max_speed=150, steps=15, delay=0.3)
        
        print("\n🏁 All tests complete!")
        
    except KeyboardInterrupt:
        print("\n⏹️ Test interrupted by user")
    
    finally:
        # Clean shutdown
        print("\n🛑 Stopping motors...")
        controller.send_motor_command(0, 0)
        time.sleep(1)
        controller.disconnect()

if __name__ == "__main__":
    main()