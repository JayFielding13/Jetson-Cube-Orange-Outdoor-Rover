#!/usr/bin/env python3
"""
RC Control Monitor
Monitors Arduino RC control and mode switching
Tests RC signal to motor control integration
"""

import serial
import json
import time
import threading
from datetime import datetime

class RCControlMonitor:
    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        self.port = port
        self.baud = baud
        self.serial_conn = None
        self.running = False
        self.latest_rc_data = None
        self.current_mode = "UNKNOWN"
        
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
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    
                    print(f"[{timestamp}] 📡 RC: CH1={data.get('ch1', 0):+4d} CH2={data.get('ch2', 0):+4d} CH9={data.get('ch9', 0):+4d} ({mode_str}) Valid={data.get('valid', False)}")
                
                # Handle mode changes
                elif 'mode_change' in data:
                    new_mode = data['mode_change']
                    self.current_mode = new_mode
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    
                    # Color-coded mode display
                    mode_color = {
                        'FAILSAFE': '🔴',
                        'MANUAL': '🟢', 
                        'AUTONOMOUS': '🔵'
                    }
                    color = mode_color.get(new_mode, '⚪')
                    
                    print(f"[{timestamp}] {color} MODE: {new_mode}")
                
                # Handle motor acknowledgments (from autonomous mode)
                elif 'motor_ack' in data:
                    ack_data = data['motor_ack']
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    print(f"[{timestamp}] 🤖 AUTO: L={ack_data.get('left', 0):+4d} R={ack_data.get('right', 0):+4d}")
                    
            else:
                # Handle non-JSON messages
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(f"[{timestamp}] 🔧 {message}")
                
        except json.JSONDecodeError:
            # Handle non-JSON messages
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"[{timestamp}] 🔧 {message}")
    
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
        print("🔄 Started RC control monitoring")
    
    def send_test_command(self, left_speed, right_speed):
        """Send a test motor command (for autonomous mode testing)"""
        if not self.serial_conn:
            return False
            
        command = f'{{"left":{left_speed},"right":{right_speed}}}\n'
        
        try:
            self.serial_conn.write(command.encode())
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"[{timestamp}] 📤 TEST CMD: L={left_speed:+4d} R={right_speed:+4d}")
            return True
        except Exception as e:
            print(f"❌ Failed to send command: {e}")
            return False

def main():
    print("=== RC Control Monitor ===")
    print("This monitor will:")
    print("1. Show RC signal data in real-time")
    print("2. Display mode changes (FAILSAFE/MANUAL/AUTONOMOUS)")
    print("3. Monitor motor control in all modes")
    print("4. Allow testing autonomous commands")
    print()
    print("Legend:")
    print("🔴 FAILSAFE   🟢 MANUAL   🔵 AUTONOMOUS")
    print("📡 RC Data    🤖 Auto Motor    📤 Test Command")
    print()
    
    # Create monitor
    monitor = RCControlMonitor()
    
    # Try to connect
    if not monitor.connect():
        print("❌ Failed to connect to Arduino. Check connection and try again.")
        return
    
    try:
        # Start monitoring
        monitor.start_monitoring()
        print("⏰ Monitoring started. Use your RC transmitter to test modes:")
        print("   • Position 1 (CH9 < -500): FAILSAFE mode")
        print("   • Position 2 (CH9 ~0): MANUAL mode") 
        print("   • Position 3 (CH9 > +500): AUTONOMOUS mode")
        print()
        print("Press 't' + Enter to send test autonomous commands")
        print("Press 'q' + Enter to quit")
        print()
        
        while True:
            user_input = input().strip().lower()
            
            if user_input == 'q':
                break
            elif user_input == 't':
                print("\n🧪 Testing autonomous mode commands...")
                print("(Switch to AUTONOMOUS mode on transmitter for these to work)")
                
                test_commands = [
                    (100, 100, "Forward"),
                    (0, 0, "Stop"),
                    (-100, -100, "Reverse"), 
                    (0, 0, "Stop"),
                    (100, -100, "Right Turn"),
                    (0, 0, "Stop"),
                    (-100, 100, "Left Turn"),
                    (0, 0, "Stop")
                ]
                
                for left, right, desc in test_commands:
                    print(f"  {desc}: L={left:+4d} R={right:+4d}")
                    monitor.send_test_command(left, right)
                    time.sleep(1.5)
                
                print("🧪 Test sequence complete\n")
        
    except KeyboardInterrupt:
        print("\n⏹️ Monitor interrupted by user")
    
    finally:
        monitor.disconnect()

if __name__ == "__main__":
    main()