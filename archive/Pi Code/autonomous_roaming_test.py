#!/usr/bin/env python3
"""
Autonomous Roaming Test
Simple test script to monitor autonomous roaming behavior
Shows roaming states and ultrasonic sensor data
"""

import serial
import json
import time
import threading

class RoamingMonitor:
    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        self.port = port
        self.baud = baud
        self.serial_conn = None
        self.running = False
        
    def connect(self):
        """Connect to Arduino"""
        try:
            self.serial_conn = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            print(f"✅ Connected to rover on {self.port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to rover: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Arduino"""
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()
            print("📱 Disconnected from rover")
    
    def read_serial_data(self):
        """Read data from rover in background thread"""
        while self.running and self.serial_conn:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode().strip()
                    if line:
                        self.process_rover_message(line)
            except Exception as e:
                print(f"❌ Serial error: {e}")
                time.sleep(0.1)
            time.sleep(0.01)
    
    def process_rover_message(self, message):
        """Process incoming message from rover"""
        try:
            if message.startswith('{') and message.endswith('}'):
                data = json.loads(message)
                
                # Roaming status
                if 'roaming' in data:
                    roaming = data['roaming']
                    state = roaming.get('state', 'UNKNOWN')
                    speed = roaming.get('speed', 0)
                    distance = roaming.get('front_distance', -1)
                    collision = roaming.get('collision_detected', False)
                    
                    # Color-coded state display
                    state_colors = {
                        'FORWARD': '🟢',
                        'TURNING': '🟡',
                        'REVERSE': '🟠',
                        'RANDOM_TURN': '🔵'
                    }
                    color = state_colors.get(state, '⚪')
                    
                    collision_status = '🔴 COLLISION' if collision else '✅ CLEAR'
                    distance_str = f"{distance:.1f}cm" if distance > 0 else "N/A"
                    
                    print(f"{color} {state:12} | Speed: {speed:3d} | Distance: {distance_str:>7} | {collision_status}")
                
                # Mode changes
                elif 'mode_change' in data:
                    mode = data['mode_change']
                    mode_emoji = {'FAILSAFE': '🔴', 'MANUAL': '🟢', 'AUTONOMOUS': '🔵'}
                    emoji = mode_emoji.get(mode, '⚪')
                    print(f"\n{emoji} MODE CHANGED: {mode}\n")
                
                # Sensor data
                elif 'sensors' in data:
                    sensors = data['sensors']
                    if 'front_distance' in sensors:
                        distance = sensors['front_distance']
                        collision = sensors.get('collision_detected', False)
                        
                        if distance > 0:
                            status = "🔴 COLLISION" if collision else "✅ CLEAR"
                            print(f"📡 Sensor: {distance:.1f}cm | {status}")
                
        except json.JSONDecodeError:
            # Non-JSON messages (initialization, etc.)
            if "initialized" in message.lower() or "arduino" in message.lower():
                print(f"🔧 {message}")
    
    def start_monitoring(self):
        """Start background thread for serial monitoring"""
        self.running = True
        self.monitor_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.monitor_thread.start()
        print("🔄 Started roaming monitor")

def main():
    print("=" * 60)
    print("🤖 AUTONOMOUS ROAMING MONITOR")
    print("=" * 60)
    print()
    print("This monitor will display:")
    print("🟢 FORWARD - Moving forward")
    print("🟡 TURNING - Avoiding obstacle")  
    print("🟠 REVERSE - Backing up")
    print("🔵 RANDOM_TURN - Exploring")
    print()
    print("Instructions:")
    print("1. Switch your RC transmitter to AUTONOMOUS mode (position 3)")
    print("2. Watch the rover roam and avoid obstacles")
    print("3. Press Ctrl+C to stop monitoring")
    print()
    
    # Create monitor
    monitor = RoamingMonitor()
    
    # Try to connect
    if not monitor.connect():
        print("❌ Failed to connect to rover. Check connection and try again.")
        return
    
    try:
        # Start monitoring
        monitor.start_monitoring()
        
        print("⏰ Monitoring started. Switch to AUTONOMOUS mode to see roaming behavior...")
        print("   Press Ctrl+C to stop")
        print()
        print("State".ljust(15) + "Speed".ljust(8) + "Distance".ljust(10) + "Collision")
        print("-" * 50)
        
        # Keep running until interrupted
        while True:
            time.sleep(0.1)
        
    except KeyboardInterrupt:
        print("\n⏹️ Monitor stopped by user")
    
    finally:
        monitor.disconnect()

if __name__ == "__main__":
    main()