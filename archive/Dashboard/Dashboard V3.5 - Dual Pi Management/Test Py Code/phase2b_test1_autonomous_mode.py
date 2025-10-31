#!/usr/bin/env python3
"""
Phase 2B Test 1: Autonomous Mode Activation
===========================================

Test Arduino gatekeeper autonomous mode activation and Pi command relay.
This test verifies the Pi can detect autonomous mode and send basic commands.
"""

import serial
import time
import sys
import json

class AutonomousArduinoInterface:
    """Arduino interface for autonomous mode testing"""
    
    def __init__(self, port='/dev/ttyUSB1', baud_rate=115200):
        self.port = port
        self.baud_rate = baud_rate
        self.serial = None
        
    def connect(self):
        """Connect to Arduino"""
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                
            self.serial = serial.Serial(self.port, self.baud_rate, timeout=1)
            time.sleep(2)  # Arduino reset delay
            return self.serial.is_open
        except Exception as e:
            print(f"Connection error: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Arduino"""
        if self.serial and self.serial.is_open:
            self.serial.close()
    
    def read_telemetry(self):
        """Read telemetry data from Arduino"""
        if not self.serial or not self.serial.is_open:
            return None
            
        try:
            self.serial.flushInput()
            time.sleep(0.1)
            
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8').strip()
                if line.startswith('{') and line.endswith('}'):
                    return json.loads(line)
        except Exception as e:
            print(f"Telemetry read error: {e}")
            
        return None
    
    def send_motor_command(self, left_speed=0, right_speed=0):
        """Send motor command to Arduino"""
        try:
            # Simple motor command format - adjust based on your protocol
            command = f"MOTOR:{left_speed},{right_speed}\n"
            self.serial.write(command.encode())
            time.sleep(0.1)
            return True
        except Exception as e:
            print(f"Motor command error: {e}")
            return False
    
    def send_test_command(self):
        """Send test command to validate Pi-Arduino communication"""
        try:
            self.serial.write(b"TEST\n")
            time.sleep(0.1)
            return True
        except Exception as e:
            print(f"Test command error: {e}")
            return False

def test_autonomous_mode():
    """Test autonomous mode activation and Pi command relay"""
    print('🔍 Phase 2B-1: Autonomous Mode Activation Test')
    print('=' * 50)
    
    # Connect to Arduino
    arduino = AutonomousArduinoInterface('/dev/ttyUSB1')
    print('📡 Connecting to Arduino gatekeeper...')
    
    if not arduino.connect():
        print('❌ Arduino connection failed!')
        return False
    
    print('✅ Arduino gatekeeper connected')
    print('🎮 Waiting for RC mode switch...')
    print('   Please switch your RC to different modes and observe:')
    print('   • FAILSAFE mode: ~-992 on ch9')
    print('   • MANUAL mode: ~0 to 8 on ch9') 
    print('   • AUTONOMOUS mode: ~+997 on ch9')
    print()
    
    try:
        mode_history = []
        autonomous_detected = False
        test_duration = 30  # 30 seconds of monitoring
        
        print(f'🔍 Monitoring for {test_duration} seconds...')
        
        for i in range(test_duration * 2):  # 2 readings per second
            telemetry = arduino.read_telemetry()
            
            if telemetry:
                ch9 = telemetry.get('ch9', 0)
                mode = telemetry.get('gatekeeper_mode', 'UNKNOWN')
                valid = telemetry.get('valid', False)
                pi_cmd_valid = telemetry.get('pi_command_valid', False)
                
                # Detect mode changes
                current_state = (mode, ch9)
                if not mode_history or mode_history[-1] != current_state:
                    print(f'📊 Mode Change: {mode} (ch9: {ch9:+4d}) valid={valid} pi_cmd={pi_cmd_valid}')
                    mode_history.append(current_state)
                
                # Check for autonomous mode
                if ch9 > 900:  # Autonomous mode threshold
                    if not autonomous_detected:
                        print('🤖 AUTONOMOUS MODE DETECTED!')
                        print('   Testing Pi command relay...')
                        autonomous_detected = True
                        
                        # Send test command
                        if arduino.send_test_command():
                            print('   ✅ Test command sent')
                        
                        # Wait a moment then send gentle motor test
                        time.sleep(1)
                        if arduino.send_motor_command(10, 10):  # Very slow forward
                            print('   ✅ Motor command sent (10, 10)')
                        
                        time.sleep(1)
                        if arduino.send_motor_command(0, 0):  # Stop
                            print('   ✅ Stop command sent (0, 0)')
            
            time.sleep(0.5)
        
        arduino.disconnect()
        print('🔌 Arduino disconnected safely')
        
        # Test results
        print('\n📊 Test Summary:')
        print(f'   Mode changes detected: {len(mode_history)}')
        for mode, ch9 in mode_history:
            print(f'   • {mode}: ch9={ch9:+4d}')
        
        if autonomous_detected:
            print('✅ Autonomous mode successfully detected and tested')
            return True
        else:
            print('⚠️  Autonomous mode not detected during test')
            print('   Please verify RC transmitter is set to autonomous position')
            return False
            
    except Exception as e:
        print(f'⚠️ Test error: {e}')
        arduino.disconnect()
        return False
    
    finally:
        print('📊 Test 2B-1 Complete')

if __name__ == "__main__":
    print("Phase 2B-1: Autonomous Mode Activation Test")
    print("This test monitors RC mode switching and tests Pi command relay")
    print("Make sure your RC transmitter is on and switch between modes")
    print("SAFETY: Ensure rover is in safe testing area with clear space")
    print()
    
    success = test_autonomous_mode()
    
    if success:
        print("\n🎉 Test 2B-1 PASSED: Autonomous mode validated")
        print("✅ Ready for motor control testing")
        sys.exit(0)
    else:
        print("\n❌ Test 2B-1 PARTIAL: Mode detection completed")
        print("🔧 Verify RC autonomous mode switch position")
        sys.exit(1)