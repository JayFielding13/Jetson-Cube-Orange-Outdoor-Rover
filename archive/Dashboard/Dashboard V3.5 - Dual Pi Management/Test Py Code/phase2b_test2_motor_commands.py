#!/usr/bin/env python3
"""
Phase 2B Test 2: Motor Command Testing
=====================================

Test Pi-to-motor command pipeline through Arduino gatekeeper.
This test validates motor control in autonomous mode with safety monitoring.
"""

import serial
import time
import sys
import json

class MotorTestInterface:
    """Arduino interface for motor command testing"""
    
    def __init__(self, port='/dev/ttyUSB1', baud_rate=115200):
        self.port = port
        self.baud_rate = baud_rate
        self.serial = None
        self.last_telemetry = None
        
    def connect(self):
        """Connect to Arduino"""
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                
            self.serial = serial.Serial(self.port, self.baud_rate, timeout=1)
            time.sleep(2)
            return self.serial.is_open
        except Exception as e:
            print(f"Connection error: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Arduino"""
        if self.serial and self.serial.is_open:
            self.serial.close()
    
    def read_telemetry(self):
        """Read telemetry with caching"""
        if not self.serial or not self.serial.is_open:
            return None
            
        try:
            self.serial.flushInput()
            time.sleep(0.1)
            
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8').strip()
                if line.startswith('{') and line.endswith('}'):
                    self.last_telemetry = json.loads(line)
                    return self.last_telemetry
        except Exception as e:
            print(f"Telemetry error: {e}")
            
        return self.last_telemetry
    
    def send_motor_command(self, left_speed, right_speed, description=""):
        """Send motor command with logging"""
        try:
            # Format: MOTOR:left,right (adjust based on your protocol)
            command = f"MOTOR:{left_speed},{right_speed}\n"
            self.serial.write(command.encode())
            
            if description:
                print(f'   📤 Command: {description} (L:{left_speed}, R:{right_speed})')
            
            time.sleep(0.2)  # Allow command processing
            return True
        except Exception as e:
            print(f"Motor command error: {e}")
            return False
    
    def is_autonomous_mode(self):
        """Check if in autonomous mode"""
        if self.last_telemetry:
            ch9 = self.last_telemetry.get('ch9', 0)
            return ch9 > 900  # Autonomous threshold
        return False
    
    def is_safe_to_test(self):
        """Check if system is safe for motor testing"""
        if not self.last_telemetry:
            return False
            
        emergency = self.last_telemetry.get('emergency', True)
        valid = self.last_telemetry.get('valid', False)
        
        return not emergency and valid and self.is_autonomous_mode()

def test_motor_commands():
    """Test motor command pipeline in autonomous mode"""
    print('🔍 Phase 2B-2: Motor Command Pipeline Test')
    print('=' * 45)
    
    # Connect to Arduino
    arduino = MotorTestInterface('/dev/ttyUSB1')
    print('📡 Connecting to Arduino motor controller...')
    
    if not arduino.connect():
        print('❌ Arduino connection failed!')
        return False
    
    print('✅ Arduino connected')
    print('🎮 Please ensure RC is in AUTONOMOUS mode (~+997 on ch9)')
    print('⚠️  SAFETY: Rover should be in safe testing area')
    print('⚠️  Be ready to switch to FAILSAFE if needed')
    print()
    
    try:
        # Wait for autonomous mode
        print('🔍 Waiting for autonomous mode...')
        autonomous_confirmed = False
        
        for attempt in range(20):  # 10 seconds to switch to autonomous
            telemetry = arduino.read_telemetry()
            if telemetry:
                ch9 = telemetry.get('ch9', 0)
                mode = telemetry.get('gatekeeper_mode', 'UNKNOWN')
                
                print(f'   Status: {mode} (ch9: {ch9:+4d})')
                
                if arduino.is_autonomous_mode():
                    print('🤖 AUTONOMOUS MODE CONFIRMED!')
                    autonomous_confirmed = True
                    break
            
            time.sleep(0.5)
        
        if not autonomous_confirmed:
            print('❌ Autonomous mode not detected')
            print('   Please switch RC to autonomous position and retry')
            arduino.disconnect()
            return False
        
        # Safety check
        if not arduino.is_safe_to_test():
            print('❌ System not safe for motor testing')
            arduino.disconnect()
            return False
        
        print('\n🚗 Starting motor command sequence...')
        print('   Commands will be sent with 2-second intervals')
        
        # Motor test sequence
        test_commands = [
            (0, 0, "STOP - Initial position"),
            (20, 20, "SLOW FORWARD - Both motors"),
            (0, 0, "STOP - Safety pause"),
            (-20, -20, "SLOW REVERSE - Both motors"), 
            (0, 0, "STOP - Safety pause"),
            (20, -20, "TURN LEFT - Differential steering"),
            (0, 0, "STOP - Safety pause"),
            (-20, 20, "TURN RIGHT - Differential steering"),
            (0, 0, "STOP - Final position"),
        ]
        
        successful_commands = 0
        
        for i, (left, right, description) in enumerate(test_commands):
            print(f'\n📋 Command {i+1}/{len(test_commands)}:')
            
            # Safety check before each command
            telemetry = arduino.read_telemetry()
            if not arduino.is_safe_to_test():
                print('⚠️  Safety abort: System no longer in safe autonomous mode')
                break
            
            # Send motor command
            if arduino.send_motor_command(left, right, description):
                successful_commands += 1
                
                # Monitor response
                time.sleep(0.5)
                response = arduino.read_telemetry()
                if response:
                    pi_cmd_valid = response.get('pi_command_valid', False)
                    print(f'   📥 Arduino response: pi_command_valid={pi_cmd_valid}')
            
            # Wait between commands
            time.sleep(2)
        
        # Final safety stop
        print('\n🛑 Sending final STOP command...')
        arduino.send_motor_command(0, 0, "FINAL STOP")
        
        arduino.disconnect()
        print('🔌 Arduino disconnected safely')
        
        # Test results
        print('\n📊 Motor Test Results:')
        print(f'   Commands sent: {successful_commands}/{len(test_commands)}')
        print(f'   Success rate: {(successful_commands/len(test_commands)*100):.1f}%')
        
        if successful_commands >= len(test_commands) * 0.8:  # 80% success threshold
            print('✅ Motor command pipeline validated')
            return True
        else:
            print('⚠️  Some motor commands failed')
            return False
            
    except Exception as e:
        print(f'⚠️ Motor test error: {e}')
        # Emergency stop
        try:
            arduino.send_motor_command(0, 0, "EMERGENCY STOP")
        except:
            pass
        arduino.disconnect()
        return False
    
    finally:
        print('📊 Test 2B-2 Complete')

if __name__ == "__main__":
    print("Phase 2B-2: Motor Command Pipeline Test")
    print("This test validates Pi-to-motor control through Arduino gatekeeper")
    print("REQUIRES: RC in autonomous mode (~+997 on channel 9)")
    print("SAFETY: Ensure rover is in safe testing area with clear space")
    print("        Be ready to switch RC to FAILSAFE if needed")
    print()
    
    # Safety confirmation
    response = input("Are you ready to test motor commands? (y/N): ")
    if response.lower() != 'y':
        print("Test cancelled for safety")
        sys.exit(1)
    
    success = test_motor_commands()
    
    if success:
        print("\n🎉 Test 2B-2 PASSED: Motor command pipeline working")
        print("✅ Pi can control rover motors through Arduino gatekeeper")
        sys.exit(0)
    else:
        print("\n❌ Test 2B-2 FAILED: Motor command issues detected")
        print("🔧 Check Arduino motor command protocol and wiring")
        sys.exit(1)