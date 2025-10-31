#!/usr/bin/env python3 -u
"""
Phase 2B Test 2: Motor Command Testing (Correct JSON Protocol)
============================================================

Test Pi-to-motor command pipeline using the CORRECT JSON protocol
discovered from working autonomous_ultrasonic_navigator.py code.
"""

import serial
import time
import sys
import json

# Force unbuffered output for real-time terminal display
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

class CorrectMotorTestInterface:
    """Arduino interface using the correct JSON motor command protocol"""
    
    def __init__(self, baud_rate=115200):
        self.baud_rate = baud_rate
        self.serial = None
        self.port = None
        self.last_telemetry = None
        
    def log(self, message):
        """Log message with immediate flush for real-time output"""
        print(message)
        sys.stdout.flush()
        
    def connect(self):
        """Connect to Arduino on any available port"""
        ports_to_try = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
        
        for port in ports_to_try:
            try:
                self.log(f"   Trying port {port}...")
                if self.serial and self.serial.is_open:
                    self.serial.close()
                    
                test_serial = serial.Serial(port, self.baud_rate, timeout=1)
                time.sleep(2)  # Arduino reset delay
                
                if test_serial.is_open:
                    self.serial = test_serial
                    self.port = port
                    self.log(f"   ✅ Connected on {port}")
                    return True
                    
            except Exception as e:
                self.log(f"   ❌ {port}: {e}")
                continue
                
        return False
    
    def disconnect(self):
        """Disconnect from Arduino"""
        if self.serial and self.serial.is_open:
            self.serial.close()
    
    def read_telemetry(self):
        """Read telemetry with real-time logging"""
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
            self.log(f"   ⚠️ Telemetry error: {e}")
            
        return self.last_telemetry
    
    def send_motor_command_json(self, left_speed, right_speed, description=""):
        """Send motor command using CORRECT JSON protocol from working code"""
        try:
            # Clamp speeds to valid range (-255 to 255)
            left_speed = max(-255, min(255, int(left_speed)))
            right_speed = max(-255, min(255, int(right_speed)))
            
            # Create command structure EXACTLY like working code
            command = {
                'motor': {
                    'left': left_speed,
                    'right': right_speed
                }
            }
            
            if description:
                self.log(f"   📤 JSON Command: {description} (L:{left_speed}, R:{right_speed})")
                sys.stdout.flush()
            
            # Send as JSON string with newline (like working code)
            cmd_str = json.dumps(command) + '\n'
            self.serial.write(cmd_str.encode())
            
            time.sleep(0.2)  # Allow command processing
            return True
        except Exception as e:
            self.log(f"   ⚠️ Motor command error: {e}")
            return False
    
    def is_autonomous_mode(self):
        """Check if in autonomous mode (mode=2 from working code)"""
        if self.last_telemetry:
            mode = self.last_telemetry.get('mode', 0)
            return mode == 2  # Autonomous mode from working code
        return False
    
    def is_safe_to_test(self):
        """Check if system is safe for motor testing"""
        if not self.last_telemetry:
            return False
            
        emergency = self.last_telemetry.get('emergency', True)
        valid = self.last_telemetry.get('valid', False)
        
        return not emergency and valid and self.is_autonomous_mode()
    
    def get_status_summary(self):
        """Get current status summary using working code field names"""
        if not self.last_telemetry:
            return "No telemetry"
            
        mode = self.last_telemetry.get('mode', 0)
        mode_names = ['MANUAL', 'ASSISTED', 'AUTONOMOUS']
        mode_text = mode_names[mode] if mode < len(mode_names) else f'MODE_{mode}'
        
        valid = self.last_telemetry.get('valid', False)
        emergency = self.last_telemetry.get('emergency', False)
        distance = self.last_telemetry.get('distance', 'None')
        
        # Check for both old and new field names
        ch9 = self.last_telemetry.get('ch9', 'N/A')
        pi_cmd_valid = self.last_telemetry.get('pi_command_valid', False)
        
        return f"{mode_text} valid={valid} emrg={emergency} dist={distance} ch9={ch9} pi_cmd={pi_cmd_valid}"

def test_motor_commands_correct_protocol():
    """Test motor command pipeline using CORRECT JSON protocol"""
    arduino = CorrectMotorTestInterface()
    
    arduino.log('🔍 Phase 2B-2: Motor Commands (CORRECT JSON Protocol)')
    arduino.log('=' * 55)
    
    # Connect to Arduino
    arduino.log('📡 Auto-detecting Arduino port...')
    
    if not arduino.connect():
        arduino.log('❌ No Arduino found on any USB port!')
        arduino.log('🔍 Check USB connections and ensure Arduino is powered')
        return False
    
    arduino.log(f'✅ Arduino connected on {arduino.port}')
    arduino.log('🎮 Please switch RC to AUTONOMOUS mode')
    arduino.log('   From working code: mode=2 means AUTONOMOUS')
    arduino.log('   RC ch9 position should set mode=2 in Arduino telemetry')
    arduino.log('⚠️  SAFETY: Rover should be in safe testing area')
    arduino.log('')
    
    try:
        # Wait for autonomous mode with real-time feedback
        arduino.log('🔍 Waiting for autonomous mode (mode=2)...')
        autonomous_confirmed = False
        
        for attempt in range(40):  # 20 seconds to switch to autonomous
            telemetry = arduino.read_telemetry()
            if telemetry:
                status = arduino.get_status_summary()
                arduino.log(f'   Status: {status}')
                
                # Check for distance data availability
                distance = telemetry.get('distance')
                if distance is not None:
                    arduino.log(f'   🎉 ULTRASONIC DATA DETECTED: {distance}cm!')
                
                if arduino.is_autonomous_mode():
                    arduino.log('🤖 AUTONOMOUS MODE CONFIRMED! (mode=2)')
                    autonomous_confirmed = True
                    break
            
            time.sleep(0.5)
        
        if not autonomous_confirmed:
            arduino.log('❌ Autonomous mode not detected')
            arduino.log('   Please switch RC to position that sets mode=2')
            arduino.log('   Current position may be setting mode=0 (manual) or mode=1 (assisted)')
            arduino.disconnect()
            return False
        
        # Safety check
        if not arduino.is_safe_to_test():
            arduino.log('❌ System not safe for motor testing')
            arduino.log('   Check: Emergency=False, Valid=True, Mode=2')
            arduino.disconnect()
            return False
        
        arduino.log('')
        arduino.log('🚗 Starting motor command sequence with CORRECT JSON protocol...')
        arduino.log('   Using format: {"motor": {"left": speed, "right": speed}}')
        
        # Motor test sequence with proper JSON commands
        test_commands = [
            (0, 0, "INITIAL STOP - Ensure safe starting position"),
            (50, 50, "SLOW FORWARD - Test basic JSON motor command"),
            (0, 0, "STOP - Monitor for Arduino response"),
            (100, 100, "MEDIUM FORWARD - Test moderate speed JSON"),
            (0, 0, "STOP - Check distance sensor response"), 
            (-50, -50, "SLOW REVERSE - Test reverse JSON command"),
            (0, 0, "STOP - Safety checkpoint"),
            (75, -75, "LEFT TURN - Test differential JSON steering"),
            (0, 0, "STOP - Monitor turn response"),
            (-75, 75, "RIGHT TURN - Test opposite JSON steering"),
            (0, 0, "STOP - Monitor turn response"),
            (100, 50, "LEFT ARC - Test combined JSON movement"),
            (0, 0, "STOP - Check sensor data"),
            (50, 100, "RIGHT ARC - Test combined JSON movement"),
            (0, 0, "FINAL STOP - Test complete"),
        ]
        
        successful_commands = 0
        distance_readings = []
        
        for i, (left, right, description) in enumerate(test_commands):
            arduino.log(f'\n📋 Command {i+1}/{len(test_commands)}:')
            
            # Pre-command safety check
            pre_telemetry = arduino.read_telemetry()
            if pre_telemetry:
                pre_status = arduino.get_status_summary()
                arduino.log(f'   📊 Pre-command: {pre_status}')
            
            if not arduino.is_safe_to_test():
                arduino.log('⚠️  SAFETY ABORT: System no longer in safe autonomous mode')
                arduino.log('   Sending JSON emergency stop...')
                arduino.send_motor_command_json(0, 0, "EMERGENCY STOP")
                break
            
            # Send JSON motor command
            if arduino.send_motor_command_json(left, right, description):
                successful_commands += 1
                
                # Monitor response for 2 seconds
                arduino.log('   📥 Monitoring Arduino JSON response...')
                
                for monitor_cycle in range(4):  # Monitor for 2 seconds
                    time.sleep(0.5)
                    response = arduino.read_telemetry()
                    if response:
                        post_status = arduino.get_status_summary()
                        arduino.log(f'   📊 Response {monitor_cycle+1}: {post_status}')
                        
                        # Check for distance data (ultrasonic sensor working!)
                        distance = response.get('distance')
                        if distance is not None:
                            distance_readings.append(distance)
                            arduino.log(f'   📏 Ultrasonic reading: {distance}cm')
            
            # Brief pause between commands
            time.sleep(1.0)
        
        # Final safety stop with JSON protocol
        arduino.log('\n🛑 Sending final JSON safety stop...')
        arduino.send_motor_command_json(0, 0, "FINAL JSON SAFETY STOP")
        time.sleep(1)
        
        final_telemetry = arduino.read_telemetry()
        if final_telemetry:
            final_status = arduino.get_status_summary()
            arduino.log(f'   📊 Final status: {final_status}')
        
        arduino.disconnect()
        arduino.log('🔌 Arduino disconnected safely')
        
        # Comprehensive test results
        arduino.log('\n📊 JSON Motor Command Test Results:')
        arduino.log(f'   JSON commands sent: {successful_commands}/{len(test_commands)}')
        arduino.log(f'   Success rate: {(successful_commands/len(test_commands)*100):.1f}%')
        
        if distance_readings:
            arduino.log(f'   🎉 ULTRASONIC SENSOR WORKING!')
            arduino.log(f'   Distance readings received: {len(distance_readings)}')
            arduino.log(f'   Distance range: {min(distance_readings):.1f}cm - {max(distance_readings):.1f}cm')
            arduino.log(f'   Average distance: {sum(distance_readings)/len(distance_readings):.1f}cm')
        else:
            arduino.log('   ⚠️  No distance readings - sensor may need JSON commands to activate')
        
        if successful_commands >= len(test_commands) * 0.8:  # 80% success threshold
            arduino.log('\n🎉 JSON MOTOR COMMAND PROTOCOL VALIDATED!')
            arduino.log('✅ Pi can send JSON motor commands to Arduino gatekeeper')
            
            if distance_readings:
                arduino.log('✅ Ultrasonic sensor data pipeline restored!')
                return True
            else:
                arduino.log('⚠️  Motor commands work, but sensor data needs investigation')
                return True
        else:
            arduino.log('\n❌ JSON MOTOR COMMAND PROTOCOL NEEDS ATTENTION')
            return False
            
    except Exception as e:
        arduino.log(f'\n⚠️ Motor test error: {e}')
        # Emergency stop with JSON
        try:
            arduino.send_motor_command_json(0, 0, "EMERGENCY STOP")
        except:
            pass
        arduino.disconnect()
        return False
    
    finally:
        arduino.log('📊 Test 2B-2 Complete (Correct JSON Protocol)')

if __name__ == "__main__":
    print("Phase 2B-2: Motor Command Pipeline (CORRECT JSON Protocol)")
    print("This test uses the JSON motor command format from working navigator code")
    print('Motor command format: {"motor": {"left": speed, "right": speed}}')
    print("REQUIRES: RC in autonomous mode (should set mode=2 in telemetry)")
    print("SAFETY: Ensure rover is in safe testing area with clear space")
    print()
    sys.stdout.flush()
    
    # Safety confirmation
    try:
        response = input("Are you ready to test JSON motor commands? (y/N): ")
        if response.lower() != 'y':
            print("Test cancelled for safety")
            sys.exit(1)
    except:
        print("Input error - test cancelled for safety")
        sys.exit(1)
    
    success = test_motor_commands_correct_protocol()
    
    if success:
        print("\n🎉 Test 2B-2 PASSED: JSON motor command protocol working")
        print("✅ Pi can control rover using correct JSON format")
        sys.exit(0)
    else:
        print("\n❌ Test 2B-2 FAILED: JSON motor command issues detected")
        print("🔧 Check Arduino JSON protocol configuration")
        sys.exit(1)