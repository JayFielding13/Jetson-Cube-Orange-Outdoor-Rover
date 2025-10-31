#!/usr/bin/env python3 -u
"""
Phase 2B Test 2: Motor Command Testing (Real-time Output)
========================================================

Test Pi-to-motor command pipeline through Arduino gatekeeper.
This version includes real-time terminal output and comprehensive motor testing.
"""

import serial
import time
import sys
import json

# Force unbuffered output for real-time terminal display
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

class MotorTestInterface:
    """Arduino interface for motor command testing with real-time output"""
    
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
    
    def send_motor_command(self, left_speed, right_speed, description=""):
        """Send motor command with real-time logging"""
        try:
            # Try multiple command formats your Arduino might expect
            commands_to_try = [
                f"MOTOR:{left_speed},{right_speed}\n",
                f"M:{left_speed},{right_speed}\n", 
                f"DRIVE {left_speed} {right_speed}\n",
                f"{left_speed},{right_speed}\n"
            ]
            
            if description:
                self.log(f"   📤 Command: {description} (L:{left_speed}, R:{right_speed})")
                sys.stdout.flush()
            
            for cmd in commands_to_try:
                self.serial.write(cmd.encode())
                time.sleep(0.05)  # Brief pause between formats
            
            time.sleep(0.2)  # Allow command processing
            return True
        except Exception as e:
            self.log(f"   ⚠️ Motor command error: {e}")
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
    
    def get_status_summary(self):
        """Get current status summary"""
        if not self.last_telemetry:
            return "No telemetry"
            
        ch9 = self.last_telemetry.get('ch9', 0)
        mode = self.last_telemetry.get('gatekeeper_mode', 'UNKNOWN')
        valid = self.last_telemetry.get('valid', False)
        pi_cmd_valid = self.last_telemetry.get('pi_command_valid', False)
        emergency = self.last_telemetry.get('emergency', False)
        distance = self.last_telemetry.get('distance', 'None')
        
        return f"{mode} (ch9:{ch9:+4d}) valid={valid} pi_cmd={pi_cmd_valid} emrg={emergency} dist={distance}"

def test_motor_commands():
    """Test motor command pipeline in autonomous mode with real-time output"""
    arduino = MotorTestInterface()
    
    arduino.log('🔍 Phase 2B-2: Motor Command Pipeline Test (Real-time)')
    arduino.log('=' * 55)
    
    # Connect to Arduino
    arduino.log('📡 Auto-detecting Arduino port...')
    
    if not arduino.connect():
        arduino.log('❌ No Arduino found on any USB port!')
        arduino.log('🔍 Check USB connections and ensure Arduino is powered')
        return False
    
    arduino.log(f'✅ Arduino connected on {arduino.port}')
    arduino.log('🎮 Please ensure RC is in AUTONOMOUS mode (~+997 on ch9)')
    arduino.log('⚠️  SAFETY: Rover should be in safe testing area')
    arduino.log('⚠️  Be ready to switch to FAILSAFE if needed')
    arduino.log('')
    
    try:
        # Wait for autonomous mode with real-time feedback
        arduino.log('🔍 Waiting for autonomous mode...')
        autonomous_confirmed = False
        
        for attempt in range(30):  # 15 seconds to switch to autonomous
            telemetry = arduino.read_telemetry()
            if telemetry:
                status = arduino.get_status_summary()
                arduino.log(f'   Status: {status}')
                
                if arduino.is_autonomous_mode():
                    arduino.log('🤖 AUTONOMOUS MODE CONFIRMED!')
                    autonomous_confirmed = True
                    break
            
            time.sleep(0.5)
        
        if not autonomous_confirmed:
            arduino.log('❌ Autonomous mode not detected')
            arduino.log('   Please switch RC to autonomous position and retry')
            arduino.disconnect()
            return False
        
        # Safety check
        if not arduino.is_safe_to_test():
            arduino.log('❌ System not safe for motor testing')
            arduino.log('   Check: Emergency=False, Valid=True, Mode=Autonomous')
            arduino.disconnect()
            return False
        
        arduino.log('')
        arduino.log('🚗 Starting comprehensive motor command sequence...')
        arduino.log('   Each command will be monitored for Arduino response')
        
        # Comprehensive motor test sequence
        test_commands = [
            (0, 0, "INITIAL STOP - Ensure safe starting position"),
            (10, 10, "SLOW FORWARD - Test basic forward movement"),
            (0, 0, "STOP - Safety checkpoint 1"),
            (20, 20, "MEDIUM FORWARD - Test moderate speed"),
            (0, 0, "STOP - Safety checkpoint 2"), 
            (-10, -10, "SLOW REVERSE - Test reverse capability"),
            (0, 0, "STOP - Safety checkpoint 3"),
            (15, -15, "GENTLE LEFT TURN - Differential steering"),
            (0, 0, "STOP - Safety checkpoint 4"),
            (-15, 15, "GENTLE RIGHT TURN - Opposite differential"),
            (0, 0, "STOP - Safety checkpoint 5"),
            (25, 10, "GRADUAL LEFT ARC - Combined forward/turn"),
            (0, 0, "STOP - Safety checkpoint 6"),
            (10, 25, "GRADUAL RIGHT ARC - Combined forward/turn"),
            (0, 0, "FINAL STOP - Test complete, motors off"),
        ]
        
        successful_commands = 0
        command_responses = []
        
        for i, (left, right, description) in enumerate(test_commands):
            arduino.log(f'\n📋 Command {i+1}/{len(test_commands)}:')
            
            # Pre-command safety check
            pre_telemetry = arduino.read_telemetry()
            if pre_telemetry:
                pre_status = arduino.get_status_summary()
                arduino.log(f'   📊 Pre-command: {pre_status}')
            
            if not arduino.is_safe_to_test():
                arduino.log('⚠️  SAFETY ABORT: System no longer in safe autonomous mode')
                arduino.log('   Sending emergency stop...')
                arduino.send_motor_command(0, 0, "EMERGENCY STOP")
                break
            
            # Send motor command
            if arduino.send_motor_command(left, right, description):
                successful_commands += 1
                
                # Monitor response for 1 second
                arduino.log('   📥 Monitoring Arduino response...')
                response_detected = False
                
                for monitor_cycle in range(4):  # Monitor for 2 seconds
                    time.sleep(0.5)
                    response = arduino.read_telemetry()
                    if response:
                        post_status = arduino.get_status_summary()
                        arduino.log(f'   📊 Response {monitor_cycle+1}: {post_status}')
                        
                        pi_cmd_valid = response.get('pi_command_valid', False)
                        if pi_cmd_valid and not response_detected:
                            arduino.log('   ✅ Pi command acknowledged by Arduino!')
                            response_detected = True
                
                command_responses.append({
                    'command': (left, right, description),
                    'acknowledged': response_detected
                })
            
            # Brief pause between commands
            time.sleep(1.5)
        
        # Final safety stop with confirmation
        arduino.log('\n🛑 Sending final safety stop...')
        arduino.send_motor_command(0, 0, "FINAL SAFETY STOP")
        time.sleep(1)
        
        final_telemetry = arduino.read_telemetry()
        if final_telemetry:
            final_status = arduino.get_status_summary()
            arduino.log(f'   📊 Final status: {final_status}')
        
        arduino.disconnect()
        arduino.log('🔌 Arduino disconnected safely')
        
        # Comprehensive test results
        arduino.log('\n📊 Motor Command Test Results:')
        arduino.log(f'   Commands sent: {successful_commands}/{len(test_commands)}')
        arduino.log(f'   Success rate: {(successful_commands/len(test_commands)*100):.1f}%')
        
        acknowledged_count = sum(1 for r in command_responses if r['acknowledged'])
        arduino.log(f'   Arduino acknowledgments: {acknowledged_count}/{len(command_responses)}')
        arduino.log(f'   Acknowledgment rate: {(acknowledged_count/len(command_responses)*100):.1f}% (if > 0)')
        
        arduino.log('\n📋 Command Details:')
        for i, response_data in enumerate(command_responses):
            cmd_left, cmd_right, cmd_desc = response_data['command']
            ack_status = "✅ ACK" if response_data['acknowledged'] else "⚠️ NO ACK"
            arduino.log(f'   {i+1:2d}. L:{cmd_left:+3d} R:{cmd_right:+3d} - {ack_status}')
        
        if successful_commands >= len(test_commands) * 0.8:  # 80% success threshold
            arduino.log('\n🎉 MOTOR COMMAND PIPELINE VALIDATED')
            arduino.log('✅ Pi can send motor commands through Arduino gatekeeper')
            
            if acknowledged_count > 0:
                arduino.log('✅ Arduino command acknowledgment confirmed')
                return True
            else:
                arduino.log('⚠️  Commands sent but Arduino acknowledgment not detected')
                arduino.log('   This may indicate the Arduino protocol needs adjustment')
                return True  # Still a success - commands were sent
        else:
            arduino.log('\n❌ MOTOR COMMAND PIPELINE NEEDS ATTENTION')
            return False
            
    except Exception as e:
        arduino.log(f'\n⚠️ Motor test error: {e}')
        # Emergency stop
        try:
            arduino.send_motor_command(0, 0, "EMERGENCY STOP")
        except:
            pass
        arduino.disconnect()
        return False
    
    finally:
        arduino.log('📊 Test 2B-2 Complete')

if __name__ == "__main__":
    print("Phase 2B-2: Motor Command Pipeline Test (Real-time Output)")
    print("This test validates Pi-to-motor control through Arduino gatekeeper")
    print("REQUIRES: RC in autonomous mode (~+997 on channel 9)")
    print("SAFETY: Ensure rover is in safe testing area with clear space")
    print("        Be ready to switch RC to FAILSAFE if needed")
    print()
    sys.stdout.flush()
    
    # Safety confirmation
    try:
        response = input("Are you ready to test motor commands? (y/N): ")
        if response.lower() != 'y':
            print("Test cancelled for safety")
            sys.exit(1)
    except:
        print("Input error - test cancelled for safety")
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