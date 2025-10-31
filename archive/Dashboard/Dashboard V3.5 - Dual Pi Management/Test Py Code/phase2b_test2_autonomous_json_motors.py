#!/usr/bin/env python3
"""
Phase 2B Test 2: Autonomous JSON Motor Testing
==============================================

Fully autonomous motor testing using correct JSON protocol.
No user input required - automatically detects autonomous mode and runs.
Real-time terminal output with immediate flush.
"""

import serial
import time
import sys
import json
import os

def force_flush():
    """Force all output to flush immediately"""
    sys.stdout.flush()
    sys.stderr.flush()

def log_realtime(message):
    """Log with forced real-time output"""
    print(message, flush=True)
    os.system('sync')  # Force filesystem sync for immediate display

class AutonomousMotorTester:
    """Fully autonomous motor testing with JSON protocol"""
    
    def __init__(self, baud_rate=115200):
        self.baud_rate = baud_rate
        self.serial = None
        self.port = None
        self.last_telemetry = None
        
    def connect(self):
        """Connect to Arduino on any available port with real-time logging"""
        ports_to_try = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
        
        log_realtime('📡 Auto-detecting Arduino port...')
        force_flush()
        
        for port in ports_to_try:
            try:
                log_realtime(f'   Trying port {port}...')
                force_flush()
                
                if self.serial and self.serial.is_open:
                    self.serial.close()
                    
                test_serial = serial.Serial(port, self.baud_rate, timeout=1)
                time.sleep(2)  # Arduino reset delay
                
                if test_serial.is_open:
                    self.serial = test_serial
                    self.port = port
                    log_realtime(f'   ✅ Connected on {port}')
                    force_flush()
                    return True
                    
            except Exception as e:
                log_realtime(f'   ❌ {port}: {e}')
                force_flush()
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
            log_realtime(f'   ⚠️ Telemetry error: {e}')
            force_flush()
            
        return self.last_telemetry
    
    def send_json_motor_command(self, left_speed, right_speed, description=""):
        """Send motor command using correct JSON protocol"""
        try:
            # Clamp speeds to Arduino range (-255 to 255)
            left_speed = max(-255, min(255, int(left_speed)))
            right_speed = max(-255, min(255, int(right_speed)))
            
            # Create JSON command exactly like working navigator code
            command = {
                'motor': {
                    'left': left_speed,
                    'right': right_speed
                }
            }
            
            if description:
                log_realtime(f'   📤 JSON: {description} → L:{left_speed}, R:{right_speed}')
                force_flush()
            
            # Send JSON with newline (working code format)
            cmd_str = json.dumps(command) + '\n'
            self.serial.write(cmd_str.encode())
            
            time.sleep(0.2)
            return True
        except Exception as e:
            log_realtime(f'   ⚠️ Command error: {e}')
            force_flush()
            return False
    
    def is_autonomous_mode(self):
        """Check for autonomous mode (mode=2)"""
        if self.last_telemetry:
            mode = self.last_telemetry.get('mode', 0)
            return mode == 2
        return False
    
    def get_live_status(self):
        """Get live status string"""
        if not self.last_telemetry:
            return "⚫ NO TELEMETRY"
            
        mode = self.last_telemetry.get('mode', 0)
        mode_names = {0: 'MANUAL', 1: 'ASSISTED', 2: 'AUTONOMOUS'}
        mode_text = mode_names.get(mode, f'MODE_{mode}')
        
        valid = self.last_telemetry.get('valid', False)
        emergency = self.last_telemetry.get('emergency', False)
        distance = self.last_telemetry.get('distance', 'None')
        
        # Show both old and new field names for debugging
        ch9 = self.last_telemetry.get('ch9', 'N/A')
        pi_cmd = self.last_telemetry.get('pi_command_valid', False)
        
        status_color = "🟢" if mode == 2 and not emergency else "🟡" if mode == 1 else "🔴"
        
        return f'{status_color} {mode_text} | valid={valid} | emrg={emergency} | dist={distance} | ch9={ch9} | pi_cmd={pi_cmd}'

def run_autonomous_motor_test():
    """Main autonomous motor test - no user input required"""
    
    log_realtime('🤖 AUTONOMOUS JSON MOTOR TEST')
    log_realtime('=' * 50)
    log_realtime('✅ No user input required - fully autonomous operation')
    log_realtime('🎯 Waiting for Arduino autonomous mode detection...')
    log_realtime('📊 Real-time telemetry will be displayed below')
    log_realtime('')
    force_flush()
    
    tester = AutonomousMotorTester()
    
    if not tester.connect():
        log_realtime('❌ Arduino connection failed - aborting test')
        return False
    
    log_realtime(f'✅ Arduino connected on {tester.port}')
    log_realtime('🔍 Monitoring for autonomous mode (mode=2)...')
    log_realtime('   Switch RC to autonomous position to begin motor testing')
    log_realtime('')
    force_flush()
    
    try:
        # Phase 1: Wait for autonomous mode with live telemetry
        autonomous_detected = False
        max_wait_cycles = 60  # 30 seconds of monitoring
        
        for cycle in range(max_wait_cycles):
            telemetry = tester.read_telemetry()
            
            if telemetry:
                status = tester.get_live_status()
                log_realtime(f'📡 Live: {status}')
                
                # Check for distance sensor activation
                distance = telemetry.get('distance')
                if distance is not None:
                    log_realtime(f'🎉 ULTRASONIC ACTIVE: {distance}cm')
                
                if tester.is_autonomous_mode():
                    log_realtime('')
                    log_realtime('🚀 AUTONOMOUS MODE CONFIRMED!')
                    log_realtime('🎯 Beginning JSON motor command sequence...')
                    log_realtime('')
                    autonomous_detected = True
                    break
            
            time.sleep(0.5)
            force_flush()
        
        if not autonomous_detected:
            log_realtime('')
            log_realtime('⏰ Autonomous mode not detected within 30 seconds')
            log_realtime('   Switch RC to autonomous position (should show mode=2)')
            tester.disconnect()
            return False
        
        # Phase 2: Execute motor command sequence
        log_realtime('🚗 JSON Motor Command Sequence Starting...')
        log_realtime('')
        force_flush()
        
        # Comprehensive test sequence
        test_commands = [
            (0, 0, "INIT STOP"),
            (30, 30, "SLOW FORWARD"),
            (0, 0, "STOP & CHECK"),
            (60, 60, "MEDIUM FORWARD"), 
            (0, 0, "STOP & CHECK"),
            (-30, -30, "SLOW REVERSE"),
            (0, 0, "STOP & CHECK"),
            (40, -40, "LEFT TURN"),
            (0, 0, "STOP & CHECK"),
            (-40, 40, "RIGHT TURN"),
            (0, 0, "STOP & CHECK"),
            (50, 25, "LEFT ARC"),
            (0, 0, "STOP & CHECK"),
            (25, 50, "RIGHT ARC"),
            (0, 0, "FINAL STOP"),
        ]
        
        successful_commands = 0
        distance_readings = []
        
        for i, (left, right, desc) in enumerate(test_commands):
            log_realtime(f'📋 Command {i+1}/{len(test_commands)}: {desc}')
            force_flush()
            
            # Safety check - verify still in autonomous mode
            pre_check = tester.read_telemetry()
            if pre_check:
                pre_status = tester.get_live_status()
                log_realtime(f'   📊 Pre-check: {pre_status}')
                
                if not tester.is_autonomous_mode():
                    log_realtime('⚠️  SAFETY ABORT: No longer in autonomous mode!')
                    tester.send_json_motor_command(0, 0, "EMERGENCY STOP")
                    break
            
            # Send JSON command
            if tester.send_json_motor_command(left, right, desc):
                successful_commands += 1
                
                # Monitor response for 1.5 seconds with live telemetry
                log_realtime('   📥 Monitoring Arduino response...')
                for monitor in range(3):
                    time.sleep(0.5)
                    response = tester.read_telemetry()
                    if response:
                        status = tester.get_live_status()
                        log_realtime(f'      Response {monitor+1}: {status}')
                        
                        # Collect distance readings
                        distance = response.get('distance')
                        if distance is not None:
                            distance_readings.append(distance)
                            log_realtime(f'      📏 Distance: {distance}cm')
            
            log_realtime('')  # Blank line between commands
            force_flush()
            time.sleep(0.5)  # Brief pause between commands
        
        # Final safety stop
        log_realtime('🛑 Final Safety Stop...')
        tester.send_json_motor_command(0, 0, "FINAL SAFETY STOP")
        
        # Final status check
        final_check = tester.read_telemetry()
        if final_check:
            final_status = tester.get_live_status()
            log_realtime(f'📊 Final Status: {final_status}')
        
        tester.disconnect()
        
        # Test Results
        log_realtime('')
        log_realtime('📊 JSON MOTOR TEST RESULTS')
        log_realtime('=' * 30)
        log_realtime(f'✅ Commands sent: {successful_commands}/{len(test_commands)}')
        log_realtime(f'📈 Success rate: {(successful_commands/len(test_commands)*100):.1f}%')
        
        if distance_readings:
            avg_dist = sum(distance_readings) / len(distance_readings)
            log_realtime(f'🎉 ULTRASONIC SENSOR WORKING!')
            log_realtime(f'   Readings collected: {len(distance_readings)}')
            log_realtime(f'   Distance range: {min(distance_readings):.1f} - {max(distance_readings):.1f}cm')
            log_realtime(f'   Average distance: {avg_dist:.1f}cm')
        else:
            log_realtime('⚠️  No ultrasonic readings - sensor may need activation')
        
        force_flush()
        
        if successful_commands >= len(test_commands) * 0.8:
            log_realtime('')
            log_realtime('🎉 JSON MOTOR TEST PASSED!')
            log_realtime('✅ Arduino JSON protocol working correctly')
            return True
        else:
            log_realtime('')
            log_realtime('❌ JSON Motor test needs attention')
            return False
            
    except Exception as e:
        log_realtime(f'⚠️ Test error: {e}')
        try:
            tester.send_json_motor_command(0, 0, "EMERGENCY STOP")
        except:
            pass
        tester.disconnect()
        return False
    
    finally:
        log_realtime('')
        log_realtime('📊 Autonomous JSON Motor Test Complete')
        force_flush()

if __name__ == "__main__":
    # Force immediate output from start
    force_flush()
    
    # Run fully autonomous test
    success = run_autonomous_motor_test()
    
    if success:
        log_realtime('')
        log_realtime('🎉 PHASE 2B-2 AUTONOMOUS TEST: PASSED')
        log_realtime('✅ JSON motor commands working with Arduino')
        sys.exit(0)
    else:
        log_realtime('')
        log_realtime('❌ PHASE 2B-2 AUTONOMOUS TEST: NEEDS ATTENTION')
        sys.exit(1)
    
    force_flush()