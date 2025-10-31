#!/usr/bin/env python3
"""
Phase 2C: Ultrasonic Sensor Validation Test
==========================================

Comprehensive ultrasonic sensor testing using proven protocols from
stable_bluetooth_ranging_v1.py working code. Tests sensor hardware,
Arduino communication, and data pipeline validation.
"""

import serial
import time
import sys
import json
import os
import statistics

def force_flush():
    """Force all output to flush immediately"""
    sys.stdout.flush()
    sys.stderr.flush()

def log_realtime(message):
    """Log with forced real-time output"""
    print(message, flush=True)
    os.system('sync')

class UltrasonicSensorValidator:
    """Comprehensive ultrasonic sensor validation using proven protocols"""
    
    def __init__(self, baud_rate=115200):
        self.baud_rate = baud_rate
        self.serial = None
        self.port = None
        self.telemetry_history = []
        
    def connect(self):
        """Connect to Arduino with sensor validation"""
        ports_to_try = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
        
        log_realtime('📡 Connecting to Arduino for sensor validation...')
        force_flush()
        
        for port in ports_to_try:
            try:
                log_realtime(f'   Testing port {port}...')
                force_flush()
                
                if self.serial and self.serial.is_open:
                    self.serial.close()
                    
                test_serial = serial.Serial(port, self.baud_rate, timeout=1)
                time.sleep(2)
                
                if test_serial.is_open:
                    self.serial = test_serial
                    self.port = port
                    log_realtime(f'   ✅ Arduino connected on {port}')
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
        """Read Arduino telemetry using proven protocol"""
        if not self.serial or not self.serial.is_open:
            return None
            
        try:
            self.serial.flushInput()
            time.sleep(0.1)
            
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8').strip()
                if line.startswith('{') and line.endswith('}'):
                    telemetry = json.loads(line)
                    self.telemetry_history.append(telemetry)
                    return telemetry
        except Exception as e:
            log_realtime(f'   ⚠️ Telemetry error: {e}')
            force_flush()
            
        return None
    
    def send_json_motor_command(self, left_speed, right_speed):
        """Send motor command to potentially activate sensor readings"""
        try:
            left_speed = max(-255, min(255, int(left_speed)))
            right_speed = max(-255, min(255, int(right_speed)))
            
            command = {
                'motor': {
                    'left': left_speed,
                    'right': right_speed
                }
            }
            
            cmd_str = json.dumps(command) + '\n'
            self.serial.write(cmd_str.encode())
            time.sleep(0.1)
            return True
        except Exception as e:
            log_realtime(f'   ⚠️ Motor command error: {e}')
            force_flush()
            return False
    
    def get_distance_from_telemetry(self, telemetry):
        """Extract distance using proven protocol from stable code"""
        if not telemetry:
            return None
            
        # Use exact same extraction as working code
        distance = telemetry.get('distance')
        if distance is None or distance < 0:
            return None
        return float(distance)
    
    def analyze_telemetry_sample(self, telemetry):
        """Analyze telemetry for sensor diagnostics"""
        if not telemetry:
            return "❌ NO TELEMETRY"
            
        # Basic telemetry info
        mode = telemetry.get('mode', 0)
        mode_names = {0: 'MANUAL', 1: 'ASSISTED', 2: 'AUTONOMOUS'}
        mode_text = mode_names.get(mode, f'MODE_{mode}')
        
        valid = telemetry.get('valid', False)
        emergency = telemetry.get('emergency', False)
        
        # Sensor-specific analysis
        distance = self.get_distance_from_telemetry(telemetry)
        
        # Check all possible distance fields for debugging
        alt_fields = []
        for key in telemetry.keys():
            if 'dist' in key.lower() or 'sensor' in key.lower() or 'ultrasonic' in key.lower():
                alt_fields.append(f"{key}={telemetry[key]}")
        
        status = f"🔵 {mode_text} | valid={valid} | emrg={emergency}"
        
        if distance is not None:
            status += f" | 🎉 DISTANCE={distance}cm"
        else:
            status += f" | ❌ distance=None"
            if alt_fields:
                status += f" | Alt: {', '.join(alt_fields)}"
        
        # Additional diagnostic fields
        ch9 = telemetry.get('ch9', 'N/A')
        pi_cmd = telemetry.get('pi_command_valid', False)
        status += f" | ch9={ch9} | pi_cmd={pi_cmd}"
        
        return status

def run_ultrasonic_sensor_validation():
    """Comprehensive ultrasonic sensor validation test"""
    
    log_realtime('🔬 PHASE 2C: ULTRASONIC SENSOR VALIDATION')
    log_realtime('=' * 50)
    log_realtime('📊 Using proven protocols from stable_bluetooth_ranging_v1.py')
    log_realtime('🎯 Testing hardware, communication, and data pipeline')
    log_realtime('')
    force_flush()
    
    validator = UltrasonicSensorValidator()
    
    if not validator.connect():
        log_realtime('❌ Arduino connection failed - cannot test sensor')
        return False
    
    log_realtime(f'✅ Connected to Arduino on {validator.port}')
    log_realtime('')
    force_flush()
    
    try:
        # Phase 1: Baseline telemetry monitoring
        log_realtime('📊 PHASE 1: Baseline Telemetry Analysis')
        log_realtime('🔍 Monitoring Arduino data stream for 15 seconds...')
        log_realtime('')
        force_flush()
        
        distance_readings = []
        telemetry_samples = []
        
        for cycle in range(30):  # 15 seconds at 0.5s intervals
            telemetry = validator.read_telemetry()
            
            if telemetry:
                status = validator.analyze_telemetry_sample(telemetry)
                log_realtime(f'   Sample {cycle+1:2d}: {status}')
                
                telemetry_samples.append(telemetry)
                
                # Collect distance readings
                distance = validator.get_distance_from_telemetry(telemetry)
                if distance is not None:
                    distance_readings.append(distance)
                    log_realtime(f'        📏 Valid reading: {distance}cm')
            
            time.sleep(0.5)
            force_flush()
        
        # Phase 1 Analysis
        log_realtime('')
        log_realtime('📊 PHASE 1 ANALYSIS:')
        log_realtime(f'   Telemetry samples: {len(telemetry_samples)}')
        log_realtime(f'   Distance readings: {len(distance_readings)}')
        
        if distance_readings:
            avg_distance = statistics.mean(distance_readings)
            log_realtime(f'   🎉 SENSOR WORKING!')
            log_realtime(f'   Average distance: {avg_distance:.1f}cm')
            log_realtime(f'   Range: {min(distance_readings):.1f} - {max(distance_readings):.1f}cm')
        else:
            log_realtime(f'   ❌ No distance readings detected')
        
        log_realtime('')
        force_flush()
        
        # Phase 2: Motor Command Activation Test
        log_realtime('📊 PHASE 2: Motor Command Activation Test')
        log_realtime('🎯 Testing if motor commands activate sensor readings...')
        log_realtime('')
        force_flush()
        
        # Try sending gentle motor commands to see if sensor activates
        test_commands = [
            (0, 0, "STOP"),
            (20, 20, "GENTLE FORWARD"),
            (0, 0, "STOP"),
            (-20, -20, "GENTLE REVERSE"),
            (0, 0, "FINAL STOP")
        ]
        
        motor_test_readings = []
        
        for i, (left, right, desc) in enumerate(test_commands):
            log_realtime(f'   Motor test {i+1}/{len(test_commands)}: {desc}')
            
            # Send motor command
            if validator.send_json_motor_command(left, right):
                log_realtime(f'     📤 JSON: L:{left}, R:{right}')
                
                # Monitor for response for 2 seconds
                for monitor in range(4):
                    time.sleep(0.5)
                    telemetry = validator.read_telemetry()
                    if telemetry:
                        distance = validator.get_distance_from_telemetry(telemetry)
                        status = validator.analyze_telemetry_sample(telemetry)
                        log_realtime(f'     📥 Response: {status}')
                        
                        if distance is not None:
                            motor_test_readings.append(distance)
                            log_realtime(f'     🎉 SENSOR ACTIVE: {distance}cm')
            
            force_flush()
        
        # Phase 2 Analysis
        log_realtime('')
        log_realtime('📊 PHASE 2 ANALYSIS:')
        log_realtime(f'   Motor command readings: {len(motor_test_readings)}')
        
        if motor_test_readings:
            log_realtime(f'   🎉 Motor commands activate sensor!')
            avg_motor_distance = statistics.mean(motor_test_readings)
            log_realtime(f'   Average during motor test: {avg_motor_distance:.1f}cm')
        else:
            log_realtime(f'   ❌ Motor commands did not activate sensor')
        
        log_realtime('')
        force_flush()
        
        # Phase 3: Autonomous Mode Test
        log_realtime('📊 PHASE 3: Autonomous Mode Sensor Test')
        log_realtime('🔍 Switch RC to autonomous mode for sensor testing...')
        log_realtime('')
        force_flush()
        
        autonomous_readings = []
        autonomous_detected = False
        
        for cycle in range(40):  # 20 seconds to switch and test
            telemetry = validator.read_telemetry()
            
            if telemetry:
                mode = telemetry.get('mode', 0)
                distance = validator.get_distance_from_telemetry(telemetry)
                status = validator.analyze_telemetry_sample(telemetry)
                
                if mode == 2:  # Autonomous mode
                    if not autonomous_detected:
                        log_realtime('   🤖 AUTONOMOUS MODE DETECTED!')
                        autonomous_detected = True
                    
                    log_realtime(f'   Auto {cycle+1:2d}: {status}')
                    
                    if distance is not None:
                        autonomous_readings.append(distance)
                        log_realtime(f'           📏 Autonomous reading: {distance}cm')
                else:
                    log_realtime(f'   Wait {cycle+1:2d}: {status}')
            
            time.sleep(0.5)
            force_flush()
        
        # Phase 3 Analysis  
        log_realtime('')
        log_realtime('📊 PHASE 3 ANALYSIS:')
        log_realtime(f'   Autonomous mode: {"✅ Detected" if autonomous_detected else "❌ Not detected"}')
        log_realtime(f'   Autonomous readings: {len(autonomous_readings)}')
        
        if autonomous_readings:
            log_realtime(f'   🎉 Sensor works in autonomous mode!')
            avg_auto_distance = statistics.mean(autonomous_readings)
            log_realtime(f'   Average in autonomous: {avg_auto_distance:.1f}cm')
        
        validator.disconnect()
        
        # Final Results
        log_realtime('')
        log_realtime('📊 ULTRASONIC SENSOR VALIDATION RESULTS')
        log_realtime('=' * 45)
        
        total_readings = len(distance_readings) + len(motor_test_readings) + len(autonomous_readings)
        log_realtime(f'✅ Total sensor readings: {total_readings}')
        log_realtime(f'   Baseline readings: {len(distance_readings)}')
        log_realtime(f'   Motor test readings: {len(motor_test_readings)}')
        log_realtime(f'   Autonomous readings: {len(autonomous_readings)}')
        
        if total_readings > 0:
            all_readings = distance_readings + motor_test_readings + autonomous_readings
            overall_avg = statistics.mean(all_readings)
            log_realtime(f'🎉 ULTRASONIC SENSOR: WORKING!')
            log_realtime(f'   Overall average: {overall_avg:.1f}cm')
            log_realtime(f'   Overall range: {min(all_readings):.1f} - {max(all_readings):.1f}cm')
            return True
        else:
            log_realtime('❌ ULTRASONIC SENSOR: NOT RESPONDING')
            log_realtime('🔧 Troubleshooting suggestions:')
            log_realtime('   1. Check sensor wiring to Arduino')
            log_realtime('   2. Verify Arduino code includes sensor reading')
            log_realtime('   3. Test sensor hardware directly')
            log_realtime('   4. Check Arduino pin configuration')
            return False
            
    except Exception as e:
        log_realtime(f'⚠️ Test error: {e}')
        validator.disconnect()
        return False
    
    finally:
        log_realtime('')
        log_realtime('📊 Phase 2C Ultrasonic Validation Complete')
        force_flush()

if __name__ == "__main__":
    force_flush()
    
    success = run_ultrasonic_sensor_validation()
    
    if success:
        log_realtime('')
        log_realtime('🎉 PHASE 2C PASSED: Ultrasonic sensor validated!')
        log_realtime('✅ Sensor hardware and communication working')
        sys.exit(0)
    else:
        log_realtime('')
        log_realtime('❌ PHASE 2C NEEDS ATTENTION: Sensor issues detected')
        log_realtime('🔧 Hardware or configuration troubleshooting required')
        sys.exit(1)