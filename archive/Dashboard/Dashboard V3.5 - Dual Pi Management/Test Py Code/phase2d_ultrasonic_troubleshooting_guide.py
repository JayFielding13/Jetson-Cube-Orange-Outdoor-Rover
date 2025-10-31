#!/usr/bin/env python3
"""
Phase 2D: Ultrasonic Sensor Hardware Troubleshooting Guide
===========================================================

Comprehensive troubleshooting guide for ultrasonic sensor hardware issues.
Based on analysis of rover_arduino_gatekeeper.ino showing sensor timeout on
pulseIn() call returning -1, causing "distance: null" in telemetry.

ISSUE IDENTIFIED:
- Arduino code includes complete ultrasonic implementation
- readUltrasonicDistance() returns -1 (timeout) 
- pulseIn(FRONT_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT) times out
- Hardware wiring or sensor malfunction likely cause

HARDWARE SPECIFICATIONS:
- FRONT_TRIG_PIN: 8 (Arduino digital pin)
- FRONT_ECHO_PIN: 12 (Arduino digital pin)
- ULTRASONIC_TIMEOUT: 5000 microseconds (5ms)
- EMERGENCY_DISTANCE: 10.0cm (safety threshold)

This guide provides systematic hardware diagnostic steps.
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
    os.system('sync')

class UltrasonicHardwareTroubleshooter:
    """Hardware troubleshooting for ultrasonic sensor timeout issues"""
    
    def __init__(self, baud_rate=115200):
        self.baud_rate = baud_rate
        self.serial = None
        self.port = None
        
        # Expected Arduino pin configuration from code analysis
        self.arduino_config = {
            'FRONT_TRIG_PIN': 8,
            'FRONT_ECHO_PIN': 12,
            'ULTRASONIC_TIMEOUT': 5000,  # microseconds
            'EMERGENCY_DISTANCE': 10.0   # cm
        }
        
    def connect(self):
        """Connect to Arduino for hardware diagnostics"""
        ports_to_try = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
        
        log_realtime('📡 Connecting to Arduino for sensor diagnostics...')
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
    
    def read_telemetry_sample(self):
        """Read single telemetry sample for analysis"""
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
            log_realtime(f'   ⚠️ Telemetry error: {e}')
            force_flush()
            
        return None
    
    def analyze_distance_field(self, telemetry):
        """Analyze distance field for diagnostic information"""
        if not telemetry:
            return "NO TELEMETRY", None
            
        distance = telemetry.get('distance')
        
        if distance is None:
            return "MISSING FIELD", None
        elif distance == "null" or distance is None:
            return "NULL VALUE", None
        elif isinstance(distance, (int, float)) and distance < 0:
            return f"NEGATIVE VALUE ({distance})", None
        elif isinstance(distance, (int, float)) and distance >= 0:
            return f"VALID READING", float(distance)
        else:
            return f"UNKNOWN FORMAT ({type(distance).__name__}: {distance})", None

def run_ultrasonic_hardware_troubleshooting():
    """Comprehensive ultrasonic sensor hardware troubleshooting"""
    
    log_realtime('🔧 PHASE 2D: ULTRASONIC SENSOR HARDWARE TROUBLESHOOTING')
    log_realtime('=' * 60)
    log_realtime('🎯 Systematic hardware diagnostic for sensor timeout issues')
    log_realtime('')
    log_realtime('ISSUE ANALYSIS:')
    log_realtime('   Arduino code: readUltrasonicDistance() returns -1 (timeout)')
    log_realtime('   Root cause: pulseIn(FRONT_ECHO_PIN, HIGH, 5000μs) timing out')
    log_realtime('   Result: JSON telemetry shows "distance: null"')
    log_realtime('')
    log_realtime('HARDWARE SPECIFICATIONS:')
    log_realtime('   FRONT_TRIG_PIN: 8 (trigger output)')
    log_realtime('   FRONT_ECHO_PIN: 12 (echo input)')
    log_realtime('   Timeout: 5000μs (5ms maximum pulse width)')
    log_realtime('   Emergency threshold: 10.0cm')
    log_realtime('')
    force_flush()
    
    troubleshooter = UltrasonicHardwareTroubleshooter()
    
    if not troubleshooter.connect():
        log_realtime('❌ Arduino connection failed - cannot diagnose hardware')
        log_realtime('')
        log_realtime('🔧 CONNECTION TROUBLESHOOTING:')
        log_realtime('   1. Check USB cable connection')
        log_realtime('   2. Verify Arduino is powered and running')
        log_realtime('   3. Check /dev/ttyUSB* permissions')
        log_realtime('   4. Try different USB ports')
        return False
    
    log_realtime(f'✅ Arduino connected on {troubleshooter.port}')
    log_realtime('')
    force_flush()
    
    try:
        # Step 1: Baseline telemetry analysis
        log_realtime('📊 STEP 1: Baseline Telemetry Analysis')
        log_realtime('=' * 45)
        log_realtime('🔍 Collecting telemetry samples to confirm timeout issue...')
        log_realtime('')
        force_flush()
        
        timeout_confirmations = 0
        valid_readings = 0
        telemetry_samples = []
        
        for sample_num in range(10):
            telemetry = troubleshooter.read_telemetry_sample()
            
            if telemetry:
                telemetry_samples.append(telemetry)
                status, distance = troubleshooter.analyze_distance_field(telemetry)
                
                log_realtime(f'   Sample {sample_num+1:2d}: {status}')
                if distance is not None:
                    log_realtime(f'             Distance: {distance}cm')
                    valid_readings += 1
                else:
                    timeout_confirmations += 1
                
                # Show full telemetry structure for first few samples
                if sample_num < 3:
                    mode = telemetry.get('mode', 'N/A')
                    emergency = telemetry.get('emergency', 'N/A')
                    valid = telemetry.get('valid', 'N/A')
                    log_realtime(f'             Mode: {mode}, Emergency: {emergency}, Valid: {valid}')
            
            time.sleep(0.5)
            force_flush()
        
        log_realtime('')
        log_realtime('📊 BASELINE ANALYSIS RESULTS:')
        log_realtime(f'   Telemetry samples: {len(telemetry_samples)}')
        log_realtime(f'   Valid distance readings: {valid_readings}')
        log_realtime(f'   Timeout confirmations: {timeout_confirmations}')
        
        if timeout_confirmations > valid_readings:
            log_realtime('   🔴 ISSUE CONFIRMED: Sensor timeouts detected')
        else:
            log_realtime('   🟢 UNEXPECTED: Sensor appears to be working')
        
        log_realtime('')
        force_flush()
        
        # Step 2: Hardware diagnostic checklist
        log_realtime('📋 STEP 2: Hardware Diagnostic Checklist')
        log_realtime('=' * 42)
        log_realtime('🔧 Systematic hardware verification steps:')
        log_realtime('')
        
        checklist = [
            "1. POWER SUPPLY CHECK",
            "   - Verify sensor VCC connected to Arduino 5V or 3.3V",
            "   - Check GND connection between sensor and Arduino",
            "   - Measure voltage at sensor VCC pin (should be 3.3V-5V)",
            "",
            "2. WIRING VERIFICATION", 
            "   - TRIG pin → Arduino Pin 8 (digital output)",
            "   - ECHO pin → Arduino Pin 12 (digital input)",
            "   - Check for loose connections or damaged wires",
            "   - Verify no shorts between TRIG and ECHO pins",
            "",
            "3. SENSOR HARDWARE TEST",
            "   - Try swapping with known good ultrasonic sensor",
            "   - Test sensor with simple Arduino sketch (non-rover code)",
            "   - Check sensor model compatibility (HC-SR04, US-100, etc.)",
            "",
            "4. ARDUINO PIN TEST",
            "   - Verify Pin 8 can output HIGH/LOW signals",
            "   - Verify Pin 12 can read digital input",
            "   - Test with multimeter or oscilloscope if available",
            "",
            "5. TIMING PARAMETER ADJUSTMENT",
            "   - Current timeout: 5000μs (5ms)",
            "   - Try increasing to 30000μs (30ms) in Arduino code",
            "   - Some sensors need longer pulse width detection",
            "",
            "6. ENVIRONMENTAL FACTORS",
            "   - Check for obstacles directly in front of sensor",
            "   - Verify sensor mounting (should face forward clearly)",
            "   - Test in different lighting conditions",
            "   - Check for electrical interference from motors",
        ]
        
        for item in checklist:
            log_realtime(f'   {item}')
        
        log_realtime('')
        force_flush()
        
        # Step 3: Live monitoring during hardware checks
        log_realtime('📡 STEP 3: Live Monitoring During Hardware Checks')
        log_realtime('=' * 48)
        log_realtime('🔍 Monitor telemetry while performing hardware checks...')
        log_realtime('   (Perform physical hardware checks now)')
        log_realtime('')
        force_flush()
        
        log_realtime('Starting 60-second live monitoring session:')
        log_realtime('(Check wiring, power, sensor connections during this time)')
        log_realtime('')
        
        improvements_detected = []
        monitoring_samples = []
        
        for monitor_cycle in range(60):  # 60 seconds of monitoring
            telemetry = troubleshooter.read_telemetry_sample()
            
            if telemetry:
                status, distance = troubleshooter.analyze_distance_field(telemetry)
                monitoring_samples.append((status, distance))
                
                if distance is not None:
                    log_realtime(f'   {monitor_cycle+1:2d}s: 🎉 SENSOR ACTIVE! Distance: {distance}cm')
                    improvements_detected.append(distance)
                else:
                    log_realtime(f'   {monitor_cycle+1:2d}s: {status}')
                
                # Check for mode changes
                mode = telemetry.get('mode', 0)
                if mode != 0:  # Not manual mode
                    mode_names = {1: 'ASSISTED', 2: 'AUTONOMOUS'}
                    log_realtime(f'        Mode change detected: {mode_names.get(mode, f"MODE_{mode}")}')
            
            time.sleep(1.0)
            force_flush()
        
        log_realtime('')
        log_realtime('📊 LIVE MONITORING RESULTS:')
        log_realtime(f'   Monitoring samples: {len(monitoring_samples)}')
        log_realtime(f'   Valid readings detected: {len(improvements_detected)}')
        
        if improvements_detected:
            avg_distance = sum(improvements_detected) / len(improvements_detected)
            log_realtime(f'   🎉 SENSOR RECOVERED!')
            log_realtime(f'   Average distance: {avg_distance:.1f}cm')
            log_realtime(f'   Range: {min(improvements_detected):.1f} - {max(improvements_detected):.1f}cm')
        else:
            log_realtime('   🔴 No sensor recovery detected')
        
        troubleshooter.disconnect()
        
        # Step 4: Recommendations and next steps
        log_realtime('')
        log_realtime('🔧 STEP 4: Troubleshooting Recommendations')
        log_realtime('=' * 42)
        
        if improvements_detected:
            log_realtime('✅ SENSOR RECOVERY DETECTED!')
            log_realtime('')
            log_realtime('SUCCESS RECOMMENDATIONS:')
            log_realtime('   1. Hardware fix appears successful')
            log_realtime('   2. Re-run Phase 2C validation to confirm')
            log_realtime('   3. Monitor for intermittent failures')
            log_realtime('   4. Consider securing loose connections')
            
        else:
            log_realtime('🔴 SENSOR STILL NOT RESPONDING')
            log_realtime('')
            log_realtime('HARDWARE REPLACEMENT RECOMMENDATIONS:')
            log_realtime('   1. Replace ultrasonic sensor with known good unit')
            log_realtime('   2. Compatible sensors: HC-SR04, US-100, JSN-SR04T')
            log_realtime('   3. Verify new sensor wiring matches Pin 8/12 config')
            log_realtime('   4. Test new sensor with simple Arduino sketch first')
            log_realtime('')
            log_realtime('ARDUINO CODE MODIFICATIONS:')
            log_realtime('   1. Increase ULTRASONIC_TIMEOUT from 5000 to 30000μs')
            log_realtime('   2. Add diagnostic Serial.print statements in readUltrasonicDistance()')
            log_realtime('   3. Consider different pulse timing parameters')
            log_realtime('')
            log_realtime('WIRING DOUBLE-CHECK:')
            log_realtime('   Sensor VCC  → Arduino 5V')
            log_realtime('   Sensor GND  → Arduino GND') 
            log_realtime('   Sensor TRIG → Arduino Pin 8')
            log_realtime('   Sensor ECHO → Arduino Pin 12')
        
        log_realtime('')
        force_flush()
        
        return len(improvements_detected) > 0
            
    except Exception as e:
        log_realtime(f'⚠️ Troubleshooting error: {e}')
        troubleshooter.disconnect()
        return False
    
    finally:
        log_realtime('')
        log_realtime('📊 Phase 2D Hardware Troubleshooting Complete')
        force_flush()

def create_arduino_diagnostic_sketch():
    """Generate simple Arduino diagnostic sketch for sensor testing"""
    
    sketch_code = '''/*
 * Ultrasonic Sensor Hardware Diagnostic Sketch
 * ============================================
 * 
 * Simple diagnostic test for ultrasonic sensor hardware
 * Use this to test sensor independently of rover code
 * 
 * Wiring:
 * - Sensor VCC  → Arduino 5V
 * - Sensor GND  → Arduino GND  
 * - Sensor TRIG → Arduino Pin 8
 * - Sensor ECHO → Arduino Pin 12
 */

#define FRONT_TRIG_PIN 8
#define FRONT_ECHO_PIN 12
#define ULTRASONIC_TIMEOUT 30000  // Extended timeout for testing

void setup() {
  Serial.begin(115200);
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  
  Serial.println("Ultrasonic Sensor Diagnostic Test");
  Serial.println("=================================");
  Serial.println("Wiring Check:");
  Serial.println("  TRIG Pin 8 → Sensor TRIG");
  Serial.println("  ECHO Pin 12 → Sensor ECHO");
  Serial.println("  5V → Sensor VCC");
  Serial.println("  GND → Sensor GND");
  Serial.println();
}

void loop() {
  // Send trigger pulse
  digitalWrite(FRONT_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(FRONT_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(FRONT_TRIG_PIN, LOW);
  
  // Read echo
  unsigned long duration = pulseIn(FRONT_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT);
  
  Serial.print("Raw pulseIn result: ");
  Serial.print(duration);
  Serial.print(" μs");
  
  if (duration == 0) {
    Serial.println(" → TIMEOUT (No echo received)");
    Serial.println("  CHECK: Wiring connections");
    Serial.println("  CHECK: Sensor power supply");
    Serial.println("  CHECK: Sensor hardware");
  } else {
    float distance = (duration * 0.0343) / 2;
    Serial.print(" → Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    
    if (distance < 2 || distance > 400) {
      Serial.println("  WARNING: Distance out of normal range");
    }
  }
  
  Serial.println();
  delay(1000);
}'''
    
    return sketch_code

def main():
    """Main troubleshooting guide entry point"""
    
    log_realtime('')
    log_realtime('🔧 ULTRASONIC SENSOR HARDWARE TROUBLESHOOTING GUIDE')
    log_realtime('=' * 60)
    log_realtime('')
    log_realtime('PURPOSE: Diagnose and fix ultrasonic sensor timeout issues')
    log_realtime('ISSUE: Arduino readUltrasonicDistance() returns -1 (timeout)')
    log_realtime('CAUSE: pulseIn() not receiving echo signal within 5ms timeout')
    log_realtime('')
    force_flush()
    
    # Run hardware troubleshooting
    success = run_ultrasonic_hardware_troubleshooting()
    
    # Generate diagnostic Arduino sketch
    log_realtime('')
    log_realtime('📝 ARDUINO DIAGNOSTIC SKETCH')
    log_realtime('=' * 32)
    log_realtime('For independent hardware testing, use this Arduino sketch:')
    log_realtime('')
    force_flush()
    
    sketch_path = '/home/jay/Desktop/Mini Rover Development/Dashboard/Dashboard V3.5 - Dual Pi Management/ultrasonic_diagnostic.ino'
    try:
        with open(sketch_path, 'w') as f:
            f.write(create_arduino_diagnostic_sketch())
        log_realtime(f'✅ Diagnostic sketch saved to: {sketch_path}')
        log_realtime('   Upload this sketch to Arduino for hardware testing')
        log_realtime('   Open Serial Monitor to see sensor readings')
    except Exception as e:
        log_realtime(f'⚠️ Could not save diagnostic sketch: {e}')
    
    log_realtime('')
    log_realtime('🎯 NEXT STEPS:')
    if success:
        log_realtime('   1. ✅ Sensor recovery detected - re-run Phase 2C validation')
        log_realtime('   2. 🔍 Monitor for intermittent failures')  
        log_realtime('   3. 🔧 Secure any loose connections found')
    else:
        log_realtime('   1. 🔧 Follow hardware checklist above')
        log_realtime('   2. 📤 Upload diagnostic sketch for isolated testing')
        log_realtime('   3. 🔄 Replace sensor if hardware checks fail')
        log_realtime('   4. 🔧 Consider increasing Arduino timeout to 30000μs')
    
    log_realtime('')
    force_flush()
    
    return success

if __name__ == "__main__":
    force_flush()
    
    success = main()
    
    if success:
        log_realtime('🎉 PHASE 2D: SENSOR RECOVERY DETECTED!')
        log_realtime('✅ Hardware troubleshooting appears successful')
        sys.exit(0)
    else:
        log_realtime('🔧 PHASE 2D: HARDWARE INTERVENTION REQUIRED')
        log_realtime('⚠️ Follow troubleshooting recommendations above')
        sys.exit(1)
    
    force_flush()