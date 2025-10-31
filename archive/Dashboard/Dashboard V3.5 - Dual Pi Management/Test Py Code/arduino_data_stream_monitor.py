#!/usr/bin/env python3
"""
Real-Time Arduino Data Stream Monitor
====================================

Live monitoring of Arduino telemetry stream with real-time terminal output.
Perfect for watching ultrasonic sensor readings while positioning the rover.

Features:
- Immediate terminal output (no buffering issues)
- Live distance readings with visual indicators
- Mode detection and status monitoring
- Timestamp logging for troubleshooting
- Emergency stop detection
"""

import serial
import time
import sys
import json
import os
from datetime import datetime

def force_flush():
    """Force all output to flush immediately"""
    sys.stdout.flush()
    sys.stderr.flush()

def log_realtime(message):
    """Log with forced real-time output and timestamp"""
    timestamp = datetime.now().strftime("%H:%M:%S")
    print(f"[{timestamp}] {message}", flush=True)
    os.system('sync')

class ArduinoDataStreamMonitor:
    """Real-time Arduino data stream monitor with live terminal output"""
    
    def __init__(self, baud_rate=115200):
        self.baud_rate = baud_rate
        self.serial = None
        self.port = None
        self.total_readings = 0
        self.distance_readings = []
        self.null_readings = 0
        self.start_time = time.time()
        
    def connect(self):
        """Connect to Arduino with auto-detection"""
        ports_to_try = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
        
        log_realtime('🔌 Auto-detecting Arduino connection...')
        force_flush()
        
        for port in ports_to_try:
            try:
                log_realtime(f'   Testing {port}...')
                force_flush()
                
                if self.serial and self.serial.is_open:
                    self.serial.close()
                    
                test_serial = serial.Serial(port, self.baud_rate, timeout=1)
                time.sleep(2)
                
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
    
    def format_distance_reading(self, distance, telemetry):
        """Format distance reading with visual indicators"""
        if distance is None or distance == "null":
            self.null_readings += 1
            return "❌ NULL (no echo/timeout)"
        
        try:
            dist_val = float(distance)
            self.distance_readings.append(dist_val)
            
            # Visual distance indicators
            if dist_val < 10:
                indicator = "🔴 VERY CLOSE"
            elif dist_val < 30:
                indicator = "🟠 CLOSE"
            elif dist_val < 100:
                indicator = "🟡 MEDIUM"
            elif dist_val < 200:
                indicator = "🟢 FAR"
            else:
                indicator = "🔵 MAX RANGE"
            
            # Check for emergency stop condition
            emergency = telemetry.get('emergency', False)
            if emergency:
                indicator += " ⚠️ EMERGENCY!"
            
            return f"📏 {dist_val:6.1f}cm {indicator}"
            
        except (ValueError, TypeError):
            return f"❓ UNKNOWN FORMAT: {distance}"
    
    def format_mode_status(self, telemetry):
        """Format mode and status information"""
        mode = telemetry.get('mode', 0)
        mode_names = {0: 'MANUAL', 1: 'ASSISTED', 2: 'AUTONOMOUS'}
        mode_text = mode_names.get(mode, f'MODE_{mode}')
        
        valid = telemetry.get('valid', False)
        emergency = telemetry.get('emergency', False)
        
        # Color coding for modes
        if mode == 2:  # Autonomous
            mode_indicator = "🤖"
        elif mode == 1:  # Assisted
            mode_indicator = "🔧"
        else:  # Manual
            mode_indicator = "🎮"
        
        status = f"{mode_indicator} {mode_text}"
        
        if not valid:
            status += " ⚠️ INVALID"
        if emergency:
            status += " 🚨 EMERGENCY"
        
        # Additional useful fields
        ch9 = telemetry.get('ch9', 'N/A')
        pi_cmd = telemetry.get('pi_command_valid', False)
        
        return f"{status} | CH9:{ch9} | PI:{pi_cmd}"
    
    def monitor_data_stream(self, duration_seconds=300):
        """Monitor Arduino data stream in real-time"""
        log_realtime(f'📡 Starting real-time data stream monitoring...')
        log_realtime(f'⏱️  Duration: {duration_seconds} seconds (Ctrl+C to stop early)')
        log_realtime('')
        log_realtime('🎯 MOVE ROVER CLOSE TO WALL to test distance readings!')
        log_realtime('📏 Watch for distance values less than 200cm')
        log_realtime('')
        log_realtime('Live Data Stream:')
        log_realtime('=' * 60)
        force_flush()
        
        end_time = time.time() + duration_seconds
        last_stats_time = time.time()
        
        try:
            while time.time() < end_time:
                try:
                    # Read raw data from Arduino
                    if self.serial.in_waiting > 0:
                        line = self.serial.readline().decode('utf-8').strip()
                        
                        # Process JSON telemetry
                        if line.startswith('{') and line.endswith('}'):
                            try:
                                telemetry = json.loads(line)
                                self.total_readings += 1
                                
                                # Format distance reading
                                distance = telemetry.get('distance')
                                distance_str = self.format_distance_reading(distance, telemetry)
                                
                                # Format mode status
                                mode_status = self.format_mode_status(telemetry)
                                
                                # Live output with reading number
                                log_realtime(f"#{self.total_readings:3d}: {distance_str} | {mode_status}")
                                force_flush()
                                
                            except json.JSONDecodeError:
                                log_realtime(f"⚠️ Invalid JSON: {line[:50]}...")
                                force_flush()
                        
                        # Show non-JSON lines (Arduino debug messages, etc.)
                        elif line.strip():
                            log_realtime(f"📤 Arduino: {line}")
                            force_flush()
                    
                    # Print periodic statistics
                    if time.time() - last_stats_time > 10:
                        self.print_live_statistics()
                        last_stats_time = time.time()
                    
                    time.sleep(0.01)  # Very fast monitoring loop
                    
                except Exception as e:
                    log_realtime(f"⚠️ Read error: {e}")
                    force_flush()
                    time.sleep(0.1)
        
        except KeyboardInterrupt:
            log_realtime('')
            log_realtime('🛑 Monitoring stopped by user')
    
    def print_live_statistics(self):
        """Print live statistics during monitoring"""
        runtime = time.time() - self.start_time
        
        log_realtime('')
        log_realtime('📊 LIVE STATISTICS:')
        log_realtime(f'   Runtime: {runtime:.0f}s')
        log_realtime(f'   Total readings: {self.total_readings}')
        log_realtime(f'   Distance readings: {len(self.distance_readings)}')
        log_realtime(f'   Null/timeout readings: {self.null_readings}')
        
        if self.distance_readings:
            avg_dist = sum(self.distance_readings) / len(self.distance_readings)
            min_dist = min(self.distance_readings)
            max_dist = max(self.distance_readings)
            log_realtime(f'   🎉 SENSOR WORKING!')
            log_realtime(f'   Distance range: {min_dist:.1f} - {max_dist:.1f}cm')
            log_realtime(f'   Average distance: {avg_dist:.1f}cm')
        else:
            log_realtime(f'   ❌ No distance readings yet')
        
        if runtime > 0:
            rate = self.total_readings / runtime
            log_realtime(f'   Reading rate: {rate:.1f}/sec')
        
        log_realtime('=' * 60)
        force_flush()

def main():
    """Main data stream monitoring"""
    log_realtime('')
    log_realtime('📡 REAL-TIME ARDUINO DATA STREAM MONITOR')
    log_realtime('=' * 50)
    log_realtime('')
    log_realtime('PURPOSE: Watch live telemetry while positioning rover near wall')
    log_realtime('GOAL: Confirm ultrasonic sensor readings < 200cm')
    log_realtime('')
    force_flush()
    
    monitor = ArduinoDataStreamMonitor()
    
    if not monitor.connect():
        log_realtime('❌ Could not connect to Arduino')
        log_realtime('')
        log_realtime('TROUBLESHOOTING:')
        log_realtime('   1. Check Arduino is connected and powered')
        log_realtime('   2. Verify USB cable connection')
        log_realtime('   3. Check /dev/ttyUSB* permissions')
        return False
    
    try:
        log_realtime(f'✅ Connected to Arduino on {monitor.port}')
        log_realtime('')
        log_realtime('🎯 READY FOR TESTING!')
        log_realtime('   Move rover close to wall (< 2 meters)')
        log_realtime('   Watch for distance readings less than 200cm')
        log_realtime('   Switch to autonomous mode to see sensor in action')
        log_realtime('')
        force_flush()
        
        # Start monitoring for 5 minutes (or until Ctrl+C)
        monitor.monitor_data_stream(300)
        
    except Exception as e:
        log_realtime(f'❌ Monitoring error: {e}')
        return False
    
    finally:
        # Final statistics
        log_realtime('')
        log_realtime('📊 FINAL MONITORING RESULTS:')
        log_realtime('=' * 35)
        
        runtime = time.time() - monitor.start_time
        log_realtime(f'Total runtime: {runtime:.1f} seconds')
        log_realtime(f'Total readings: {monitor.total_readings}')
        log_realtime(f'Distance readings: {len(monitor.distance_readings)}')
        log_realtime(f'Null readings: {monitor.null_readings}')
        
        if monitor.distance_readings:
            avg_dist = sum(monitor.distance_readings) / len(monitor.distance_readings)
            min_dist = min(monitor.distance_readings)
            max_dist = max(monitor.distance_readings)
            log_realtime('')
            log_realtime('🎉 ULTRASONIC SENSOR SUCCESS!')
            log_realtime(f'   Readings collected: {len(monitor.distance_readings)}')
            log_realtime(f'   Distance range: {min_dist:.1f} - {max_dist:.1f}cm')
            log_realtime(f'   Average distance: {avg_dist:.1f}cm')
            
            # Check if we got close readings (wall test)
            close_readings = [d for d in monitor.distance_readings if d < 100]
            if close_readings:
                log_realtime(f'   🎯 Wall test successful: {len(close_readings)} readings < 100cm')
            else:
                log_realtime(f'   📏 No close readings - try moving rover closer to wall')
        
        else:
            log_realtime('')
            log_realtime('❌ No distance readings detected')
            log_realtime('   Sensor may still have hardware issues')
        
        monitor.disconnect()
        log_realtime('')
        log_realtime('👋 Data stream monitoring complete')
        
        return len(monitor.distance_readings) > 0

if __name__ == "__main__":
    force_flush()
    
    try:
        success = main()
        
        if success:
            log_realtime('')
            log_realtime('✅ SENSOR MONITORING: SUCCESS')
            log_realtime('🎉 Distance readings detected!')
            sys.exit(0)
        else:
            log_realtime('')
            log_realtime('⚠️ SENSOR MONITORING: NEEDS ATTENTION') 
            log_realtime('🔧 Consider hardware troubleshooting')
            sys.exit(1)
            
    except KeyboardInterrupt:
        log_realtime('')
        log_realtime('🛑 Monitoring interrupted by user')
        sys.exit(0)
    
    force_flush()