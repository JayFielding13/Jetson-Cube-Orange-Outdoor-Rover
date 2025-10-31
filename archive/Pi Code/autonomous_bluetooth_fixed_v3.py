#!/usr/bin/env python3
"""
Autonomous Bluetooth-Hybrid Navigator v3
FIXED: Real RSSI values using callback-based BLE scanning
Part of the Mini Rover Development Project
"""

import serial
import json
import time
import threading
import math
import random
import asyncio
import sys

# Bluetooth imports with graceful fallback
try:
    from bleak import BleakScanner
    BLUETOOTH_AVAILABLE = True
    print("✅ Bluetooth (bleak) library available")
except ImportError:
    print("⚠️ Bluetooth library not available - install with: pip install bleak")
    BLUETOOTH_AVAILABLE = False

class BluetoothHybridNavigatorV3:
    """
    FIXED autonomous rover navigator with real RSSI values from callback scanning
    Built on stable ultrasonic foundation with accurate BLE distance tracking
    """
    
    def __init__(self, arduino_port='/dev/ttyUSB0', baud=115200):
        self.arduino_port = arduino_port
        self.baud = baud
        self.arduino = None
        self.running = False
        
        # Robot state from Arduino gatekeeper
        self.robot_state = {
            'mode': 0,                    # 0=Manual, 1=Assisted, 2=Autonomous
            'rc_valid': False,            # RC signal validity
            'emergency_stop': False,      # Emergency stop status
            'ultrasonic_distance': 200.0, # Distance in cm
            'last_update': time.time()    # Last data timestamp
        }
        
        # Fixed Bluetooth state with callback scanning
        self.bluetooth_state = {
            'enabled': BLUETOOTH_AVAILABLE,
            'target_mac': 'DD:34:02:09:CA:1E',  # Specific BlueCharm MAC
            'target_name': 'BlueCharm_190853',  # Actual device name
            'target_rssi': None,
            'target_distance': None,
            'last_detection': 0,
            'scanner_active': False,
            'detection_count': 0,
            'rssi_history': [],  # For stability tracking
            'scan_failures': 0,
            'scan_successes': 0,
            'strongest_rssi': -200,
            'detection_timeout_count': 0,
            'callback_detections': 0,
            'last_callback_time': 0
        }
        
        # BLE scanner object for callback mode
        self.ble_scanner = None
        
        # Navigation state machine (inherited from stable code)
        self.nav_state = 'STRAIGHT'
        self.nav_start_time = time.time()
        self.turn_direction = 'left'
        self.stuck_counter = 0
        
        # Movement parameters (inherited - proven stable)
        self.cruise_speed = 100       # Normal forward speed
        self.slow_speed = 60          # Cautious forward speed
        self.turn_speed = 80          # Turning speed
        self.reverse_speed = 70       # Backing up speed
        
        # Navigation thresholds (inherited - proven stable)
        self.danger_threshold = 30.0   # Emergency stop distance (cm)
        self.caution_threshold = 60.0  # Slow down distance (cm)
        self.safe_distance = 100.0     # Safe following distance (cm)
        self.stuck_threshold = 5       # Consecutive stuck detections
        
        # Navigation timing (enhanced for better stuck recovery)
        self.exploration_interval = 8.0  # Seconds before random exploration
        self.turn_duration = 1.5         # Seconds for avoidance turns
        self.backup_duration = 2.0       # Seconds for backing up (increased)
        self.stuck_reset_time = 10.0     # Reset stuck counter interval
        self.aggressive_backup_duration = 3.0  # Extended backup for stuck recovery
        
        # Fixed Bluetooth parameters with callback scanning
        self.bluetooth_timeout = 5.0         # Timeout for lost signal
        self.bluetooth_weight = 0.0          # Start with 0% influence (Step 1)
        self.target_bluetooth_distance = 3.0  # Preferred distance in meters
        
        # Statistics tracking (enhanced)
        self.stats = {
            'total_runtime': 0,
            'distance_measurements': 0,
            'obstacle_avoidances': 0,
            'exploration_turns': 0,
            'emergency_stops': 0,
            'stuck_recoveries': 0,
            'bluetooth_detections': 0,
            'bluetooth_navigation_events': 0,
            'bluetooth_callback_events': 0
        }
        
        print("🚀 Bluetooth-Hybrid Navigator v3 (FIXED RSSI) Initialized")
        print(f"📊 Danger: {self.danger_threshold}cm | Caution: {self.caution_threshold}cm | Safe: {self.safe_distance}cm")
        print(f"📡 Bluetooth: {'Enabled' if self.bluetooth_state['enabled'] else 'Disabled'}")
        print(f"📡 BT Method: Callback-based scanning for real RSSI values")
        print(f"📡 BT Weight: {self.bluetooth_weight*100:.0f}% navigation influence")
    
    def bluetooth_detection_callback(self, device, advertisement_data):
        """Callback function for real-time BLE detection with real RSSI"""
        if device.address.upper() == self.bluetooth_state['target_mac'].upper():
            # Get real RSSI from advertisement data
            rssi = advertisement_data.rssi
            current_time = time.time()
            
            self.bluetooth_state['callback_detections'] += 1
            self.bluetooth_state['last_callback_time'] = current_time
            self.stats['bluetooth_callback_events'] += 1
            
            # Process the detection
            detection = {
                'name': device.name or 'BlueCharm_190853',
                'address': device.address,
                'rssi': rssi,
                'timestamp': current_time
            }
            
            self.process_bluetooth_detection(detection)
    
    def connect_arduino(self):
        """Establish connection to Arduino gatekeeper (unchanged - stable)"""
        try:
            self.arduino = serial.Serial(self.arduino_port, self.baud, timeout=1)
            time.sleep(2)  # Allow Arduino to reset
            print(f"✅ Arduino connected on {self.arduino_port}")
            return True
        except Exception as e:
            print(f"❌ Arduino connection failed: {e}")
            return False
    
    def start(self):
        """Start the hybrid navigator with callback-based BLE scanning"""
        if not self.connect_arduino():
            return False
        
        self.running = True
        self.stats['start_time'] = time.time()
        
        # Start communication threads (inherited)
        self.arduino_thread = threading.Thread(target=self.arduino_loop, daemon=True)
        self.navigation_thread = threading.Thread(target=self.navigation_loop, daemon=True)
        
        # Start Fixed Bluetooth thread with callback scanning
        if self.bluetooth_state['enabled']:
            self.bluetooth_thread = threading.Thread(target=self.bluetooth_loop, daemon=True)
            self.bluetooth_thread.start()
            print("📡 Fixed callback-based Bluetooth scanner started")
        
        self.arduino_thread.start()
        self.navigation_thread.start()
        
        print("🚀 Fixed Navigator started")
        print("🎯 Ready for autonomous navigation with REAL Bluetooth RSSI tracking")
        return True
    
    def arduino_loop(self):
        """Handle continuous Arduino communication (unchanged - stable)"""
        consecutive_failures = 0
        max_failures = 10
        
        while self.running and self.arduino:
            try:
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode().strip()
                    if line and line.startswith('{'):
                        try:
                            data = json.loads(line)
                            self.process_arduino_data(data)
                            consecutive_failures = 0
                        except json.JSONDecodeError as e:
                            print(f"⚠️ JSON decode error: {e}")
                            consecutive_failures += 1
                
                # Check for communication timeout
                if time.time() - self.robot_state['last_update'] > 5.0:
                    print("⚠️ Arduino communication timeout")
                    consecutive_failures += 1
                
                # Handle persistent failures
                if consecutive_failures >= max_failures:
                    print("❌ Too many Arduino communication failures")
                    self.emergency_stop()
                    break
                    
            except Exception as e:
                print(f"❌ Arduino communication error: {e}")
                consecutive_failures += 1
                time.sleep(0.1)
            
            time.sleep(0.02)  # 50Hz communication loop
    
    def process_arduino_data(self, data):
        """Process incoming data from Arduino (unchanged - stable)"""
        # Extract distance with validation
        distance = data.get('distance')
        if distance is None or distance < 0:
            distance = 200.0  # Safe default
        
        # Update robot state
        self.robot_state.update({
            'mode': data.get('mode', 0),
            'rc_valid': data.get('valid', False),
            'emergency_stop': data.get('emergency', False),
            'ultrasonic_distance': float(distance),
            'last_update': time.time()
        })
        
        # Track statistics
        self.stats['distance_measurements'] += 1
        
        # Log emergency conditions
        if self.robot_state['emergency_stop']:
            self.stats['emergency_stops'] += 1
    
    def bluetooth_loop(self):
        """Fixed Bluetooth loop with callback-based scanning"""
        if not self.bluetooth_state['enabled']:
            return
        
        # Create event loop for this thread
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        print("📡 Starting fixed callback-based Bluetooth detection...")
        
        while self.running:
            try:
                # Only scan in autonomous mode to avoid interference
                if self.robot_state.get('mode', 0) == 2:
                    if not self.bluetooth_state['scanner_active']:
                        # Start callback scanner
                        self.bluetooth_state['scanner_active'] = True
                        task = loop.create_task(self.start_callback_scanner())
                        
                    # Check for detection timeout
                    current_time = time.time()
                    if current_time - self.bluetooth_state['last_detection'] > self.bluetooth_timeout:
                        if self.bluetooth_state['target_rssi'] is not None:
                            print("📡 BlueCharm signal lost (timeout)")
                            self.bluetooth_state['target_rssi'] = None
                            self.bluetooth_state['target_distance'] = None
                            self.bluetooth_state['detection_timeout_count'] += 1
                else:
                    # Stop scanner when not in autonomous mode
                    if self.bluetooth_state['scanner_active']:
                        self.bluetooth_state['scanner_active'] = False
                        if self.ble_scanner:
                            try:
                                loop.run_until_complete(self.ble_scanner.stop())
                            except:
                                pass
                
                time.sleep(1.0)  # Check every second
                
            except Exception as e:
                print(f"❌ Fixed Bluetooth loop error: {e}")
                self.bluetooth_state['scanner_active'] = False
                time.sleep(2.0)  # Back off on errors
    
    async def start_callback_scanner(self):
        """Start the callback-based BLE scanner"""
        try:
            print("📡 Starting callback BLE scanner...")
            self.ble_scanner = BleakScanner(self.bluetooth_detection_callback)
            await self.ble_scanner.start()
            
            # Keep scanner running until stopped
            while self.running and self.bluetooth_state['scanner_active']:
                await asyncio.sleep(1.0)
            
            await self.ble_scanner.stop()
            print("📡 Callback BLE scanner stopped")
            
        except Exception as e:
            print(f"❌ Callback scanner error: {e}")
            self.bluetooth_state['scanner_active'] = False
    
    def process_bluetooth_detection(self, detection):
        """Process BlueCharm BLE detection with REAL RSSI values"""
        self.bluetooth_state['target_rssi'] = detection['rssi']
        self.bluetooth_state['last_detection'] = detection['timestamp']
        self.bluetooth_state['detection_count'] += 1
        self.stats['bluetooth_detections'] += 1
        
        # Track RSSI history for stability analysis
        self.bluetooth_state['rssi_history'].append(detection['rssi'])
        if len(self.bluetooth_state['rssi_history']) > 10:
            self.bluetooth_state['rssi_history'].pop(0)
        
        # Track strongest signal seen
        if detection['rssi'] > self.bluetooth_state['strongest_rssi']:
            self.bluetooth_state['strongest_rssi'] = detection['rssi']
        
        # Convert RSSI to estimated distance (BLE optimized)
        distance = self.rssi_to_distance(detection['rssi'])
        self.bluetooth_state['target_distance'] = distance
        
        # Enhanced logging with real RSSI values
        avg_rssi = sum(self.bluetooth_state['rssi_history']) / len(self.bluetooth_state['rssi_history'])
        if len(self.bluetooth_state['rssi_history']) > 2:
            rssi_range = max(self.bluetooth_state['rssi_history']) - min(self.bluetooth_state['rssi_history'])
            stability = "stable" if rssi_range < 8 else "variable"
        else:
            stability = "initial"
        
        # Log every 3rd detection to avoid spam
        if self.bluetooth_state['detection_count'] % 3 == 0:
            print(f"📡 BlueCharm REAL RSSI #{self.bluetooth_state['detection_count']}:")
            print(f"   RSSI: {detection['rssi']}dBm | Distance: ~{distance:.1f}m")
            print(f"   Avg RSSI: {avg_rssi:.0f}dBm ({stability}) | Callbacks: {self.bluetooth_state['callback_detections']}")
    
    def rssi_to_distance(self, rssi):
        """Convert RSSI to approximate distance (BLE beacon optimized with real values)"""
        if rssi == 0 or rssi < -100:
            return 30.0  # Safe default for out of range
        
        # BLE beacon distance estimation with real RSSI values
        # Typical BlueCharm/BLE beacon TX power at 1 meter
        tx_power = -59
        ratio = (tx_power - rssi) / 20.0
        distance = math.pow(10, ratio)
        
        # Apply realistic bounds for BLE beacons
        return max(0.3, min(distance, 30.0))  # BLE effective range
    
    def navigation_loop(self):
        """Main autonomous navigation logic (unchanged for Step 1)"""
        last_stuck_check = time.time()
        
        while self.running:
            try:
                # Only navigate in autonomous mode
                if self.robot_state.get('mode', 0) != 2:
                    time.sleep(0.5)
                    continue
                
                # Get current sensor readings
                ultrasonic_distance = self.robot_state.get('ultrasonic_distance', 200.0)
                current_time = time.time()
                
                # Check for stuck condition periodically
                if current_time - last_stuck_check > self.stuck_reset_time:
                    if self.stuck_counter > 0:
                        self.stuck_counter = max(0, self.stuck_counter - 1)
                    last_stuck_check = current_time
                
                # For Step 1: Pure ultrasonic navigation (no Bluetooth influence yet)
                left_speed, right_speed = self.compute_ultrasonic_navigation(ultrasonic_distance, current_time)
                
                # Send motor commands to Arduino
                self.send_motor_command(left_speed, right_speed)
                
                # Enhanced status logging (every 3 seconds)
                if current_time % 3 < 0.5:
                    self.log_enhanced_status(ultrasonic_distance)
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                self.send_motor_command(0, 0)
                time.sleep(1.0)
            
            time.sleep(0.5)  # 2Hz navigation loop for smooth control
    
    def compute_ultrasonic_navigation(self, distance, current_time):
        """Original proven ultrasonic navigation logic (unchanged)"""
        time_in_state = current_time - self.nav_start_time
        left_speed = 0
        right_speed = 0
        
        # State machine for navigation behavior (proven stable)
        if self.nav_state == 'STRAIGHT':
            if distance < self.danger_threshold:
                # Emergency avoidance
                self.nav_state = 'AVOIDING'
                self.nav_start_time = current_time
                self.turn_direction = self.choose_turn_direction()
                self.stats['obstacle_avoidances'] += 1
                print(f"🚨 EMERGENCY! Obstacle at {distance:.1f}cm - turning {self.turn_direction}")
                
            elif distance < self.caution_threshold:
                # Cautious forward movement
                speed_factor = (distance - self.danger_threshold) / (self.caution_threshold - self.danger_threshold)
                speed = self.slow_speed * max(0.3, speed_factor)
                left_speed = speed
                right_speed = speed
                print(f"⚠️ CAUTION: Obstacle at {distance:.1f}cm - slowing to {speed:.0f}")
                
            elif time_in_state > self.exploration_interval:
                # Periodic exploration
                self.nav_state = 'EXPLORING'
                self.nav_start_time = current_time
                self.stats['exploration_turns'] += 1
                print(f"🔍 EXPLORING: Random turn after {time_in_state:.1f}s")
                
            else:
                # Normal forward movement
                left_speed = self.cruise_speed
                right_speed = self.cruise_speed
        
        elif self.nav_state == 'AVOIDING':
            # Execute avoidance maneuver
            if self.turn_direction == 'left':
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
            else:
                left_speed = self.turn_speed
                right_speed = -self.turn_speed
            
            # Check if avoidance is complete
            if time_in_state > self.turn_duration and distance > self.safe_distance:
                self.nav_state = 'STRAIGHT'
                self.nav_start_time = current_time
                print(f"✅ CLEAR: Path ahead at {distance:.1f}cm - resuming forward")
            elif time_in_state > self.turn_duration * 2:
                # Stuck in avoidance - try backing up
                self.nav_state = 'BACKING'
                self.nav_start_time = current_time
                self.stuck_counter += 1
                print(f"🔄 STUCK: Backing up (count: {self.stuck_counter})")
        
        elif self.nav_state == 'BACKING':
            # Back up to create space - more aggressive when stuck
            backup_speed = self.reverse_speed
            if self.stuck_counter >= 3:
                backup_speed = min(120, self.reverse_speed * 1.5)  # Faster backup when very stuck
            
            left_speed = -backup_speed
            right_speed = -backup_speed
            
            # Dynamic backup duration based on stuck level
            required_backup_time = self.backup_duration
            if self.stuck_counter >= 3:
                required_backup_time = self.aggressive_backup_duration
            
            if time_in_state > required_backup_time:
                if self.stuck_counter >= self.stuck_threshold:
                    # Multiple stuck detections - try different strategy
                    self.nav_state = 'STUCK_RECOVERY'
                    self.nav_start_time = current_time
                    self.stats['stuck_recoveries'] += 1
                    print(f"🆘 STUCK RECOVERY: Multiple stuck detections")
                else:
                    # Return to avoidance with more aggressive turn
                    self.nav_state = 'AVOIDING'
                    self.nav_start_time = current_time
                    self.turn_direction = 'left' if self.turn_direction == 'right' else 'right'
                    print(f"🔄 RETRY: Switching to {self.turn_direction} turn (stuck: {self.stuck_counter})")
        
        elif self.nav_state == 'STUCK_RECOVERY':
            # Aggressive stuck recovery - longer backing and turning
            recovery_backup_time = self.aggressive_backup_duration
            recovery_turn_time = self.turn_duration * 3  # Even longer turns
            
            if time_in_state < recovery_backup_time:
                # Extended aggressive backing
                backup_speed = min(150, self.reverse_speed * 1.8)
                left_speed = -backup_speed
                right_speed = -backup_speed
                if time_in_state % 2 < 1:  # Log every 2 seconds
                    print(f"🆘 AGGRESSIVE BACKUP: {time_in_state:.1f}s at {backup_speed} speed")
            elif time_in_state < recovery_backup_time + recovery_turn_time:
                # Extended aggressive turning with higher speed
                turn_speed = min(120, self.turn_speed * 1.5)
                left_speed = -turn_speed
                right_speed = turn_speed
                if time_in_state % 2 < 1:
                    print(f"🆘 AGGRESSIVE TURN: {time_in_state - recovery_backup_time:.1f}s")
            else:
                # Reset and try again
                self.nav_state = 'STRAIGHT'
                self.nav_start_time = current_time
                self.stuck_counter = 0
                print("🔄 RECOVERY COMPLETE: Resetting navigation")
        
        elif self.nav_state == 'EXPLORING':
            # Random exploration turn
            left_speed = -self.turn_speed
            right_speed = self.turn_speed
            
            if time_in_state > self.turn_duration:
                self.nav_state = 'STRAIGHT'
                self.nav_start_time = current_time
                print("🔍 EXPLORATION COMPLETE: Resuming forward")
        
        return left_speed, right_speed
    
    def choose_turn_direction(self):
        """Choose turn direction (unchanged - proven stable)"""
        # For now, alternate directions to avoid getting stuck
        if hasattr(self, 'last_turn_direction'):
            self.last_turn_direction = 'right' if self.last_turn_direction == 'left' else 'left'
            return self.last_turn_direction
        else:
            self.last_turn_direction = 'left'
            return 'left'
    
    def send_motor_command(self, left_speed, right_speed):
        """Send motor command to Arduino gatekeeper (unchanged - stable)"""
        if not self.arduino or not self.running:
            return
        
        # Clamp speeds to valid range
        left_speed = max(-255, min(255, int(left_speed)))
        right_speed = max(-255, min(255, int(right_speed)))
        
        # Create command structure
        command = {
            'motor': {
                'left': left_speed,
                'right': right_speed
            }
        }
        
        try:
            cmd_str = json.dumps(command) + '\n'
            self.arduino.write(cmd_str.encode())
        except Exception as e:
            print(f"❌ Motor command failed: {e}")
    
    def emergency_stop(self):
        """Emergency stop - send zero speeds (unchanged - stable)"""
        self.send_motor_command(0, 0)
        print("🛑 EMERGENCY STOP EXECUTED")
    
    def log_enhanced_status(self, ultrasonic_distance):
        """Enhanced status logging with REAL Bluetooth RSSI info"""
        runtime = time.time() - self.stats['start_time']
        mode_text = ['MANUAL', 'ASSISTED', 'AUTONOMOUS'][self.robot_state.get('mode', 0)]
        
        status_line = f"🧭 {mode_text} | {self.nav_state} | US: {ultrasonic_distance:.1f}cm"
        
        # Add REAL Bluetooth info if available
        if self.bluetooth_state['enabled']:
            if self.bluetooth_state['target_distance']:
                bt_distance = self.bluetooth_state['target_distance']
                bt_rssi = self.bluetooth_state['target_rssi']
                status_line += f" | BT: {bt_distance:.1f}m ({bt_rssi}dBm REAL)"
            else:
                status_line += f" | BT: No signal"
            
            # Add callback statistics
            callbacks = self.bluetooth_state['callback_detections']
            status_line += f" | CB: {callbacks}"
        
        status_line += f" | Runtime: {runtime:.0f}s"
        print(status_line)
        
        if self.stuck_counter > 0:
            print(f"   Stuck counter: {self.stuck_counter}/{self.stuck_threshold}")
        
        # Periodic enhanced statistics
        if int(runtime) % 30 == 0 and runtime > 0:
            self.print_enhanced_statistics()
    
    def print_enhanced_statistics(self):
        """Print enhanced navigation and REAL Bluetooth statistics"""
        runtime = time.time() - self.stats['start_time']
        
        print("📊 === Enhanced Navigation Statistics ===")
        print(f"   Runtime: {runtime:.0f} seconds")
        print(f"   Distance measurements: {self.stats['distance_measurements']}")
        print(f"   Obstacle avoidances: {self.stats['obstacle_avoidances']}")
        print(f"   Exploration turns: {self.stats['exploration_turns']}")
        print(f"   Emergency stops: {self.stats['emergency_stops']}")
        print(f"   Stuck recoveries: {self.stats['stuck_recoveries']}")
        
        # Enhanced REAL Bluetooth statistics
        if self.bluetooth_state['enabled']:
            print("📡 === REAL RSSI Bluetooth Statistics ===")
            print(f"   Callback detections: {self.bluetooth_state['callback_detections']}")
            print(f"   BlueCharm detections: {self.stats['bluetooth_detections']}")
            print(f"   Callback events: {self.stats['bluetooth_callback_events']}")
            
            if runtime > 0:
                detection_rate = self.stats['bluetooth_detections'] / runtime
                callback_rate = self.bluetooth_state['callback_detections'] / runtime
                print(f"   Detection rate: {detection_rate:.2f}/sec")
                print(f"   Callback rate: {callback_rate:.2f}/sec")
            
            print(f"   Strongest RSSI seen: {self.bluetooth_state['strongest_rssi']} dBm")
            print(f"   Detection timeouts: {self.bluetooth_state['detection_timeout_count']}")
            
            if self.bluetooth_state['rssi_history']:
                avg_rssi = sum(self.bluetooth_state['rssi_history']) / len(self.bluetooth_state['rssi_history'])
                print(f"   Recent avg RSSI: {avg_rssi:.1f} dBm")
        
        if runtime > 0:
            print(f"   Avg measurements/sec: {self.stats['distance_measurements']/runtime:.1f}")
    
    def stop(self):
        """Stop the navigator and clean up resources (enhanced)"""
        print("🛑 Stopping Fixed Navigator...")
        self.running = False
        
        # Stop BLE scanner
        if self.bluetooth_state['enabled'] and self.bluetooth_state['scanner_active']:
            self.bluetooth_state['scanner_active'] = False
            if self.ble_scanner:
                try:
                    # Note: scanner cleanup happens in bluetooth_loop
                    pass
                except:
                    pass
        
        # Send stop command
        if self.arduino:
            self.send_motor_command(0, 0)
            time.sleep(0.2)
            
            try:
                self.arduino.close()
            except:
                pass
        
        # Print final enhanced statistics
        if 'start_time' in self.stats:
            runtime = time.time() - self.stats['start_time']
            print(f"📊 Final runtime: {runtime:.1f} seconds")
            self.print_enhanced_statistics()
        
        print("👋 Fixed Navigator shutdown complete")

def main():
    """Main entry point for the FIXED hybrid navigator"""
    print("=" * 70)
    print("🤖 AUTONOMOUS BLUETOOTH-HYBRID NAVIGATOR v3 (FIXED RSSI)")
    print("=" * 70)
    print()
    print("Features:")
    print("  ✅ Proven ultrasonic obstacle detection and avoidance")
    print("  ✅ Arduino gatekeeper integration")
    print("  ✅ Enhanced state-based navigation with improved stuck recovery")
    print("  ✅ Comprehensive error handling and statistics")
    print("  📡 FIXED: Real RSSI values using callback-based BLE scanning")
    print("  🛡️ Safety-first design - ultrasonic overrides Bluetooth")
    print("  📊 Accurate distance tracking with real signal strength")
    print("  🔧 Step 1: Passive detection (0% navigation influence)")
    print()
    
    if not BLUETOOTH_AVAILABLE:
        print("⚠️ WARNING: Bluetooth library not available")
        print("   Install with: pip install bleak")
        print("   Navigator will run in ultrasonic-only mode")
        print()
    
    navigator = BluetoothHybridNavigatorV3()
    
    try:
        if navigator.start():
            print("✅ FIXED Navigator ready - switch to autonomous mode")
            print("📡 Real RSSI Bluetooth tracking will activate automatically")
            print("🔍 Watch for REAL signal strength values!")
            print("Press Ctrl+C to stop...")
            
            while True:
                time.sleep(1)
        else:
            print("❌ Failed to start navigator")
    
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested by user...")
    
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
    
    finally:
        navigator.stop()

if __name__ == "__main__":
    main()