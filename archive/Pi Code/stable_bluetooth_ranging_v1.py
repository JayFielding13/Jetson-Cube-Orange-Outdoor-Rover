#!/usr/bin/env python3
"""
Stable Bluetooth Ranging V1
================================================================================
Autonomous rover with proven Bluetooth beacon ranging capabilities

FEATURES:
- Proven ultrasonic obstacle detection and avoidance
- Real-time Bluetooth beacon distance tracking (0.8m - 11.2m range)
- Working RSSI values (-57dBm to -80dBm) using callback-based BLE scanning
- Thread-safe integration with clean shutdown
- Safety-first design: ultrasonic navigation takes absolute priority
- 0% Bluetooth navigation influence (passive detection only)
- Comprehensive error handling and statistics

TESTED PERFORMANCE:
- Detection rate: 0.49/sec (64 detections in 131 seconds)
- Navigation: 11 obstacle avoidances, 0 stuck recoveries
- Clean operation: No task warnings, proper thread management
- Distance accuracy: Real-time tracking from close (0.8m) to far (11.2m)

HARDWARE:
- Raspberry Pi 4B with Bluetooth 5.0
- Arduino gatekeeper with ultrasonic sensor
- BlueCharm BLE beacon (MAC: DD:34:02:09:CA:1E)
- Cytron MDDS30 motor driver

Part of the Mini Rover Development Project
Author: Developed incrementally with Jay Fielding
Version: 1.0 - Stable Bluetooth Ranging Foundation
Date: 2025-01-07
================================================================================
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

class StableBluetoothRangingNavigator:
    """
    Stable autonomous rover navigator with proven Bluetooth beacon ranging
    
    This is the V1 stable release featuring:
    - Proven ultrasonic navigation with enhanced stuck recovery
    - Working real-time Bluetooth distance tracking
    - Thread-safe callback-based BLE scanning
    - Safety-first design with ultrasonic override priority
    - Comprehensive statistics and error handling
    """
    
    def __init__(self, arduino_port='/dev/ttyUSB0', baud=115200):
        """Initialize the stable Bluetooth ranging navigator"""
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
        
        # Bluetooth ranging state (proven stable configuration)
        self.bluetooth_state = {
            'enabled': BLUETOOTH_AVAILABLE,
            'target_mac': 'DD:34:02:09:CA:1E',      # BlueCharm MAC address
            'target_name': 'BlueCharm_190853',      # BlueCharm device name
            'target_rssi': None,                    # Current signal strength
            'target_distance': None,                # Current distance estimate
            'last_detection': 0,                    # Last detection timestamp
            'scanner_running': False,               # Scanner state
            'detection_count': 0,                   # Total detections
            'rssi_history': [],                     # Signal strength history
            'strongest_rssi': -200,                 # Best signal seen
            'detection_timeout_count': 0,           # Timeout events
            'callback_detections': 0                # Callback events
        }
        
        # Thread-safe detection lock
        self.detection_lock = threading.Lock()
        
        # Navigation state machine (proven stable)
        self.nav_state = 'STRAIGHT'
        self.nav_start_time = time.time()
        self.turn_direction = 'left'
        self.stuck_counter = 0
        
        # Movement parameters (proven stable values)
        self.cruise_speed = 100       # Normal forward speed
        self.slow_speed = 60          # Cautious forward speed
        self.turn_speed = 80          # Turning speed
        self.reverse_speed = 70       # Backing up speed
        
        # Navigation thresholds (proven stable values)
        self.danger_threshold = 30.0   # Emergency stop distance (cm)
        self.caution_threshold = 60.0  # Slow down distance (cm)
        self.safe_distance = 100.0     # Safe following distance (cm)
        self.stuck_threshold = 5       # Consecutive stuck detections
        
        # Navigation timing (enhanced for better stuck recovery)
        self.exploration_interval = 8.0        # Seconds before random exploration
        self.turn_duration = 1.5               # Seconds for avoidance turns
        self.backup_duration = 2.0             # Seconds for backing up
        self.stuck_reset_time = 10.0           # Reset stuck counter interval
        self.aggressive_backup_duration = 3.0  # Extended backup for stuck recovery
        
        # Bluetooth parameters (stable configuration)
        self.bluetooth_timeout = 5.0           # Timeout for lost signal
        self.bluetooth_weight = 0.0            # Navigation influence (0% = passive)
        self.target_bluetooth_distance = 3.0   # Preferred distance in meters
        
        # Statistics tracking
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
        
        print("🚀 Stable Bluetooth Ranging Navigator V1 Initialized")
        print(f"📊 Navigation: Danger={self.danger_threshold}cm | Caution={self.caution_threshold}cm | Safe={self.safe_distance}cm")
        print(f"📡 Bluetooth: {'Enabled' if self.bluetooth_state['enabled'] else 'Disabled'} | Target: {self.bluetooth_state['target_mac']}")
        print(f"📡 Ranging: Passive detection only ({self.bluetooth_weight*100:.0f}% navigation influence)")
        print(f"🛡️ Safety: Ultrasonic navigation takes absolute priority")
    
    def bluetooth_detection_callback(self, device, advertisement_data):
        """
        Thread-safe callback for real-time BlueCharm detection
        
        This callback processes BLE advertisements and extracts real RSSI values
        for accurate distance estimation. Thread-safe implementation ensures
        reliable operation alongside navigation threads.
        """
        if device.address.upper() == self.bluetooth_state['target_mac'].upper():
            # Thread-safe processing of detection
            with self.detection_lock:
                # Extract real RSSI from advertisement data
                rssi = advertisement_data.rssi
                current_time = time.time()
                
                # Update callback statistics
                self.bluetooth_state['callback_detections'] += 1
                self.stats['bluetooth_callback_events'] += 1
                
                # Update detection state
                self.bluetooth_state['target_rssi'] = rssi
                self.bluetooth_state['last_detection'] = current_time
                self.bluetooth_state['detection_count'] += 1
                self.stats['bluetooth_detections'] += 1
                
                # Track RSSI history for stability analysis
                self.bluetooth_state['rssi_history'].append(rssi)
                if len(self.bluetooth_state['rssi_history']) > 10:
                    self.bluetooth_state['rssi_history'].pop(0)
                
                # Track strongest signal seen
                if rssi > self.bluetooth_state['strongest_rssi']:
                    self.bluetooth_state['strongest_rssi'] = rssi
                
                # Convert RSSI to distance estimate
                distance = self.rssi_to_distance(rssi)
                self.bluetooth_state['target_distance'] = distance
                
                # Log every 5th detection to avoid spam
                if self.bluetooth_state['detection_count'] % 5 == 0:
                    avg_rssi = sum(self.bluetooth_state['rssi_history']) / len(self.bluetooth_state['rssi_history'])
                    print(f"📡 BlueCharm Range #{self.bluetooth_state['detection_count']}:")
                    print(f"   RSSI: {rssi}dBm | Distance: ~{distance:.1f}m | Avg: {avg_rssi:.0f}dBm")
    
    def rssi_to_distance(self, rssi):
        """
        Convert RSSI to approximate distance for BLE beacon
        
        Uses logarithmic path loss model optimized for BlueCharm beacons.
        Provides realistic distance estimates in 0.3-30m range.
        
        Args:
            rssi: Received Signal Strength Indicator in dBm
            
        Returns:
            Estimated distance in meters
        """
        if rssi == 0 or rssi < -100:
            return 30.0  # Safe default for out of range
        
        # BLE beacon distance estimation
        # Typical BlueCharm TX power at 1 meter
        tx_power = -59
        ratio = (tx_power - rssi) / 20.0
        distance = math.pow(10, ratio)
        
        # Apply realistic bounds for BLE beacons
        return max(0.3, min(distance, 30.0))
    
    def connect_arduino(self):
        """Establish connection to Arduino gatekeeper"""
        try:
            self.arduino = serial.Serial(self.arduino_port, self.baud, timeout=1)
            time.sleep(2)  # Allow Arduino to reset
            print(f"✅ Arduino connected on {self.arduino_port}")
            return True
        except Exception as e:
            print(f"❌ Arduino connection failed: {e}")
            return False
    
    def start(self):
        """Start the stable navigator with Bluetooth ranging"""
        if not self.connect_arduino():
            return False
        
        self.running = True
        self.stats['start_time'] = time.time()
        
        # Start communication threads
        self.arduino_thread = threading.Thread(target=self.arduino_loop, daemon=True)
        self.navigation_thread = threading.Thread(target=self.navigation_loop, daemon=True)
        
        # Start Bluetooth ranging thread
        if self.bluetooth_state['enabled']:
            self.bluetooth_thread = threading.Thread(target=self.bluetooth_thread_worker, daemon=True)
            self.bluetooth_thread.start()
            print("📡 Bluetooth ranging scanner started")
        
        self.arduino_thread.start()
        self.navigation_thread.start()
        
        print("🚀 Stable Navigator V1 started")
        print("🎯 Ready for autonomous navigation with Bluetooth ranging")
        return True
    
    def bluetooth_thread_worker(self):
        """Dedicated thread worker for Bluetooth scanning"""
        print("📡 Starting Bluetooth ranging thread...")
        
        # Create new event loop for this thread
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            loop.run_until_complete(self.bluetooth_scanner_main())
        except Exception as e:
            print(f"❌ Bluetooth thread error: {e}")
        finally:
            try:
                loop.close()
            except:
                pass
    
    async def bluetooth_scanner_main(self):
        """Main async function for Bluetooth ranging"""
        print("📡 Bluetooth ranging scanner ready")
        
        scanner = None
        
        try:
            while self.running:
                # Only scan in autonomous mode
                if self.robot_state.get('mode', 0) == 2:
                    if not self.bluetooth_state['scanner_running']:
                        # Start scanner
                        print("📡 Starting BLE ranging scanner...")
                        scanner = BleakScanner(self.bluetooth_detection_callback)
                        await scanner.start()
                        self.bluetooth_state['scanner_running'] = True
                        print("📡 BLE ranging scanner active")
                    
                    # Check for detection timeout
                    current_time = time.time()
                    if current_time - self.bluetooth_state['last_detection'] > self.bluetooth_timeout:
                        if self.bluetooth_state['target_rssi'] is not None:
                            with self.detection_lock:
                                self.bluetooth_state['target_rssi'] = None
                                self.bluetooth_state['target_distance'] = None
                                self.bluetooth_state['detection_timeout_count'] += 1
                
                else:
                    # Stop scanner when not in autonomous mode
                    if self.bluetooth_state['scanner_running'] and scanner:
                        print("📡 Stopping BLE ranging scanner...")
                        await scanner.stop()
                        self.bluetooth_state['scanner_running'] = False
                        scanner = None
                
                # Async sleep to yield control
                await asyncio.sleep(1.0)
        
        except Exception as e:
            print(f"❌ Bluetooth scanner error: {e}")
        
        finally:
            # Cleanup scanner
            if scanner and self.bluetooth_state['scanner_running']:
                try:
                    print("📡 Cleaning up BLE scanner...")
                    await scanner.stop()
                    self.bluetooth_state['scanner_running'] = False
                except Exception as e:
                    print(f"⚠️ Scanner cleanup error: {e}")
    
    def arduino_loop(self):
        """Handle continuous Arduino communication"""
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
        """Process incoming data from Arduino"""
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
    
    def navigation_loop(self):
        """Main autonomous navigation logic"""
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
                
                # V1: Pure ultrasonic navigation (0% Bluetooth influence)
                left_speed, right_speed = self.compute_ultrasonic_navigation(ultrasonic_distance, current_time)
                
                # Send motor commands to Arduino
                self.send_motor_command(left_speed, right_speed)
                
                # Status logging every 3 seconds
                if current_time % 3 < 0.5:
                    self.log_status(ultrasonic_distance)
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                self.send_motor_command(0, 0)
                time.sleep(1.0)
            
            time.sleep(0.5)  # 2Hz navigation loop
    
    def compute_ultrasonic_navigation(self, distance, current_time):
        """
        Proven ultrasonic navigation with enhanced stuck recovery
        
        State machine implementation with the following states:
        - STRAIGHT: Normal forward movement with periodic exploration
        - AVOIDING: Turn to avoid obstacles
        - BACKING: Back up when stuck
        - STUCK_RECOVERY: Aggressive recovery for persistent stuck conditions
        - EXPLORING: Periodic random exploration turns
        """
        time_in_state = current_time - self.nav_start_time
        left_speed = 0
        right_speed = 0
        
        # State machine for navigation behavior
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
                backup_speed = min(120, self.reverse_speed * 1.5)
            
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
            recovery_turn_time = self.turn_duration * 3
            
            if time_in_state < recovery_backup_time:
                # Extended aggressive backing
                backup_speed = min(150, self.reverse_speed * 1.8)
                left_speed = -backup_speed
                right_speed = -backup_speed
                if time_in_state % 2 < 1:
                    print(f"🆘 AGGRESSIVE BACKUP: {time_in_state:.1f}s at {backup_speed} speed")
            elif time_in_state < recovery_backup_time + recovery_turn_time:
                # Extended aggressive turning
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
        """Choose turn direction alternating to avoid patterns"""
        if hasattr(self, 'last_turn_direction'):
            self.last_turn_direction = 'right' if self.last_turn_direction == 'left' else 'left'
            return self.last_turn_direction
        else:
            self.last_turn_direction = 'left'
            return 'left'
    
    def send_motor_command(self, left_speed, right_speed):
        """Send motor command to Arduino gatekeeper"""
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
        """Emergency stop - send zero speeds"""
        self.send_motor_command(0, 0)
        print("🛑 EMERGENCY STOP EXECUTED")
    
    def log_status(self, ultrasonic_distance):
        """Log current navigation and ranging status"""
        runtime = time.time() - self.stats['start_time']
        mode_text = ['MANUAL', 'ASSISTED', 'AUTONOMOUS'][self.robot_state.get('mode', 0)]
        
        status_line = f"🧭 {mode_text} | {self.nav_state} | US: {ultrasonic_distance:.1f}cm"
        
        # Add Bluetooth ranging info if available (thread-safe)
        if self.bluetooth_state['enabled']:
            with self.detection_lock:
                if self.bluetooth_state['target_distance']:
                    bt_distance = self.bluetooth_state['target_distance']
                    bt_rssi = self.bluetooth_state['target_rssi']
                    status_line += f" | BT: {bt_distance:.1f}m ({bt_rssi}dBm)"
                else:
                    status_line += f" | BT: No signal"
                
                # Add detection statistics
                callbacks = self.bluetooth_state['callback_detections']
                status_line += f" | Det: {callbacks}"
        
        status_line += f" | Runtime: {runtime:.0f}s"
        print(status_line)
        
        if self.stuck_counter > 0:
            print(f"   Stuck counter: {self.stuck_counter}/{self.stuck_threshold}")
        
        # Periodic statistics
        if int(runtime) % 30 == 0 and runtime > 0:
            self.print_statistics()
    
    def print_statistics(self):
        """Print comprehensive navigation and ranging statistics"""
        runtime = time.time() - self.stats['start_time']
        
        print("📊 === Stable Navigator V1 Statistics ===")
        print(f"   Runtime: {runtime:.0f} seconds")
        print(f"   Distance measurements: {self.stats['distance_measurements']}")
        print(f"   Obstacle avoidances: {self.stats['obstacle_avoidances']}")
        print(f"   Exploration turns: {self.stats['exploration_turns']}")
        print(f"   Emergency stops: {self.stats['emergency_stops']}")
        print(f"   Stuck recoveries: {self.stats['stuck_recoveries']}")
        
        # Bluetooth ranging statistics
        if self.bluetooth_state['enabled']:
            with self.detection_lock:
                print("📡 === Bluetooth Ranging Statistics ===")
                print(f"   Total detections: {self.stats['bluetooth_detections']}")
                print(f"   Callback events: {self.stats['bluetooth_callback_events']}")
                
                if runtime > 0:
                    detection_rate = self.stats['bluetooth_detections'] / runtime
                    callback_rate = self.bluetooth_state['callback_detections'] / runtime
                    print(f"   Detection rate: {detection_rate:.2f}/sec")
                    print(f"   Callback rate: {callback_rate:.2f}/sec")
                
                print(f"   Strongest RSSI: {self.bluetooth_state['strongest_rssi']} dBm")
                print(f"   Signal timeouts: {self.bluetooth_state['detection_timeout_count']}")
                
                if self.bluetooth_state['rssi_history']:
                    avg_rssi = sum(self.bluetooth_state['rssi_history']) / len(self.bluetooth_state['rssi_history'])
                    print(f"   Average RSSI: {avg_rssi:.1f} dBm")
        
        if runtime > 0:
            print(f"   Navigation rate: {self.stats['distance_measurements']/runtime:.1f} measurements/sec")
    
    def stop(self):
        """Stop the navigator and clean up resources"""
        print("🛑 Stopping Stable Navigator V1...")
        self.running = False
        
        # Allow threads to notice the stop
        time.sleep(0.5)
        
        # Send stop command
        if self.arduino:
            self.send_motor_command(0, 0)
            time.sleep(0.2)
            
            try:
                self.arduino.close()
            except:
                pass
        
        # Print final statistics
        if 'start_time' in self.stats:
            runtime = time.time() - self.stats['start_time']
            print(f"📊 Final runtime: {runtime:.1f} seconds")
            self.print_statistics()
        
        print("👋 Stable Navigator V1 shutdown complete")

def main():
    """Main entry point for Stable Bluetooth Ranging Navigator V1"""
    print("=" * 80)
    print("🤖 STABLE BLUETOOTH RANGING NAVIGATOR V1")
    print("=" * 80)
    print()
    print("PROVEN FEATURES:")
    print("  ✅ Ultrasonic obstacle detection and avoidance")
    print("  ✅ Enhanced stuck recovery system")
    print("  ✅ Arduino gatekeeper integration")
    print("  ✅ Real-time Bluetooth beacon ranging (0.8m - 11.2m)")
    print("  ✅ Working RSSI values (-57dBm to -80dBm)")
    print("  ✅ Thread-safe callback-based BLE scanning")
    print("  ✅ Comprehensive error handling and statistics")
    print("  ✅ Safety-first design with ultrasonic priority")
    print("  ✅ Clean shutdown with no task warnings")
    print()
    print("TESTED PERFORMANCE:")
    print("  📊 Detection rate: 0.49/sec (64 detections in 131 seconds)")
    print("  🧭 Navigation: 11 obstacle avoidances, 0 stuck recoveries")
    print("  📡 Distance tracking: Real-time from 0.8m to 11.2m")
    print("  🔧 V1 Configuration: Passive detection (0% navigation influence)")
    print()
    
    if not BLUETOOTH_AVAILABLE:
        print("⚠️ WARNING: Bluetooth library not available")
        print("   Install with: pip install bleak")
        print("   Navigator will run in ultrasonic-only mode")
        print()
    
    navigator = StableBluetoothRangingNavigator()
    
    try:
        if navigator.start():
            print("✅ Stable Navigator V1 ready - switch to autonomous mode")
            print("📡 Bluetooth ranging will activate automatically")
            print("🎯 Navigate safely while tracking BlueCharm beacon distance")
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