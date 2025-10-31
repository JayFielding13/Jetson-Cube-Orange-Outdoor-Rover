#!/usr/bin/env python3
"""
Bluetooth Tracking Navigator V4
================================================================================
Forward movement signal strength tracking with intelligent seeking

FEATURES:
- Built on proven Stable Bluetooth Ranging V1 foundation
- Forward movement signal strength tracking (no circular scanning)
- Simple turn-and-test behavior for signal improvement
- Proper motor speeds that actually move the rover
- Target hold logic for precise 0.5m distance maintenance
- Safety-first design: ultrasonic navigation takes absolute priority
- Real-time Bluetooth beacon distance tracking (0.8m - 11.2m range)
- Working RSSI values (-57dBm to -80dBm) using callback-based BLE scanning
- Thread-safe integration with clean shutdown

NEW IN V4:
- 🚶 Forward movement signal tracking (move and monitor RSSI)
- 🔄 Simple turn-and-test behavior when signal weakens
- ⚡ Proper motor speeds (80+ minimum) that actually move the rover
- 🎯 Straight-line approach with course corrections
- 📊 Signal improvement detection during movement
- 🏁 Target hold logic - stops and maintains 0.5m distance
- 🔧 Simplified state machine for reliable operation

HARDWARE:
- Raspberry Pi 4B with Bluetooth 5.0
- Arduino gatekeeper with ultrasonic sensor
- BlueCharm BLE beacon (MAC: DD:34:02:09:CA:1E)
- Cytron MDDS30 motor driver

Part of the Mini Rover Development Project
Author: Developed incrementally with Jay Fielding
Version: 4.0 - Forward Movement Signal Tracking
Date: 2025-01-11
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

class BluetoothTrackingNavigator:
    """
    Forward movement signal strength tracking rover navigator
    
    This is the V4 tracking release featuring:
    - Proven ultrasonic navigation with enhanced stuck recovery
    - Working real-time Bluetooth distance tracking
    - Forward movement signal strength tracking
    - Simple turn-and-test behavior for course corrections
    - Target hold logic for precise distance maintenance
    - Thread-safe callback-based BLE scanning
    - Safety-first design with ultrasonic override priority
    - Proper motor speeds that actually move the rover
    """
    
    def __init__(self, arduino_port='/dev/ttyUSB0', baud=115200):
        """Initialize the forward movement signal tracking navigator"""
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
        
        # Enhanced Bluetooth state for forward tracking
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
            'callback_detections': 0,               # Callback events
            # New forward tracking state
            'tracking_mode': False,                 # Active forward tracking
            'approaching_mode': False,              # Close approach mode
            'holding_mode': False,                  # Maintaining target distance
            'movement_start_rssi': None,            # RSSI when movement started
            'movement_start_time': 0,               # Movement timing
            'signal_improving': False,              # Signal getting stronger
            'signal_degrading': False,              # Signal getting weaker
            'turn_test_mode': False,                # Testing turn direction
            'test_direction': 'left',               # Current test direction
            'turn_test_start_rssi': None,           # RSSI when turn test started
            'at_target_start_time': 0,              # Time when reached target
            'last_direction_change': 0              # Last course correction time
        }
        
        # Thread-safe detection lock
        self.detection_lock = threading.Lock()
        
        # Simplified navigation state machine for tracking
        self.nav_state = 'SEARCHING'  # Start in search mode
        self.nav_start_time = time.time()
        self.turn_direction = 'left'
        self.stuck_counter = 0
        
        # Movement parameters (increased for actual movement)
        self.cruise_speed = 100       # Normal forward speed
        self.tracking_speed = 80      # Forward tracking speed
        self.slow_speed = 70          # Cautious forward speed
        self.turn_speed = 85          # Turning speed (increased)
        self.reverse_speed = 80       # Backing up speed (increased)
        self.approach_speed = 60      # Gentle approach speed (increased)
        self.hold_speed = 40          # Target adjustment speed (increased)
        
        # Navigation thresholds (proven stable values)
        self.danger_threshold = 30.0   # Emergency stop distance (cm)
        self.caution_threshold = 60.0  # Slow down distance (cm)
        self.safe_distance = 100.0     # Safe following distance (cm)
        self.stuck_threshold = 5       # Consecutive stuck detections
        
        # Navigation timing (simplified for tracking)
        self.exploration_interval = 8.0        # Seconds before random exploration
        self.turn_duration = 2.0               # Seconds for avoidance turns (increased)
        self.backup_duration = 2.5             # Seconds for backing up (increased)
        self.stuck_reset_time = 10.0           # Reset stuck counter interval
        self.aggressive_backup_duration = 3.5  # Extended backup for stuck recovery (increased)
        self.tracking_duration = 6.0           # Time to track forward before evaluation
        self.turn_test_duration = 3.0          # Time to test turn direction
        self.hold_check_interval = 2.0         # Check target distance interval
        
        # Signal strength tracking parameters
        self.bluetooth_timeout = 10.0          # Timeout for lost signal
        self.bluetooth_weight = 0.35           # Navigation influence (35% when tracking)
        self.target_bluetooth_distance = 0.5   # Target approach distance (0.5m)
        self.target_tolerance = 0.15           # Target distance tolerance (±0.15m)
        self.approach_threshold = 1.0          # Start approach mode (1.0m)
        self.tracking_threshold = 3.0          # Start tracking when signal detected (3.0m)
        self.signal_improvement_threshold = 2  # dBm improvement to consider direction good
        self.signal_degradation_threshold = 3  # dBm degradation to trigger turn test
        self.movement_evaluation_samples = 5   # Samples to evaluate movement effectiveness
        
        # Enhanced statistics tracking
        self.stats = {
            'total_runtime': 0,
            'distance_measurements': 0,
            'obstacle_avoidances': 0,
            'exploration_turns': 0,
            'emergency_stops': 0,
            'stuck_recoveries': 0,
            'bluetooth_detections': 0,
            'bluetooth_navigation_events': 0,
            'bluetooth_callback_events': 0,
            'tracking_activations': 0,
            'approach_activations': 0,
            'hold_activations': 0,
            'target_reaches': 0,
            'signal_improvements': 0,
            'course_corrections': 0,
            'turn_tests': 0
        }
        
        print("🚀 Bluetooth Tracking Navigator V4 Initialized")
        print(f"📊 Navigation: Danger={self.danger_threshold}cm | Caution={self.caution_threshold}cm | Safe={self.safe_distance}cm")
        print(f"📡 Bluetooth: {'Enabled' if self.bluetooth_state['enabled'] else 'Disabled'} | Target: {self.bluetooth_state['target_mac']}")
        print(f"🚶 Tracking: Target distance {self.target_bluetooth_distance}m ±{self.target_tolerance}m | Influence {self.bluetooth_weight*100:.0f}%")
        print(f"📈 Signal: {self.signal_improvement_threshold}dBm improvement | {self.signal_degradation_threshold}dBm degradation triggers")
        print(f"⚡ Motor: Min speeds {self.hold_speed}-{self.tracking_speed} (proper movement)")
        print(f"🛡️ Safety: Ultrasonic navigation takes absolute priority")
    
    def bluetooth_detection_callback(self, device, advertisement_data):
        """
        Thread-safe callback for real-time BlueCharm detection and tracking analysis
        
        Enhanced for forward movement signal tracking
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
                previous_rssi = self.bluetooth_state['target_rssi']
                self.bluetooth_state['target_rssi'] = rssi
                self.bluetooth_state['last_detection'] = current_time
                self.bluetooth_state['detection_count'] += 1
                self.stats['bluetooth_detections'] += 1
                
                # Track RSSI history for tracking analysis
                self.bluetooth_state['rssi_history'].append(rssi)
                if len(self.bluetooth_state['rssi_history']) > 15:  # Moderate history for tracking
                    self.bluetooth_state['rssi_history'].pop(0)
                
                # Track strongest signal seen
                if rssi > self.bluetooth_state['strongest_rssi']:
                    self.bluetooth_state['strongest_rssi'] = rssi
                
                # Signal improvement/degradation detection
                if previous_rssi and len(self.bluetooth_state['rssi_history']) >= 3:
                    recent_avg = sum(self.bluetooth_state['rssi_history'][-3:]) / 3
                    older_avg = sum(self.bluetooth_state['rssi_history'][-6:-3]) / 3 if len(self.bluetooth_state['rssi_history']) >= 6 else previous_rssi
                    
                    rssi_change = recent_avg - older_avg
                    
                    if rssi_change >= self.signal_improvement_threshold:
                        if not self.bluetooth_state['signal_improving']:
                            self.bluetooth_state['signal_improving'] = True
                            self.bluetooth_state['signal_degrading'] = False
                            self.stats['signal_improvements'] += 1
                    elif rssi_change <= -self.signal_degradation_threshold:
                        if not self.bluetooth_state['signal_degrading']:
                            self.bluetooth_state['signal_degrading'] = True
                            self.bluetooth_state['signal_improving'] = False
                
                # Convert RSSI to distance estimate
                distance = self.rssi_to_distance(rssi)
                self.bluetooth_state['target_distance'] = distance
                
                # Activate appropriate mode based on distance
                if distance <= self.target_bluetooth_distance + self.target_tolerance:
                    # At target distance - activate hold mode
                    if not self.bluetooth_state['holding_mode']:
                        self.bluetooth_state['holding_mode'] = True
                        self.bluetooth_state['at_target_start_time'] = current_time
                        self.stats['hold_activations'] += 1
                        self.stats['target_reaches'] += 1
                        print(f"🏁 TARGET HOLD: BlueCharm at {distance:.1f}m - holding position")
                elif distance <= self.approach_threshold:
                    # Close - activate approach mode
                    if not self.bluetooth_state['approaching_mode']:
                        self.bluetooth_state['approaching_mode'] = True
                        self.stats['approach_activations'] += 1
                        print(f"🎯 APPROACH MODE: BlueCharm at {distance:.1f}m - gentle approach")
                elif distance <= self.tracking_threshold:
                    # Medium range - activate tracking mode
                    if not self.bluetooth_state['tracking_mode']:
                        self.bluetooth_state['tracking_mode'] = True
                        self.bluetooth_state['movement_start_rssi'] = rssi
                        self.bluetooth_state['movement_start_time'] = current_time
                        self.stats['tracking_activations'] += 1
                        print(f"🚶 TRACKING MODE: BlueCharm at {distance:.1f}m - forward tracking")
                
                # Log every 5th detection with mode info
                if self.bluetooth_state['detection_count'] % 5 == 0:
                    avg_rssi = sum(self.bluetooth_state['rssi_history']) / len(self.bluetooth_state['rssi_history'])
                    if self.bluetooth_state['holding_mode']:
                        mode = "HOLDING"
                    elif self.bluetooth_state['approaching_mode']:
                        mode = "APPROACH"
                    elif self.bluetooth_state['tracking_mode']:
                        mode = "TRACKING"
                        if self.bluetooth_state['signal_improving']:
                            mode += "↗"
                        elif self.bluetooth_state['signal_degrading']:
                            mode += "↘"
                    else:
                        mode = "PASSIVE"
                    
                    print(f"📡 BlueCharm {mode} #{self.bluetooth_state['detection_count']}:")
                    print(f"   RSSI: {rssi}dBm | Distance: ~{distance:.1f}m | Avg: {avg_rssi:.0f}dBm")
    
    def rssi_to_distance(self, rssi):
        """Convert RSSI to approximate distance for BLE beacon"""
        if rssi == 0 or rssi < -100:
            return 30.0  # Safe default for out of range
        
        # BLE beacon distance estimation
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
        """Start the tracking navigator with Bluetooth forward tracking"""
        if not self.connect_arduino():
            return False
        
        self.running = True
        self.stats['start_time'] = time.time()
        
        # Start communication threads
        self.arduino_thread = threading.Thread(target=self.arduino_loop, daemon=True)
        self.navigation_thread = threading.Thread(target=self.navigation_loop, daemon=True)
        
        # Start Bluetooth tracking thread
        if self.bluetooth_state['enabled']:
            self.bluetooth_thread = threading.Thread(target=self.bluetooth_thread_worker, daemon=True)
            self.bluetooth_thread.start()
            print("📡 BlueCharm tracking scanner started")
        
        self.arduino_thread.start()
        self.navigation_thread.start()
        
        print("🚀 Tracking Navigator V4 started")
        print("🚶 Ready for autonomous forward movement signal tracking")
        return True
    
    def bluetooth_thread_worker(self):
        """Dedicated thread worker for Bluetooth scanning"""
        print("📡 Starting BlueCharm tracking thread...")
        
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
        """Main async function for Bluetooth tracking scanning"""
        print("📡 BlueCharm tracking scanner ready")
        
        scanner = None
        
        try:
            while self.running:
                # Only scan in autonomous mode
                if self.robot_state.get('mode', 0) == 2:
                    if not self.bluetooth_state['scanner_running']:
                        # Start scanner
                        print("📡 Starting BLE tracking scanner...")
                        scanner = BleakScanner(self.bluetooth_detection_callback)
                        await scanner.start()
                        self.bluetooth_state['scanner_running'] = True
                        print("📡 BLE tracking scanner active")
                    
                    # Check for detection timeout and mode management
                    current_time = time.time()
                    time_since_detection = current_time - self.bluetooth_state['last_detection']
                    
                    # Reset modes on timeout
                    if time_since_detection > self.bluetooth_timeout:
                        if self.bluetooth_state['target_rssi'] is not None:
                            with self.detection_lock:
                                self.bluetooth_state['target_rssi'] = None
                                self.bluetooth_state['target_distance'] = None
                                self.bluetooth_state['detection_timeout_count'] += 1
                                
                                # Reset all tracking modes
                                modes_active = any([
                                    self.bluetooth_state['tracking_mode'],
                                    self.bluetooth_state['approaching_mode'],
                                    self.bluetooth_state['holding_mode']
                                ])
                                
                                if modes_active:
                                    print(f"📡 Signal timeout ({time_since_detection:.1f}s) - ending all tracking modes")
                                    self.bluetooth_state['tracking_mode'] = False
                                    self.bluetooth_state['approaching_mode'] = False
                                    self.bluetooth_state['holding_mode'] = False
                                    self.bluetooth_state['signal_improving'] = False
                                    self.bluetooth_state['signal_degrading'] = False
                
                else:
                    # Stop scanner when not in autonomous mode
                    if self.bluetooth_state['scanner_running'] and scanner:
                        print("📡 Stopping BLE tracking scanner...")
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
                    print("📡 Cleaning up BLE tracking scanner...")
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
        """Main autonomous navigation logic with forward movement signal tracking"""
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
                
                # V4: Enhanced navigation with forward movement signal tracking
                left_speed, right_speed = self.compute_tracking_navigation(ultrasonic_distance, current_time)
                
                # Send motor commands to Arduino
                self.send_motor_command(left_speed, right_speed)
                
                # Status logging every 3 seconds
                if current_time % 3 < 0.5:
                    self.log_tracking_status(ultrasonic_distance)
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                self.send_motor_command(0, 0)
                time.sleep(1.0)
            
            time.sleep(0.5)  # 2Hz navigation loop
    
    def compute_tracking_navigation(self, ultrasonic_distance, current_time):
        """
        Enhanced navigation with forward movement signal tracking
        
        Simplified state machine with tracking modes:
        - SEARCHING: Looking for BlueCharm signal
        - FORWARD_TRACKING: Moving forward while monitoring signal strength
        - TURN_TESTING: Testing left/right turns for signal improvement
        - APPROACHING: Gentle approach to target
        - HOLDING: Maintaining target distance
        - Standard ultrasonic states: AVOIDING, BACKING, STUCK_RECOVERY
        """
        time_in_state = current_time - self.nav_start_time
        base_left_speed = 0
        base_right_speed = 0
        
        # SAFETY FIRST: Ultrasonic obstacle detection always takes priority
        if ultrasonic_distance < self.danger_threshold:
            # Emergency avoidance overrides all Bluetooth behavior
            if self.nav_state not in ['AVOIDING', 'BACKING', 'STUCK_RECOVERY']:
                self.nav_state = 'AVOIDING'
                self.nav_start_time = current_time
                self.turn_direction = self.choose_turn_direction()
                self.stats['obstacle_avoidances'] += 1
                print(f"🚨 EMERGENCY! Obstacle at {ultrasonic_distance:.1f}cm - overriding tracking")
        
        # Get current Bluetooth state (thread-safe)
        bluetooth_distance = None
        tracking_mode = False
        approaching_mode = False
        holding_mode = False
        signal_improving = False
        signal_degrading = False
        
        with self.detection_lock:
            bluetooth_distance = self.bluetooth_state.get('target_distance')
            tracking_mode = self.bluetooth_state.get('tracking_mode', False)
            approaching_mode = self.bluetooth_state.get('approaching_mode', False)
            holding_mode = self.bluetooth_state.get('holding_mode', False)
            signal_improving = self.bluetooth_state.get('signal_improving', False)
            signal_degrading = self.bluetooth_state.get('signal_degrading', False)
        
        # Simplified state machine with forward tracking behavior
        if self.nav_state == 'AVOIDING':
            # Execute avoidance maneuver (unchanged - safety priority)
            if self.turn_direction == 'left':
                base_left_speed = -self.turn_speed
                base_right_speed = self.turn_speed
            else:
                base_left_speed = self.turn_speed
                base_right_speed = -self.turn_speed
            
            # Check if avoidance is complete
            if time_in_state > self.turn_duration and ultrasonic_distance > self.safe_distance:
                # Return to appropriate mode based on Bluetooth state
                if holding_mode:
                    self.nav_state = 'HOLDING'
                elif approaching_mode:
                    self.nav_state = 'APPROACHING'
                elif tracking_mode:
                    self.nav_state = 'FORWARD_TRACKING'
                else:
                    self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                print(f"✅ CLEAR: Path ahead at {ultrasonic_distance:.1f}cm - resuming {self.nav_state.lower()}")
            elif time_in_state > self.turn_duration * 2:
                # Stuck in avoidance - try backing up
                self.nav_state = 'BACKING'
                self.nav_start_time = current_time
                self.stuck_counter += 1
                print(f"🔄 STUCK: Backing up (count: {self.stuck_counter})")
        
        elif self.nav_state == 'BACKING':
            # Back up to create space (unchanged - proven stable)
            backup_speed = self.reverse_speed
            if self.stuck_counter >= 3:
                backup_speed = min(130, self.reverse_speed * 1.5)
            
            base_left_speed = -backup_speed
            base_right_speed = -backup_speed
            
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
            # Aggressive stuck recovery (unchanged - proven stable)
            recovery_backup_time = self.aggressive_backup_duration
            recovery_turn_time = self.turn_duration * 3
            
            if time_in_state < recovery_backup_time:
                # Extended aggressive backing
                backup_speed = min(160, self.reverse_speed * 1.8)
                base_left_speed = -backup_speed
                base_right_speed = -backup_speed
                if time_in_state % 2 < 1:
                    print(f"🆘 AGGRESSIVE BACKUP: {time_in_state:.1f}s at {backup_speed} speed")
            elif time_in_state < recovery_backup_time + recovery_turn_time:
                # Extended aggressive turning
                turn_speed = min(130, self.turn_speed * 1.5)
                base_left_speed = -turn_speed
                base_right_speed = turn_speed
                if time_in_state % 2 < 1:
                    print(f"🆘 AGGRESSIVE TURN: {time_in_state - recovery_backup_time:.1f}s")
            else:
                # Reset and try again
                self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                self.stuck_counter = 0
                print("🔄 RECOVERY COMPLETE: Resetting to search mode")
        
        elif self.nav_state == 'HOLDING' and holding_mode and bluetooth_distance:
            # Target hold mode - maintain precise 0.5m distance
            distance_error = bluetooth_distance - self.target_bluetooth_distance
            
            if abs(distance_error) <= self.target_tolerance:
                # At perfect distance - stop movement
                base_left_speed = 0
                base_right_speed = 0
                if time_in_state % 5 < 1:  # Log every 5 seconds
                    print(f"🏁 TARGET PERFECT: Holding {bluetooth_distance:.1f}m distance")
            elif distance_error > 0:
                # Too far - gentle approach
                adjustment_speed = min(self.hold_speed, abs(distance_error) * 40)
                base_left_speed = adjustment_speed
                base_right_speed = adjustment_speed
                if time_in_state % 3 < 1:
                    print(f"📏 ADJUST CLOSER: {bluetooth_distance:.1f}m → {self.target_bluetooth_distance}m")
            else:
                # Too close - gentle retreat
                adjustment_speed = min(self.hold_speed, abs(distance_error) * 40)
                base_left_speed = -adjustment_speed
                base_right_speed = -adjustment_speed
                if time_in_state % 3 < 1:
                    print(f"📏 ADJUST FARTHER: {bluetooth_distance:.1f}m → {self.target_bluetooth_distance}m")
            
            # Exit hold mode if signal lost or too far
            if not holding_mode or (bluetooth_distance and bluetooth_distance > self.approach_threshold):
                self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                print("📡 HOLD LOST: Signal lost or too far - returning to search")
        
        elif self.nav_state == 'APPROACHING' and approaching_mode and bluetooth_distance:
            # Gentle approach mode - move toward target
            if bluetooth_distance <= self.target_bluetooth_distance + self.target_tolerance:
                # Close enough to switch to hold mode
                self.nav_state = 'HOLDING'
                self.nav_start_time = current_time
                print(f"🎯 SWITCHING TO HOLD: {bluetooth_distance:.1f}m - target reached")
            else:
                # Move toward signal with adaptive speed based on distance
                if ultrasonic_distance < self.caution_threshold:
                    # Cautious movement when obstacles near
                    speed_factor = (ultrasonic_distance - self.danger_threshold) / (self.caution_threshold - self.danger_threshold)
                    speed = self.slow_speed * max(0.4, speed_factor)
                else:
                    # Normal approach speed
                    speed = self.approach_speed
                
                base_left_speed = speed
                base_right_speed = speed
                
                if time_in_state % 4 < 1:
                    print(f"🎯 APPROACHING: Moving toward {bluetooth_distance:.1f}m signal at {speed:.0f} speed")
            
            # Exit approach mode if signal lost
            if not approaching_mode:
                self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                print("📡 APPROACH LOST: Signal lost - returning to search")
        
        elif self.nav_state == 'FORWARD_TRACKING' and tracking_mode:
            # Forward tracking mode - move forward and monitor signal strength
            if bluetooth_distance and bluetooth_distance <= self.approach_threshold:
                # Close enough to switch to approach mode
                self.nav_state = 'APPROACHING'
                self.nav_start_time = current_time
                print(f"🎯 SWITCHING TO APPROACH: {bluetooth_distance:.1f}m")
            elif signal_degrading and time_in_state > 3.0:
                # Signal getting worse - try turn test
                self.nav_state = 'TURN_TESTING'
                self.nav_start_time = current_time
                self.stats['course_corrections'] += 1
                with self.detection_lock:
                    self.bluetooth_state['turn_test_mode'] = True
                    self.bluetooth_state['test_direction'] = 'left'
                    self.bluetooth_state['turn_test_start_rssi'] = self.bluetooth_state['target_rssi']
                print(f"🔄 SIGNAL DEGRADING: Testing left turn for improvement")
            else:
                # Continue forward tracking
                if ultrasonic_distance < self.caution_threshold:
                    # Cautious movement when obstacles near
                    speed_factor = (ultrasonic_distance - self.danger_threshold) / (self.caution_threshold - self.danger_threshold)
                    speed = self.slow_speed * max(0.4, speed_factor)
                else:
                    # Normal tracking speed
                    speed = self.tracking_speed
                
                base_left_speed = speed
                base_right_speed = speed
                
                if time_in_state % 4 < 1:
                    signal_text = "↗improving" if signal_improving else "↘degrading" if signal_degrading else "stable"
                    print(f"🚶 FORWARD TRACKING: {bluetooth_distance:.1f}m signal {signal_text}")
            
            # Exit tracking mode if signal lost
            if not tracking_mode:
                self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                print("📡 TRACKING LOST: Signal lost - returning to search")
        
        elif self.nav_state == 'TURN_TESTING':
            # Turn testing mode - test left/right turns for signal improvement
            if time_in_state < self.turn_test_duration:
                # Execute test turn
                with self.detection_lock:
                    test_direction = self.bluetooth_state.get('test_direction', 'left')
                
                if test_direction == 'left':
                    base_left_speed = -self.turn_speed
                    base_right_speed = self.turn_speed
                else:
                    base_left_speed = self.turn_speed
                    base_right_speed = -self.turn_speed
                
                if time_in_state % 2 < 1:
                    print(f"🔄 TESTING: {test_direction} turn for signal improvement")
            else:
                # Evaluate test results
                with self.detection_lock:
                    current_rssi = self.bluetooth_state.get('target_rssi')
                    start_rssi = self.bluetooth_state.get('turn_test_start_rssi')
                    test_direction = self.bluetooth_state.get('test_direction', 'left')
                    
                    self.stats['turn_tests'] += 1
                    
                    if current_rssi and start_rssi:
                        improvement = current_rssi - start_rssi
                        
                        if improvement >= self.signal_improvement_threshold:
                            # Good direction - continue forward tracking
                            self.nav_state = 'FORWARD_TRACKING'
                            self.nav_start_time = current_time
                            self.bluetooth_state['signal_improving'] = True
                            self.bluetooth_state['signal_degrading'] = False
                            print(f"✅ TURN TEST SUCCESS: {test_direction} improved by {improvement:.1f}dBm")
                        elif test_direction == 'left':
                            # Try right turn
                            self.bluetooth_state['test_direction'] = 'right'
                            self.nav_start_time = current_time
                            print(f"🔄 TRYING RIGHT: Left turn didn't help, testing right")
                        else:
                            # Neither direction helped - continue forward
                            self.nav_state = 'FORWARD_TRACKING'
                            self.nav_start_time = current_time
                            print(f"⚠️ NO IMPROVEMENT: Continuing forward tracking")
                    else:
                        # No signal - return to search
                        self.nav_state = 'SEARCHING'
                        self.nav_start_time = current_time
                        print(f"📡 NO SIGNAL: Returning to search")
                    
                    self.bluetooth_state['turn_test_mode'] = False
        
        elif self.nav_state == 'SEARCHING':
            # Search pattern when no BlueCharm signal
            mode_detected = tracking_mode or approaching_mode or holding_mode
            
            if mode_detected:
                # Signal detected - switch to appropriate mode
                if holding_mode:
                    self.nav_state = 'HOLDING'
                elif approaching_mode:
                    self.nav_state = 'APPROACHING'
                elif tracking_mode:
                    self.nav_state = 'FORWARD_TRACKING'
                
                self.nav_start_time = current_time
                distance_text = f"{bluetooth_distance:.1f}m" if bluetooth_distance else "unknown"
                print(f"📡 SIGNAL DETECTED: Switching to {self.nav_state.lower()} mode ({distance_text})")
            elif time_in_state > self.turn_duration * 2:
                # Execute search turn
                base_left_speed = -self.turn_speed
                base_right_speed = self.turn_speed
                
                if time_in_state > self.turn_duration * 4:
                    # Reset search pattern
                    self.nav_start_time = current_time
                    print(f"🔄 SEARCH PATTERN: Looking for signal...")
            else:
                # Search forward movement
                if ultrasonic_distance > self.safe_distance:
                    base_left_speed = self.slow_speed
                    base_right_speed = self.slow_speed
                else:
                    # Wait before turning if obstacle ahead
                    base_left_speed = 0
                    base_right_speed = 0
        
        else:
            # Default fallback
            self.nav_state = 'SEARCHING'
            self.nav_start_time = current_time
            print("⚠️ Unknown navigation state - defaulting to search")
        
        return base_left_speed, base_right_speed
    
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
    
    def log_tracking_status(self, ultrasonic_distance):
        """Log current navigation and tracking status"""
        runtime = time.time() - self.stats['start_time']
        mode_text = ['MANUAL', 'ASSISTED', 'AUTONOMOUS'][self.robot_state.get('mode', 0)]
        
        status_line = f"🧭 {mode_text} | {self.nav_state} | US: {ultrasonic_distance:.1f}cm"
        
        # Add Bluetooth tracking info if available (thread-safe)
        if self.bluetooth_state['enabled']:
            with self.detection_lock:
                if self.bluetooth_state['target_distance']:
                    bt_distance = self.bluetooth_state['target_distance']
                    bt_rssi = self.bluetooth_state['target_rssi']
                    
                    # Determine mode symbol
                    if self.bluetooth_state['holding_mode']:
                        mode_symbol = "HOLD"
                    elif self.bluetooth_state['approaching_mode']:
                        mode_symbol = "APPR"
                    elif self.bluetooth_state['tracking_mode']:
                        mode_symbol = "TRACK"
                        if self.bluetooth_state['signal_improving']:
                            mode_symbol += "↗"
                        elif self.bluetooth_state['signal_degrading']:
                            mode_symbol += "↘"
                    else:
                        mode_symbol = "PASS"
                    
                    status_line += f" | BT: {bt_distance:.1f}m ({bt_rssi}dBm {mode_symbol})"
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
            self.print_tracking_statistics()
    
    def print_tracking_statistics(self):
        """Print comprehensive navigation and tracking statistics"""
        runtime = time.time() - self.stats['start_time']
        
        print("📊 === Tracking Navigator V4 Statistics ===")
        print(f"   Runtime: {runtime:.0f} seconds")
        print(f"   Distance measurements: {self.stats['distance_measurements']}")
        print(f"   Obstacle avoidances: {self.stats['obstacle_avoidances']}")
        print(f"   Emergency stops: {self.stats['emergency_stops']}")
        print(f"   Stuck recoveries: {self.stats['stuck_recoveries']}")
        
        # Forward tracking statistics
        print("🚶 === Forward Movement Tracking Statistics ===")
        print(f"   Tracking activations: {self.stats['tracking_activations']}")
        print(f"   Approach activations: {self.stats['approach_activations']}")
        print(f"   Hold activations: {self.stats['hold_activations']}")
        print(f"   Target reaches: {self.stats['target_reaches']}")
        print(f"   Signal improvements: {self.stats['signal_improvements']}")
        print(f"   Course corrections: {self.stats['course_corrections']}")
        print(f"   Turn tests: {self.stats['turn_tests']}")
        
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
                
                # Current mode status
                current_modes = []
                if self.bluetooth_state['tracking_mode']:
                    current_modes.append("TRACKING")
                if self.bluetooth_state['approaching_mode']:
                    current_modes.append("APPROACHING")
                if self.bluetooth_state['holding_mode']:
                    current_modes.append("HOLDING")
                
                if current_modes:
                    print(f"   Current modes: {', '.join(current_modes)}")
        
        if runtime > 0:
            print(f"   Navigation rate: {self.stats['distance_measurements']/runtime:.1f} measurements/sec")
    
    def stop(self):
        """Stop the navigator and clean up resources"""
        print("🛑 Stopping Tracking Navigator V4...")
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
            self.print_tracking_statistics()
        
        print("👋 Tracking Navigator V4 shutdown complete")

def main():
    """Main entry point for Bluetooth Tracking Navigator V4"""
    print("=" * 80)
    print("🤖 BLUETOOTH TRACKING NAVIGATOR V4")
    print("=" * 80)
    print()
    print("FORWARD MOVEMENT SIGNAL TRACKING FEATURES:")
    print("  ✅ Ultrasonic obstacle detection and avoidance")
    print("  ✅ Enhanced stuck recovery system")
    print("  ✅ Arduino gatekeeper integration")
    print("  ✅ Real-time Bluetooth beacon ranging (0.8m - 11.2m)")
    print("  ✅ Working RSSI values (-57dBm to -80dBm)")
    print("  ✅ Thread-safe callback-based BLE scanning")
    print("  🚶 NEW: Forward movement signal strength tracking")
    print("  🔄 NEW: Turn-and-test behavior for course corrections")
    print("  ⚡ NEW: Proper motor speeds (80+) that actually move rover")
    print("  🎯 NEW: Straight-line approach with signal monitoring")
    print("  🏁 NEW: Precise target hold at 0.5m ±0.15m")
    print("  ✅ Safety-first design with ultrasonic priority")
    print("  ✅ Clean shutdown with no task warnings")
    print()
    print("FORWARD TRACKING BEHAVIOR:")
    print("  🔍 SEARCHING: Look for BlueCharm signal")
    print("  🚶 FORWARD_TRACKING: Move forward while monitoring signal")
    print("  🔄 TURN_TESTING: Test left/right turns when signal degrades")
    print("  🎯 APPROACHING: Gentle approach to target")
    print("  🏁 HOLDING: Maintain precise 0.5m distance")
    print("  🛡️ SAFETY: Ultrasonic overrides all Bluetooth behavior")
    print()
    print("SIGNAL TRACKING ALGORITHM:")
    print(f"  📈 {2}dBm improvement threshold to continue direction")
    print(f"  📉 {3}dBm degradation threshold triggers turn test")
    print(f"  🎯 Target: 0.5m ±0.15m with micro-adjustments")
    print(f"  ⚡ Motor speeds: 40-100 (guaranteed movement)")
    print()
    
    if not BLUETOOTH_AVAILABLE:
        print("⚠️ WARNING: Bluetooth library not available")
        print("   Install with: pip install bleak")
        print("   Navigator will run in ultrasonic-only mode")
        print()
    
    navigator = BluetoothTrackingNavigator()
    
    try:
        if navigator.start():
            print("✅ Tracking Navigator V4 ready - switch to autonomous mode")
            print("📡 BlueCharm forward tracking will activate automatically")
            print("🚶 Rover will move forward and track signal strength changes")
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