#!/usr/bin/env python3
"""
Bluetooth Smart Navigator V5
================================================================================
Intelligent obstacle avoidance with RSSI smoothing and minimal turning

FEATURES:
- Built on proven V4 forward movement signal tracking foundation
- Intelligent obstacle avoidance with minimal turning
- RSSI smoothing to reduce distance reading jumps
- Smart turning that stops when path is clear
- Forward movement signal strength tracking
- Target hold logic for precise 0.5m distance maintenance
- Safety-first design: ultrasonic navigation takes absolute priority
- Real-time Bluetooth beacon distance tracking with stability
- Working RSSI values with noise reduction
- Thread-safe integration with clean shutdown

NEW IN V5:
- 🧠 Intelligent obstacle avoidance (turn only until clear)
- 📊 RSSI smoothing to reduce distance jumps
- 🔄 Smart turning with real-time path checking
- 📈 Weighted average distance calculations
- 🎯 Improved approach stability
- 🛡️ Enhanced stuck detection and recovery

HARDWARE:
- Raspberry Pi 4B with Bluetooth 5.0
- Arduino gatekeeper with ultrasonic sensor
- BlueCharm BLE beacon (MAC: DD:34:02:09:CA:1E)
- Cytron MDDS30 motor driver

Part of the Mini Rover Development Project
Author: Developed incrementally with Jay Fielding
Version: 5.0 - Smart Navigation with Intelligent Avoidance
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
from collections import deque

# Bluetooth imports with graceful fallback
try:
    from bleak import BleakScanner
    BLUETOOTH_AVAILABLE = True
    print("✅ Bluetooth (bleak) library available")
except ImportError:
    print("⚠️ Bluetooth library not available - install with: pip install bleak")
    BLUETOOTH_AVAILABLE = False

class BluetoothSmartNavigator:
    """
    Smart Bluetooth navigator with intelligent obstacle avoidance and RSSI smoothing
    
    This is the V5 smart release featuring:
    - Proven forward movement signal tracking from V4
    - Intelligent obstacle avoidance with minimal turning
    - RSSI smoothing to reduce distance reading jumps
    - Smart turning that stops when path is clear
    - Enhanced approach stability and target holding
    - Thread-safe callback-based BLE scanning
    - Safety-first design with ultrasonic override priority
    """
    
    def __init__(self, arduino_port='/dev/ttyUSB0', baud=115200):
        """Initialize the smart Bluetooth navigator"""
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
        
        # Enhanced Bluetooth state with smoothing
        self.bluetooth_state = {
            'enabled': BLUETOOTH_AVAILABLE,
            'target_mac': 'DD:34:02:09:CA:1E',      # BlueCharm MAC address
            'target_name': 'BlueCharm_190853',      # BlueCharm device name
            'target_rssi': None,                    # Current signal strength
            'target_distance': None,                # Current distance estimate
            'smoothed_distance': None,              # Smoothed distance for stability
            'last_detection': 0,                    # Last detection timestamp
            'scanner_running': False,               # Scanner state
            'detection_count': 0,                   # Total detections
            'rssi_history': deque(maxlen=10),       # RSSI history for smoothing
            'distance_history': deque(maxlen=8),    # Distance history for smoothing
            'strongest_rssi': -200,                 # Best signal seen
            'detection_timeout_count': 0,           # Timeout events
            'callback_detections': 0,               # Callback events
            # Forward tracking state (proven from V4)
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
        
        # Enhanced navigation state machine
        self.nav_state = 'SEARCHING'  # Start in search mode
        self.nav_start_time = time.time()
        self.turn_direction = 'left'
        self.stuck_counter = 0
        
        # Smart avoidance state
        self.smart_turning = False
        self.turn_start_time = 0
        self.min_turn_time = 0.8      # Minimum turn time before checking
        self.max_turn_time = 4.0      # Maximum turn time before giving up
        self.clear_distance_threshold = 80.0  # Distance to consider path clear
        
        # Movement parameters (proven from V4)
        self.cruise_speed = 100       # Normal forward speed
        self.tracking_speed = 80      # Forward tracking speed
        self.slow_speed = 70          # Cautious forward speed
        self.turn_speed = 85          # Turning speed
        self.reverse_speed = 80       # Backing up speed
        self.approach_speed = 60      # Gentle approach speed
        self.hold_speed = 40          # Target adjustment speed
        
        # Navigation thresholds (proven stable values)
        self.danger_threshold = 30.0   # Emergency stop distance (cm)
        self.caution_threshold = 60.0  # Slow down distance (cm)
        self.safe_distance = 100.0     # Safe following distance (cm)
        self.stuck_threshold = 4       # Consecutive stuck detections (reduced)
        
        # Navigation timing (optimized for smart avoidance)
        self.exploration_interval = 8.0        # Seconds before random exploration
        self.turn_duration = 2.0               # Default turn duration (fallback)
        self.backup_duration = 2.5             # Seconds for backing up
        self.stuck_reset_time = 12.0           # Reset stuck counter interval (increased)
        self.aggressive_backup_duration = 3.5  # Extended backup for stuck recovery
        self.tracking_duration = 6.0           # Time to track forward before evaluation
        self.turn_test_duration = 3.0          # Time to test turn direction
        self.hold_check_interval = 2.0         # Check target distance interval
        
        # Signal strength tracking parameters (proven from V4)
        self.bluetooth_timeout = 10.0          # Timeout for lost signal
        self.bluetooth_weight = 0.35           # Navigation influence when tracking
        self.target_bluetooth_distance = 0.5   # Target approach distance (0.5m)
        self.target_tolerance = 0.15           # Target distance tolerance (±0.15m)
        self.approach_threshold = 1.0          # Start approach mode (1.0m)
        self.tracking_threshold = 3.0          # Start tracking when signal detected (3.0m)
        self.signal_improvement_threshold = 2  # dBm improvement to consider direction good
        self.signal_degradation_threshold = 3  # dBm degradation to trigger turn test
        
        # RSSI smoothing parameters
        self.rssi_smoothing_weight = 0.3       # Weight for new RSSI readings (30% new, 70% history)
        self.distance_smoothing_weight = 0.4   # Weight for new distance readings (40% new, 60% history)
        self.outlier_rejection_threshold = 2.0 # Reject distance changes > 2x previous
        
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
            'turn_tests': 0,
            'smart_turns': 0,
            'rssi_outliers_rejected': 0
        }
        
        print("🚀 Bluetooth Smart Navigator V5 Initialized")
        print(f"📊 Navigation: Danger={self.danger_threshold}cm | Caution={self.caution_threshold}cm | Safe={self.safe_distance}cm")
        print(f"📡 Bluetooth: {'Enabled' if self.bluetooth_state['enabled'] else 'Disabled'} | Target: {self.bluetooth_state['target_mac']}")
        print(f"🚶 Tracking: Target distance {self.target_bluetooth_distance}m ±{self.target_tolerance}m | Influence {self.bluetooth_weight*100:.0f}%")
        print(f"📈 Signal: {self.signal_improvement_threshold}dBm improvement | {self.signal_degradation_threshold}dBm degradation triggers")
        print(f"🧠 Smart Turn: Stop when clear >{self.clear_distance_threshold}cm | Min {self.min_turn_time}s")
        print(f"📊 RSSI Smoothing: {self.rssi_smoothing_weight*100:.0f}% new data | Outlier rejection >{self.outlier_rejection_threshold}x")
        print(f"🛡️ Safety: Ultrasonic navigation takes absolute priority")
    
    def smooth_rssi(self, new_rssi):
        """Apply smoothing to RSSI readings to reduce noise"""
        if not self.bluetooth_state['rssi_history']:
            # First reading
            return new_rssi
        
        # Calculate weighted average with recent history
        history_weight = 1.0 - self.rssi_smoothing_weight
        recent_avg = sum(list(self.bluetooth_state['rssi_history'])[-3:]) / min(3, len(self.bluetooth_state['rssi_history']))
        
        smoothed_rssi = (self.rssi_smoothing_weight * new_rssi) + (history_weight * recent_avg)
        return smoothed_rssi
    
    def smooth_distance(self, new_distance):
        """Apply smoothing to distance readings with outlier rejection"""
        if not self.bluetooth_state['distance_history']:
            # First reading
            return new_distance
        
        # Outlier rejection - check if new reading is too different
        recent_avg = sum(list(self.bluetooth_state['distance_history'])[-3:]) / min(3, len(self.bluetooth_state['distance_history']))
        distance_ratio = new_distance / recent_avg if recent_avg > 0 else 1.0
        
        if distance_ratio > self.outlier_rejection_threshold or distance_ratio < (1.0 / self.outlier_rejection_threshold):
            # Reject outlier, use previous smoothed value
            self.stats['rssi_outliers_rejected'] += 1
            return self.bluetooth_state['smoothed_distance'] if self.bluetooth_state['smoothed_distance'] else new_distance
        
        # Calculate weighted average with recent history
        history_weight = 1.0 - self.distance_smoothing_weight
        smoothed_distance = (self.distance_smoothing_weight * new_distance) + (history_weight * recent_avg)
        
        return smoothed_distance
    
    def bluetooth_detection_callback(self, device, advertisement_data):
        """
        Thread-safe callback for real-time BlueCharm detection with smoothing
        
        Enhanced with RSSI and distance smoothing for stability
        """
        if device.address.upper() == self.bluetooth_state['target_mac'].upper():
            # Thread-safe processing of detection
            with self.detection_lock:
                # Extract real RSSI from advertisement data
                raw_rssi = advertisement_data.rssi
                current_time = time.time()
                
                # Apply RSSI smoothing
                smoothed_rssi = self.smooth_rssi(raw_rssi)
                
                # Update callback statistics
                self.bluetooth_state['callback_detections'] += 1
                self.stats['bluetooth_callback_events'] += 1
                
                # Update detection state with smoothed values
                previous_rssi = self.bluetooth_state['target_rssi']
                self.bluetooth_state['target_rssi'] = smoothed_rssi
                self.bluetooth_state['last_detection'] = current_time
                self.bluetooth_state['detection_count'] += 1
                self.stats['bluetooth_detections'] += 1
                
                # Track RSSI history for smoothing and analysis
                self.bluetooth_state['rssi_history'].append(smoothed_rssi)
                
                # Track strongest signal seen
                if smoothed_rssi > self.bluetooth_state['strongest_rssi']:
                    self.bluetooth_state['strongest_rssi'] = smoothed_rssi
                
                # Convert RSSI to distance estimate and apply smoothing
                raw_distance = self.rssi_to_distance(smoothed_rssi)
                smoothed_distance = self.smooth_distance(raw_distance)
                
                # Update distance state
                self.bluetooth_state['target_distance'] = raw_distance  # Keep raw for immediate decisions
                self.bluetooth_state['smoothed_distance'] = smoothed_distance  # Use for display/logging
                self.bluetooth_state['distance_history'].append(smoothed_distance)
                
                # Signal improvement/degradation detection (using smoothed values)
                if previous_rssi and len(self.bluetooth_state['rssi_history']) >= 3:
                    recent_avg = sum(list(self.bluetooth_state['rssi_history'])[-3:]) / 3
                    older_avg = sum(list(self.bluetooth_state['rssi_history'])[-6:-3]) / 3 if len(self.bluetooth_state['rssi_history']) >= 6 else previous_rssi
                    
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
                
                # Use smoothed distance for mode activation decisions
                distance_for_decisions = smoothed_distance
                
                # Activate appropriate mode based on smoothed distance
                if distance_for_decisions <= self.target_bluetooth_distance + self.target_tolerance:
                    # At target distance - activate hold mode
                    if not self.bluetooth_state['holding_mode']:
                        self.bluetooth_state['holding_mode'] = True
                        self.bluetooth_state['at_target_start_time'] = current_time
                        self.stats['hold_activations'] += 1
                        self.stats['target_reaches'] += 1
                        print(f"🏁 TARGET HOLD: BlueCharm at {distance_for_decisions:.1f}m - holding position")
                elif distance_for_decisions <= self.approach_threshold:
                    # Close - activate approach mode
                    if not self.bluetooth_state['approaching_mode']:
                        self.bluetooth_state['approaching_mode'] = True
                        self.stats['approach_activations'] += 1
                        print(f"🎯 APPROACH MODE: BlueCharm at {distance_for_decisions:.1f}m - gentle approach")
                elif distance_for_decisions <= self.tracking_threshold:
                    # Medium range - activate tracking mode
                    if not self.bluetooth_state['tracking_mode']:
                        self.bluetooth_state['tracking_mode'] = True
                        self.bluetooth_state['movement_start_rssi'] = smoothed_rssi
                        self.bluetooth_state['movement_start_time'] = current_time
                        self.stats['tracking_activations'] += 1
                        print(f"🚶 TRACKING MODE: BlueCharm at {distance_for_decisions:.1f}m - forward tracking")
                
                # Log every 5th detection with mode info (using smoothed values for display)
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
                    print(f"   RSSI: {smoothed_rssi:.0f}dBm (raw: {raw_rssi}) | Distance: ~{distance_for_decisions:.1f}m | Avg: {avg_rssi:.0f}dBm")
                    if self.stats['rssi_outliers_rejected'] > 0:
                        print(f"   Outliers rejected: {self.stats['rssi_outliers_rejected']}")
    
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
    
    def is_path_clear(self, ultrasonic_distance):
        """Check if path ahead is clear for smart turning"""
        return ultrasonic_distance > self.clear_distance_threshold
    
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
        """Start the smart navigator with enhanced Bluetooth tracking"""
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
            print("📡 BlueCharm smart tracking scanner started")
        
        self.arduino_thread.start()
        self.navigation_thread.start()
        
        print("🚀 Smart Navigator V5 started")
        print("🧠 Ready for intelligent navigation with smart obstacle avoidance")
        return True
    
    def bluetooth_thread_worker(self):
        """Dedicated thread worker for Bluetooth scanning"""
        print("📡 Starting BlueCharm smart tracking thread...")
        
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
        """Main async function for Bluetooth smart tracking scanning"""
        print("📡 BlueCharm smart tracking scanner ready")
        
        scanner = None
        
        try:
            while self.running:
                # Only scan in autonomous mode
                if self.robot_state.get('mode', 0) == 2:
                    if not self.bluetooth_state['scanner_running']:
                        # Start scanner
                        print("📡 Starting BLE smart tracking scanner...")
                        scanner = BleakScanner(self.bluetooth_detection_callback)
                        await scanner.start()
                        self.bluetooth_state['scanner_running'] = True
                        print("📡 BLE smart tracking scanner active")
                    
                    # Check for detection timeout and mode management
                    current_time = time.time()
                    time_since_detection = current_time - self.bluetooth_state['last_detection']
                    
                    # Reset modes on timeout
                    if time_since_detection > self.bluetooth_timeout:
                        if self.bluetooth_state['target_rssi'] is not None:
                            with self.detection_lock:
                                self.bluetooth_state['target_rssi'] = None
                                self.bluetooth_state['target_distance'] = None
                                self.bluetooth_state['smoothed_distance'] = None
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
                        print("📡 Stopping BLE smart tracking scanner...")
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
                    print("📡 Cleaning up BLE smart tracking scanner...")
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
        """Main autonomous navigation logic with smart obstacle avoidance"""
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
                
                # V5: Enhanced navigation with smart obstacle avoidance
                left_speed, right_speed = self.compute_smart_navigation(ultrasonic_distance, current_time)
                
                # Send motor commands to Arduino
                self.send_motor_command(left_speed, right_speed)
                
                # Status logging every 3 seconds
                if current_time % 3 < 0.5:
                    self.log_smart_status(ultrasonic_distance)
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                self.send_motor_command(0, 0)
                time.sleep(1.0)
            
            time.sleep(0.5)  # 2Hz navigation loop
    
    def compute_smart_navigation(self, ultrasonic_distance, current_time):
        """
        Enhanced navigation with smart obstacle avoidance and signal tracking
        
        Smart state machine with intelligent turning:
        - SEARCHING: Looking for BlueCharm signal
        - FORWARD_TRACKING: Moving forward while monitoring signal strength
        - TURN_TESTING: Testing left/right turns for signal improvement
        - APPROACHING: Gentle approach to target
        - HOLDING: Maintaining target distance
        - SMART_AVOIDING: Intelligent obstacle avoidance with minimal turning
        - BACKING, STUCK_RECOVERY: Enhanced recovery mechanisms
        """
        time_in_state = current_time - self.nav_start_time
        base_left_speed = 0
        base_right_speed = 0
        
        # SAFETY FIRST: Ultrasonic obstacle detection always takes priority
        if ultrasonic_distance < self.danger_threshold and self.nav_state not in ['SMART_AVOIDING', 'BACKING', 'STUCK_RECOVERY']:
            # Emergency avoidance with smart turning
            self.nav_state = 'SMART_AVOIDING'
            self.nav_start_time = current_time
            self.turn_start_time = current_time
            self.smart_turning = True
            self.turn_direction = self.choose_turn_direction()
            self.stats['obstacle_avoidances'] += 1
            self.stats['smart_turns'] += 1
            print(f"🧠 SMART AVOIDING! Obstacle at {ultrasonic_distance:.1f}cm - turning {self.turn_direction} until clear")
        
        # Get current Bluetooth state (thread-safe)
        bluetooth_distance = None
        smoothed_distance = None
        tracking_mode = False
        approaching_mode = False
        holding_mode = False
        signal_improving = False
        signal_degrading = False
        
        with self.detection_lock:
            bluetooth_distance = self.bluetooth_state.get('target_distance')
            smoothed_distance = self.bluetooth_state.get('smoothed_distance')
            tracking_mode = self.bluetooth_state.get('tracking_mode', False)
            approaching_mode = self.bluetooth_state.get('approaching_mode', False)
            holding_mode = self.bluetooth_state.get('holding_mode', False)
            signal_improving = self.bluetooth_state.get('signal_improving', False)
            signal_degrading = self.bluetooth_state.get('signal_degrading', False)
        
        # Use smoothed distance for display but raw distance for critical decisions
        display_distance = smoothed_distance if smoothed_distance else bluetooth_distance
        
        # Enhanced state machine with smart avoidance
        if self.nav_state == 'SMART_AVOIDING':
            # Intelligent obstacle avoidance - turn until path is clear
            turn_time = current_time - self.turn_start_time
            
            if turn_time < self.min_turn_time:
                # Minimum turn time to ensure movement
                if self.turn_direction == 'left':
                    base_left_speed = -self.turn_speed
                    base_right_speed = self.turn_speed
                else:
                    base_left_speed = self.turn_speed
                    base_right_speed = -self.turn_speed
            elif self.is_path_clear(ultrasonic_distance):
                # Path is clear - stop turning and resume appropriate mode
                self.smart_turning = False
                
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
                print(f"✅ SMART CLEAR: Path clear at {ultrasonic_distance:.1f}cm after {turn_time:.1f}s turn - resuming {self.nav_state.lower()}")
            elif turn_time > self.max_turn_time:
                # Maximum turn time reached - try backing up
                self.nav_state = 'BACKING'
                self.nav_start_time = current_time
                self.stuck_counter += 1
                self.smart_turning = False
                print(f"🔄 SMART STUCK: Max turn time reached - backing up (count: {self.stuck_counter})")
            else:
                # Continue turning until clear
                if self.turn_direction == 'left':
                    base_left_speed = -self.turn_speed
                    base_right_speed = self.turn_speed
                else:
                    base_left_speed = self.turn_speed
                    base_right_speed = -self.turn_speed
                
                if turn_time % 2 < 1:  # Log every 2 seconds
                    print(f"🧠 SMART TURNING: {self.turn_direction} for {turn_time:.1f}s, obstacle at {ultrasonic_distance:.1f}cm")
        
        elif self.nav_state == 'BACKING':
            # Enhanced backing with smart recovery
            backup_speed = self.reverse_speed
            if self.stuck_counter >= 3:
                backup_speed = min(130, self.reverse_speed * 1.5)
            
            base_left_speed = -backup_speed
            base_right_speed = -backup_speed
            
            # Smart backup duration based on stuck level
            required_backup_time = self.backup_duration
            if self.stuck_counter >= 3:
                required_backup_time = self.aggressive_backup_duration
            
            if time_in_state > required_backup_time:
                if self.stuck_counter >= self.stuck_threshold:
                    # Multiple stuck detections - try different strategy
                    self.nav_state = 'STUCK_RECOVERY'
                    self.nav_start_time = current_time
                    self.stats['stuck_recoveries'] += 1
                    print(f"🆘 STUCK RECOVERY: Multiple stuck detections ({self.stuck_counter})")
                else:
                    # Return to smart avoidance with opposite direction
                    self.nav_state = 'SMART_AVOIDING'
                    self.nav_start_time = current_time
                    self.turn_start_time = current_time
                    self.smart_turning = True
                    self.turn_direction = 'left' if self.turn_direction == 'right' else 'right'
                    print(f"🧠 SMART RETRY: Switching to {self.turn_direction} turn (stuck: {self.stuck_counter})")
        
        elif self.nav_state == 'STUCK_RECOVERY':
            # Enhanced stuck recovery with smart techniques
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
        
        elif self.nav_state == 'HOLDING' and holding_mode and display_distance:
            # Target hold mode - maintain precise 0.5m distance (using smoothed distance)
            distance_error = display_distance - self.target_bluetooth_distance
            
            if abs(distance_error) <= self.target_tolerance:
                # At perfect distance - stop movement
                base_left_speed = 0
                base_right_speed = 0
                if time_in_state % 5 < 1:  # Log every 5 seconds
                    print(f"🏁 TARGET PERFECT: Holding {display_distance:.1f}m distance")
            elif distance_error > 0:
                # Too far - gentle approach
                adjustment_speed = min(self.hold_speed, abs(distance_error) * 40)
                base_left_speed = adjustment_speed
                base_right_speed = adjustment_speed
                if time_in_state % 3 < 1:
                    print(f"📏 ADJUST CLOSER: {display_distance:.1f}m → {self.target_bluetooth_distance}m")
            else:
                # Too close - gentle retreat
                adjustment_speed = min(self.hold_speed, abs(distance_error) * 40)
                base_left_speed = -adjustment_speed
                base_right_speed = -adjustment_speed
                if time_in_state % 3 < 1:
                    print(f"📏 ADJUST FARTHER: {display_distance:.1f}m → {self.target_bluetooth_distance}m")
            
            # Exit hold mode if signal lost or too far
            if not holding_mode or (display_distance and display_distance > self.approach_threshold):
                self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                print("📡 HOLD LOST: Signal lost or too far - returning to search")
        
        elif self.nav_state == 'APPROACHING' and approaching_mode and display_distance:
            # Gentle approach mode (using smoothed distance for stability)
            if display_distance <= self.target_bluetooth_distance + self.target_tolerance:
                # Close enough to switch to hold mode
                self.nav_state = 'HOLDING'
                self.nav_start_time = current_time
                print(f"🎯 SWITCHING TO HOLD: {display_distance:.1f}m - target reached")
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
                    print(f"🎯 APPROACHING: Moving toward {display_distance:.1f}m signal at {speed:.0f} speed")
            
            # Exit approach mode if signal lost
            if not approaching_mode:
                self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                print("📡 APPROACH LOST: Signal lost - returning to search")
        
        elif self.nav_state == 'FORWARD_TRACKING' and tracking_mode:
            # Forward tracking mode (using raw distance for immediate decisions)
            if bluetooth_distance and bluetooth_distance <= self.approach_threshold:
                # Close enough to switch to approach mode
                self.nav_state = 'APPROACHING'
                self.nav_start_time = current_time
                print(f"🎯 SWITCHING TO APPROACH: {display_distance:.1f}m")
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
                    print(f"🚶 FORWARD TRACKING: {display_distance:.1f}m signal {signal_text}")
            
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
                distance_text = f"{display_distance:.1f}m" if display_distance else "unknown"
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
    
    def log_smart_status(self, ultrasonic_distance):
        """Log current navigation and smart tracking status"""
        runtime = time.time() - self.stats['start_time']
        mode_text = ['MANUAL', 'ASSISTED', 'AUTONOMOUS'][self.robot_state.get('mode', 0)]
        
        status_line = f"🧭 {mode_text} | {self.nav_state}"
        
        # Add smart turning indicator
        if self.smart_turning:
            status_line += f"🧠"
        
        status_line += f" | US: {ultrasonic_distance:.1f}cm"
        
        # Add Bluetooth smart tracking info if available (thread-safe)
        if self.bluetooth_state['enabled']:
            with self.detection_lock:
                smoothed_distance = self.bluetooth_state.get('smoothed_distance')
                if smoothed_distance:
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
                    
                    status_line += f" | BT: {smoothed_distance:.1f}m ({bt_rssi:.0f}dBm {mode_symbol})"
                    
                    # Add outlier rejection count if any
                    if self.stats['rssi_outliers_rejected'] > 0:
                        status_line += f" | Outliers: {self.stats['rssi_outliers_rejected']}"
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
            self.print_smart_statistics()
    
    def print_smart_statistics(self):
        """Print comprehensive navigation and smart tracking statistics"""
        runtime = time.time() - self.stats['start_time']
        
        print("📊 === Smart Navigator V5 Statistics ===")
        print(f"   Runtime: {runtime:.0f} seconds")
        print(f"   Distance measurements: {self.stats['distance_measurements']}")
        print(f"   Obstacle avoidances: {self.stats['obstacle_avoidances']}")
        print(f"   Emergency stops: {self.stats['emergency_stops']}")
        print(f"   Stuck recoveries: {self.stats['stuck_recoveries']}")
        
        # Smart navigation statistics
        print("🧠 === Smart Navigation Statistics ===")
        print(f"   Smart turns: {self.stats['smart_turns']}")
        print(f"   Tracking activations: {self.stats['tracking_activations']}")
        print(f"   Approach activations: {self.stats['approach_activations']}")
        print(f"   Hold activations: {self.stats['hold_activations']}")
        print(f"   Target reaches: {self.stats['target_reaches']}")
        print(f"   Signal improvements: {self.stats['signal_improvements']}")
        print(f"   Course corrections: {self.stats['course_corrections']}")
        print(f"   Turn tests: {self.stats['turn_tests']}")
        
        # RSSI smoothing statistics
        print("📊 === RSSI Smoothing Statistics ===")
        print(f"   Outliers rejected: {self.stats['rssi_outliers_rejected']}")
        
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
                
                if len(self.bluetooth_state['rssi_history']) > 0:
                    avg_rssi = sum(self.bluetooth_state['rssi_history']) / len(self.bluetooth_state['rssi_history'])
                    print(f"   Average RSSI: {avg_rssi:.1f} dBm")
                
                if len(self.bluetooth_state['distance_history']) > 0:
                    avg_distance = sum(self.bluetooth_state['distance_history']) / len(self.bluetooth_state['distance_history'])
                    print(f"   Average smoothed distance: {avg_distance:.1f}m")
                
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
        print("🛑 Stopping Smart Navigator V5...")
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
            self.print_smart_statistics()
        
        print("👋 Smart Navigator V5 shutdown complete")

def main():
    """Main entry point for Bluetooth Smart Navigator V5"""
    print("=" * 80)
    print("🤖 BLUETOOTH SMART NAVIGATOR V5")
    print("=" * 80)
    print()
    print("INTELLIGENT NAVIGATION FEATURES:")
    print("  ✅ Ultrasonic obstacle detection and avoidance")
    print("  ✅ Enhanced stuck recovery system")
    print("  ✅ Arduino gatekeeper integration")
    print("  ✅ Real-time Bluetooth beacon ranging (0.8m - 11.2m)")
    print("  ✅ Working RSSI values with noise reduction")
    print("  ✅ Thread-safe callback-based BLE scanning")
    print("  🚶 PROVEN: Forward movement signal strength tracking")
    print("  🔄 PROVEN: Turn-and-test behavior for course corrections")
    print("  🧠 NEW: Intelligent obstacle avoidance (turn only until clear)")
    print("  📊 NEW: RSSI smoothing to reduce distance jumps")
    print("  🎯 NEW: Enhanced approach stability")
    print("  🏁 NEW: Precise target hold at 0.5m ±0.15m")
    print("  ✅ Safety-first design with ultrasonic priority")
    print("  ✅ Clean shutdown with no task warnings")
    print()
    print("SMART NAVIGATION BEHAVIOR:")
    print("  🔍 SEARCHING: Look for BlueCharm signal")
    print("  🚶 FORWARD_TRACKING: Move forward while monitoring signal")
    print("  🔄 TURN_TESTING: Test left/right turns when signal degrades")
    print("  🧠 SMART_AVOIDING: Turn only until path is clear")
    print("  🎯 APPROACHING: Gentle approach to target")
    print("  🏁 HOLDING: Maintain precise 0.5m distance")
    print("  🛡️ SAFETY: Ultrasonic overrides all Bluetooth behavior")
    print()
    print("INTELLIGENT IMPROVEMENTS:")
    print(f"  🧠 Smart turning: Stop when clear >{80}cm | Min {0.8}s | Max {4.0}s")
    print(f"  📊 RSSI smoothing: 30% new data | Distance smoothing: 40% new data")
    print(f"  🎯 Outlier rejection: >2x distance change rejection")
    print(f"  📈 Signal tracking: 2dBm improvement | 3dBm degradation triggers")
    print()
    
    if not BLUETOOTH_AVAILABLE:
        print("⚠️ WARNING: Bluetooth library not available")
        print("   Install with: pip install bleak")
        print("   Navigator will run in ultrasonic-only mode")
        print()
    
    navigator = BluetoothSmartNavigator()
    
    try:
        if navigator.start():
            print("✅ Smart Navigator V5 ready - switch to autonomous mode")
            print("📡 BlueCharm smart tracking will activate automatically")
            print("🧠 Rover will use intelligent navigation with minimal turning")
            print("📊 RSSI smoothing will reduce distance reading jumps")
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