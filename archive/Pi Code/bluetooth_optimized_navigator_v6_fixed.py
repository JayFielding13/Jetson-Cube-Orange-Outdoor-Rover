#!/usr/bin/env python3
"""
Bluetooth Optimized Navigator V6 Fixed
================================================================================
HOLDING mode bug fix - BlueCharm movement detection and smoothing reset

FEATURES:
- Built on proven V4 forward movement signal tracking foundation
- Optimized tracking range (15m threshold instead of 3m)
- Reduced smoothing aggressiveness for better signal detection
- Longer movement phases for proper signal gradient analysis
- Intelligent obstacle avoidance from V5 (but less aggressive)
- Target hold logic for precise 0.5m distance maintenance
- Safety-first design: ultrasonic navigation takes absolute priority
- Real-time Bluetooth beacon distance tracking with balanced stability
- Working RSSI values with smart noise reduction
- Thread-safe integration with clean shutdown

FIXES FROM V6:
- 🐛 HOLDING MODE BUG FIX: Detects when BlueCharm moves significantly
- 📡 RSSI Change Detection: Exits HOLDING mode if RSSI changes >10dBm
- 🔄 Smoothing Reset: Clears distance smoothing when BlueCharm moves
- 🎯 Proper Movement Tracking: No more "stuck" distance readings
- 🛡️ State Management: Clean holding state initialization and cleanup

BUG FIXED:
Previously, when BlueCharm moved after rover reached HOLDING mode at 0.5m:
- RSSI changed dramatically (-67dBm to -84dBm)
- Raw distance calculation showed ~13m+ 
- But outlier rejection kept smoothed distance "stuck" at 0.5m
- HOLDING mode never exited because it only checked smoothed distance

NOW FIXED:
- Monitors RSSI when entering HOLDING mode
- Detects >10dBm RSSI changes indicating BlueCharm movement
- Resets distance smoothing to allow fresh calculations
- Exits HOLDING mode and returns to SEARCHING when BlueCharm moves

HARDWARE:
- Raspberry Pi 4B with Bluetooth 5.0
- Arduino gatekeeper with ultrasonic sensor
- BlueCharm BLE beacon (MAC: DD:34:02:09:CA:1E)
- Cytron MDDS30 motor driver

Part of the Mini Rover Development Project
Author: Developed incrementally with Jay Fielding
Version: 6.0 Fixed - HOLDING Mode Bug Fix
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

class BluetoothOptimizedNavigator:
    """
    Optimized Bluetooth navigator with effective signal gradient tracking
    
    This is the V6 Fixed release featuring:
    - Proven forward movement signal tracking from V4
    - Optimized tracking thresholds for real-world signal ranges
    - Balanced RSSI smoothing for stability without over-filtering
    - Longer movement phases for effective signal gradient detection
    - Smart obstacle avoidance with minimal over-turning
    - Enhanced approach stability and target holding
    - Thread-safe callback-based BLE scanning
    - Safety-first design with ultrasonic override priority
    - FIXED: HOLDING mode bug where distance readings got stuck
    """
    
    def __init__(self, arduino_port='/dev/ttyUSB0', baud=115200):
        """Initialize the optimized Bluetooth navigator with HOLDING bug fix"""
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
        
        # Optimized Bluetooth state with balanced smoothing
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
            'rssi_history': deque(maxlen=8),        # RSSI history for smoothing (reduced)
            'distance_history': deque(maxlen=6),    # Distance history for smoothing (reduced)
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
            'last_direction_change': 0,             # Last course correction time
            'holding_start_rssi': None,             # RSSI when HOLDING mode started
            'holding_start_time': 0                 # Time when HOLDING mode started
        }
        
        # Thread-safe detection lock
        self.detection_lock = threading.Lock()
        
        # Enhanced navigation state machine
        self.nav_state = 'SEARCHING'  # Start in search mode
        self.nav_start_time = time.time()
        self.turn_direction = 'left'
        self.stuck_counter = 0
        
        # Optimized smart avoidance state (less aggressive)
        self.smart_turning = False
        self.turn_start_time = 0
        self.min_turn_time = 1.0      # Minimum turn time before checking (increased)
        self.max_turn_time = 3.5      # Maximum turn time before giving up (reduced)
        self.clear_distance_threshold = 90.0  # Distance to consider path clear (increased)
        
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
        self.stuck_threshold = 4       # Consecutive stuck detections
        self.backup_duration = 1.8     # Time to back up when stuck
        self.turn_duration = 1.5       # Time to turn when avoiding
        self.aggressive_backup_duration = 2.5  # Extended backup time
        
        # Optimized Bluetooth navigation parameters
        self.target_bluetooth_distance = 0.5   # Target distance (m)
        self.target_tolerance = 0.15           # Distance tolerance (m)
        self.bluetooth_weight = 0.35           # Bluetooth influence weight (reduced)
        self.tracking_threshold = 15.0         # Start tracking when signal detected (15m - MAJOR INCREASE)
        self.approach_threshold = 1.2          # Switch to approach mode (m)
        self.signal_improvement_threshold = 1.5 # RSSI improvement to continue direction (dBm)
        self.signal_degradation_threshold = 4.0 # RSSI degradation to change direction (dBm)
        self.bluetooth_timeout = 12.0          # Signal timeout (s)
        self.tracking_duration = 8.0           # Time to track forward before evaluation (increased)
        self.turn_test_duration = 2.5          # Time for each turn test
        self.exploration_interval = 10.0       # Search turn interval (increased)
        
        # Optimized RSSI and distance smoothing (less aggressive)
        self.rssi_smoothing_weight = 0.5       # 50% new data vs 50% history (increased)
        self.distance_smoothing_weight = 0.6   # 60% new data vs 40% history (increased)
        self.outlier_rejection_threshold = 3.0 # Reject distances >3x different (balanced)
        
        # Performance statistics
        self.stats = {
            'distance_measurements': 0,
            'obstacle_avoidances': 0,
            'emergency_stops': 0,
            'stuck_recoveries': 0,
            'smart_turns': 0,
            'tracking_activations': 0,
            'approach_activations': 0,
            'hold_activations': 0,
            'target_reaches': 0,
            'signal_improvements': 0,
            'course_corrections': 0,
            'turn_tests': 0,
            'long_tracking_phases': 0,
            'bluetooth_detections': 0,
            'bluetooth_callback_events': 0,
            'rssi_outliers_rejected': 0
        }
        
        print("🚀 Bluetooth Optimized Navigator V6 Fixed Initialized")
        print(f"📊 Navigation: Danger={self.danger_threshold}cm | Caution={self.caution_threshold}cm | Safe={self.safe_distance}cm")
        print(f"📡 Bluetooth: {'Enabled' if self.bluetooth_state['enabled'] else 'Disabled'} | Target: {self.bluetooth_state['target_mac']}")
        print(f"🚶 Tracking: Target distance {self.target_bluetooth_distance}m ±{self.target_tolerance}m | Influence {self.bluetooth_weight*100:.0f}%")
        print(f"📈 Signal: {self.signal_improvement_threshold}dBm improvement | {self.signal_degradation_threshold}dBm degradation triggers")
        print(f"📏 Ranges: Tracking <{self.tracking_threshold}m | Approach <{self.approach_threshold}m | Target {self.target_bluetooth_distance}m")
        print(f"⏱️ Timing: {self.tracking_duration}s tracking phases | {self.exploration_interval}s exploration")
        print(f"🧠 Smart Turn: Clear >{self.clear_distance_threshold}cm | {self.min_turn_time}-{self.max_turn_time}s")
        print(f"📊 Smoothing: {self.rssi_smoothing_weight*100:.0f}% new RSSI | {self.distance_smoothing_weight*100:.0f}% new distance | >{self.outlier_rejection_threshold}x outlier threshold")
        print(f"🐛 HOLDING BUG FIX: Will detect BlueCharm movement >10dBm RSSI change")
        print(f"🛡️ Safety: Ultrasonic navigation takes absolute priority")
    
    def smooth_rssi(self, new_rssi):
        """Apply balanced smoothing to RSSI readings"""
        if not self.bluetooth_state['rssi_history']:
            # First reading
            return new_rssi
        
        # Calculate weighted average with recent history (less aggressive)
        history_weight = 1.0 - self.rssi_smoothing_weight
        recent_avg = sum(list(self.bluetooth_state['rssi_history'])[-3:]) / min(3, len(self.bluetooth_state['rssi_history']))
        
        smoothed_rssi = (self.rssi_smoothing_weight * new_rssi) + (history_weight * recent_avg)
        return smoothed_rssi
    
    def smooth_distance(self, new_distance):
        """Apply balanced smoothing to distance readings with reduced outlier rejection"""
        if not self.bluetooth_state['distance_history']:
            # First reading
            return new_distance
        
        # Less aggressive outlier rejection
        recent_avg = sum(list(self.bluetooth_state['distance_history'])[-3:]) / min(3, len(self.bluetooth_state['distance_history']))
        distance_ratio = new_distance / recent_avg if recent_avg > 0 else 1.0
        
        if distance_ratio > self.outlier_rejection_threshold or distance_ratio < (1.0 / self.outlier_rejection_threshold):
            # Reject extreme outliers only
            self.stats['rssi_outliers_rejected'] += 1
            return self.bluetooth_state['smoothed_distance'] if self.bluetooth_state['smoothed_distance'] else new_distance
        
        # Calculate weighted average with more emphasis on new data
        history_weight = 1.0 - self.distance_smoothing_weight
        smoothed_distance = (self.distance_smoothing_weight * new_distance) + (history_weight * recent_avg)
        
        return smoothed_distance
    
    def bluetooth_detection_callback(self, device, advertisement_data):
        """
        Thread-safe callback for real-time BlueCharm detection with optimized smoothing
        
        Enhanced with balanced RSSI and distance smoothing for effective signal tracking
        """
        if device.address.upper() == self.bluetooth_state['target_mac'].upper():
            # Thread-safe processing of detection
            with self.detection_lock:
                # Extract real RSSI from advertisement data
                raw_rssi = advertisement_data.rssi
                current_time = time.time()
                
                # Apply balanced RSSI smoothing
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
                
                # Convert RSSI to distance estimate and apply balanced smoothing
                raw_distance = self.rssi_to_distance(smoothed_rssi)
                smoothed_distance = self.smooth_distance(raw_distance)
                
                # Update distance state
                self.bluetooth_state['target_distance'] = raw_distance  # Keep raw for immediate decisions
                self.bluetooth_state['smoothed_distance'] = smoothed_distance  # Use for display/logging
                self.bluetooth_state['distance_history'].append(smoothed_distance)
                
                # Signal improvement/degradation detection (using smoothed values)
                if previous_rssi and len(self.bluetooth_state['rssi_history']) >= 4:
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
                    else:
                        # Signal stable
                        self.bluetooth_state['signal_improving'] = False
                        self.bluetooth_state['signal_degrading'] = False
                
                # Update navigation modes based on optimized distance
                self.update_navigation_modes(smoothed_distance)
                
                # Periodic detailed logging with balanced frequency
                if self.bluetooth_state['detection_count'] % 5 == 0:
                    print(f"📡 BlueCharm {self.determine_mode_text()} #{self.bluetooth_state['detection_count']}:")
                    print(f"   RSSI: {smoothed_rssi:.0f}dBm (raw: {raw_rssi}) | Distance: ~{smoothed_distance:.1f}m | Avg: {sum(list(self.bluetooth_state['rssi_history'])[-3:]) / min(3, len(self.bluetooth_state['rssi_history'])):.0f}dBm")
                    if self.stats['rssi_outliers_rejected'] > 0:
                        print(f"   Outliers rejected: {self.stats['rssi_outliers_rejected']}")
    
    def determine_mode_text(self):
        """Determine display text for current tracking mode"""
        if self.bluetooth_state['holding_mode']:
            return "HOLDING"
        elif self.bluetooth_state['approaching_mode']:
            return "APPROACH"
        elif self.bluetooth_state['tracking_mode']:
            if self.bluetooth_state['signal_improving']:
                return "TRACKING↗"
            elif self.bluetooth_state['signal_degrading']:
                return "TRACKING↘"
            else:
                return "TRACKING"
        else:
            return "PASSIVE"
    
    def update_navigation_modes(self, distance):
        """Update Bluetooth navigation modes based on optimized distance thresholds"""
        if distance is None:
            return
        
        # Reset all modes first
        self.bluetooth_state['tracking_mode'] = False
        self.bluetooth_state['approaching_mode'] = False
        self.bluetooth_state['holding_mode'] = False
        
        # Determine appropriate mode based on optimized distance
        if distance <= self.target_bluetooth_distance + self.target_tolerance:
            # Within target range - hold position
            self.bluetooth_state['holding_mode'] = True
            if not hasattr(self, '_hold_mode_announced'):
                print(f"🏁 TARGET HOLD: BlueCharm at {distance:.1f}m - holding position")
                self.stats['hold_activations'] += 1
                self.stats['target_reaches'] += 1
                self._hold_mode_announced = True
        elif distance <= self.approach_threshold:
            # Close enough for gentle approach
            self.bluetooth_state['approaching_mode'] = True
            if not hasattr(self, '_approach_mode_announced'):
                print(f"🎯 APPROACH MODE: BlueCharm at {distance:.1f}m - gentle approach")
                self.stats['approach_activations'] += 1
                self._approach_mode_announced = True
        elif distance <= self.tracking_threshold:
            # Within tracking range for forward movement
            self.bluetooth_state['tracking_mode'] = True
            if not hasattr(self, '_tracking_mode_announced'):
                print(f"🚶 TRACKING MODE: BlueCharm at {distance:.1f}m - forward tracking activated")
                print(f"📡 SIGNAL DETECTED: Switching to forward_tracking mode ({distance:.1f}m)")
                self.stats['tracking_activations'] += 1
                self._tracking_mode_announced = True
        
        # Reset announcements when changing between modes
        if not self.bluetooth_state['tracking_mode']:
            self._tracking_mode_announced = False
        if not self.bluetooth_state['approaching_mode']:
            self._approach_mode_announced = False
        if not self.bluetooth_state['holding_mode']:
            self._hold_mode_announced = False
    
    def rssi_to_distance(self, rssi):
        """Convert RSSI to estimated distance using calibrated formula"""
        if rssi >= -50:
            return 0.5  # Very close
        elif rssi >= -60:
            return 0.8 + ((-50 - rssi) * 0.3)  # 0.8-3.8m
        elif rssi >= -70:
            return 3.8 + ((-60 - rssi) * 0.6)  # 3.8-9.8m  
        elif rssi >= -80:
            return 9.8 + ((-70 - rssi) * 1.2)  # 9.8-21.8m
        else:
            return 21.8 + ((-80 - rssi) * 2.0)  # >21.8m
    
    def connect_arduino(self):
        """Establish connection to Arduino gatekeeper"""
        try:
            self.arduino = serial.Serial(self.arduino_port, self.baud, timeout=0.1)
            time.sleep(2)  # Allow Arduino to reset
            print(f"✅ Arduino connected on {self.arduino_port}")
            return True
        except Exception as e:
            print(f"❌ Arduino connection failed: {e}")
            return False
    
    def start_navigation(self):
        """Start the main navigation system"""
        if not self.connect_arduino():
            return False
        
        self.running = True
        
        # Start Bluetooth tracking thread if available
        if self.bluetooth_state['enabled']:
            self.bluetooth_thread = threading.Thread(target=self.bluetooth_thread_worker, daemon=True)
            self.bluetooth_thread.start()
        
        # Start Arduino communication thread
        self.arduino_thread = threading.Thread(target=self.arduino_loop, daemon=True)
        self.arduino_thread.start()
        
        print("🚀 Optimized Navigator V6 Fixed started")
        print("🎯 Ready for effective signal gradient tracking with longer movement phases")
        print("🐛 HOLDING mode bug fixed - will detect BlueCharm movement")
        return True
    
    def bluetooth_thread_worker(self):
        """Dedicated thread worker for Bluetooth scanning"""
        print("📡 Starting BlueCharm optimized tracking thread...")
        
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
        """Main async function for Bluetooth optimized tracking scanning"""
        print("📡 BlueCharm optimized tracking scanner ready")
        
        scanner = None
        
        try:
            while self.running:
                # Only scan in autonomous mode
                if self.robot_state.get('mode', 0) == 2:
                    if not self.bluetooth_state['scanner_running']:
                        # Start scanner
                        print("📡 Starting BLE optimized tracking scanner...")
                        scanner = BleakScanner(self.bluetooth_detection_callback)
                        await scanner.start()
                        self.bluetooth_state['scanner_running'] = True
                        print("📡 BLE optimized tracking scanner active")
                    
                    # Check for detection timeout and mode management
                    current_time = time.time()
                    time_since_detection = current_time - self.bluetooth_state['last_detection']
                    
                    # Reset modes on timeout (longer timeout)
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
                                    self.bluetooth_state['holding_start_rssi'] = None  # Reset holding state
                
                else:
                    # Stop scanner when not in autonomous mode
                    if self.bluetooth_state['scanner_running'] and scanner:
                        print("📡 Stopping BLE optimized tracking scanner...")
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
                    print("📡 Cleaning up BLE optimized tracking scanner...")
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
                # Try to read Arduino data
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode('utf-8').strip()
                    if line:
                        try:
                            data = json.loads(line)
                            self.process_arduino_data(data)
                            consecutive_failures = 0
                        except json.JSONDecodeError:
                            pass  # Ignore malformed JSON
                
                # Navigation decision and motor control
                self.navigate_and_control()
                time.sleep(0.05)  # 20Hz control loop
                
            except Exception as e:
                consecutive_failures += 1
                if consecutive_failures >= max_failures:
                    print(f"❌ Arduino communication failed: {e}")
                    break
                time.sleep(0.1)
    
    def process_arduino_data(self, data):
        """Process incoming Arduino sensor data"""
        self.robot_state.update(data)
        self.robot_state['last_update'] = time.time()
        self.stats['distance_measurements'] += 1
    
    def navigate_and_control(self):
        """Main navigation decision and motor control logic"""
        current_time = time.time()
        ultrasonic_distance = self.robot_state.get('ultrasonic_distance', 200.0)
        
        # Safety check - Arduino gatekeeper will override if needed
        if ultrasonic_distance < self.danger_threshold:
            self.stats['emergency_stops'] += 1
        elif ultrasonic_distance < self.caution_threshold:
            # Caution zone - let state machine handle but note it
            pass
        
        # Get base movement from navigation state machine
        base_left_speed, base_right_speed = self.navigation_state_machine(current_time, ultrasonic_distance)
        
        # Send motor commands to Arduino gatekeeper
        self.send_motor_command(base_left_speed, base_right_speed)
        
        # Display status with optimized frequency
        self.display_navigation_status(current_time, ultrasonic_distance)
    
    def navigation_state_machine(self, current_time, ultrasonic_distance):
        """Enhanced navigation state machine with optimized Bluetooth integration"""
        base_left_speed = 0
        base_right_speed = 0
        
        time_in_state = current_time - self.nav_start_time
        
        # Get Bluetooth state (thread-safe)
        with self.detection_lock:
            bluetooth_distance = self.bluetooth_state.get('smoothed_distance')  # Use smoothed for display
            bluetooth_signal = self.bluetooth_state.get('target_rssi')
            tracking_mode = self.bluetooth_state.get('tracking_mode', False)
            approaching_mode = self.bluetooth_state.get('approaching_mode', False)
            holding_mode = self.bluetooth_state.get('holding_mode', False)
            signal_improving = self.bluetooth_state.get('signal_improving', False)
            signal_degrading = self.bluetooth_state.get('signal_degrading', False)
        
        # Use smoothed distance for display, raw for decisions when needed
        display_distance = bluetooth_distance
        
        # Emergency ultrasonic override
        if ultrasonic_distance < self.danger_threshold:
            # Arduino gatekeeper handles this automatically
            self.nav_state = 'AVOIDING'
            self.nav_start_time = current_time
            self.stats['obstacle_avoidances'] += 1
            print(f"🚨 EMERGENCY! Obstacle at {ultrasonic_distance:.1f}cm - overriding tracking")
            return 0, 0  # Stop immediately
        
        # Smart obstacle avoidance with balanced turning
        elif self.nav_state == 'SMART_AVOIDING':
            if not self.smart_turning:
                # Initialize smart turn
                self.smart_turning = True
                self.turn_start_time = current_time
                print(f"🧠 SMART AVOIDING! Obstacle at {ultrasonic_distance:.1f}cm - turning {self.turn_direction} until clear")
            
            smart_turn_time = current_time - self.turn_start_time
            
            # Check if path is clear
            if ultrasonic_distance > self.clear_distance_threshold and smart_turn_time >= self.min_turn_time:
                # Path clear - resume previous activity
                self.smart_turning = False
                self.stats['smart_turns'] += 1
                
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
                print(f"✅ SMART CLEAR: Path clear at {ultrasonic_distance:.1f}cm after {smart_turn_time:.1f}s turn - resuming {self.nav_state.lower()}")
            elif smart_turn_time >= self.max_turn_time:
                # Max turn time reached - try backing up
                self.smart_turning = False
                self.nav_state = 'BACKING'
                self.nav_start_time = current_time
                print(f"🔄 SMART STUCK: Max turn time reached - backing up (count: {self.stuck_counter + 1})")
            else:
                # Continue turning
                if self.turn_direction == 'left':
                    base_left_speed = -self.turn_speed
                    base_right_speed = self.turn_speed
                else:
                    base_left_speed = self.turn_speed
                    base_right_speed = -self.turn_speed
                
                if smart_turn_time % 2 < 1:
                    print(f"🧠 SMART TURNING: {self.turn_direction} for {smart_turn_time:.1f}s, obstacle at {ultrasonic_distance:.1f}cm")
        
        # Standard obstacle avoidance states
        elif self.nav_state == 'AVOIDING':
            if ultrasonic_distance > self.safe_distance:
                # Obstacle cleared
                self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                print(f"✅ CLEAR: Path ahead at {ultrasonic_distance:.1f}cm - resuming searching")
            elif time_in_state > self.turn_duration:
                # Tried turning, now back up
                self.nav_state = 'BACKING'
                self.nav_start_time = current_time
                self.stuck_counter += 1
                print(f"🔄 STUCK: Backing up (count: {self.stuck_counter})")
            else:
                # Turn to avoid
                if self.turn_direction == 'left':
                    base_left_speed = -self.turn_speed
                    base_right_speed = self.turn_speed
                else:
                    base_left_speed = self.turn_speed
                    base_right_speed = -self.turn_speed
        
        elif self.nav_state == 'BACKING':
            required_backup_time = self.backup_duration
            if self.stuck_counter >= 2:
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
            # Enhanced stuck recovery
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
            # Target hold mode - maintain precise 0.5m distance
            
            # Initialize holding start RSSI if first time in HOLDING mode
            with self.detection_lock:
                if self.bluetooth_state['holding_start_rssi'] is None:
                    self.bluetooth_state['holding_start_rssi'] = self.bluetooth_state.get('target_rssi')
                    self.bluetooth_state['holding_start_time'] = current_time
                    print(f"🏁 HOLDING INITIALIZED: Started at {self.bluetooth_state['holding_start_rssi']:.0f}dBm")
                
                # Check for significant RSSI change indicating BlueCharm movement
                current_rssi = self.bluetooth_state.get('target_rssi')
                holding_start_rssi = self.bluetooth_state['holding_start_rssi']
                
                rssi_change_detected = False
                if current_rssi and holding_start_rssi:
                    rssi_change = abs(current_rssi - holding_start_rssi)
                    if rssi_change > 10.0:  # >10dBm change indicates significant movement
                        rssi_change_detected = True
                        print(f"🚨 BLUECHARM MOVED: RSSI changed {rssi_change:.1f}dBm ({holding_start_rssi:.0f} → {current_rssi:.0f})")
                        print(f"🔄 RESETTING TRACKING: BlueCharm moved - clearing smoothing and returning to search")
                        
                        # Reset smoothing to allow fresh distance calculations
                        self.bluetooth_state['smoothed_distance'] = None
                        self.bluetooth_state['distance_history'].clear()
                        self.bluetooth_state['rssi_history'].clear()
                        self.bluetooth_state['holding_start_rssi'] = None
            
            # Exit HOLDING mode if BlueCharm moved significantly
            if rssi_change_detected:
                self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                print("🔄 EXITING HOLD: BlueCharm moved - returning to search mode")
            else:
                # Normal holding behavior
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
                    with self.detection_lock:
                        self.bluetooth_state['holding_start_rssi'] = None
                    self.nav_state = 'SEARCHING'
                    self.nav_start_time = current_time
                    print("📡 HOLD LOST: Signal lost or too far - returning to search")
        
        elif self.nav_state == 'APPROACHING' and approaching_mode and display_distance:
            # Gentle approach mode
            if display_distance <= self.target_bluetooth_distance + self.target_tolerance:
                # Close enough to switch to hold mode
                with self.detection_lock:
                    self.bluetooth_state['holding_start_rssi'] = None  # Reset to initialize fresh
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
            # Extended forward tracking mode for effective signal gradient detection
            if bluetooth_distance and bluetooth_distance <= self.approach_threshold:
                # Close enough to switch to approach mode
                self.nav_state = 'APPROACHING'
                self.nav_start_time = current_time
                print(f"🎯 SWITCHING TO APPROACH: {display_distance:.1f}m")
            elif signal_degrading and time_in_state > self.tracking_duration:
                # Signal getting worse after sufficient tracking time - try turn test
                self.nav_state = 'TURN_TESTING'
                self.nav_start_time = current_time
                self.stats['course_corrections'] += 1
                self.stats['long_tracking_phases'] += 1
                with self.detection_lock:
                    self.bluetooth_state['turn_test_mode'] = True
                    self.bluetooth_state['test_direction'] = 'left'
                    self.bluetooth_state['turn_test_start_rssi'] = self.bluetooth_state['target_rssi']
                print(f"🔄 SIGNAL DEGRADING: Testing left turn after {time_in_state:.1f}s tracking")
            else:
                # Continue extended forward tracking
                if ultrasonic_distance < self.caution_threshold:
                    # Cautious movement when obstacles near
                    speed_factor = (ultrasonic_distance - self.danger_threshold) / (self.caution_threshold - self.danger_threshold)
                    speed = self.slow_speed * max(0.4, speed_factor)
                else:
                    # Normal tracking speed
                    speed = self.tracking_speed
                
                base_left_speed = speed
                base_right_speed = speed
                
                if time_in_state % 6 < 1:  # Log every 6 seconds for longer phases
                    signal_text = "↗improving" if signal_improving else "↘degrading" if signal_degrading else "stable"
                    print(f"🚶 EXTENDED TRACKING: {display_distance:.1f}m signal {signal_text} (t={time_in_state:.1f}s)")
            
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
                            # Neither direction helped - continue forward tracking
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
                    with self.detection_lock:
                        self.bluetooth_state['holding_start_rssi'] = None  # Reset to initialize fresh
                    self.nav_state = 'HOLDING'
                elif approaching_mode:
                    self.nav_state = 'APPROACHING'
                elif tracking_mode:
                    self.nav_state = 'FORWARD_TRACKING'
                
                self.nav_start_time = current_time
                distance_text = f"{display_distance:.1f}m" if display_distance else "unknown"
                print(f"📡 SIGNAL DETECTED: Switching to {self.nav_state.lower()} mode ({distance_text})")
            elif time_in_state > self.exploration_interval:
                # Execute search turn (longer intervals)
                base_left_speed = -self.turn_speed
                base_right_speed = self.turn_speed
                
                if time_in_state > self.exploration_interval * 1.5:
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
        
        # Handle smart obstacle avoidance
        if ultrasonic_distance < self.caution_threshold and self.nav_state not in ['SMART_AVOIDING', 'AVOIDING', 'BACKING', 'STUCK_RECOVERY']:
            self.nav_state = 'SMART_AVOIDING'
            self.nav_start_time = current_time
            self.turn_start_time = current_time
            self.smart_turning = True
            self.turn_direction = self.choose_turn_direction()
            return 0, 0  # Let next iteration handle the turn
        
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
        if not self.arduino:
            return
        
        try:
            # Clamp speeds to valid range
            left_speed = max(-255, min(255, int(left_speed)))
            right_speed = max(-255, min(255, int(right_speed)))
            
            command = {
                'cmd': 'motor',
                'left': left_speed,
                'right': right_speed
            }
            
            command_json = json.dumps(command) + '\n'
            self.arduino.write(command_json.encode())
            
        except Exception as e:
            print(f"❌ Motor command error: {e}")
    
    def display_navigation_status(self, current_time, ultrasonic_distance):
        """Display navigation status with optimized frequency"""
        # Status display every 3 seconds
        if not hasattr(self, 'last_status_time'):
            self.last_status_time = 0
        
        if current_time - self.last_status_time >= 3.0:
            # Get thread-safe Bluetooth state
            with self.detection_lock:
                bluetooth_distance = self.bluetooth_state.get('smoothed_distance')
                bluetooth_signal = self.bluetooth_state.get('target_rssi')
                detection_count = self.bluetooth_state.get('detection_count', 0)
                outliers_rejected = self.stats.get('rssi_outliers_rejected', 0)
            
            # Format display values
            runtime = int(current_time - (time.time() - 300 if hasattr(self, 'start_time') else current_time))  # Approximate
            
            if bluetooth_distance and bluetooth_signal:
                bt_status = f"{bluetooth_distance:.1f}m ({bluetooth_signal:.0f}dBm {self.determine_mode_text()})"
                if outliers_rejected > 0:
                    bt_status += f" | Outliers: {outliers_rejected}"
            elif bluetooth_signal:
                bt_status = f"Signal only ({bluetooth_signal:.0f}dBm)"
            else:
                bt_status = "No signal"
            
            # Create status line
            print(f"🧭 AUTONOMOUS | {self.nav_state} | US: {ultrasonic_distance:.1f}cm | BT: {bt_status} | Det: {detection_count} | Runtime: {runtime}s")
            
            # Show stuck counter if relevant
            if self.stuck_counter > 0:
                print(f"   Stuck counter: {self.stuck_counter}/{self.stuck_threshold}")
            
            self.last_status_time = current_time
        
        # Statistics display every 30 seconds
        if not hasattr(self, 'last_stats_time'):
            self.last_stats_time = 0
        
        if current_time - self.last_stats_time >= 30.0:
            self.display_statistics()
            self.last_stats_time = current_time
    
    def display_statistics(self):
        """Display comprehensive navigation statistics"""
        runtime = int(time.time() - self.nav_start_time) if hasattr(self, 'nav_start_time') else 0
        
        print(f"📊 === Optimized Navigator V6 Fixed Statistics ===")
        print(f"   Runtime: {runtime} seconds")
        print(f"   Distance measurements: {self.stats['distance_measurements']}")
        print(f"   Obstacle avoidances: {self.stats['obstacle_avoidances']}")
        print(f"   Emergency stops: {self.stats['emergency_stops']}")
        print(f"   Stuck recoveries: {self.stats['stuck_recoveries']}")
        
        print(f"🎯 === Optimized Navigation Statistics ===")
        print(f"   Smart turns: {self.stats['smart_turns']}")
        print(f"   Tracking activations: {self.stats['tracking_activations']}")
        print(f"   Approach activations: {self.stats['approach_activations']}")
        print(f"   Hold activations: {self.stats['hold_activations']}")
        print(f"   Target reaches: {self.stats['target_reaches']}")
        print(f"   Signal improvements: {self.stats['signal_improvements']}")
        print(f"   Course corrections: {self.stats['course_corrections']}")
        print(f"   Turn tests: {self.stats['turn_tests']}")
        print(f"   Long tracking phases: {self.stats['long_tracking_phases']}")
        
        print(f"📊 === Optimized RSSI Statistics ===")
        print(f"   Outliers rejected: {self.stats['rssi_outliers_rejected']}")
        
        with self.detection_lock:
            detection_rate = self.stats['bluetooth_detections'] / max(1, runtime)
            callback_rate = self.stats['bluetooth_callback_events'] / max(1, runtime)
            strongest_rssi = self.bluetooth_state.get('strongest_rssi', -200)
            timeout_count = self.bluetooth_state.get('detection_timeout_count', 0)
            
            avg_rssi = 0
            if self.bluetooth_state['rssi_history']:
                avg_rssi = sum(self.bluetooth_state['rssi_history']) / len(self.bluetooth_state['rssi_history'])
            
            avg_distance = 0
            if self.bluetooth_state['distance_history']:
                avg_distance = sum(self.bluetooth_state['distance_history']) / len(self.bluetooth_state['distance_history'])
            
            current_modes = []
            if self.bluetooth_state.get('tracking_mode'):
                current_modes.append('TRACKING')
            if self.bluetooth_state.get('approaching_mode'):
                current_modes.append('APPROACHING')
            if self.bluetooth_state.get('holding_mode'):
                current_modes.append('HOLDING')
        
        print(f"📡 === Bluetooth Ranging Statistics ===")
        print(f"   Total detections: {self.stats['bluetooth_detections']}")
        print(f"   Callback events: {self.stats['bluetooth_callback_events']}")
        print(f"   Detection rate: {detection_rate:.2f}/sec")
        print(f"   Callback rate: {callback_rate:.2f}/sec")
        if strongest_rssi > -200:
            print(f"   Strongest RSSI: {strongest_rssi} dBm")
        print(f"   Signal timeouts: {timeout_count}")
        if avg_rssi != 0:
            print(f"   Average RSSI: {avg_rssi:.1f} dBm")
        if avg_distance != 0:
            print(f"   Average smoothed distance: {avg_distance:.1f}m")
        if current_modes:
            print(f"   Current modes: {', '.join(current_modes)}")
        
        nav_rate = self.stats['distance_measurements'] / max(1, runtime)
        print(f"   Navigation rate: {nav_rate:.1f} measurements/sec")
    
    def stop_navigation(self):
        """Clean shutdown of navigation system"""
        print("🛑 Stopping Optimized Navigator V6 Fixed...")
        self.running = False
        
        # Send stop command to motors
        if self.arduino:
            self.send_motor_command(0, 0)
            time.sleep(0.1)
            self.arduino.close()
        
        print(f"📊 Final runtime: {time.time() - self.nav_start_time:.1f} seconds")
        self.display_statistics()
        print("👋 Optimized Navigator V6 Fixed shutdown complete")

def main():
    """Main function"""
    navigator = BluetoothOptimizedNavigator()
    
    print("================================================================================")
    print("🤖 BLUETOOTH OPTIMIZED NAVIGATOR V6 FIXED")
    print("================================================================================")
    print("")
    print("OPTIMIZED TRACKING FEATURES:")
    print("  ✅ Ultrasonic obstacle detection and avoidance")
    print("  ✅ Enhanced stuck recovery system")
    print("  ✅ Arduino gatekeeper integration")
    print("  ✅ Real-time Bluetooth beacon ranging (0.8m - 30m)")
    print("  ✅ Working RSSI values with balanced noise reduction")
    print("  ✅ Thread-safe callback-based BLE scanning")
    print("  🚶 PROVEN: Forward movement signal strength tracking")
    print("  🔄 PROVEN: Turn-and-test behavior for course corrections")
    print("  🧠 BALANCED: Smart obstacle avoidance without over-turning")
    print("  📊 OPTIMIZED: RSSI smoothing for stability without over-filtering")
    print("  ⏱️ ENHANCED: Longer movement phases for effective gradient detection")
    print("  🎯 PROVEN: Precise target hold at 0.5m ±0.15m")
    print("  ✅ Safety-first design with ultrasonic priority")
    print("  ✅ Clean shutdown with no task warnings")
    print("")
    print("🐛 HOLDING MODE BUG FIX:")
    print("  ✅ Detects when BlueCharm moves significantly (>10dBm RSSI change)")
    print("  ✅ Exits HOLDING mode and resets smoothing when BlueCharm moves")
    print("  ✅ No more 'stuck' distance readings at 0.5m")
    print("  ✅ Proper movement tracking and re-acquisition")
    print("")
    print("OPTIMIZED NAVIGATION BEHAVIOR:")
    print("  🔍 SEARCHING: Look for BlueCharm signal")
    print("  🚶 FORWARD_TRACKING: Extended movement (8s) while monitoring signal")
    print("  🔄 TURN_TESTING: Test left/right turns when signal degrades")
    print("  🧠 SMART_AVOIDING: Balanced turning until path is clear")
    print("  🎯 APPROACHING: Gentle approach to target")
    print("  🏁 HOLDING: Maintain precise 0.5m distance with movement detection")
    print("  🛡️ SAFETY: Ultrasonic overrides all Bluetooth behavior")
    print("")
    print("KEY OPTIMIZATIONS:")
    print("  📏 Tracking range: <15m (vs 3m) for real-world signal detection")
    print("  ⏱️ Movement phases: 8s tracking (vs 3s) for gradient analysis")
    print("  📊 RSSI smoothing: 50% new data (vs 30%) for responsiveness")
    print("  🎯 Signal thresholds: 1.5dBm improvement | 4dBm degradation")
    print("  🧠 Smart turning: 1-3.5s duration | Clear at >90cm")
    print("  ⏱️ Exploration: 10s intervals for fewer interruptions")
    print("  🐛 BlueCharm movement detection: >10dBm RSSI change exits HOLDING")
    print("")
    
    if not navigator.start_navigation():
        print("❌ Failed to start navigation system")
        return
    
    print("✅ Optimized Navigator V6 Fixed ready - switch to autonomous mode")
    print("📡 BlueCharm optimized tracking will activate automatically")
    print("🎯 Rover will use longer movement phases for effective signal tracking")
    print("📊 Balanced smoothing will provide stability without over-filtering")
    print("🐛 HOLDING mode will detect BlueCharm movement and reset tracking")
    print("Press Ctrl+C to stop...")
    
    try:
        while navigator.running:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested by user...")
    finally:
        navigator.stop_navigation()

if __name__ == "__main__":
    main()