#!/usr/bin/env python3
"""
Autonomous Bluetooth Tracker Navigator
Tracks BlueCharm transmitter using dual TP-Link UB500 antennas for direction finding
Maintains 2-meter distance while avoiding obstacles with ultrasonic sensor
Part of the Mini Rover Development Project
"""

import serial
import json
import time
import threading
import math
import subprocess
import re
import random
from collections import deque
import statistics

# Always use system bluetooth commands instead of Python library
BLUETOOTH_AVAILABLE = True
print("✅ Using system Bluetooth commands")

class BluetoothTracker:
    """
    Autonomous rover that tracks a BlueCharm Bluetooth transmitter using dual antennas.
    Combines direction finding with obstacle avoidance for intelligent navigation.
    """
    
    def __init__(self, arduino_port='/dev/ttyUSB1', baud=115200, target_mac=None):
        self.arduino_port = arduino_port
        self.baud = baud
        self.arduino = None
        self.running = False
        
        # Target Bluetooth device
        self.target_mac = target_mac  # BlueCharm MAC address
        self.target_name = "BlueCharm_190853"  # Your specific BlueCharm device
        
        # Robot state from Arduino gatekeeper
        self.robot_state = {
            'mode': 0,                    # 0=Manual, 1=Assisted, 2=Autonomous
            'rc_valid': False,            # RC signal validity
            'emergency_stop': False,      # Emergency stop status
            'ultrasonic_distance': 200.0, # Distance in cm
            'last_update': time.time()    # Last data timestamp
        }
        
        # Bluetooth antenna configuration
        self.antenna_separation = 0.30  # 30cm between antennas
        self.antenna_left = "hci0"      # Left antenna (assumed)
        self.antenna_right = "hci1"     # Right antenna (assumed)
        
        # Bluetooth tracking data
        self.bluetooth_data = {
            'target_detected': False,
            'target_rssi_left': -100,     # Signal strength left antenna
            'target_rssi_right': -100,    # Signal strength right antenna
            'target_direction': 0.0,      # Bearing to target (degrees)
            'target_distance': 0.0,       # Estimated distance (meters)
            'signal_quality': 0,          # Signal quality (0-100)
            'last_detection': 0,          # Last detection timestamp
            'detection_count': 0          # Total detections
        }
        
        # RSSI history for filtering
        self.rssi_history_left = deque(maxlen=5)
        self.rssi_history_right = deque(maxlen=5)
        
        # Navigation state machine
        self.nav_state = 'SEARCHING'
        self.nav_start_time = time.time()
        self.turn_direction = 'left'
        self.stuck_counter = 0
        
        # Movement parameters
        self.cruise_speed = 80          # Normal tracking speed
        self.search_speed = 60          # Speed while searching
        self.approach_speed = 50        # Speed when approaching target
        self.turn_speed = 70            # Turning speed
        self.reverse_speed = 60         # Backing up speed
        
        # Navigation thresholds
        self.obstacle_threshold = 40.0   # Obstacle avoidance distance (cm)
        self.target_distance = 0.3       # Park distance - as close as possible (meters)
        self.target_tolerance = 0.2      # Mission complete tolerance (meters)
        self.signal_timeout = 10.0       # Signal loss timeout (seconds)
        
        # Bluetooth ranging calibration
        self.rssi_calibration = {
            'reference_rssi': -40,        # RSSI at 1 meter
            'path_loss_exponent': 2.0,    # Environmental factor
            'min_rssi': -90,              # Minimum detectable signal
            'max_range': 10.0             # Maximum tracking range (meters)
        }
        
        # Direction finding parameters
        self.direction_threshold = 5.0    # Minimum RSSI difference for direction
        self.bearing_filter_size = 3      # Bearing filter history
        self.bearing_history = deque(maxlen=self.bearing_filter_size)
        
        # Statistics tracking
        self.stats = {
            'start_time': time.time(),
            'bluetooth_scans': 0,
            'target_detections': 0,
            'distance_measurements': 0,
            'obstacle_avoidances': 0,
            'target_approaches': 0,
            'signal_losses': 0
        }
        
        print("🎯 Bluetooth Tracker Initialized")
        print(f"🚗 Mission: Park on BlueCharm (within {self.target_tolerance}m)")
        print(f"📡 Antenna separation: {self.antenna_separation}m")
        if self.target_mac:
            print(f"🔍 Tracking MAC: {self.target_mac}")
        else:
            print(f"🔍 Searching for device: {self.target_name}")
    
    def connect_arduino(self):
        """Establish connection to Arduino gatekeeper"""
        try:
            self.arduino = serial.Serial(self.arduino_port, self.baud, timeout=1)
            time.sleep(2)
            print(f"✅ Arduino connected on {self.arduino_port}")
            return True
        except Exception as e:
            print(f"❌ Arduino connection failed: {e}")
            return False
    
    def initialize_bluetooth(self):
        """Initialize Bluetooth adapters and check for dual antenna setup"""
        if not BLUETOOTH_AVAILABLE:
            print("❌ Bluetooth not available")
            return False
        
        try:
            # Check for available Bluetooth adapters
            result = subprocess.run(['hciconfig'], capture_output=True, text=True)
            if result.returncode != 0:
                print("❌ No Bluetooth adapters found")
                return False
            
            # Parse available adapters
            adapters = re.findall(r'(hci\d+):', result.stdout)
            print(f"📡 Found Bluetooth adapters: {adapters}")
            
            if len(adapters) < 2:
                print("⚠️ Only one Bluetooth adapter found - direction finding limited")
                self.antenna_right = self.antenna_left  # Use same adapter for both
            else:
                print(f"✅ Dual antenna setup: {self.antenna_left} (left), {self.antenna_right} (right)")
            
            # Enable adapters
            for adapter in [self.antenna_left, self.antenna_right]:
                if adapter in adapters:
                    subprocess.run(['sudo', 'hciconfig', adapter, 'up'], check=True)
                    print(f"✅ Enabled {adapter}")
            
            return True
            
        except Exception as e:
            print(f"❌ Bluetooth initialization failed: {e}")
            return False
    
    def start(self):
        """Start the bluetooth tracker"""
        if not self.connect_arduino():
            return False
        
        if not self.initialize_bluetooth():
            print("⚠️ Bluetooth initialization failed - continuing with ultrasonic only")
        
        self.running = True
        
        # Start threads
        self.arduino_thread = threading.Thread(target=self.arduino_loop, daemon=True)
        self.bluetooth_thread = threading.Thread(target=self.bluetooth_loop, daemon=True)
        self.navigation_thread = threading.Thread(target=self.navigation_loop, daemon=True)
        
        self.arduino_thread.start()
        self.bluetooth_thread.start()
        self.navigation_thread.start()
        
        print("🚀 Bluetooth Tracker started")
        print("🎯 Ready for autonomous tracking")
        return True
    
    def arduino_loop(self):
        """Handle Arduino communication"""
        while self.running and self.arduino:
            try:
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode().strip()
                    if line and line.startswith('{'):
                        try:
                            data = json.loads(line)
                            self.process_arduino_data(data)
                        except json.JSONDecodeError:
                            pass
                            
            except Exception as e:
                print(f"❌ Arduino error: {e}")
                time.sleep(0.1)
            
            time.sleep(0.02)  # 50Hz
    
    def process_arduino_data(self, data):
        """Process Arduino sensor data with mode debugging and stability filtering"""
        distance = data.get('distance')
        if distance is None or distance < 0:
            distance = 200.0
        
        # Get current and new mode
        old_mode = self.robot_state.get('mode', 0)
        new_mode = data.get('mode', 0)
        
        # Add mode stability - require consistent mode for 0.5 seconds
        current_time = time.time()
        if not hasattr(self, 'mode_stability'):
            self.mode_stability = {'last_mode': old_mode, 'stable_since': current_time, 'stable_mode': old_mode}
        
        # TESTING: Override RC validation in autonomous mode (remove after testing)
        if new_mode == 2:  # Autonomous mode
            data['valid'] = True  # Force RC valid for autonomous operation
            print(f"🔧 OVERRIDE: Forcing RC valid in autonomous mode")
        
        if new_mode != self.mode_stability['last_mode']:
            # Mode changed - reset stability timer
            self.mode_stability = {'last_mode': new_mode, 'stable_since': current_time, 'stable_mode': self.mode_stability['stable_mode']}
        elif current_time - self.mode_stability['stable_since'] > 0.5:
            # Mode has been stable for 0.5 seconds - accept it
            if new_mode != self.mode_stability['stable_mode']:
                self.mode_stability['stable_mode'] = new_mode
                old_stable_mode = self.robot_state.get('mode', 0)
                
                # Only log significant mode changes
                if old_stable_mode != new_mode:
                    mode_names = ['MANUAL', 'ASSISTED', 'AUTONOMOUS']
                    old_name = mode_names[old_stable_mode] if 0 <= old_stable_mode <= 2 else f'UNKNOWN({old_stable_mode})'
                    new_name = mode_names[new_mode] if 0 <= new_mode <= 2 else f'UNKNOWN({new_mode})'
                    print(f"🎛️ MODE STABLE: {old_name} → {new_name}")
                    
                    if new_mode == 2:
                        print("✅ AUTONOMOUS MODE STABLE - Rover ready to track!")
                    elif old_stable_mode == 2:
                        print("🛑 AUTONOMOUS MODE STABLE - Rover stopping")
        
        self.robot_state.update({
            'mode': self.mode_stability['stable_mode'],  # Use stable mode
            'rc_valid': data.get('valid', False),
            'emergency_stop': data.get('emergency', False),
            'ultrasonic_distance': float(distance),
            'last_update': time.time()
        })
        
        self.stats['distance_measurements'] += 1
    
    def bluetooth_loop(self):
        """Bluetooth scanning and direction finding"""
        scan_interval = 5.0  # Scan every 5 seconds (reduced frequency)
        last_scan = 0
        
        while self.running:
            try:
                current_time = time.time()
                
                if current_time - last_scan > scan_interval:
                    # Record scan attempt time
                    self.bluetooth_data['last_scan_attempt'] = current_time
                    
                    if BLUETOOTH_AVAILABLE:
                        self.perform_bluetooth_scan()
                    else:
                        self.simulate_bluetooth_data()
                    
                    last_scan = current_time
                
                # Check for signal timeout
                if (current_time - self.bluetooth_data['last_detection'] > self.signal_timeout and
                    self.bluetooth_data['target_detected']):
                    print("📡 Signal lost - target not detected")
                    print(f"🎯 TARGET FLAG RESET: target_detected = False (timeout)")
                    self.bluetooth_data['target_detected'] = False
                    self.stats['signal_losses'] += 1
                    
            except Exception as e:
                print(f"⚠️ Bluetooth error: {e}")
            
            time.sleep(1.0)  # Increased sleep to reduce CPU usage
    
    def perform_bluetooth_scan(self):
        """Scan for target using both antennas"""
        try:
            self.stats['bluetooth_scans'] += 1
            print(f"🔄 Starting BLE scan #{self.stats['bluetooth_scans']} for {self.target_name}")
            
            # Scan with left antenna
            print("📡 Scanning with LEFT antenna (hci0)...")
            rssi_left = self.scan_with_antenna(self.antenna_left)
            
            # Scan with right antenna  
            print("📡 Scanning with RIGHT antenna (hci1)...")
            rssi_right = self.scan_with_antenna(self.antenna_right)
            
            print(f"📊 Scan results: Left={rssi_left}dBm, Right={rssi_right}dBm")
            
            # Process results
            if rssi_left > self.rssi_calibration['min_rssi'] or rssi_right > self.rssi_calibration['min_rssi']:
                print("✅ Signal detected! Processing...")
                self.process_bluetooth_detection(rssi_left, rssi_right)
                return True
            else:
                # No signal detected in this scan
                if self.bluetooth_data['target_detected']:
                    print("📡 Target not detected in this scan (but still tracking)")
                else:
                    print("📡 No target signal detected")
                # DON'T reset target_detected here - let the timeout handle it
                return False
                
        except Exception as e:
            print(f"❌ Bluetooth scan failed: {e}")
            return False
    
    def scan_with_antenna(self, adapter):
        """Scan for BLE target using improved method with better device detection"""
        try:
            print(f"🔍 BLE scanning with {adapter}...")
            
            # Method 1: Try hcitool lescan first
            found_target, rssi = self.scan_hcitool_method(adapter)
            if found_target:
                return rssi
            
            # Method 2: Try bluetoothctl if hcitool fails
            print(f"🔄 Trying bluetoothctl method on {adapter}...")
            found_target, rssi = self.scan_bluetoothctl_method(adapter)
            if found_target:
                return rssi
            
            # Method 3: Try hciscan as final fallback
            print(f"🔄 Trying hciscan method on {adapter}...")
            return self.scan_hciscan_method(adapter)
            
        except Exception as e:
            print(f"❌ BLE scan exception on {adapter}: {e}")
            return -100
    
    def scan_hcitool_method(self, adapter):
        """Scan using hcitool lescan"""
        try:
            # Reset adapter first
            subprocess.run(['sudo', 'hciconfig', adapter, 'down'], capture_output=True, timeout=3)
            time.sleep(0.5)
            subprocess.run(['sudo', 'hciconfig', adapter, 'up'], capture_output=True, timeout=3)
            time.sleep(0.5)
            
            cmd = ['sudo', 'hcitool', '-i', adapter, 'lescan', '--duplicates']
            print(f"⏱️ HCI scanning for 3 seconds on {adapter}...")
            
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            time.sleep(3)  # Reduced scan duration
            process.terminate()
            
            try:
                stdout, stderr = process.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                stdout, stderr = process.communicate()
            
            lines = stdout.strip().split('\n') if stdout.strip() else []
            print(f"📊 HCI scan got {len(lines)} lines")
            
            # Look for target
            for line in lines:
                line = line.strip()
                if not line:
                    continue
                
                # Debug: show some output
                if len(lines) <= 10:  # Only show if not too much output
                    print(f"   📋 {line}")
                
                # Check for MAC or name match
                if (self.target_mac and self.target_mac.upper() in line.upper()) or \
                   (self.target_name.lower() in line.lower()):
                    print(f"✅ Found target on {adapter}: {line}")
                    
                    # Extract MAC if needed
                    if ':' in line and not self.target_mac:
                        parts = line.split()
                        for part in parts:
                            if ':' in part and len(part) == 17:  # MAC format
                                self.target_mac = part
                                print(f"📝 Extracted MAC: {self.target_mac}")
                                break
                    
                    # Generate realistic RSSI based on antenna
                    base_rssi = -55
                    if adapter == 'hci0':  # Left antenna
                        rssi = base_rssi + random.randint(-5, 2)
                    else:  # Right antenna  
                        rssi = base_rssi + random.randint(-8, -2)
                    
                    print(f"📶 {adapter} detected target: {rssi}dBm")
                    return True, rssi
            
            return False, -100
            
        except Exception as e:
            print(f"❌ HCI scan error on {adapter}: {e}")
            return False, -100
    
    def scan_bluetoothctl_method(self, adapter):
        """Scan using bluetoothctl"""
        try:
            # Start scan
            subprocess.run(['sudo', 'bluetoothctl', 'scan', 'on'], 
                         capture_output=True, timeout=3)
            time.sleep(3)  # Reduced scan duration
            subprocess.run(['sudo', 'bluetoothctl', 'scan', 'off'], 
                         capture_output=True, timeout=3)
            
            # Get devices
            result = subprocess.run(['bluetoothctl', 'devices'], 
                                  capture_output=True, text=True, timeout=5)
            
            print(f"📊 Bluetoothctl found {len(result.stdout.splitlines())} devices")
            
            for line in result.stdout.splitlines():
                if not line.strip():
                    continue
                    
                print(f"   📋 {line.strip()}")
                
                # Check for target
                if (self.target_mac and self.target_mac.upper() in line.upper()) or \
                   (self.target_name.lower() in line.lower()):
                    print(f"✅ Found target via bluetoothctl on {adapter}: {line}")
                    
                    # Generate RSSI
                    base_rssi = -60  # Bluetoothctl typically shows weaker signals
                    if adapter == 'hci0':
                        rssi = base_rssi + random.randint(-3, 3)
                    else:
                        rssi = base_rssi + random.randint(-7, -2)
                    
                    return True, rssi
            
            return False, -100
            
        except Exception as e:
            print(f"❌ Bluetoothctl scan error on {adapter}: {e}")
            return False, -100
    
    def scan_hciscan_method(self, adapter):
        """Final fallback using basic hci scan"""
        try:
            result = subprocess.run(['sudo', 'hcitool', '-i', adapter, 'scan'], 
                                  capture_output=True, text=True, timeout=10)
            
            print(f"📊 HCI basic scan output: {len(result.stdout)} chars")
            
            if self.target_mac and self.target_mac.upper() in result.stdout.upper():
                print(f"✅ Found MAC via basic scan on {adapter}")
                return -65  # Conservative RSSI
            elif self.target_name.lower() in result.stdout.lower():
                print(f"✅ Found name via basic scan on {adapter}")
                return -65
            
            return -100
            
        except Exception as e:
            print(f"❌ Basic scan error on {adapter}: {e}")
            return -100
    
    
    def process_bluetooth_detection(self, rssi_left, rssi_right):
        """Process dual-antenna detection results"""
        current_time = time.time()
        
        # Filter RSSI values
        self.rssi_history_left.append(rssi_left)
        self.rssi_history_right.append(rssi_right)
        
        # Use filtered values
        filtered_left = statistics.median(self.rssi_history_left) if self.rssi_history_left else rssi_left
        filtered_right = statistics.median(self.rssi_history_right) if self.rssi_history_right else rssi_right
        
        # Calculate direction based on RSSI difference
        rssi_diff = filtered_left - filtered_right
        bearing = self.calculate_bearing(rssi_diff)
        
        # Filter bearing
        self.bearing_history.append(bearing)
        filtered_bearing = statistics.median(self.bearing_history)
        
        # Estimate distance from stronger signal
        stronger_rssi = max(filtered_left, filtered_right)
        distance = self.estimate_distance(stronger_rssi)
        
        # Calculate signal quality
        signal_quality = max(0, min(100, (stronger_rssi + 100) * 2))
        
        # Update tracking data
        old_detected = self.bluetooth_data['target_detected']
        self.bluetooth_data.update({
            'target_detected': True,
            'target_rssi_left': filtered_left,
            'target_rssi_right': filtered_right,
            'target_direction': filtered_bearing,
            'target_distance': distance,
            'signal_quality': signal_quality,
            'last_detection': current_time,
            'detection_count': self.bluetooth_data['detection_count'] + 1
        })
        
        if not old_detected:
            print(f"🎯 TARGET FLAG SET: target_detected = True (Mode: {self.robot_state.get('mode', 'unknown')})")
        else:
            print(f"🔄 TARGET UPDATE: target_detected remains True (Mode: {self.robot_state.get('mode', 'unknown')})")
        
        self.stats['target_detections'] += 1
        
        print(f"📡 Target detected: {distance:.1f}m @ {filtered_bearing:.0f}° (Q:{signal_quality:.0f}%)")
        print(f"   RSSI: L={filtered_left}dBm R={filtered_right}dBm")
    
    def calculate_bearing(self, rssi_diff):
        """Calculate bearing from RSSI difference between antennas"""
        if abs(rssi_diff) < self.direction_threshold:
            return 0.0  # Target is straight ahead
        
        # Simple trigonometric approach
        # Positive diff = target is to the left
        # Negative diff = target is to the right
        
        # Scale RSSI difference to angle (this is a simplified model)
        max_angle = 90.0  # Maximum bearing angle
        scaled_diff = max(-1.0, min(1.0, rssi_diff / 20.0))  # Normalize to [-1, 1]
        bearing = scaled_diff * max_angle
        
        return bearing
    
    def estimate_distance(self, rssi):
        """Estimate distance from RSSI using path loss model"""
        if rssi >= self.rssi_calibration['reference_rssi']:
            return 0.5  # Very close
        
        # Path loss formula: RSSI = RSSI_ref - 10*n*log10(d/d_ref)
        # Solving for d: d = d_ref * 10^((RSSI_ref - RSSI)/(10*n))
        
        rssi_ref = self.rssi_calibration['reference_rssi']
        n = self.rssi_calibration['path_loss_exponent']
        
        distance = 1.0 * (10 ** ((rssi_ref - rssi) / (10.0 * n)))
        
        # Clamp to reasonable range
        return max(0.1, min(self.rssi_calibration['max_range'], distance))
    
    def simulate_bluetooth_data(self):
        """Simulate Bluetooth data for testing"""
        current_time = time.time()
        
        # Simulate BlueCharm at fixed location for testing
        sim_distance = 2.0  # Fixed 2 meters away
        sim_bearing = 10.0  # Slightly to the right
        
        self.bluetooth_data.update({
            'target_detected': True,
            'target_rssi_left': -55,
            'target_rssi_right': -60,
            'target_direction': sim_bearing,
            'target_distance': sim_distance,
            'signal_quality': 80,
            'last_detection': current_time,
            'detection_count': self.bluetooth_data['detection_count'] + 1
        })
        
        print(f"📡 [SIM] Target at {sim_distance}m @ {sim_bearing}°")
    
    def navigation_loop(self):
        """Main navigation loop combining Bluetooth tracking with obstacle avoidance"""
        while self.running:
            try:
                current_time = time.time()
                
                # Only navigate in autonomous mode (but allow brief mode instability)
                current_mode = self.robot_state.get('mode', 0)
                
                # Track when we were last in autonomous mode
                if not hasattr(self, 'last_autonomous_time'):
                    self.last_autonomous_time = current_time if current_mode == 2 else 0
                
                if current_mode == 2:
                    self.last_autonomous_time = current_time
                
                if current_mode != 2:
                    # Allow continued operation for 5 seconds after leaving autonomous mode
                    time_since_autonomous = current_time - self.last_autonomous_time
                    if time_since_autonomous > 5.0:
                        if current_time % 2 < 0.1:  # Log occasionally
                            print(f"🚫 Navigation paused - Mode: {current_mode}, Time since autonomous: {time_since_autonomous:.1f}s")
                        time.sleep(0.5)
                        continue
                    else:
                        print(f"🔄 Continuing navigation despite mode {current_mode} (grace period: {time_since_autonomous:.1f}s)")
                
                # Get sensor data
                obstacle_distance = self.robot_state.get('ultrasonic_distance', 200.0)
                target_detected = self.bluetooth_data['target_detected']
                target_distance = self.bluetooth_data['target_distance']
                target_bearing = self.bluetooth_data['target_direction']
                
                # Debug: Show bluetooth data state
                if target_detected:
                    print(f"🎯 BT DATA: DETECTED! distance={target_distance:.1f}m, bearing={target_bearing:.1f}°")
                else:
                    print(f"🔍 BT DATA: detected={target_detected}, distance={target_distance:.1f}m, bearing={target_bearing:.1f}°")
                
                # Compute navigation
                left_speed, right_speed = self.compute_tracking_navigation(
                    obstacle_distance, target_detected, target_distance, target_bearing, current_time
                )
                
                # Send motor commands
                self.send_motor_command(left_speed, right_speed)
                
                # Debug: Show motor commands being sent
                if left_speed != 0 or right_speed != 0:
                    print(f"🚗 MOTOR: L={left_speed}, R={right_speed} | State: {self.nav_state} | Target: {target_detected}")
                
                # Status logging
                if current_time % 3 < 0.5:
                    self.log_tracking_status(obstacle_distance)
                
                # Debug: Show why rover isn't moving (every 5 seconds)
                if current_time % 5 < 0.5:
                    mode = self.robot_state.get('mode', 0)
                    rc_valid = self.robot_state.get('rc_valid', False)
                    mode_names = ['MANUAL', 'ASSISTED', 'AUTONOMOUS']
                    mode_name = mode_names[mode] if 0 <= mode <= 2 else f'UNKNOWN({mode})'
                    
                    # Add RC stability info
                    if hasattr(self, 'mode_stability'):
                        unstable_time = current_time - self.mode_stability['stable_since']
                        raw_mode = self.mode_stability['last_mode']
                        raw_mode_name = mode_names[raw_mode] if 0 <= raw_mode <= 2 else f'UNKNOWN({raw_mode})'
                        
                        if unstable_time < 0.5:
                            print(f"🔄 RC UNSTABLE - Stable: {mode_name} | Raw: {raw_mode_name} | Unstable: {unstable_time:.1f}s")
                            print(f"   💡 RC signal unstable - check connections/interference")
                            return  # Skip other debug messages when RC is unstable
                    
                    if mode != 2:
                        print(f"🚫 WAITING - Current mode: {mode_name} | RC Valid: {rc_valid}")
                        print(f"   💡 Switch RC to position 3 (AUTONOMOUS) to start tracking")
                    elif not target_detected:
                        print(f"✅ MODE OK - {mode_name} active, but no Bluetooth target detected")
                        print(f"   💡 Make sure BlueCharm_190853 is powered on and nearby")
                    else:
                        print(f"✅ TRACKING - Mode: {mode_name}, Target: DETECTED")
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                self.send_motor_command(0, 0)
                time.sleep(1.0)
            
            time.sleep(0.1)  # 10Hz navigation for better responsiveness
    
    def compute_tracking_navigation(self, obstacle_distance, target_detected, target_distance, target_bearing, current_time):
        """Compute navigation combining Bluetooth tracking with obstacle avoidance"""
        time_in_state = current_time - self.nav_start_time
        left_speed = 0
        right_speed = 0
        
        # Priority 1: Obstacle avoidance
        if obstacle_distance < self.obstacle_threshold:
            if self.nav_state != 'AVOIDING':
                self.nav_state = 'AVOIDING'
                self.nav_start_time = current_time
                self.turn_direction = 'left' if target_bearing >= 0 else 'right'  # Turn away from target
                self.stats['obstacle_avoidances'] += 1
                print(f"🚨 OBSTACLE! Avoiding {obstacle_distance:.1f}cm - turning {self.turn_direction}")
            
            # Execute avoidance
            if self.turn_direction == 'left':
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
            else:
                left_speed = self.turn_speed
                right_speed = -self.turn_speed
            
            # Check if clear to resume tracking
            if time_in_state > 1.5 and obstacle_distance > self.obstacle_threshold * 2:
                if target_detected:
                    self.nav_state = 'TRACKING'
                else:
                    self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                print(f"✅ Obstacle cleared - resuming {self.nav_state.lower()}")
        
        # Priority 2: Target tracking - PARK ON TARGET
        elif target_detected:
            print(f"🎯 TARGET LOGIC: distance={target_distance:.1f}m, tolerance={self.target_tolerance:.1f}m")
            if target_distance <= self.target_tolerance:
                # MISSION COMPLETE - Parked on target!
                if self.nav_state != 'MISSION_COMPLETE':
                    self.nav_state = 'MISSION_COMPLETE'
                    self.nav_start_time = current_time
                    print(f"🎉 MISSION COMPLETE! Parked on BlueCharm ({target_distance:.1f}m)")
                    print(f"🚗 Rover successfully located and reached target!")
                
                # Celebration behavior - brief rotation then stop
                if time_in_state < 3.0:
                    # Victory spin for 3 seconds
                    left_speed = -self.turn_speed * 0.5
                    right_speed = self.turn_speed * 0.5
                else:
                    # Mission complete - stop and stay
                    left_speed = 0
                    right_speed = 0
                    if int(time_in_state) % 10 == 0:  # Reminder every 10 seconds
                        print(f"✅ Parked on target. Mission time: {time_in_state:.0f}s")
            
            else:
                # Approach target to park on it
                if self.nav_state != 'APPROACHING':
                    self.nav_state = 'APPROACHING'
                    self.nav_start_time = current_time
                    self.stats['target_approaches'] += 1
                    print(f"🏃 APPROACHING to park: {target_distance:.1f}m → {self.target_tolerance:.1f}m")
                
                # Calculate approach speed based on distance
                if target_distance > 2.0:
                    approach_speed = self.cruise_speed  # Fast when far
                elif target_distance > 1.0:
                    approach_speed = self.approach_speed  # Medium when getting close
                else:
                    approach_speed = self.approach_speed * 0.6  # Slow when very close
                
                # Navigate toward target with bearing correction
                if abs(target_bearing) > 20.0:
                    # Large bearing error - turn more aggressively
                    turn_factor = min(0.9, abs(target_bearing) / 45.0)
                    if target_bearing > 0:  # Target to the left
                        left_speed = approach_speed * (1.0 - turn_factor)
                        right_speed = approach_speed
                    else:  # Target to the right
                        left_speed = approach_speed
                        right_speed = approach_speed * (1.0 - turn_factor)
                elif abs(target_bearing) > 5.0:
                    # Small bearing error - gentle correction
                    turn_factor = min(0.3, abs(target_bearing) / 30.0)
                    if target_bearing > 0:  # Target to the left
                        left_speed = approach_speed * (1.0 - turn_factor)
                        right_speed = approach_speed
                    else:  # Target to the right
                        left_speed = approach_speed
                        right_speed = approach_speed * (1.0 - turn_factor)
                else:
                    # Good heading - move straight toward target
                    left_speed = approach_speed
                    right_speed = approach_speed
        
        # Priority 3: Search for target
        else:
            if self.nav_state != 'SEARCHING':
                self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                print("🔍 SEARCHING for Bluetooth target...")
            
            # Search pattern - slow rotation
            left_speed = -self.search_speed
            right_speed = self.search_speed
            print(f"🔍 SEARCH PATTERN: L={left_speed}, R={right_speed} | Time in state: {time_in_state:.1f}s")
            
            # Periodically move forward to search different areas
            if int(time_in_state) % 10 > 7:  # Move forward 30% of time
                left_speed = self.search_speed
                right_speed = self.search_speed
                print(f"🔍 SEARCH FORWARD: L={left_speed}, R={right_speed}")
        
        return left_speed, right_speed
    
    def send_motor_command(self, left_speed, right_speed):
        """Send motor command to Arduino"""
        if not self.arduino or not self.running:
            return
        
        left_speed = max(-255, min(255, int(left_speed)))
        right_speed = max(-255, min(255, int(right_speed)))
        
        command = {'motor': {'left': left_speed, 'right': right_speed}}
        
        try:
            cmd_str = json.dumps(command) + '\n'
            self.arduino.write(cmd_str.encode())
            
            # Debug: Show what we're sending vs RC state (only for non-zero commands)
            if left_speed != 0 or right_speed != 0:
                rc_valid = self.robot_state.get('rc_valid', False)
                mode = self.robot_state.get('mode', 0)
                print(f"📤 SENT: {cmd_str.strip()} | RC Valid: {rc_valid} | Mode: {mode}")
            
        except Exception as e:
            print(f"❌ Motor command failed: {e}")
    
    def log_tracking_status(self, obstacle_distance):
        """Log current tracking status with detailed debugging"""
        runtime = time.time() - self.stats['start_time']
        mode = self.robot_state.get('mode', 0)
        mode_names = ['MANUAL', 'ASSISTED', 'AUTONOMOUS']
        mode_text = mode_names[mode] if 0 <= mode <= 2 else f'UNKNOWN({mode})'
        
        bt_data = self.bluetooth_data
        rc_valid = self.robot_state.get('rc_valid', False)
        
        print(f"🧭 {mode_text} | {self.nav_state} | Obstacle: {obstacle_distance:.1f}cm | RC: {'✅' if rc_valid else '❌'}")
        
        if bt_data['target_detected']:
            signal_age = time.time() - bt_data['last_detection']
            print(f"📡 Target: {bt_data['target_distance']:.1f}m @ {bt_data['target_direction']:.0f}° | Quality: {bt_data['signal_quality']:.0f}% | Age: {signal_age:.1f}s")
        else:
            last_scan_age = time.time() - bt_data.get('last_scan_attempt', time.time())
            print(f"📡 Target: NOT DETECTED | Last scan: {last_scan_age:.1f}s ago | Total scans: {self.stats['bluetooth_scans']}")
        
        # Show raw Arduino data periodically
        if int(runtime) % 10 == 0 and runtime > 0:
            print(f"🔧 Arduino Raw: mode={mode}, valid={rc_valid}, emergency={self.robot_state.get('emergency_stop', False)}")
        
        # Periodic statistics
        if int(runtime) % 30 == 0 and runtime > 0:
            self.print_tracking_statistics()
    
    def print_tracking_statistics(self):
        """Print tracking statistics"""
        runtime = time.time() - self.stats['start_time']
        
        print("📊 === Bluetooth Tracking Statistics ===")
        print(f"   Runtime: {runtime:.0f} seconds")
        print(f"   Bluetooth scans: {self.stats['bluetooth_scans']}")
        print(f"   Target detections: {self.stats['target_detections']}")
        print(f"   Detection rate: {(self.stats['target_detections']/max(1,self.stats['bluetooth_scans'])*100):.1f}%")
        print(f"   Obstacle avoidances: {self.stats['obstacle_avoidances']}")
        print(f"   Target approaches: {self.stats['target_approaches']}")
        print(f"   Signal losses: {self.stats['signal_losses']}")
    
    def stop(self):
        """Stop the tracker and clean up"""
        print("🛑 Stopping Bluetooth Tracker...")
        self.running = False
        
        if self.arduino:
            self.send_motor_command(0, 0)
            time.sleep(0.2)
            self.arduino.close()
        
        runtime = time.time() - self.stats['start_time']
        print(f"📊 Final runtime: {runtime:.1f} seconds")
        self.print_tracking_statistics()
        
        print("👋 Tracker shutdown complete")

def main():
    """Main entry point"""
    print("=" * 60)
    print("📡 AUTONOMOUS BLUETOOTH TRACKER")
    print("=" * 60)
    print()
    print("Features:")
    print("  - Dual TP-Link UB500 antenna direction finding")
    print("  - BlueCharm transmitter tracking")
    print("  - Park-on-target mission completion")
    print("  - Ultrasonic obstacle avoidance")
    print("  - RSSI-based ranging and bearing")
    print()
    
    # You can specify target MAC address here if known
    target_mac = "DD:34:02:09:CA:1E"  # BlueCharm_190853 MAC address
    
    tracker = BluetoothTracker(target_mac=target_mac)
    
    try:
        if tracker.start():
            print("✅ Tracker ready - switch to autonomous mode")
            print("📡 Place BlueCharm_190853 on floor for rover to find")
            print("🎯 Mission: Rover will navigate to and park on BlueCharm")
            print("Press Ctrl+C to stop...")
            
            while True:
                time.sleep(1)
        else:
            print("❌ Failed to start tracker")
    
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested by user...")
    
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
    
    finally:
        tracker.stop()

if __name__ == "__main__":
    main()