#!/usr/bin/env python3
"""
Bluetooth LiDAR Navigator V8-LITE - CORE FUNCTIONALITY ONLY
================================================================================
Streamlined version focused on reliable navigation without visualization overhead

KEY FEATURES PRESERVED:
✅ Fixed LiDAR left/right orientation (CRITICAL navigation fix)
✅ All proven V6 Bluetooth navigation capabilities  
✅ Integrated RPLidar A1 for 360° obstacle detection
✅ Multi-sensor fusion (ultrasonic + LiDAR)
✅ Enhanced obstacle avoidance with sector analysis
✅ Automatic USB device detection and assignment
✅ Robot body-aware navigation with physical dimensions
✅ Optimized navigation speeds and thresholds

REMOVED FOR PERFORMANCE:
❌ LiDAR visualization (caused Pi overload)
❌ Matplotlib dependencies and threading
❌ Animation and GUI components
❌ Excessive debug output

HARDWARE:
- Raspberry Pi 4B with Bluetooth 5.0
- Arduino gatekeeper with ultrasonic sensor
- BlueCharm BLE beacon (MAC: DD:34:02:09:CA:1E)
- RPLidar A1 on auto-detected USB port
- Cytron MDDS30 motor driver

Part of the Mini Rover Development Project
Author: Developed incrementally with Jay Fielding
Version: 8-LITE - Core Navigation Only
Date: 2025-01-18
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
import numpy as np
import queue

# Additional imports for device detection
import serial.tools.list_ports
import glob
import os

# Bluetooth imports with graceful fallback
try:
    from bleak import BleakScanner
    BLUETOOTH_AVAILABLE = True
    print("✅ Bluetooth (bleak) library available")
except ImportError:
    print("⚠️ Bluetooth library not available - install with: pip install bleak")
    BLUETOOTH_AVAILABLE = False

# LiDAR imports with graceful fallback
try:
    from rplidar import RPLidar
    LIDAR_AVAILABLE = True
    print("✅ LiDAR (rplidar) library available")
except ImportError:
    print("⚠️ LiDAR library not available - install with: pip install rplidar")
    LIDAR_AVAILABLE = False

class RobotPhysicalProfile:
    """
    Physical dimensions and clearance calculations for the rover
    All measurements from LiDAR center point for accurate navigation
    """
    
    def __init__(self):
        # Robot overall dimensions
        self.total_length = 35.0      # cm (front to back)
        self.total_width = 28.0       # cm (left to right)
        
        # LiDAR position relative to robot edges
        self.lidar_to_front = 23.0    # cm from LiDAR center to front edge
        self.lidar_to_rear = 35.0 - 23.0  # 12.0 cm to rear edge
        self.lidar_to_left = 13.0     # cm from LiDAR center to left edge  
        self.lidar_to_right = 28.0 - 13.0  # 15.0 cm to right edge
        
        # Safety margins for navigation
        self.safety_margin = 5.0      # cm extra buffer for safe navigation
        self.tight_space_margin = 3.0 # cm reduced margin for confined spaces
        
        # Calculated minimum safe distances (LiDAR reading to obstacle)
        self.min_front_distance = self.lidar_to_front + self.safety_margin   # 28.0 cm
        self.min_rear_distance = self.lidar_to_rear + self.safety_margin     # 17.0 cm
        self.min_left_distance = self.lidar_to_left + self.safety_margin     # 18.0 cm
        self.min_right_distance = self.lidar_to_right + self.safety_margin   # 20.0 cm
        
        print(f"🤖 Robot Physical Profile Loaded:")
        print(f"   Dimensions: {self.total_length}cm × {self.total_width}cm")
        print(f"   LiDAR Position: {self.lidar_to_front}cm from front, {self.lidar_to_left}cm from left")
        print(f"   Safe Distances: F={self.min_front_distance}cm R={self.min_rear_distance}cm L={self.min_left_distance}cm R={self.min_right_distance}cm")
        
        # Corridor width requirements
        self.min_corridor_width = self.total_width + (self.safety_margin * 2)      # 38.0 cm
        self.tight_corridor_width = self.total_width + (self.tight_space_margin * 2) # 34.0 cm
        
        print(f"   Corridor Requirements: {self.min_corridor_width}cm normal, {self.tight_corridor_width}cm tight")
    
    def get_safe_distance(self, lidar_reading_mm, direction):
        """
        Convert LiDAR reading to safe navigation distance accounting for robot body
        """
        reading_cm = lidar_reading_mm / 10.0
        
        if direction == 'front':
            return max(0, reading_cm - self.lidar_to_front)
        elif direction == 'rear':
            return max(0, reading_cm - self.lidar_to_rear)
        elif direction == 'left':
            return max(0, reading_cm - self.lidar_to_left)
        elif direction == 'right':
            return max(0, reading_cm - self.lidar_to_right)
        else:
            return reading_cm  # Default for unknown directions

class USBDeviceDetector:
    """
    Automatic USB device detection and assignment
    Identifies Arduino and LiDAR devices on available ports
    """
    
    def __init__(self):
        self.arduino_port = None
        self.arduino_baud = None
        self.lidar_port = None
        print("🔍 USB Device Detector initialized")
    
    def detect_devices(self):
        """Automatically detect and assign Arduino and LiDAR ports"""
        print("🔍 === USB Device Detection Started ===")
        print("🔍 Scanning for USB devices...")
        
        # Get list of available serial ports
        ports = serial.tools.list_ports.comports()
        usb_ports = [port for port in ports if 'USB' in port.device or 'ttyUSB' in port.device or 'ttyACM' in port.device]
        
        if not usb_ports:
            print("❌ No USB serial devices found")
            return False
        
        print(f"📋 Found {len(usb_ports)} USB serial devices:")
        for i, port in enumerate(usb_ports, 1):
            print(f"   {i}. {port.device} - {port.description}")
            if hasattr(port, 'vid') and hasattr(port, 'pid'):
                print(f"      VID:PID = {port.vid:04X}:{port.pid:04X}")
            if hasattr(port, 'manufacturer') and port.manufacturer:
                print(f"      Manufacturer: {port.manufacturer}")
        
        print("\\n🧪 Testing ports for device identification...")
        
        arduino_candidates = []
        lidar_candidates = []
        
        for port in usb_ports:
            print(f"\\n📍 Testing {port.device}...")
            
            # Test for Arduino
            arduino_result = self._test_arduino(port.device)
            if arduino_result:
                if isinstance(arduino_result, dict):
                    arduino_candidates.append(arduino_result)
                else:
                    arduino_candidates.append({'port': port.device, 'baud': 115200})
            
            # Test for LiDAR
            if self._test_lidar(port.device):
                lidar_candidates.append({'port': port.device})
        
        # Assign detected devices
        print(f"\\n📊 Detection Results:")
        print(f"   Arduino candidates: {len(arduino_candidates)}")
        print(f"   LiDAR candidates: {len(lidar_candidates)}")
        
        # Assign Arduino
        if arduino_candidates:
            self.arduino_port = arduino_candidates[0]['port']
            self.arduino_baud = arduino_candidates[0]['baud']
            print(f"✅ Arduino assigned to: {self.arduino_port} @ {self.arduino_baud} baud")
        else:
            print(f"❌ No Arduino detected on any port")
        
        # Assign LiDAR
        if lidar_candidates:
            self.lidar_port = lidar_candidates[0]['port']
            print(f"✅ LiDAR assigned to: {self.lidar_port}")
        else:
            print(f"❌ No LiDAR detected on any port")
        
        # Final summary
        print(f"\\n📋 Final Device Assignment:")
        print(f"   Arduino: {self.arduino_port or 'Not found'}")
        print(f"   LiDAR: {self.lidar_port or 'Not found'}")
        
        return self.arduino_port is not None or self.lidar_port is not None
    
    def _test_arduino(self, port):
        """Enhanced Arduino detection with multiple baud rates and diagnostics"""
        print(f"🤖 Testing for Arduino on {port}...")
        
        # Try common baud rates
        baud_rates = [115200, 9600, 57600]
        
        for baud in baud_rates:
            try:
                print(f"   Trying {baud} baud...")
                ser = serial.Serial(port, baud, timeout=3)
                time.sleep(1.5)  # Longer wait for Arduino to initialize
                
                # Check what data is available
                data_received = []
                
                # Try to read some data - wait longer for JSON after startup
                for attempt in range(15):  # More attempts for Arduino startup
                    if ser.in_waiting > 0:
                        try:
                            line = ser.readline().decode('utf-8', errors='ignore').strip()
                            if line:
                                data_received.append(line)
                                
                                # Show all data for first baud rate, minimal for others
                                if baud == 115200:
                                    print(f"   Raw data: {line[:100]}...")  # Show first 100 chars
                                
                                # Special detection for Arduino Gatekeeper startup
                                if "Arduino Gatekeeper" in line and "Initialized" in line:
                                    print(f"   🎯 Arduino Gatekeeper detected! Waiting for JSON data...")
                                    # Continue waiting for JSON, don't exit yet
                                
                                if line.startswith('{'):
                                    try:
                                        data = json.loads(line)
                                        if 'distance' in data or 'emergency' in data or 'mode' in data:
                                            ser.close()
                                            print(f"✅ Arduino detected on {port} at {baud} baud")
                                            print(f"   Valid JSON data: {data}")
                                            return {'port': port, 'baud': baud}
                                    except json.JSONDecodeError as e:
                                        if baud == 115200:
                                            print(f"   JSON decode error: {e}")
                        except UnicodeDecodeError:
                            if baud == 115200:
                                print(f"   Unicode decode error at {baud} baud")
                    time.sleep(0.3)  # Longer wait for JSON data
                
                ser.close()
                
                if data_received:
                    # Check if we detected Arduino Gatekeeper startup messages
                    for line in data_received:
                        if "Arduino Gatekeeper" in line and "Initialized" in line:
                            print(f"   🎯 Arduino Gatekeeper confirmed at {baud} baud (startup mode)")
                            return {'port': port, 'baud': baud}
                    
                    print(f"   Data received but not valid Arduino JSON")
                else:
                    print(f"   No data received at {baud} baud")
                    
            except Exception as e:
                print(f"   Serial error at {baud} baud: {e}")
        
        print(f"❌ No Arduino detected on {port} (tried {baud_rates})")
        return False
    
    def _test_lidar(self, port):
        """Test if port has LiDAR responding"""
        print(f"📡 Testing for LiDAR on {port}...")
        try:
            lidar = RPLidar(port, baudrate=115200, timeout=2)
            info = lidar.get_info()
            
            if info and 'model' in info:
                print(f"✅ LiDAR detected on {port}")
                print(f"   Model: {info['model']}")
                print(f"   Firmware: {info['firmware']}")
                if 'hardware' in info:
                    print(f"   Hardware: {info['hardware']}")
                lidar.disconnect()
                return True
                
        except Exception as e:
            print(f"❌ LiDAR test failed on {port}: {e}")
            
        return False
    
    def get_arduino_config(self):
        """Get Arduino connection configuration"""
        if self.arduino_port:
            return self.arduino_port, getattr(self, 'arduino_baud', 115200)
        return None, None
    
    def get_lidar_config(self):
        """Get LiDAR connection configuration"""
        if self.lidar_port:
            return self.lidar_port
        return None

class LidarProcessor:
    """
    LiDAR processor focused purely on navigation data - NO VISUALIZATION
    """
    
    def __init__(self, port=None):
        self.lidar = None
        self.port = port
        self.running = False
        self.data_queue = queue.Queue(maxsize=3)
        self.scan_thread = None
        
        # LiDAR navigation data
        self.obstacle_sectors = {}
        self.clear_directions = []
        self.closest_obstacle = {'distance': 8000, 'angle': 0}
        self.last_scan_time = 0
        self.scan_count = 0
        
        # Optimized navigation sectors (8 directions) - 0° = forward
        self.sectors = {
            'front': (337.5, 22.5),      # Forward
            'front_right': (22.5, 67.5), # Front-right
            'right': (67.5, 112.5),      # Right
            'back_right': (112.5, 157.5), # Back-right  
            'back': (157.5, 202.5),      # Backward
            'back_left': (202.5, 247.5), # Back-left
            'left': (247.5, 292.5),      # Left
            'front_left': (292.5, 337.5) # Front-left
        }
        
        # Optimized LiDAR settings - less restrictive for better navigation
        self.min_distance = 100      # Filter close noise (mm)
        self.max_distance = 8000     # Maximum useful range (mm)
        self.sector_obstacle_threshold = 600  # mm - consider sector blocked
        self.clear_path_threshold = 900      # mm - consider sector clear for navigation
        
        print("🔄 LiDAR Processor initialized (LITE - NO VISUALIZATION)")
        print(f"📏 Range: {self.min_distance}-{self.max_distance}mm")
        print(f"🚧 Obstacle threshold: {self.sector_obstacle_threshold}mm")
        print(f"✅ Clear path threshold: {self.clear_path_threshold}mm")
    
    def connect_lidar(self, port):
        """Connect to LiDAR device"""
        if not LIDAR_AVAILABLE:
            print("❌ LiDAR library not available")
            return False
        
        try:
            print(f"🔗 Connecting to RPLidar A1 on {port}")
            self.lidar = RPLidar(port, baudrate=115200, timeout=3)
            
            # Get device info
            info = self.lidar.get_info()
            print(f"📋 LiDAR Device info: {info}")
            
            # Get health status
            health = self.lidar.get_health()
            print(f"💚 LiDAR Health: {health}")
            
            # Start motor
            print("🔄 Starting LiDAR motor...")
            self.lidar.start_motor()
            time.sleep(3)
            
            print("✅ LiDAR connected and ready")
            return True
            
        except Exception as e:
            print(f"❌ LiDAR connection error: {e}")
            return False
    
    def start_scanning(self):
        """Start LiDAR scanning for navigation only"""
        if not self.lidar:
            print("❌ LiDAR not connected")
            return False
        
        self.running = True
        self.scan_thread = threading.Thread(target=self._scan_worker, daemon=True)
        self.scan_thread.start()
        
        print("📡 LiDAR scanning started - NAVIGATION ONLY")
        return True
    
    def _scan_worker(self):
        """Background thread for continuous LiDAR scanning - NAVIGATION ONLY"""
        try:
            scan_count = 0
            for scan in self.lidar.iter_scans():
                if not self.running:
                    break
                
                scan_count += 1
                
                # Process for navigation only
                processed_data = self._process_scan(scan)
                
                # Update navigation queue
                if processed_data:
                    if self.data_queue.full():
                        try:
                            self.data_queue.get_nowait()
                        except queue.Empty:
                            pass
                    
                    try:
                        self.data_queue.put(processed_data, block=False)
                        self.last_scan_time = time.time()
                        self.scan_count += 1
                        
                        # Reduced logging frequency for performance
                        if scan_count % 100 == 0:  # Every 100 scans instead of 20
                            print(f"📡 Navigation scan #{scan_count}: {len(processed_data['valid_points'])} points, {len(processed_data['clear_directions'])}/8 clear")
                            
                            # Debug sector mappings every 500 scans to verify orientation fix
                            if scan_count % 500 == 0:
                                sectors_debug = {k: f"{v:.0f}mm" for k, v in processed_data['obstacle_sectors'].items()}
                                print(f"🧭 SECTORS: {sectors_debug}")
                            
                    except queue.Full:
                        pass
                        
        except Exception as e:
            print(f"❌ LiDAR scan error: {e}")
    
    def _process_scan(self, scan):
        """Process raw LiDAR scan into navigation data - FIXED LEFT/RIGHT ORIENTATION"""
        valid_points = []
        sector_distances = {sector: [] for sector in self.sectors}
        closest_dist = self.max_distance
        closest_angle = 0
        
        for measurement in scan:
            quality, angle, distance = measurement
            
            # CRITICAL FIX: Correct left/right mirroring for navigation
            corrected_angle = -angle if angle != 0 else 0  # Flip left/right to match rover orientation
            # Normalize to 0-360 range
            if corrected_angle < 0:
                corrected_angle += 360
            
            # Filter valid measurements
            if self.min_distance <= distance <= self.max_distance:
                valid_points.append((corrected_angle, distance))
                
                # Track closest obstacle
                if distance < closest_dist:
                    closest_dist = distance
                    closest_angle = corrected_angle
                
                # Categorize by sector using CORRECTED angle
                for sector_name, (start_angle, end_angle) in self.sectors.items():
                    if start_angle > end_angle:  # Handle wrap-around (e.g., front sector)
                        if corrected_angle >= start_angle or corrected_angle <= end_angle:
                            sector_distances[sector_name].append(distance)
                    else:
                        if start_angle <= corrected_angle <= end_angle:
                            sector_distances[sector_name].append(distance)
        
        # Calculate sector minimums (closest obstacle in each direction)
        obstacle_sectors = {}
        clear_directions = []
        
        for sector, distances in sector_distances.items():
            if distances:
                min_dist = min(distances)
                obstacle_sectors[sector] = min_dist
                
                # Determine if direction is clear for navigation
                if min_dist > self.clear_path_threshold:
                    clear_directions.append(sector)
            else:
                # No obstacles detected in this sector
                obstacle_sectors[sector] = self.max_distance
                clear_directions.append(sector)
        
        return {
            'valid_points': valid_points,
            'obstacle_sectors': obstacle_sectors,
            'clear_directions': clear_directions,
            'closest_obstacle': {'distance': closest_dist, 'angle': closest_angle},
            'timestamp': time.time(),
            'point_count': len(valid_points)
        }
    
    def get_navigation_data(self):
        """Get latest LiDAR navigation data"""
        try:
            data = self.data_queue.get_nowait()
            
            # Update internal navigation state
            self.obstacle_sectors = data['obstacle_sectors']
            self.clear_directions = data['clear_directions']
            self.closest_obstacle = data['closest_obstacle']
            
            return data
            
        except queue.Empty:
            # Return last known data if available
            if hasattr(self, 'obstacle_sectors') and self.obstacle_sectors:
                return {
                    'obstacle_sectors': self.obstacle_sectors,
                    'clear_directions': self.clear_directions,
                    'closest_obstacle': self.closest_obstacle,
                    'timestamp': self.last_scan_time,
                    'stale': True
                }
            else:
                return None
    
    def get_front_distance(self):
        """Get distance to closest obstacle in front sector"""
        if 'front' in self.obstacle_sectors:
            return self.obstacle_sectors['front'] / 10.0  # Convert mm to cm
        return 200.0  # Safe default
    
    def is_direction_clear(self, direction, min_distance_cm=60):
        """Check if a specific direction is clear for navigation"""
        if direction in self.obstacle_sectors:
            distance_cm = self.obstacle_sectors[direction] / 10.0
            return distance_cm > min_distance_cm
        return False
    
    def get_best_turn_direction(self):
        """Get best turn direction based on LiDAR data"""
        if not self.obstacle_sectors:
            return 'left'  # Default
        
        # Check left and right sectors
        left_distance = self.obstacle_sectors.get('left', 0)
        right_distance = self.obstacle_sectors.get('right', 0)
        front_left_distance = self.obstacle_sectors.get('front_left', 0)
        front_right_distance = self.obstacle_sectors.get('front_right', 0)
        
        # Calculate composite clearance scores
        left_score = left_distance + (front_left_distance * 0.5)
        right_score = right_distance + (front_right_distance * 0.5)
        
        return 'left' if left_score > right_score else 'right'
    
    def stop_scanning(self):
        """Stop LiDAR scanning"""
        self.running = False
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
                print("✅ LiDAR disconnected")
            except:
                pass

class BluetoothLidarNavigator:
    """
    Core navigation system with LiDAR and Bluetooth integration
    V8-LITE: Focused on performance and reliability
    """
    
    def __init__(self, arduino_port=None, baud=115200, target_mac="DD:34:02:09:CA:1E"):
        self.arduino = None
        self.arduino_port = arduino_port
        self.baud = baud
        self.running = False
        
        # Device detection
        self.device_detector = USBDeviceDetector()
        
        # Robot physical profile for body-aware navigation
        self.robot_profile = RobotPhysicalProfile()
        
        # LiDAR processor - NO VISUALIZATION
        self.lidar_processor = LidarProcessor() if LIDAR_AVAILABLE else None
        self.lidar_enabled = False
        
        # Core navigation parameters - optimized for performance
        self.danger_threshold = 12.0    # cm - immediate danger
        self.caution_threshold = 20.0   # cm - slow down
        self.safe_distance = 35.0       # cm - normal operation
        
        # LiDAR-specific thresholds
        self.lidar_danger_threshold = 12.0   # cm
        self.lidar_caution_threshold = 20.0  # cm
        
        # Bluetooth configuration
        self.bluetooth_state = {
            'enabled': BLUETOOTH_AVAILABLE,
            'target_mac': target_mac.upper(),
            'target_rssi': -100,
            'target_distance': None,
            'last_detection': 0,
            'detection_count': 0,
            'strongest_rssi': -100,
            'rssi_history': deque(maxlen=10),
            'distance_history': deque(maxlen=10),
            'smoothed_distance': None,
            'tracking_mode': False,
            'approaching_mode': False,
            'holding_mode': False,
            'signal_improving': False,
            'signal_degrading': False,
            'course_correction_count': 0,
            'last_course_correction': 0
        }
        
        # Navigation state
        self.nav_state = 'SEARCHING'
        self.nav_start_time = time.time()
        self.turn_direction = 'left'
        self.stuck_counter = 0
        
        # Optimized speeds and timings
        self.cruise_speed = 120        # Normal forward speed
        self.tracking_speed = 100      # Speed when tracking target
        self.turn_speed = 100          # Turn speed
        self.slow_speed = 60           # Slow speed near obstacles
        self.hold_speed = 40           # Fine positioning speed
        self.reverse_speed = 80        # Backup speed
        
        self.turn_duration = 1.5       # Turn duration
        self.backup_duration = 1.0     # Backup duration
        self.exploration_interval = 8.0 # Search turn interval
        self.stuck_reset_time = 30.0   # Reset stuck counter
        
        # Bluetooth navigation thresholds - optimized for closer approach
        self.target_bluetooth_distance = 0.3    # Target distance (was 0.5m)
        self.target_tolerance = 0.2              # Distance tolerance
        self.tracking_threshold = 15.0           # Start tracking at 15m
        self.approach_threshold = 1.0            # Switch to approach mode
        
        # Signal analysis thresholds
        self.signal_improvement_threshold = 1.0  # dBm improvement to detect
        self.signal_degradation_threshold = 2.0  # dBm degradation to detect
        self.course_correction_threshold = 1.5   # Distance increase for correction
        self.max_course_corrections_per_minute = 3  # Limit corrections
        
        # Robot state tracking
        self.robot_state = {
            'mode': 0,
            'rc_valid': False,
            'emergency_stop': False,
            'ultrasonic_distance': 200.0,
            'last_update': time.time()
        }
        
        # Threading
        self.detection_lock = threading.Lock()
        
        # Statistics
        self.stats = {
            'start_time': time.time(),
            'distance_measurements': 0,
            'obstacle_avoidances': 0,
            'exploration_turns': 0,
            'stuck_recoveries': 0,
            'bluetooth_detections': 0,
            'bluetooth_navigation_events': 0,
            'course_corrections': 0,
            'tracking_activations': 0,
            'approach_activations': 0,
            'hold_activations': 0,
            'target_reaches': 0,
            'lidar_scans_processed': 0,
            'lidar_obstacle_detections': 0,
            'lidar_enhanced_turns': 0
        }
        
        print("🚀 Bluetooth LiDAR Navigator V8-LITE")
        print(f"📊 Navigation: Danger={self.danger_threshold}cm | Caution={self.caution_threshold}cm | Safe={self.safe_distance}cm")
        print(f"🔄 LiDAR: Danger={self.lidar_danger_threshold}cm | Caution={self.lidar_caution_threshold}cm")
        print(f"📡 Bluetooth: Target {self.target_bluetooth_distance}m ±{self.target_tolerance}m")
        print(f"🎯 LITE: No visualization - core navigation only")
    
    def smooth_rssi(self, new_rssi):
        """Apply optimized smoothing to RSSI readings"""
        if not self.bluetooth_state['rssi_history']:
            return new_rssi
        
        # Simpler smoothing for faster response
        weight = 0.6  # Higher weight on new reading
        recent_avg = sum(list(self.bluetooth_state['rssi_history'])[-2:]) / min(2, len(self.bluetooth_state['rssi_history']))
        
        smoothed_rssi = (weight * new_rssi) + ((1.0 - weight) * recent_avg)
        return smoothed_rssi
    
    def smooth_distance(self, new_distance):
        """Apply optimized smoothing to distance readings"""
        if not self.bluetooth_state['distance_history']:
            return new_distance
        
        # Simpler outlier rejection
        recent_avg = sum(list(self.bluetooth_state['distance_history'])[-2:]) / min(2, len(self.bluetooth_state['distance_history']))
        distance_ratio = new_distance / recent_avg if recent_avg > 0 else 1.0
        
        # More lenient outlier rejection
        if distance_ratio > 2.5 or distance_ratio < 0.4:
            return self.bluetooth_state['smoothed_distance'] if self.bluetooth_state['smoothed_distance'] else new_distance
        
        # Simple weighted average
        weight = 0.7  # Higher weight on new reading
        smoothed_distance = (weight * new_distance) + ((1.0 - weight) * recent_avg)
        
        return smoothed_distance
    
    def bluetooth_detection_callback(self, device, advertisement_data):
        """Optimized callback for BlueCharm detection"""
        if device.address.upper() == self.bluetooth_state['target_mac'].upper():
            with self.detection_lock:
                raw_rssi = advertisement_data.rssi
                current_time = time.time()
                
                smoothed_rssi = self.smooth_rssi(raw_rssi)
                
                previous_rssi = self.bluetooth_state['target_rssi']
                self.bluetooth_state['target_rssi'] = smoothed_rssi
                self.bluetooth_state['last_detection'] = current_time
                self.bluetooth_state['detection_count'] += 1
                self.stats['bluetooth_detections'] += 1
                
                self.bluetooth_state['rssi_history'].append(smoothed_rssi)
                
                if smoothed_rssi > self.bluetooth_state['strongest_rssi']:
                    self.bluetooth_state['strongest_rssi'] = smoothed_rssi
                
                raw_distance = self.rssi_to_distance(smoothed_rssi)
                smoothed_distance = self.smooth_distance(raw_distance)
                
                self.bluetooth_state['target_distance'] = raw_distance
                self.bluetooth_state['smoothed_distance'] = smoothed_distance
                self.bluetooth_state['distance_history'].append(smoothed_distance)
                
                # Optimized signal analysis - only when rover is moving
                if previous_rssi and len(self.bluetooth_state['rssi_history']) >= 3:
                    recent_avg = sum(list(self.bluetooth_state['rssi_history'])[-2:]) / 2
                    older_avg = sum(list(self.bluetooth_state['rssi_history'])[-4:-2]) / 2 if len(self.bluetooth_state['rssi_history']) >= 4 else previous_rssi
                    
                    rssi_change = recent_avg - older_avg
                    
                    # Only consider signal changes if rover has moved recently
                    rover_moving = hasattr(self, 'last_command_time') and (current_time - getattr(self, 'last_command_time', 0)) < 2.0
                    rover_moving = rover_moving and hasattr(self, 'last_speeds') and (abs(getattr(self, 'last_speeds', [0,0])[0]) > 10 or abs(getattr(self, 'last_speeds', [0,0])[1]) > 10)
                    
                    if rover_moving:
                        if rssi_change >= self.signal_improvement_threshold:
                            if not self.bluetooth_state['signal_improving']:
                                self.bluetooth_state['signal_improving'] = True
                                self.bluetooth_state['signal_degrading'] = False
                        elif rssi_change <= -self.signal_degradation_threshold:
                            if not self.bluetooth_state['signal_degrading']:
                                self.bluetooth_state['signal_degrading'] = True
                                self.bluetooth_state['signal_improving'] = False
                    else:
                        # Reset signal change flags when stationary
                        self.bluetooth_state['signal_improving'] = False
                        self.bluetooth_state['signal_degrading'] = False
                
                # Optimized mode activation
                distance_for_decisions = smoothed_distance
                
                if distance_for_decisions <= self.target_bluetooth_distance + self.target_tolerance:
                    if not self.bluetooth_state['holding_mode']:
                        self.bluetooth_state['holding_mode'] = True
                        self.stats['hold_activations'] += 1
                        self.stats['target_reaches'] += 1
                        print(f"🏁 TARGET HOLD: BlueCharm at {distance_for_decisions:.1f}m - holding position")
                elif distance_for_decisions <= self.approach_threshold:
                    if not self.bluetooth_state['approaching_mode']:
                        self.bluetooth_state['approaching_mode'] = True
                        self.stats['approach_activations'] += 1
                        print(f"🎯 APPROACH MODE: BlueCharm at {distance_for_decisions:.1f}m")
                elif distance_for_decisions <= self.tracking_threshold:
                    if not self.bluetooth_state['tracking_mode']:
                        self.bluetooth_state['tracking_mode'] = True
                        self.stats['tracking_activations'] += 1
                        print(f"🚶 TRACKING MODE: BlueCharm at {distance_for_decisions:.1f}m")
                
                # Reduced logging frequency for performance
                if self.bluetooth_state['detection_count'] % 20 == 0:  # Was 10, now 20
                    avg_rssi = sum(self.bluetooth_state['rssi_history']) / len(self.bluetooth_state['rssi_history'])
                    mode = "HOLD" if self.bluetooth_state['holding_mode'] else "APPR" if self.bluetooth_state['approaching_mode'] else "TRACK" if self.bluetooth_state['tracking_mode'] else "SCAN"
                    print(f"📡 BlueCharm {mode} #{self.bluetooth_state['detection_count']}: {smoothed_rssi:.0f}dBm | ~{distance_for_decisions:.1f}m")
    
    def rssi_to_distance(self, rssi):
        """Convert RSSI to approximate distance for BLE beacon"""
        if rssi == 0 or rssi < -100:
            return 30.0
        
        tx_power = -59
        ratio = (tx_power - rssi) / 20.0
        distance = math.pow(10, ratio)
        
        return max(0.2, min(distance, 30.0))  # Closer minimum distance
    
    def get_enhanced_obstacle_distance(self):
        """Get robot-aware obstacle distance with smart sensor fusion"""
        ultrasonic_distance = self.robot_state.get('ultrasonic_distance', 200.0)
        
        # Get LiDAR front distance if available
        if self.lidar_enabled and self.lidar_processor:
            lidar_raw_distance = self.lidar_processor.get_front_distance()
            
            if lidar_raw_distance < 200.0:  # LiDAR has valid reading
                # Convert LiDAR reading to actual robot clearance
                lidar_robot_clearance = self.robot_profile.get_safe_distance(
                    lidar_raw_distance * 10, 'front'
                ) / 10.0
                
                # Smart fusion: detect likely ultrasonic false readings
                difference = abs(lidar_robot_clearance - ultrasonic_distance)
                
                # If ultrasonic is much smaller AND both are in close range, likely false reading
                if (ultrasonic_distance < 15.0 and 
                    lidar_robot_clearance > 50.0 and 
                    difference > 40.0):
                    print(f"⚠️ Ultrasonic false reading detected: US={ultrasonic_distance:.1f}cm, LiDAR={lidar_robot_clearance:.1f}cm - using LiDAR")
                    return lidar_robot_clearance, 'lidar_corrected'
                
                # Use the more conservative reading for normal operation
                enhanced_distance = min(ultrasonic_distance, lidar_robot_clearance)
                return enhanced_distance, 'smart_fusion'
        
        return ultrasonic_distance, 'ultrasonic'
    
    def get_enhanced_turn_direction(self):
        """Enhanced turn direction selection using LiDAR data"""
        if self.lidar_enabled and self.lidar_processor:
            lidar_direction = self.lidar_processor.get_best_turn_direction()
            self.stats['lidar_enhanced_turns'] += 1
            return lidar_direction
        
        # Fallback to alternating logic
        if hasattr(self, 'last_turn_direction'):
            self.last_turn_direction = 'right' if self.last_turn_direction == 'left' else 'left'
            return self.last_turn_direction
        else:
            self.last_turn_direction = 'left'
            return 'left'
    
    def connect_arduino(self):
        """Establish connection to Arduino gatekeeper with auto-detection"""
        if not self.arduino_port or not self.baud:
            detected_port, detected_baud = self.device_detector.get_arduino_config()
            if detected_port:
                self.arduino_port = detected_port
                self.baud = detected_baud
                print(f"📍 Using detected Arduino: {self.arduino_port} @ {self.baud} baud")
            else:
                print("❌ No Arduino detected and no manual configuration provided")
                return False
        
        try:
            self.arduino = serial.Serial(self.arduino_port, self.baud, timeout=1)
            time.sleep(2)
            print(f"✅ Arduino connected on {self.arduino_port}")
            return True
        except Exception as e:
            print(f"❌ Arduino connection failed: {e}")
            return False
    
    def start(self):
        """Start the navigator with automatic device detection"""
        print("🔍 Starting device detection...")
        
        # Run device detection first
        detection_success = self.device_detector.detect_devices()
        
        if not detection_success:
            print("⚠️ No devices detected, attempting manual fallback...")
            fallback_configs = [
                ('/dev/ttyUSB0', 115200),
                ('/dev/ttyUSB1', 115200),
                ('/dev/ttyACM0', 115200)
            ]
            
            for port, baud in fallback_configs:
                try:
                    if os.path.exists(port):
                        self.arduino_port = port
                        self.baud = baud
                        print(f"🔄 Trying fallback: {port} @ {baud}")
                        break
                except:
                    continue
        
        # Connect to Arduino
        if not self.connect_arduino():
            print("❌ Arduino connection failed - continuing without Arduino")
        
        # Connect to LiDAR if available
        if LIDAR_AVAILABLE and self.lidar_processor:
            lidar_port = self.device_detector.get_lidar_config()
            if lidar_port:
                print(f"📍 Using detected LiDAR: {lidar_port}")
                if self.lidar_processor.connect_lidar(lidar_port):
                    if self.lidar_processor.start_scanning():
                        self.lidar_enabled = True
                        print("✅ LiDAR navigation enhancement enabled")
        
        if not self.lidar_enabled:
            print("⚠️ LiDAR not available - using ultrasonic only")
        
        # Start background threads
        if self.arduino:
            arduino_thread = threading.Thread(target=self.arduino_loop, daemon=True)
            arduino_thread.start()
        
        if BLUETOOTH_AVAILABLE:
            print("📡 Starting BlueCharm tracking thread...")
            bluetooth_thread = threading.Thread(target=self.bluetooth_loop, daemon=True)
            bluetooth_thread.start()
            print("📡 BlueCharm tracking scanner ready")
        
        # Start navigation thread
        navigation_thread = threading.Thread(target=self.navigation_loop, daemon=True)
        navigation_thread.start()
        
        print("🚀 Navigator V8-LITE started with LiDAR + Ultrasonic + Bluetooth (NO VISUALIZATION)")
        print("🎯 Ready for core navigation functionality")
        print("✅ Navigator V8-LITE ready with LiDAR + Ultrasonic + Bluetooth")
        print("📡 Switch to autonomous mode for navigation")
        print("🚀 Core navigation optimized for performance")
        print("Press Ctrl+C to stop...")
    
    def arduino_loop(self):
        """Handle continuous Arduino communication - very lenient timeout handling"""
        consecutive_failures = 0
        max_failures = 100  # Much more tolerance during active navigation
        last_data_time = time.time()
        
        while self.running and self.arduino:
            try:
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode().strip()
                    if line and line.startswith('{'):
                        try:
                            data = json.loads(line)
                            self.process_arduino_data(data)
                            consecutive_failures = 0
                            last_data_time = time.time()
                        except json.JSONDecodeError as e:
                            consecutive_failures += 1
                
                # Very lenient timeout check - 20 seconds for busy navigation
                if time.time() - last_data_time > 20.0:
                    consecutive_failures += 1
                    if consecutive_failures % 30 == 0:  # Log every 30 failures
                        print(f"⚠️ Arduino communication timeout (failures: {consecutive_failures}) - navigation may be busy")
                
                if consecutive_failures >= max_failures:
                    print("❌ Too many Arduino communication failures - emergency stop")
                    self.emergency_stop()
                    break
                    
            except Exception as e:
                print(f"❌ Arduino communication error: {e}")
                consecutive_failures += 1
                time.sleep(0.1)
            
            time.sleep(0.05)  # Slightly slower to reduce CPU load
    
    def process_arduino_data(self, data):
        """Process incoming data from Arduino"""
        distance = data.get('distance')
        if distance is None or distance < 0:
            distance = 200.0
        
        self.robot_state.update({
            'mode': data.get('mode', 0),
            'rc_valid': data.get('valid', False),
            'emergency_stop': data.get('emergency', False),
            'ultrasonic_distance': float(distance),
            'last_update': time.time()
        })
        
        self.stats['distance_measurements'] += 1
        
        if self.robot_state['emergency_stop']:
            self.stats['emergency_stops'] += 1
    
    def navigation_loop(self):
        """Optimized autonomous navigation logic"""
        last_stuck_check = time.time()
        
        while self.running:
            try:
                if self.robot_state.get('mode', 0) != 2:
                    time.sleep(0.5)
                    continue
                
                # Get enhanced sensor readings
                obstacle_distance, sensor_source = self.get_enhanced_obstacle_distance()
                current_time = time.time()
                
                # Process LiDAR data if available
                lidar_data = None
                if self.lidar_enabled and self.lidar_processor:
                    lidar_data = self.lidar_processor.get_navigation_data()
                    if lidar_data and not lidar_data.get('stale', False):
                        self.stats['lidar_scans_processed'] += 1
                        # Mark that we have LiDAR data for display
                        lidar_data['has_data'] = True
                
                # Check for stuck condition
                if current_time - last_stuck_check > self.stuck_reset_time:
                    if self.stuck_counter > 0:
                        self.stuck_counter = max(0, self.stuck_counter - 1)
                    last_stuck_check = current_time
                
                # Optimized navigation computation
                left_speed, right_speed = self.compute_optimized_navigation(
                    obstacle_distance, current_time, lidar_data, sensor_source
                )
                
                self.send_motor_command(left_speed, right_speed)
                
                # Reduced status logging frequency to lower CPU load
                if current_time % 10 < 0.5:  # Was every 8s, now every 10s
                    self.log_optimized_status(obstacle_distance, sensor_source, lidar_data)
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                self.send_motor_command(0, 0)
                time.sleep(1.0)
            
            time.sleep(0.05)  # Fast response for autonomous mode
    
    def compute_optimized_navigation(self, obstacle_distance, current_time, lidar_data, sensor_source):
        """
        Optimized navigation with reduced processing overhead
        """
        time_in_state = current_time - self.nav_start_time
        base_left_speed = 0
        base_right_speed = 0
        
        # Check if emergency stop is preventing movement
        if hasattr(self, 'emergency_stop_active') and self.emergency_stop_active:
            return 0, 0
        
        # Less sensitive danger detection
        if obstacle_distance < self.danger_threshold:
            self.nav_state = 'AVOIDING'
            self.nav_start_time = current_time
            self.turn_direction = self.get_enhanced_turn_direction()
            self.stats['obstacle_avoidances'] += 1
            print(f"🛑 AVOIDING! {sensor_source} obstacle at {obstacle_distance:.1f}cm - turning {self.turn_direction}")
        
        # Get Bluetooth state
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
        
        display_distance = smoothed_distance if smoothed_distance else bluetooth_distance
        
        # Optimized state machine
        if self.nav_state == 'AVOIDING':
            if time_in_state < self.turn_duration:
                # Turn away from obstacle
                if self.turn_direction == 'left':
                    base_left_speed = -self.turn_speed
                    base_right_speed = self.turn_speed
                else:
                    base_left_speed = self.turn_speed
                    base_right_speed = -self.turn_speed
            else:
                # Check if clear
                if obstacle_distance > self.caution_threshold:
                    if holding_mode:
                        self.nav_state = 'HOLDING'
                    elif approaching_mode:
                        self.nav_state = 'APPROACHING'
                    elif tracking_mode:
                        self.nav_state = 'TRACKING'
                    else:
                        self.nav_state = 'SEARCHING'
                    
                    self.nav_start_time = current_time
                    print(f"✅ CLEAR: Path clear at {obstacle_distance:.1f}cm - resuming {self.nav_state.lower()}")
                else:
                    # Continue turning or backup
                    if time_in_state > self.turn_duration * 2:
                        self.nav_state = 'BACKING'
                        self.nav_start_time = current_time
                        self.stuck_counter += 1
                        print(f"🔄 BACKING: Still blocked - backing up (count: {self.stuck_counter})")
                    else:
                        # Continue turning
                        if self.turn_direction == 'left':
                            base_left_speed = -self.turn_speed
                            base_right_speed = self.turn_speed
                        else:
                            base_left_speed = self.turn_speed
                            base_right_speed = -self.turn_speed
        
        elif self.nav_state == 'BACKING':
            base_left_speed = -self.reverse_speed
            base_right_speed = -self.reverse_speed
            
            if time_in_state > self.backup_duration:
                self.nav_state = 'AVOIDING'
                self.nav_start_time = current_time
                self.turn_direction = self.get_enhanced_turn_direction()
                print(f"🧠 RETRY: Switching to {self.turn_direction} turn after backup")
        
        elif self.nav_state == 'HOLDING' and holding_mode and display_distance:
            # Optimized holding with closer target
            distance_error = display_distance - self.target_bluetooth_distance
            
            if abs(distance_error) <= self.target_tolerance:
                base_left_speed = 0
                base_right_speed = 0
                if time_in_state % 15 < 1:  # Reduced logging
                    print(f"🏁 TARGET PERFECT: Holding {display_distance:.1f}m distance")
            elif distance_error > 0:
                # Move closer
                if obstacle_distance < self.caution_threshold:
                    adjustment_speed = min(self.hold_speed * 0.7, abs(distance_error) * 30)
                else:
                    adjustment_speed = min(self.hold_speed, abs(distance_error) * 50)
                base_left_speed = adjustment_speed
                base_right_speed = adjustment_speed
                if time_in_state % 8 < 1:
                    print(f"📏 CLOSER: {display_distance:.1f}m → {self.target_bluetooth_distance}m")
            else:
                # Move back
                adjustment_speed = min(self.hold_speed, abs(distance_error) * 50)
                base_left_speed = -adjustment_speed
                base_right_speed = -adjustment_speed
                if time_in_state % 8 < 1:
                    print(f"📏 FARTHER: {display_distance:.1f}m → {self.target_bluetooth_distance}m")
            
            if not holding_mode or (display_distance and display_distance > self.approach_threshold):
                self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                print("📡 HOLD LOST: Signal lost - returning to search")
        
        elif self.nav_state == 'APPROACHING' and approaching_mode and display_distance:
            # Optimized approaching
            if display_distance <= self.target_bluetooth_distance + self.target_tolerance:
                self.nav_state = 'HOLDING'
                self.nav_start_time = current_time
                print(f"🎯 SWITCHING TO HOLD: {display_distance:.1f}m")
            else:
                # Approach with obstacle awareness
                if obstacle_distance < self.caution_threshold:
                    speed_factor = (obstacle_distance - self.danger_threshold) / (self.caution_threshold - self.danger_threshold)
                    speed = self.slow_speed * max(0.5, speed_factor)
                else:
                    speed = self.hold_speed
                
                base_left_speed = speed
                base_right_speed = speed
                
                if time_in_state % 8 < 1:
                    print(f"🎯 APPROACHING: {display_distance:.1f}m target")
            
            if not approaching_mode:
                self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                print("📡 APPROACH LOST: Signal lost")
        
        elif self.nav_state == 'TRACKING' and tracking_mode and display_distance:
            # Optimized tracking with course correction limiting
            current_time_for_corrections = current_time
            
            # Reset course correction counter every minute
            if current_time_for_corrections - self.bluetooth_state.get('last_course_correction', 0) > 60:
                self.bluetooth_state['course_correction_count'] = 0
            
            # Course correction logic - more lenient
            if bluetooth_distance and display_distance:
                distance_change = bluetooth_distance - self.bluetooth_state.get('movement_start_distance', bluetooth_distance)
                
                # More lenient course correction threshold
                if (distance_change > self.course_correction_threshold and 
                    self.bluetooth_state['course_correction_count'] < self.max_course_corrections_per_minute):
                    
                    self.bluetooth_state['course_correction_count'] += 1
                    self.bluetooth_state['last_course_correction'] = current_time_for_corrections
                    self.stats['course_corrections'] += 1
                    
                    print(f"🔄 COURSE CORRECTION #{self.bluetooth_state['course_correction_count']}: Distance increased {distance_change:.1f}m")
                    
                    # Brief turn instead of full turn test
                    if self.turn_direction == 'left':
                        base_left_speed = -self.turn_speed * 0.7
                        base_right_speed = self.turn_speed * 0.7
                    else:
                        base_left_speed = self.turn_speed * 0.7
                        base_right_speed = -self.turn_speed * 0.7
                    
                    time.sleep(0.5)  # Brief turn
                    self.turn_direction = 'right' if self.turn_direction == 'left' else 'left'
            
            if display_distance and display_distance <= self.approach_threshold:
                self.nav_state = 'APPROACHING'
                self.nav_start_time = current_time
                print(f"🎯 SWITCHING TO APPROACH: {display_distance:.1f}m")
            else:
                # Forward tracking with obstacle awareness
                if obstacle_distance < self.caution_threshold:
                    speed_factor = (obstacle_distance - self.danger_threshold) / (self.caution_threshold - self.danger_threshold)
                    speed = self.slow_speed * max(0.5, speed_factor)
                else:
                    speed = self.tracking_speed
                
                base_left_speed = speed
                base_right_speed = speed
                
                if time_in_state % 12 < 1:  # Reduced logging frequency
                    signal_text = "↗improving" if signal_improving else "↘degrading" if signal_degrading else "stable"
                    print(f"🚶 TRACKING: {display_distance:.1f}m signal {signal_text}")
            
            if not tracking_mode:
                self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                print("📡 TRACKING LOST: Signal lost")
        
        elif self.nav_state == 'SEARCHING':
            # Optimized search behavior
            mode_detected = tracking_mode or approaching_mode or holding_mode
            
            if mode_detected:
                if holding_mode:
                    self.nav_state = 'HOLDING'
                elif approaching_mode:
                    self.nav_state = 'APPROACHING'
                elif tracking_mode:
                    self.nav_state = 'TRACKING'
                    with self.detection_lock:
                        self.bluetooth_state['movement_start_distance'] = bluetooth_distance
                
                self.nav_start_time = current_time
                distance_text = f"{display_distance:.1f}m" if display_distance else "unknown"
                print(f"📡 SIGNAL DETECTED: Switching to {self.nav_state.lower()} mode ({distance_text})")
            elif time_in_state > self.exploration_interval:
                # Search turns with enhanced direction selection
                turn_direction = self.get_enhanced_turn_direction()
                if turn_direction == 'left':
                    base_left_speed = -self.turn_speed
                    base_right_speed = self.turn_speed
                else:
                    base_left_speed = self.turn_speed
                    base_right_speed = -self.turn_speed
                
                if time_in_state > self.exploration_interval * 1.5:
                    self.nav_start_time = current_time
                    self.stats['exploration_turns'] += 1
                    print(f"🔄 SEARCH: Looking for BlueCharm signal...")
            else:
                # Forward search movement
                if obstacle_distance > self.safe_distance:
                    base_left_speed = self.cruise_speed
                    base_right_speed = self.cruise_speed
                else:
                    base_left_speed = 0
                    base_right_speed = 0
        
        # Final obstacle check
        if obstacle_distance < self.caution_threshold and self.nav_state not in ['AVOIDING', 'BACKING']:
            # More decisive obstacle avoidance
            base_left_speed = 0
            base_right_speed = 0
        
        return base_left_speed, base_right_speed
    
    def send_motor_command(self, left_speed, right_speed):
        """Send motor command to Arduino gatekeeper"""
        if not self.arduino:
            if abs(left_speed) > 0 or abs(right_speed) > 0:
                print(f"🚫 MOTOR BLOCKED: Would send L={left_speed}, R={right_speed} (No Arduino)")
            return
        
        if not self.running:
            return
        
        left_speed = max(-255, min(255, int(left_speed)))
        right_speed = max(-255, min(255, int(right_speed)))
        
        # Track movement for signal analysis
        self.last_command_time = time.time()
        self.last_speeds = [left_speed, right_speed]
        
        command = {
            'left': left_speed,
            'right': right_speed
        }
        
        try:
            cmd_str = json.dumps(command) + '\\n'
            self.arduino.write(cmd_str.encode())
        except Exception as e:
            print(f"❌ Motor command failed: {e}")
    
    def emergency_stop(self):
        """Emergency stop with auto-reset"""
        self.emergency_stop_active = True
        self.send_motor_command(0, 0)
        print("🛑 EMERGENCY STOP EXECUTED")
        
        # Auto-reset emergency stop after 10 seconds if communication resumes
        def reset_emergency():
            time.sleep(10)
            if hasattr(self, 'emergency_stop_active'):
                self.emergency_stop_active = False
                print("🔄 Emergency stop reset - resuming operations")
        
        threading.Thread(target=reset_emergency, daemon=True).start()
    
    def log_optimized_status(self, obstacle_distance, sensor_source, lidar_data):
        """Log navigation status with reduced frequency"""
        runtime = time.time() - self.stats['start_time']
        mode_text = ['MANUAL', 'ASSISTED', 'AUTONOMOUS'][self.robot_state.get('mode', 0)]
        
        status_line = f"🎯 {mode_text} | {self.nav_state}"
        
        # Sensor display
        if sensor_source == 'robot_fusion':
            status_line += f" | FUSION: {obstacle_distance:.0f}cm"
        else:
            status_line += f" | {sensor_source.upper()}: {obstacle_distance:.1f}cm"
        
        # LiDAR information
        if lidar_data and lidar_data.get('has_data', False) and 'obstacle_sectors' in lidar_data and not lidar_data.get('stale', False):
            clear_count = len(lidar_data.get('clear_directions', []))
            closest = lidar_data.get('closest_obstacle', {})
            
            if closest.get('distance', 8000) < 8000:
                raw_distance = closest['distance']/10
                status_line += f" | LiDAR: {raw_distance:.0f}cm@{closest['angle']:.0f}°"
            else:
                status_line += f" | LiDAR: Clear"
            
            status_line += f" | {clear_count}/8 clear"
        elif self.lidar_enabled:
            # Check if we have ANY LiDAR activity
            if hasattr(self, 'lidar_processor') and self.lidar_processor and self.lidar_processor.scan_count > 0:
                status_line += f" | LiDAR: Processing ({self.lidar_processor.scan_count} scans)"
            else:
                status_line += f" | LiDAR: No data"
        
        # Bluetooth info
        if self.bluetooth_state['enabled']:
            with self.detection_lock:
                smoothed_distance = self.bluetooth_state.get('smoothed_distance')
                if smoothed_distance:
                    bt_rssi = self.bluetooth_state['target_rssi']
                    
                    if self.bluetooth_state['holding_mode']:
                        mode_symbol = "HOLD"
                    elif self.bluetooth_state['approaching_mode']:
                        mode_symbol = "APPR"
                    elif self.bluetooth_state['tracking_mode']:
                        mode_symbol = "TRACK"
                    else:
                        mode_symbol = "SCAN"
                    
                    status_line += f" | BT: {smoothed_distance:.1f}m ({bt_rssi:.0f}dBm {mode_symbol})"
                else:
                    status_line += f" | BT: No signal"
                
                corrections = self.bluetooth_state.get('course_correction_count', 0)
                if corrections > 0:
                    status_line += f" | Corrections: {corrections}"
        
        status_line += f" | Runtime: {runtime:.0f}s"
        print(status_line)
        
        if self.stuck_counter > 0:
            print(f"   Stuck counter: {self.stuck_counter}")
        
        # Statistics every 60 seconds for reduced output
        if int(runtime) % 60 == 0 and runtime > 0:
            self.print_optimized_statistics()
    
    def print_optimized_statistics(self):
        """Print navigation statistics with reduced frequency"""
        runtime = time.time() - self.stats['start_time']
        
        print("📊 === Navigator V8-LITE Statistics ===")
        print(f"   Runtime: {runtime:.0f} seconds")
        print(f"   Distance measurements: {self.stats['distance_measurements']}")
        print(f"   Obstacle avoidances: {self.stats['obstacle_avoidances']}")
        print(f"   Course corrections: {self.stats['course_corrections']}")
        
        # Navigation performance
        print("🎯 === Navigation Performance ===")
        print(f"   Tracking activations: {self.stats['tracking_activations']}")
        print(f"   Approach activations: {self.stats['approach_activations']}")
        print(f"   Hold activations: {self.stats['hold_activations']}")
        print(f"   Target reaches: {self.stats['target_reaches']}")
        
        # LiDAR statistics
        if self.lidar_enabled:
            print("🔄 === LiDAR Performance ===")
            print(f"   LiDAR scans processed: {self.stats['lidar_scans_processed']}")
            print(f"   LiDAR enhanced turns: {self.stats['lidar_enhanced_turns']}")
            
            if runtime > 0 and self.stats['lidar_scans_processed'] > 0:
                scan_rate = self.stats['lidar_scans_processed'] / runtime
                print(f"   LiDAR scan rate: {scan_rate:.2f}/sec")
        
        # Bluetooth statistics
        if self.bluetooth_state['enabled']:
            with self.detection_lock:
                print("📡 === Bluetooth Performance ===")
                print(f"   Total detections: {self.stats['bluetooth_detections']}")
                
                if runtime > 0:
                    detection_rate = self.stats['bluetooth_detections'] / runtime
                    print(f"   Detection rate: {detection_rate:.2f}/sec")
                
                print(f"   Strongest RSSI: {self.bluetooth_state['strongest_rssi']:.1f} dBm")
                
                if len(self.bluetooth_state['distance_history']) > 0:
                    avg_distance = sum(self.bluetooth_state['distance_history']) / len(self.bluetooth_state['distance_history'])
                    print(f"   Average distance: {avg_distance:.1f}m")
    
    async def bluetooth_scan_async(self):
        """Asynchronous Bluetooth scanning loop with proper cleanup"""
        scanner = None
        try:
            while self.running:
                try:
                    print("📡 Starting BLE scanner...")
                    scanner = BleakScanner(detection_callback=self.bluetooth_detection_callback)
                    await scanner.start()
                    print("📡 BLE scanner active")
                    
                    # Scan for a period, then restart to prevent resource buildup
                    scan_duration = 0
                    while self.running and scan_duration < 30:
                        await asyncio.sleep(1)
                        scan_duration += 1
                    
                    # Properly stop scanner
                    if scanner:
                        await scanner.stop()
                        print("📡 BLE scanner stopped")
                        scanner = None
                    
                    if self.running:
                        await asyncio.sleep(2)  # Pause before restart
                        
                except Exception as scan_error:
                    print(f"❌ BLE scan error: {scan_error}")
                    if scanner:
                        try:
                            await scanner.stop()
                        except:
                            pass
                        scanner = None
                    await asyncio.sleep(5)  # Wait longer on error
                    
        except Exception as e:
            print(f"❌ Bluetooth loop error: {e}")
        finally:
            # Ensure cleanup
            if scanner:
                try:
                    await scanner.stop()
                except:
                    pass
    
    def bluetooth_loop(self):
        """Bluetooth scanning loop wrapper with improved cleanup"""
        loop = None
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.bluetooth_scan_async())
        except Exception as e:
            print(f"❌ Bluetooth thread error: {e}")
        finally:
            try:
                if loop and not loop.is_closed():
                    # Cancel any remaining tasks
                    pending = asyncio.all_tasks(loop)
                    for task in pending:
                        task.cancel()
                    if pending:
                        loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
                    loop.close()
            except Exception as cleanup_error:
                print(f"⚠️ Bluetooth cleanup warning: {cleanup_error}")
    
    def stop(self):
        """Stop the navigator and clean up all resources"""
        print("🛑 Stopping Navigator V8-LITE...")
        self.running = False
        
        # Give time for threads to recognize the stop signal
        print("⏳ Waiting for threads to stop...")
        time.sleep(2.0)
        
        # Stop LiDAR if running
        if self.lidar_enabled and self.lidar_processor:
            print("🔄 Stopping LiDAR...")
            self.lidar_processor.stop_scanning()
        
        # Stop Arduino safely
        if self.arduino:
            print("🤖 Stopping Arduino...")
            self.send_motor_command(0, 0)
            time.sleep(0.2)
            
            try:
                self.arduino.close()
                print("✅ Arduino disconnected")
            except Exception as e:
                print(f"⚠️ Arduino disconnect warning: {e}")
        
        # Print final statistics
        if 'start_time' in self.stats:
            runtime = time.time() - self.stats['start_time']
            print(f"📊 Final runtime: {runtime:.1f} seconds")
            self.print_optimized_statistics()
        
        print("👋 Navigator V8-LITE shutdown complete")

def main():
    """Main entry point for Navigator V8-LITE"""
    print("=" * 80)
    print("🚀 BLUETOOTH LIDAR NAVIGATOR V8-LITE")
    print("=" * 80)
    print()
    print("CORE FUNCTIONALITY ONLY:")
    print("  ✅ Fixed LiDAR left/right orientation (CRITICAL)")
    print("  ✅ LiDAR-enhanced navigation without visualization")
    print("  ✅ Optimized Bluetooth target tracking")
    print("  ✅ Multi-sensor fusion (LiDAR + Ultrasonic)")
    print("  ✅ Robot body-aware obstacle avoidance")
    print("  ✅ Automatic USB device detection")
    print("  ✅ Fast autonomous mode response")
    print()
    print("REMOVED FOR PERFORMANCE:")
    print("  ❌ LiDAR visualization (caused Pi overload)")
    print("  ❌ Matplotlib and GUI components")
    print("  ❌ Excessive debug output")
    print()
    
    sensor_status = []
    if LIDAR_AVAILABLE:
        sensor_status.append("✅ LiDAR Ready (Navigation Only)")
    else:
        sensor_status.append("❌ LiDAR Unavailable (pip install rplidar)")
    
    if BLUETOOTH_AVAILABLE:
        sensor_status.append("✅ Bluetooth Ready")
    else:
        sensor_status.append("❌ Bluetooth Unavailable (pip install bleak)")
    
    print("SENSOR STATUS:")
    for status in sensor_status:
        print(f"  {status}")
    print()
    
    # Create and start navigator
    navigator = BluetoothLidarNavigator()
    navigator.running = True
    
    try:
        navigator.start()
        
        # Keep main thread alive
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\\n🛑 Shutdown requested by user...")
        navigator.stop()
    except Exception as e:
        print(f"❌ Navigation system error: {e}")
        navigator.stop()

if __name__ == "__main__":
    main()