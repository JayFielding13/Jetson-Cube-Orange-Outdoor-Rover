#!/usr/bin/env python3
"""
Bluetooth LiDAR Navigator V7
================================================================================
Enhanced navigation with integrated RPLidar A1 for object detection and mapping
Built on the proven V6 foundation with added LiDAR capabilities

FEATURES:
- All proven V6 navigation capabilities (Bluetooth, ultrasonic, state machine)
- Integrated RPLidar A1 for 360° obstacle detection
- Enhanced object detection and path planning
- LiDAR-assisted obstacle avoidance
- Thread-safe LiDAR data processing
- Fallback to ultrasonic if LiDAR fails
- Real-time obstacle mapping
- Improved navigation decision making

NEW LIDAR CAPABILITIES:
- 360° obstacle detection vs single-direction ultrasonic
- Object classification (walls, obstacles, clear paths)
- Direction-specific obstacle distances
- Enhanced path planning using LiDAR data
- Sector-based navigation analysis
- Real-time obstacle mapping for better decisions

HARDWARE:
- Raspberry Pi 4B with Bluetooth 5.0
- Arduino gatekeeper with ultrasonic sensor
- BlueCharm BLE beacon (MAC: DD:34:02:09:CA:1E)
- RPLidar A1 on /dev/ttyUSB0
- Cytron MDDS30 motor driver

Part of the Mini Rover Development Project
Author: Developed incrementally with Jay Fielding
Version: 7.0 - LiDAR Integration
Date: 2025-01-13
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

# Visualization imports with graceful fallback
try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    VISUALIZATION_AVAILABLE = True
    print("✅ Visualization (matplotlib) library available")
except ImportError:
    print("⚠️ Visualization library not available - install with: pip install matplotlib")
    VISUALIZATION_AVAILABLE = False

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
        
        # Tight space minimums (for confined navigation)
        self.tight_front_distance = self.lidar_to_front + self.tight_space_margin  # 26.0 cm
        self.tight_rear_distance = self.lidar_to_rear + self.tight_space_margin    # 15.0 cm
        self.tight_left_distance = self.lidar_to_left + self.tight_space_margin    # 16.0 cm
        self.tight_right_distance = self.lidar_to_right + self.tight_space_margin  # 18.0 cm
        
        # Required corridor width to navigate safely
        self.min_corridor_width = self.total_width + (2 * self.safety_margin)  # 38.0 cm
        self.tight_corridor_width = self.total_width + (2 * self.tight_space_margin)  # 34.0 cm
        
        print(f"🤖 Robot Physical Profile Loaded:")
        print(f"   Dimensions: {self.total_length}cm × {self.total_width}cm")
        print(f"   LiDAR Position: {self.lidar_to_front}cm from front, {self.lidar_to_left}cm from left")
        print(f"   Safe Distances: F={self.min_front_distance}cm R={self.min_rear_distance}cm L={self.min_left_distance}cm R={self.min_right_distance}cm")
        print(f"   Corridor Requirements: {self.min_corridor_width}cm normal, {self.tight_corridor_width}cm tight")
    
    def get_safe_distance(self, lidar_distance, sector, tight_space=False):
        """
        Convert LiDAR reading to actual robot clearance
        Returns how much space the robot body actually has
        """
        if tight_space:
            margins = {
                'front': self.tight_front_distance,
                'front_left': self.tight_front_distance, 
                'front_right': self.tight_front_distance,
                'left': self.tight_left_distance,
                'right': self.tight_right_distance,
                'back_left': self.tight_rear_distance,
                'back_right': self.tight_rear_distance,
                'back': self.tight_rear_distance
            }
        else:
            margins = {
                'front': self.min_front_distance,
                'front_left': self.min_front_distance,
                'front_right': self.min_front_distance, 
                'left': self.min_left_distance,
                'right': self.min_right_distance,
                'back_left': self.min_rear_distance,
                'back_right': self.min_rear_distance,
                'back': self.min_rear_distance
            }
        
        required_clearance = margins.get(sector, self.min_front_distance)
        actual_clearance = lidar_distance - required_clearance
        
        return max(0, actual_clearance)
    
    def can_fit_through_gap(self, left_distance, right_distance, tight_space=False):
        """
        Determine if robot can physically fit through a detected gap
        """
        total_gap_width = left_distance + right_distance
        required_width = self.tight_corridor_width if tight_space else self.min_corridor_width
        
        fits = total_gap_width >= required_width
        margin = total_gap_width - required_width if fits else 0
        
        return fits, margin
    
    def get_safe_approach_distance(self, target_distance, approach_sector='front'):
        """
        Calculate how close robot can safely approach a Bluetooth target
        """
        if approach_sector in ['front', 'front_left', 'front_right']:
            min_distance = self.lidar_to_front / 100.0  # Convert to meters
        elif approach_sector in ['left', 'back_left']:
            min_distance = self.lidar_to_left / 100.0
        elif approach_sector in ['right', 'back_right']:
            min_distance = self.lidar_to_right / 100.0
        else:  # rear
            min_distance = self.lidar_to_rear / 100.0
        
        # Add safety margin and ensure minimum distance
        safe_distance = min_distance + 0.05  # 5cm safety margin in meters
        return max(0.15, safe_distance)  # Never closer than 15cm
    
    def analyze_corridor_width(self, left_distance, right_distance):
        """
        Analyze if rover can navigate a corridor and what strategy to use
        """
        total_width = left_distance + right_distance
        
        if total_width >= self.min_corridor_width:
            return 'normal', total_width - self.min_corridor_width
        elif total_width >= self.tight_corridor_width:
            return 'tight', total_width - self.tight_corridor_width
        else:
            return 'blocked', 0

class USBDeviceDetector:
    """
    Automatically detects and identifies USB devices connected to the system
    Prevents port conflicts by finding the correct port for each device
    """
    
    def __init__(self):
        self.detected_devices = {}
        self.arduino_port = None
        self.lidar_port = None
        
        print("🔍 USB Device Detector initialized")
    
    def scan_usb_ports(self):
        """Scan all available USB/serial ports"""
        print("🔍 Scanning for USB devices...")
        
        # Get all available serial ports
        available_ports = []
        
        # Method 1: Using serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        for port in ports:
            available_ports.append({
                'device': port.device,
                'description': port.description,
                'manufacturer': getattr(port, 'manufacturer', 'Unknown'),
                'vid': getattr(port, 'vid', None),
                'pid': getattr(port, 'pid', None),
                'serial_number': getattr(port, 'serial_number', 'Unknown')
            })
        
        # Method 2: Direct filesystem scan for Linux
        if os.name == 'posix':  # Linux/Unix systems
            for device_path in glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'):
                if not any(p['device'] == device_path for p in available_ports):
                    available_ports.append({
                        'device': device_path,
                        'description': 'USB Serial Device',
                        'manufacturer': 'Unknown',
                        'vid': None,
                        'pid': None,
                        'serial_number': 'Unknown'
                    })
        
        print(f"📋 Found {len(available_ports)} USB serial devices:")
        for i, port in enumerate(available_ports):
            print(f"   {i+1}. {port['device']} - {port['description']}")
            if port['vid'] and port['pid']:
                print(f"      VID:PID = {port['vid']:04X}:{port['pid']:04X}")
            if port['manufacturer'] != 'Unknown':
                print(f"      Manufacturer: {port['manufacturer']}")
        
        return available_ports
    
    def identify_arduino(self, port_path, timeout=3):
        """
        Attempt to identify if a port contains an Arduino by looking for JSON responses
        """
        try:
            print(f"🤖 Testing for Arduino on {port_path}...")
            
            # Try common Arduino baud rates
            baud_rates = [115200, 9600, 57600, 38400]
            
            for baud in baud_rates:
                try:
                    with serial.Serial(port_path, baud, timeout=1) as ser:
                        time.sleep(2)  # Allow Arduino to reset and start sending data
                        
                        # Clear any existing data
                        ser.flushInput()
                        
                        # Look for JSON-formatted data (Arduino gatekeeper sends JSON)
                        start_time = time.time()
                        json_count = 0
                        
                        while time.time() - start_time < timeout:
                            if ser.in_waiting > 0:
                                try:
                                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                                    if line.startswith('{') and line.endswith('}'):
                                        # Try to parse as JSON
                                        data = json.loads(line)
                                        if any(key in data for key in ['mode', 'distance', 'valid', 'emergency']):
                                            json_count += 1
                                            if json_count >= 2:  # Multiple valid JSON responses
                                                print(f"✅ Arduino detected on {port_path} at {baud} baud")
                                                print(f"   Sample data: {data}")
                                                return baud
                                except (json.JSONDecodeError, UnicodeDecodeError):
                                    continue
                            time.sleep(0.1)
                        
                except (serial.SerialException, OSError):
                    continue
            
            print(f"❌ No Arduino detected on {port_path}")
            return None
            
        except Exception as e:
            print(f"❌ Error testing Arduino on {port_path}: {e}")
            return None
    
    def identify_lidar(self, port_path, timeout=3):
        """
        Attempt to identify if a port contains an RPLidar by testing the protocol
        """
        try:
            print(f"📡 Testing for LiDAR on {port_path}...")
            
            # Try to connect with RPLidar protocol
            try:
                lidar = RPLidar(port_path, baudrate=115200, timeout=1)
                
                # Try to get device info (RPLidar specific command)
                info = lidar.get_info()
                
                if info and 'model' in info:
                    print(f"✅ LiDAR detected on {port_path}")
                    print(f"   Model: {info.get('model', 'Unknown')}")
                    print(f"   Firmware: {info.get('firmware', 'Unknown')}")
                    print(f"   Hardware: {info.get('hardware', 'Unknown')}")
                    lidar.disconnect()
                    return True
                
                lidar.disconnect()
                
            except Exception as lidar_error:
                # If RPLidar library fails, try manual protocol detection
                try:
                    with serial.Serial(port_path, 115200, timeout=1) as ser:
                        # Send RPLidar GET_INFO command (0xA5 0x50)
                        ser.write(b'\xA5\x50')
                        time.sleep(0.1)
                        
                        response = ser.read(100)
                        if len(response) > 7 and response[0] == 0xA5 and response[1] == 0x5A:
                            print(f"✅ LiDAR protocol detected on {port_path}")
                            return True
                        
                except (serial.SerialException, OSError):
                    pass
            
            print(f"❌ No LiDAR detected on {port_path}")
            return False
            
        except Exception as e:
            print(f"❌ Error testing LiDAR on {port_path}: {e}")
            return False
    
    def detect_devices(self):
        """
        Main device detection routine - scans ports and identifies devices
        """
        print("🔍 === USB Device Detection Started ===")
        
        # Scan for available ports
        available_ports = self.scan_usb_ports()
        
        if not available_ports:
            print("❌ No USB serial devices found!")
            return False
        
        arduino_candidates = []
        lidar_candidates = []
        
        # Test each port for device identification
        print("\n🧪 Testing ports for device identification...")
        
        for port_info in available_ports:
            port_path = port_info['device']
            
            # Skip if port is already identified
            if port_path in [self.arduino_port, self.lidar_port]:
                continue
            
            print(f"\n📍 Testing {port_path}...")
            
            # Test for Arduino first (less intrusive)
            arduino_baud = self.identify_arduino(port_path)
            if arduino_baud:
                arduino_candidates.append({
                    'port': port_path,
                    'baud': arduino_baud,
                    'info': port_info
                })
                continue  # Don't test for LiDAR if Arduino found
            
            # Test for LiDAR
            if LIDAR_AVAILABLE and self.identify_lidar(port_path):
                lidar_candidates.append({
                    'port': port_path,
                    'info': port_info
                })
        
        # Assign detected devices
        print(f"\n📊 Detection Results:")
        print(f"   Arduino candidates: {len(arduino_candidates)}")
        print(f"   LiDAR candidates: {len(lidar_candidates)}")
        
        # Assign Arduino
        if arduino_candidates:
            self.arduino_port = arduino_candidates[0]['port']
            self.arduino_baud = arduino_candidates[0]['baud']
            print(f"✅ Arduino assigned to: {self.arduino_port} @ {self.arduino_baud} baud")
            
            if len(arduino_candidates) > 1:
                print(f"⚠️ Multiple Arduino candidates found - using first one")
        else:
            print(f"❌ No Arduino detected on any port")
        
        # Assign LiDAR
        if lidar_candidates:
            self.lidar_port = lidar_candidates[0]['port']
            print(f"✅ LiDAR assigned to: {self.lidar_port}")
            
            if len(lidar_candidates) > 1:
                print(f"⚠️ Multiple LiDAR candidates found - using first one")
        else:
            print(f"❌ No LiDAR detected on any port")
        
        # Final summary
        print(f"\n📋 Final Device Assignment:")
        print(f"   Arduino: {self.arduino_port or 'Not found'}")
        print(f"   LiDAR: {self.lidar_port or 'Not found'}")
        
        # Store results
        self.detected_devices = {
            'arduino': {
                'port': self.arduino_port,
                'baud': getattr(self, 'arduino_baud', 115200),
                'detected': self.arduino_port is not None
            },
            'lidar': {
                'port': self.lidar_port,
                'detected': self.lidar_port is not None
            }
        }
        
        return self.arduino_port is not None or self.lidar_port is not None
    
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
    LiDAR data processor for enhanced navigation
    Provides 360° obstacle detection and path analysis
    """
    
    def __init__(self, port=None, enable_visualization=True):
        self.lidar = None
        self.port = port  # Will be set by device detector
        self.running = False
        self.data_queue = queue.Queue(maxsize=3)
        self.scan_thread = None
        
        # Visualization components
        self.enable_visualization = enable_visualization and VISUALIZATION_AVAILABLE
        self.viz_data_queue = queue.Queue(maxsize=3)
        self.viz_thread = None
        self.fig = None
        self.ax = None
        self.line = None
        self.animation = None
        
        # LiDAR navigation data
        self.obstacle_sectors = {}  # Sector-based obstacle distances
        self.clear_directions = []  # Available navigation directions
        self.closest_obstacle = {'distance': 8000, 'angle': 0}
        self.last_scan_time = 0
        self.scan_count = 0
        
        # Navigation sectors (8 directions)
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
        
        # LiDAR settings
        self.min_distance = 100    # Filter close noise (mm)
        self.max_distance = 8000   # Maximum useful range (mm)
        self.sector_obstacle_threshold = 800  # mm - consider sector blocked
        self.clear_path_threshold = 800      # mm - consider direction clear (was 1200, too restrictive)
        
        print("🔄 LiDAR Processor initialized")
        print(f"📏 Range: {self.min_distance}-{self.max_distance}mm")
        print(f"🚧 Obstacle threshold: {self.sector_obstacle_threshold}mm")
        print(f"✅ Clear path threshold: {self.clear_path_threshold}mm")
        if self.enable_visualization:
            print("📊 Real-time visualization enabled")
        else:
            print("📊 Visualization disabled")
    
    def connect(self):
        """Connect to LiDAR sensor"""
        if not LIDAR_AVAILABLE:
            print("❌ LiDAR library not available")
            return False
        
        if not self.port:
            print("❌ No LiDAR port specified")
            return False
        
        try:
            print(f"🔗 Connecting to RPLidar A1 on {self.port}")
            self.lidar = RPLidar(self.port, baudrate=115200, timeout=3)
            
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
            
            # Setup visualization if enabled
            if self.enable_visualization:
                self._setup_visualization()
            
            return True
            
        except Exception as e:
            print(f"❌ LiDAR connection error: {e}")
            return False
    
    def _setup_visualization(self):
        """Setup matplotlib visualization for real-time LiDAR display"""
        try:
            print("📊 Setting up real-time LiDAR visualization...")
            
            # Create figure and polar subplot
            self.fig, self.ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(projection='polar'))
            self.ax.set_ylim(0, 6000)  # 6 meter range for visualization
            self.ax.set_title('Rover LiDAR - Real-time Navigation Data')
            self.ax.grid(True)
            
            # Initialize empty plot with rover-specific styling
            self.line, = self.ax.plot([], [], 'ro', markersize=2, alpha=0.7, label='Obstacles')
            
            # Add sector boundaries for navigation zones
            sector_angles = [0, 45, 90, 135, 180, 225, 270, 315, 360]
            for angle in sector_angles:
                self.ax.axvline(math.radians(angle), color='gray', alpha=0.3, linestyle='--')
            
            # Add range circles
            for radius in [1000, 2000, 3000, 4000, 5000]:
                circle = plt.Circle((0, 0), radius, fill=False, color='lightgray', alpha=0.5)
                
            self.ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.0))
            
            print("📊 Visualization setup complete")
            
        except Exception as e:
            print(f"❌ Visualization setup error: {e}")
            self.enable_visualization = False
    
    def _animate(self, frame):
        """Animation function for matplotlib real-time display"""
        try:
            viz_data = self.viz_data_queue.get_nowait()
            angles, distances = viz_data
            self.line.set_data(angles, distances)
            
            # Update title with current stats
            self.ax.set_title(f'Rover LiDAR - {len(angles)} points | Clear: {len(self.clear_directions)}/8 sectors')
            
            return self.line,
            
        except queue.Empty:
            return self.line,
    
    def start_scanning(self):
        """Start background LiDAR scanning thread"""
        if not self.lidar:
            print("❌ LiDAR not connected")
            return False
        
        self.running = True
        self.scan_thread = threading.Thread(target=self._scan_worker, daemon=True)
        self.scan_thread.start()
        
        # Start visualization if enabled
        if self.enable_visualization and self.fig:
            self._start_visualization()
        
        print("📡 LiDAR scanning started")
        return True
    
    def _start_visualization(self):
        """Start the visualization - X11 forwarding support"""
        try:
            print("📊 Starting real-time LiDAR visualization...")
            
            # Check if we have a display available - be more permissive for X11 forwarding
            import os
            display_available = 'DISPLAY' in os.environ
            
            if not display_available:
                print("⚠️ No DISPLAY environment variable - disabling visualization")
                print("📊 LiDAR data collection continues normally without visualization")
                self.enable_visualization = False
                return
            
            print(f"📊 Display detected ({os.environ.get('DISPLAY', 'unknown')}) - attempting visualization...")
            
            # Start animation (this will only work if running locally with display)
            self.animation = animation.FuncAnimation(
                self.fig, self._animate, interval=100, blit=True, cache_frame_data=False
            )
            
            # Try to show the plot non-blocking
            try:
                plt.ion()  # Turn on interactive mode
                plt.show(block=False)
                plt.pause(0.1)  # Allow time for window to appear
                print("📊 Visualization window should now be visible")
            except Exception as viz_error:
                print(f"⚠️ Visualization display failed: {viz_error}")
                print("💡 Note: For X11 forwarding, ensure 'ssh -X' was used")
                print("📊 LiDAR data collection continues normally")
                self.enable_visualization = False
            
        except Exception as e:
            print(f"❌ Visualization start error: {e}")
            self.enable_visualization = False
    
    def stop_scanning(self):
        """Stop LiDAR scanning and cleanup"""
        self.running = False
        
        if self.scan_thread:
            self.scan_thread.join(timeout=2)
        
        # Cleanup visualization
        if self.enable_visualization:
            try:
                if self.animation:
                    self.animation.event_source.stop()
                if self.fig:
                    plt.close(self.fig)
            except:
                pass
        
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
                print("🔌 LiDAR disconnected")
            except:
                pass
    
    def _scan_worker(self):
        """Background thread for continuous LiDAR scanning"""
        try:
            for scan in self.lidar.iter_scans():
                if not self.running:
                    break
                
                # Process scan data
                processed_data = self._process_scan(scan)
                
                if processed_data:
                    # Add to navigation queue, remove old if full
                    if self.data_queue.full():
                        try:
                            self.data_queue.get_nowait()
                        except queue.Empty:
                            pass
                    
                    try:
                        self.data_queue.put(processed_data, block=False)
                        self.last_scan_time = time.time()
                        self.scan_count += 1
                        
                        # Feed visualization data if enabled
                        if self.enable_visualization:
                            self._update_visualization_data(processed_data['valid_points'])
                        
                        if self.scan_count % 10 == 0:
                            print(f"📡 LiDAR scan #{self.scan_count}: {len(processed_data['valid_points'])} points, {len(processed_data['clear_directions'])}/8 clear sectors")
                            
                    except queue.Full:
                        pass
                        
        except Exception as e:
            print(f"❌ LiDAR scan error: {e}")
    
    def _update_visualization_data(self, valid_points):
        """Update visualization queue with latest scan data"""
        try:
            if len(valid_points) > 20:  # Only update if we have enough points
                angles = []
                distances = []
                
                for angle, distance in valid_points:
                    angles.append(math.radians(angle))
                    distances.append(distance)
                
                # Add to visualization queue, remove old if full
                if self.viz_data_queue.full():
                    try:
                        self.viz_data_queue.get_nowait()
                    except queue.Empty:
                        pass
                
                try:
                    self.viz_data_queue.put((angles, distances), block=False)
                except queue.Full:
                    pass  # Skip if queue is full
                    
        except Exception as e:
            print(f"❌ Visualization data update error: {e}")
    
    def _process_scan(self, scan):
        """Process raw LiDAR scan into navigation data"""
        valid_points = []
        sector_distances = {sector: [] for sector in self.sectors}
        closest_dist = self.max_distance
        closest_angle = 0
        
        for measurement in scan:
            quality, angle, distance = measurement
            
            # Filter valid measurements
            if self.min_distance <= distance <= self.max_distance:
                valid_points.append((angle, distance))
                
                # Track closest obstacle
                if distance < closest_dist:
                    closest_dist = distance
                    closest_angle = angle
                
                # Categorize by sector
                for sector_name, (start_angle, end_angle) in self.sectors.items():
                    if start_angle > end_angle:  # Handle wrap-around (e.g., front sector)
                        if angle >= start_angle or angle <= end_angle:
                            sector_distances[sector_name].append(distance)
                    else:
                        if start_angle <= angle <= end_angle:
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
        """Get distance to closest obstacle in front sector (for ultrasonic compatibility)"""
        if 'front' in self.obstacle_sectors:
            return self.obstacle_sectors['front'] / 10.0  # Convert mm to cm
        return 200.0  # Safe default
    
    def is_direction_clear(self, direction, min_distance_cm=80):
        """Check if a specific direction is clear for navigation"""
        if direction in self.obstacle_sectors:
            distance_cm = self.obstacle_sectors[direction] / 10.0
            return distance_cm > min_distance_cm
        return False
    
    def get_best_turn_direction(self):
        """Determine best turn direction based on LiDAR data"""
        if not self.obstacle_sectors:
            return 'left'  # Default
        
        # Check left and right sectors
        left_clear = self.is_direction_clear('left', 100)
        right_clear = self.is_direction_clear('right', 100)
        front_left_clear = self.is_direction_clear('front_left', 100)
        front_right_clear = self.is_direction_clear('front_right', 100)
        
        # Score directions based on clearance
        left_score = 0
        right_score = 0
        
        if left_clear:
            left_score += 2
        if front_left_clear:
            left_score += 1
        if right_clear:
            right_score += 2
        if front_right_clear:
            right_score += 1
        
        # Add distance-based scoring
        if 'left' in self.obstacle_sectors:
            left_score += min(5, self.obstacle_sectors['left'] / 1000)
        if 'right' in self.obstacle_sectors:
            right_score += min(5, self.obstacle_sectors['right'] / 1000)
        
        return 'left' if left_score >= right_score else 'right'


class BluetoothLidarNavigator:
    """
    Enhanced Bluetooth navigator with integrated LiDAR capabilities
    Built on proven V6 foundation with added 360° obstacle detection
    """
    
    def __init__(self, arduino_port=None, baud=None):
        """Initialize the enhanced Bluetooth LiDAR navigator"""
        # Initialize device detector
        self.device_detector = USBDeviceDetector()
        
        # These will be set by device detection
        self.arduino_port = arduino_port
        self.baud = baud
        self.arduino = None
        self.running = False
        
        # Initialize robot physical profile for precise navigation
        self.robot_profile = RobotPhysicalProfile()
        
        # Initialize LiDAR processor (port will be set by detector)
        self.lidar_processor = LidarProcessor() if LIDAR_AVAILABLE else None
        self.lidar_enabled = False
        
        # Robot state from Arduino gatekeeper (unchanged from V6)
        self.robot_state = {
            'mode': 0,                    
            'rc_valid': False,            
            'emergency_stop': False,      
            'ultrasonic_distance': 200.0, 
            'last_update': time.time()    
        }
        
        # Bluetooth state (unchanged from V6 - proven working)
        self.bluetooth_state = {
            'enabled': BLUETOOTH_AVAILABLE,
            'target_mac': 'DD:34:02:09:CA:1E',      
            'target_name': 'BlueCharm_190853',      
            'target_rssi': None,                    
            'target_distance': None,                
            'smoothed_distance': None,              
            'last_detection': 0,                    
            'scanner_running': False,               
            'detection_count': 0,                   
            'rssi_history': deque(maxlen=8),        
            'distance_history': deque(maxlen=6),    
            'strongest_rssi': -200,                 
            'detection_timeout_count': 0,           
            'callback_detections': 0,               
            'tracking_mode': False,                 
            'approaching_mode': False,              
            'holding_mode': False,                  
            'movement_start_rssi': None,            
            'movement_start_time': 0,               
            'signal_improving': False,              
            'signal_degrading': False,              
            'movement_start_distance': None,        
            'last_distance': None,                  
            'wrong_direction_detected': False,      
            'turn_test_mode': False,                
            'test_direction': 'left',               
            'turn_test_start_rssi': None,           
            'at_target_start_time': 0,              
            'last_direction_change': 0,             
            'holding_start_rssi': None,             
            'holding_start_time': 0                 
        }
        
        # Thread-safe detection lock
        self.detection_lock = threading.Lock()
        
        # Navigation state machine (enhanced with LiDAR awareness)
        self.nav_state = 'SEARCHING'
        self.nav_start_time = time.time()
        self.turn_direction = 'left'
        self.stuck_counter = 0
        
        # Confined space navigation
        self.confined_space_mode = False
        self.confined_space_start_time = 0
        self.confined_space_strategy = 0  # 0=forward, 1=back_turn, 2=long_back
        self.confined_space_attempts = 0
        
        # Smart avoidance state
        self.smart_turning = False
        self.turn_start_time = 0
        self.min_turn_time = 1.0      
        self.max_turn_time = 3.5      
        self.clear_distance_threshold = 90.0  
        
        # Motor speeds (unchanged from V6)
        self.cruise_speed = 100      
        self.tracking_speed = 80     
        self.slow_speed = 70         
        self.turn_speed = 85         
        self.reverse_speed = 80      
        self.approach_speed = 60     
        self.hold_speed = 40         
        
        # AGGRESSIVE robot-aware navigation thresholds per user request
        self.danger_threshold = 15.0   # Aggressive emergency stop (ultrasonic)
        self.caution_threshold = 25.0  # Aggressive slow down (ultrasonic) 
        self.safe_distance = 40.0      # Aggressive safe following (ultrasonic)
        
        # AGGRESSIVE LiDAR thresholds - user wants to get within 15cm of objects
        self.lidar_danger_threshold = 15.0   # 15cm aggressive emergency stop
        self.lidar_caution_threshold = 25.0  # 25cm aggressive slow down  
        self.lidar_safe_threshold = 40.0     # 40cm aggressive safe distance
        self.stuck_threshold = 4       
        
        # Timing parameters (unchanged from V6)
        self.exploration_interval = 10.0       
        self.turn_duration = 2.0               
        self.backup_duration = 2.5             
        self.stuck_reset_time = 15.0           
        self.aggressive_backup_duration = 3.5  
        self.tracking_duration = 3.0           
        self.turn_test_duration = 0.6          
        self.hold_check_interval = 2.0         
        
        # Robot-aware Bluetooth parameters
        self.bluetooth_timeout = 12.0          
        self.bluetooth_weight = 0.35           
        # Target distance now accounts for robot front clearance
        self.target_bluetooth_distance = self.robot_profile.get_safe_approach_distance(0.5, 'front')
        self.target_tolerance = 0.15           
        self.approach_threshold = 1.2          
        self.tracking_threshold = 15.0         
        self.signal_improvement_threshold = 1.0 
        self.signal_degradation_threshold = 2.0 
        self.distance_increase_threshold = 1.5  
        self.course_correction_threshold = 2.0  
        
        # RSSI smoothing (unchanged from V6)
        self.rssi_smoothing_weight = 0.5       
        self.distance_smoothing_weight = 0.6   
        self.outlier_rejection_threshold = 3.0 
        
        # Enhanced statistics with LiDAR tracking
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
            'rssi_outliers_rejected': 0,
            'long_tracking_phases': 0,
            'lidar_scans_processed': 0,
            'lidar_obstacle_detections': 0,
            'lidar_enhanced_turns': 0,
            'sensor_fallbacks': 0,
            'confined_space_detections': 0,
            'confined_space_escapes': 0,
            'confined_space_time': 0
        }
        
        print("🚀 Bluetooth LiDAR Navigator V7 - AGGRESSIVE Object Detection")
        print(f"📊 AGGRESSIVE Navigation: Danger={self.danger_threshold}cm | Caution={self.caution_threshold}cm | Safe={self.safe_distance}cm")
        print(f"🔄 AGGRESSIVE LiDAR: {'Enabled' if LIDAR_AVAILABLE else 'Disabled'} | Danger={self.lidar_danger_threshold:.0f}cm | Caution={self.lidar_caution_threshold:.0f}cm | Safe={self.lidar_safe_threshold:.0f}cm")
        print(f"📡 Bluetooth: {'Enabled' if self.bluetooth_state['enabled'] else 'Disabled'} | Target: {self.bluetooth_state['target_mac']}")
        print(f"🚶 Tracking: Target distance {self.target_bluetooth_distance}m ±{self.target_tolerance}m")
        print(f"📈 Signal: {self.signal_improvement_threshold}dBm improvement | {self.signal_degradation_threshold}dBm degradation")
        print(f"🛡️ AGGRESSIVE Safety: 15cm obstacle thresholds with robot body awareness")
    
    # Copy all V6 Bluetooth methods unchanged (proven working)
    def smooth_rssi(self, new_rssi):
        """Apply balanced smoothing to RSSI readings"""
        if not self.bluetooth_state['rssi_history']:
            return new_rssi
        
        history_weight = 1.0 - self.rssi_smoothing_weight
        recent_avg = sum(list(self.bluetooth_state['rssi_history'])[-3:]) / min(3, len(self.bluetooth_state['rssi_history']))
        
        smoothed_rssi = (self.rssi_smoothing_weight * new_rssi) + (history_weight * recent_avg)
        return smoothed_rssi
    
    def smooth_distance(self, new_distance):
        """Apply balanced smoothing to distance readings with reduced outlier rejection"""
        if not self.bluetooth_state['distance_history']:
            return new_distance
        
        recent_avg = sum(list(self.bluetooth_state['distance_history'])[-3:]) / min(3, len(self.bluetooth_state['distance_history']))
        distance_ratio = new_distance / recent_avg if recent_avg > 0 else 1.0
        
        if distance_ratio > self.outlier_rejection_threshold or distance_ratio < (1.0 / self.outlier_rejection_threshold):
            self.stats['rssi_outliers_rejected'] += 1
            return self.bluetooth_state['smoothed_distance'] if self.bluetooth_state['smoothed_distance'] else new_distance
        
        history_weight = 1.0 - self.distance_smoothing_weight
        smoothed_distance = (self.distance_smoothing_weight * new_distance) + (history_weight * recent_avg)
        
        return smoothed_distance
    
    def bluetooth_detection_callback(self, device, advertisement_data):
        """Thread-safe callback for real-time BlueCharm detection (unchanged from V6)"""
        if device.address.upper() == self.bluetooth_state['target_mac'].upper():
            with self.detection_lock:
                raw_rssi = advertisement_data.rssi
                current_time = time.time()
                
                smoothed_rssi = self.smooth_rssi(raw_rssi)
                
                self.bluetooth_state['callback_detections'] += 1
                self.stats['bluetooth_callback_events'] += 1
                
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
                
                # Signal analysis (unchanged from V6)
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
                
                # Mode activation (unchanged from V6)
                distance_for_decisions = smoothed_distance
                
                if distance_for_decisions <= self.target_bluetooth_distance + self.target_tolerance:
                    if not self.bluetooth_state['holding_mode']:
                        self.bluetooth_state['holding_mode'] = True
                        self.bluetooth_state['at_target_start_time'] = current_time
                        self.stats['hold_activations'] += 1
                        self.stats['target_reaches'] += 1
                        print(f"🏁 TARGET HOLD: BlueCharm at {distance_for_decisions:.1f}m - holding position")
                elif distance_for_decisions <= self.approach_threshold:
                    if not self.bluetooth_state['approaching_mode']:
                        self.bluetooth_state['approaching_mode'] = True
                        self.stats['approach_activations'] += 1
                        print(f"🎯 APPROACH MODE: BlueCharm at {distance_for_decisions:.1f}m - gentle approach")
                elif distance_for_decisions <= self.tracking_threshold:
                    if not self.bluetooth_state['tracking_mode']:
                        self.bluetooth_state['tracking_mode'] = True
                        self.bluetooth_state['movement_start_rssi'] = smoothed_rssi
                        self.bluetooth_state['movement_start_time'] = current_time
                        self.stats['tracking_activations'] += 1
                        print(f"🚶 TRACKING MODE: BlueCharm at {distance_for_decisions:.1f}m - forward tracking activated")
                
                # Logging (unchanged from V6)
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
    
    def rssi_to_distance(self, rssi):
        """Convert RSSI to approximate distance for BLE beacon (unchanged from V6)"""
        if rssi == 0 or rssi < -100:
            return 30.0
        
        tx_power = -59
        ratio = (tx_power - rssi) / 20.0
        distance = math.pow(10, ratio)
        
        return max(0.3, min(distance, 30.0))
    
    def is_path_clear(self, ultrasonic_distance, lidar_distance=None, tight_space=False):
        """Robot-aware path clearance check using both ultrasonic and LiDAR"""
        # Primary check with ultrasonic
        ultrasonic_clear = ultrasonic_distance > self.clear_distance_threshold
        
        # Robot-aware LiDAR check if available
        if lidar_distance is not None:
            # Use robot body dimensions for clearance calculation
            if tight_space:
                required_clearance = self.robot_profile.tight_front_distance
            else:
                required_clearance = self.lidar_caution_threshold
            
            lidar_robot_clearance = self.robot_profile.get_safe_distance(
                lidar_distance * 10, 'front', tight_space
            ) / 10.0
            
            lidar_clear = lidar_robot_clearance > 5.0  # 5cm aggressive robot clearance
            
            # Both sensors must agree for clearance
            return ultrasonic_clear and lidar_clear
        
        return ultrasonic_clear
    
    def get_enhanced_obstacle_distance(self):
        """Get robot-aware obstacle distance from available sensors"""
        ultrasonic_distance = self.robot_state.get('ultrasonic_distance', 200.0)
        
        # Get LiDAR front distance if available
        if self.lidar_enabled and self.lidar_processor:
            lidar_raw_distance = self.lidar_processor.get_front_distance()
            
            if lidar_raw_distance < 200.0:  # LiDAR has valid reading
                # Convert LiDAR reading to actual robot clearance
                lidar_robot_clearance = self.robot_profile.get_safe_distance(
                    lidar_raw_distance * 10, 'front'  # Convert cm to mm for get_safe_distance
                ) / 10.0  # Convert back to cm
                
                # Use the more conservative (closer) reading  
                enhanced_distance = min(ultrasonic_distance, lidar_robot_clearance)
                return enhanced_distance, 'robot_aware_fusion'
        
        return ultrasonic_distance, 'ultrasonic'
    
    def get_enhanced_turn_direction(self):
        """Enhanced turn direction selection using LiDAR data"""
        # Get LiDAR recommendation if available
        if self.lidar_enabled and self.lidar_processor:
            lidar_direction = self.lidar_processor.get_best_turn_direction()
            self.stats['lidar_enhanced_turns'] += 1
            return lidar_direction
        
        # Fallback to V6 logic
        return self.choose_turn_direction()
    
    def connect_arduino(self):
        """Establish connection to Arduino gatekeeper with auto-detection"""
        # Use detected Arduino configuration if not manually specified
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
        """Start the enhanced navigator with automatic device detection"""
        print("🔍 Starting device detection...")
        
        # Run device detection first
        detection_success = self.device_detector.detect_devices()
        
        if not detection_success:
            print("⚠️ No devices detected, attempting manual fallback...")
            # Try common fallback configurations
            fallback_configs = [
                ('/dev/ttyUSB0', 115200),
                ('/dev/ttyUSB1', 115200), 
                ('/dev/ttyACM0', 115200),
                ('/dev/ttyACM1', 115200)
            ]
            
            arduino_connected = False
            for port, baud in fallback_configs:
                try:
                    print(f"🔄 Trying fallback Arduino connection: {port} @ {baud}")
                    test_arduino = serial.Serial(port, baud, timeout=1)
                    test_arduino.close()
                    self.arduino_port = port
                    self.baud = baud
                    arduino_connected = True
                    print(f"✅ Fallback Arduino connection successful: {port}")
                    break
                except Exception:
                    continue
            
            if not arduino_connected:
                print("❌ All fallback Arduino connections failed")
                self.arduino_port = '/dev/ttyUSB0'  # Last resort
                self.baud = 115200
        
        # Connect to Arduino
        if not self.connect_arduino():
            return False
        
        # Initialize LiDAR if available and detected
        if LIDAR_AVAILABLE and self.lidar_processor:
            detected_lidar_port = self.device_detector.get_lidar_config()
            
            if detected_lidar_port:
                self.lidar_processor.port = detected_lidar_port
                print(f"📍 Using detected LiDAR: {detected_lidar_port}")
            else:
                # Try LiDAR fallback ports (excluding Arduino port)
                lidar_fallback_ports = ['/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB0', '/dev/ttyACM1']
                if self.arduino_port in lidar_fallback_ports:
                    lidar_fallback_ports.remove(self.arduino_port)
                
                lidar_found = False
                for port in lidar_fallback_ports:
                    try:
                        print(f"🔄 Trying fallback LiDAR port: {port}")
                        if self.device_detector.identify_lidar(port):
                            self.lidar_processor.port = port
                            lidar_found = True
                            print(f"✅ Fallback LiDAR found: {port}")
                            break
                    except Exception:
                        continue
                
                if not lidar_found:
                    print("⚠️ No LiDAR found on any fallback port")
            
            # Attempt LiDAR connection
            if self.lidar_processor.port:
                if self.lidar_processor.connect():
                    if self.lidar_processor.start_scanning():
                        self.lidar_enabled = True
                        print("✅ LiDAR navigation enhancement enabled")
                    else:
                        print("⚠️ LiDAR scanning failed - using ultrasonic only")
                else:
                    print("⚠️ LiDAR connection failed - using ultrasonic only")
            else:
                print("⚠️ No LiDAR port available - using ultrasonic only")
        else:
            print("⚠️ LiDAR not available - using ultrasonic only")
        
        self.running = True
        self.stats['start_time'] = time.time()
        
        # Start communication threads (unchanged from V6)
        self.arduino_thread = threading.Thread(target=self.arduino_loop, daemon=True)
        self.navigation_thread = threading.Thread(target=self.navigation_loop, daemon=True)
        
        # Start Bluetooth tracking thread (unchanged from V6)
        if self.bluetooth_state['enabled']:
            self.bluetooth_thread = threading.Thread(target=self.bluetooth_thread_worker, daemon=True)
            self.bluetooth_thread.start()
            print("📡 BlueCharm tracking scanner started")
        
        self.arduino_thread.start()
        self.navigation_thread.start()
        
        sensor_modes = []
        if self.lidar_enabled:
            sensor_modes.append("LiDAR")
        sensor_modes.append("Ultrasonic")
        if self.bluetooth_state['enabled']:
            sensor_modes.append("Bluetooth")
        
        print(f"🚀 Enhanced Navigator V7 started with {' + '.join(sensor_modes)}")
        print("🎯 Ready for enhanced object detection and navigation")
        return True
    
    # Copy all V6 thread methods unchanged (proven working)
    def bluetooth_thread_worker(self):
        """Dedicated thread worker for Bluetooth scanning (unchanged from V6)"""
        print("📡 Starting BlueCharm tracking thread...")
        
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
        """Main async function for Bluetooth scanning (unchanged from V6)"""
        print("📡 BlueCharm tracking scanner ready")
        
        scanner = None
        
        try:
            while self.running:
                if self.robot_state.get('mode', 0) == 2:
                    if not self.bluetooth_state['scanner_running']:
                        print("📡 Starting BLE scanner...")
                        scanner = BleakScanner(self.bluetooth_detection_callback)
                        await scanner.start()
                        self.bluetooth_state['scanner_running'] = True
                        print("📡 BLE scanner active")
                    
                    current_time = time.time()
                    time_since_detection = current_time - self.bluetooth_state['last_detection']
                    
                    if time_since_detection > self.bluetooth_timeout:
                        if self.bluetooth_state['target_rssi'] is not None:
                            with self.detection_lock:
                                self.bluetooth_state['target_rssi'] = None
                                self.bluetooth_state['target_distance'] = None
                                self.bluetooth_state['smoothed_distance'] = None
                                self.bluetooth_state['detection_timeout_count'] += 1
                                
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
                                    self.bluetooth_state['holding_start_rssi'] = None
                
                else:
                    if self.bluetooth_state['scanner_running'] and scanner:
                        print("📡 Stopping BLE scanner...")
                        await scanner.stop()
                        self.bluetooth_state['scanner_running'] = False
                        scanner = None
                
                await asyncio.sleep(1.0)
        
        except Exception as e:
            print(f"❌ Bluetooth scanner error: {e}")
        
        finally:
            if scanner and self.bluetooth_state['scanner_running']:
                try:
                    print("📡 Cleaning up BLE scanner...")
                    await scanner.stop()
                    self.bluetooth_state['scanner_running'] = False
                except Exception as e:
                    print(f"⚠️ Scanner cleanup error: {e}")
    
    def arduino_loop(self):
        """Handle continuous Arduino communication (unchanged from V6)"""
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
                
                if time.time() - self.robot_state['last_update'] > 5.0:
                    print("⚠️ Arduino communication timeout")
                    consecutive_failures += 1
                
                if consecutive_failures >= max_failures:
                    print("❌ Too many Arduino communication failures")
                    self.emergency_stop()
                    break
                    
            except Exception as e:
                print(f"❌ Arduino communication error: {e}")
                consecutive_failures += 1
                time.sleep(0.1)
            
            time.sleep(0.02)
    
    def process_arduino_data(self, data):
        """Process incoming data from Arduino (unchanged from V6)"""
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
        """Main autonomous navigation logic with enhanced LiDAR integration"""
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
                
                # Check for stuck condition
                if current_time - last_stuck_check > self.stuck_reset_time:
                    if self.stuck_counter > 0:
                        self.stuck_counter = max(0, self.stuck_counter - 1)
                    last_stuck_check = current_time
                
                # Enhanced navigation with LiDAR awareness
                left_speed, right_speed = self.compute_enhanced_navigation(
                    obstacle_distance, current_time, lidar_data, sensor_source
                )
                
                self.send_motor_command(left_speed, right_speed)
                
                # Robot-aware status logging
                if current_time % 3 < 0.5:
                    self.log_enhanced_status(obstacle_distance, sensor_source, lidar_data)
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                self.send_motor_command(0, 0)
                time.sleep(1.0)
            
            time.sleep(0.5)
    
    def compute_enhanced_navigation(self, obstacle_distance, current_time, lidar_data, sensor_source):
        """
        Enhanced navigation with LiDAR integration
        Uses LiDAR data to improve obstacle detection and path planning
        """
        time_in_state = current_time - self.nav_start_time
        base_left_speed = 0
        base_right_speed = 0
        
        # Determine effective danger threshold based on available sensors
        if sensor_source == 'fusion':
            danger_threshold = min(self.danger_threshold, self.lidar_danger_threshold)
            caution_threshold = min(self.caution_threshold, self.lidar_caution_threshold)
        else:
            danger_threshold = self.danger_threshold
            caution_threshold = self.caution_threshold
        
        # ROBOT-AWARE CONFINED SPACE DETECTION
        clear_sectors = len(lidar_data.get('clear_directions', [])) if lidar_data else 4
        obstacle_sectors = lidar_data.get('obstacle_sectors', {}) if lidar_data else {}
        
        # Robot-aware corridor analysis
        corridor_status = 'open'
        corridor_width = 0
        robot_clearances = {}
        
        if obstacle_sectors:
            # Calculate actual robot clearances for each direction
            for sector, distance_mm in obstacle_sectors.items():
                robot_clearances[sector] = self.robot_profile.get_safe_distance(
                    distance_mm, sector, tight_space=False
                )
            
            # Analyze corridor width using robot dimensions
            left_distance = obstacle_sectors.get('left', 8000)
            right_distance = obstacle_sectors.get('right', 8000)
            
            if left_distance < 8000 and right_distance < 8000:
                corridor_status, corridor_width = self.robot_profile.analyze_corridor_width(
                    left_distance, right_distance
                )
            
            # Check if robot can physically navigate forward
            front_clearance = min(
                robot_clearances.get('front', 1000),
                robot_clearances.get('front_left', 1000),
                robot_clearances.get('front_right', 1000)
            ) / 10.0  # Convert mm to cm
            
            side_clearances = {
                'left': robot_clearances.get('left', 1000) / 10.0,
                'right': robot_clearances.get('right', 1000) / 10.0
            }
        
        # Confined space triggers based on robot dimensions
        confined_triggers = []
        
        # EXTREMELY RESTRICTIVE confined space triggers - robot with 400cm clearance is NOT confined!
        
        # Only trigger if TRULY trapped with minimal clearances in ALL directions
        actually_trapped = False
        
        if obstacle_sectors:
            # Check if ALL movement directions are severely blocked
            front_severely_blocked = front_clearance < 20.0  # Less than 20cm to move forward
            left_severely_blocked = side_clearances['left'] < 20.0   # Less than 20cm to turn left
            right_severely_blocked = side_clearances['right'] < 20.0 # Less than 20cm to turn right
            very_few_sectors = clear_sectors <= 1  # Almost no clear sectors
            
            # Only confined if ALL directions blocked AND very few sectors clear
            if front_severely_blocked and left_severely_blocked and right_severely_blocked and very_few_sectors:
                confined_triggers.append(f"truly_trapped(F:{front_clearance:.0f}cm L:{side_clearances['left']:.0f}cm R:{side_clearances['right']:.0f}cm sectors:{clear_sectors}/8)")
                actually_trapped = True
            
        # NEVER trigger confined space if robot has good clearance in any direction
        if (front_clearance > 50.0 or 
            side_clearances['left'] > 50.0 or 
            side_clearances['right'] > 50.0 or
            clear_sectors >= 3):
            actually_trapped = False
            confined_triggers = []  # Clear any triggers - robot is NOT confined
            
        # Definitely not confined if robot has excellent clearance (like 400cm)
        if (side_clearances['left'] > 100.0 or 
            side_clearances['right'] > 100.0 or
            front_clearance > 100.0):
            actually_trapped = False
            confined_triggers = []  # Robot has plenty of space
            
        confined_detected = actually_trapped and len(confined_triggers) > 0
        
        if confined_detected and not self.confined_space_mode:
            self.confined_space_mode = True
            self.confined_space_start_time = current_time
            self.confined_space_strategy = 0
            self.confined_space_attempts = 0
            self.stats['confined_space_detections'] += 1
            
            triggers_text = " + ".join(confined_triggers)
            print(f"🏠 ROBOT-AWARE CONFINED SPACE: {triggers_text}")
            print(f"   Robot needs: {self.robot_profile.total_length}cm×{self.robot_profile.total_width}cm + margins")
            if robot_clearances:
                print(f"   Actual clearances: F={front_clearance:.0f}cm L={side_clearances['left']:.0f}cm R={side_clearances['right']:.0f}cm")
                
        elif (clear_sectors > 3 and 
              (not obstacle_sectors or front_clearance > 15.0 or 
               side_clearances['left'] > 15.0 or side_clearances['right'] > 15.0) and 
              self.confined_space_mode):
            self.confined_space_mode = False
            self.stats['confined_space_escapes'] += 1
            self.stats['confined_space_time'] += current_time - self.confined_space_start_time
            print(f"🌟 ROBOT ESCAPED: {clear_sectors}/8 sectors, front={front_clearance:.0f}cm L={side_clearances['left']:.0f}cm R={side_clearances['right']:.0f}cm")

        # CONFINED SPACE NAVIGATION: Special handling for tight spaces
        if self.confined_space_mode and lidar_data:
            return self.navigate_confined_space(obstacle_distance, current_time, lidar_data, sensor_source)

        # ENHANCED SAFETY: Multi-sensor obstacle detection
        if obstacle_distance < danger_threshold and self.nav_state not in ['SMART_AVOIDING', 'BACKING', 'STUCK_RECOVERY']:
            # Emergency avoidance with enhanced turn direction selection
            self.nav_state = 'SMART_AVOIDING'
            self.nav_start_time = current_time
            self.turn_start_time = current_time
            self.smart_turning = True
            self.turn_direction = self.get_enhanced_turn_direction()
            self.stats['obstacle_avoidances'] += 1
            self.stats['smart_turns'] += 1
            if lidar_data:
                self.stats['lidar_obstacle_detections'] += 1
            # Show both raw sensor reading and robot clearance
            if sensor_source == 'robot_aware_fusion' and lidar_data:
                raw_lidar = lidar_data.get('obstacle_sectors', {}).get('front', 8000) / 10.0
                print(f"🧠 AGGRESSIVE ROBOT-AWARE AVOIDING! LiDAR={raw_lidar:.0f}cm → Robot clearance={obstacle_distance:.0f}cm - turning {self.turn_direction}")
            else:
                print(f"🧠 AGGRESSIVE AVOIDING! {sensor_source.upper()} obstacle at {obstacle_distance:.1f}cm - turning {self.turn_direction}")
        
        # Get Bluetooth state (thread-safe, unchanged from V6)
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
        
        # Enhanced state machine with LiDAR awareness
        if self.nav_state == 'SMART_AVOIDING':
            # Enhanced obstacle avoidance with LiDAR path checking
            turn_time = current_time - self.turn_start_time
            
            if turn_time < self.min_turn_time:
                # Minimum turn time
                if self.turn_direction == 'left':
                    base_left_speed = -self.turn_speed
                    base_right_speed = self.turn_speed
                else:
                    base_left_speed = self.turn_speed
                    base_right_speed = -self.turn_speed
            else:
                # Enhanced path clearance check
                lidar_front_distance = None
                if lidar_data and 'obstacle_sectors' in lidar_data:
                    lidar_front_distance = lidar_data['obstacle_sectors'].get('front', 8000) / 10.0  # Convert to cm
                
                path_clear = self.is_path_clear(obstacle_distance, lidar_front_distance)
                
                if path_clear:
                    # Path is clear - resume appropriate mode
                    self.smart_turning = False
                    
                    if holding_mode:
                        self.nav_state = 'HOLDING'
                    elif approaching_mode:
                        self.nav_state = 'APPROACHING'
                    elif tracking_mode:
                        self.nav_state = 'FORWARD_TRACKING'
                    else:
                        self.nav_state = 'SEARCHING'
                    
                    self.nav_start_time = current_time
                    if lidar_front_distance is not None:
                        sensor_text = f"{sensor_source} (robot clearance: {lidar_front_distance:.0f}cm)"
                    else:
                        sensor_text = sensor_source
                    print(f"✅ AGGRESSIVE CLEAR: Path clear via {sensor_text} - resuming {self.nav_state.lower()}")
                elif turn_time > self.max_turn_time:
                    # Maximum turn time - try backing up
                    self.nav_state = 'BACKING'
                    self.nav_start_time = current_time
                    self.stuck_counter += 1
                    self.smart_turning = False
                    print(f"🔄 ENHANCED STUCK: Max turn time reached - backing up (count: {self.stuck_counter})")
                else:
                    # Continue turning
                    if self.turn_direction == 'left':
                        base_left_speed = -self.turn_speed
                        base_right_speed = self.turn_speed
                    else:
                        base_left_speed = self.turn_speed
                        base_right_speed = -self.turn_speed
                    
                    if turn_time % 2 < 1:
                        print(f"🧠 ENHANCED TURNING: {self.turn_direction} for {turn_time:.1f}s, {sensor_source} obstacle at {obstacle_distance:.1f}cm")
        
        # Rest of the navigation states are largely unchanged from V6
        # but now use enhanced obstacle detection
        elif self.nav_state == 'BACKING':
            backup_speed = self.reverse_speed
            if self.stuck_counter >= 3:
                backup_speed = min(130, self.reverse_speed * 1.5)
            
            base_left_speed = -backup_speed
            base_right_speed = -backup_speed
            
            required_backup_time = self.backup_duration
            if self.stuck_counter >= 3:
                required_backup_time = self.aggressive_backup_duration
            
            if time_in_state > required_backup_time:
                if self.stuck_counter >= self.stuck_threshold:
                    self.nav_state = 'STUCK_RECOVERY'
                    self.nav_start_time = current_time
                    self.stats['stuck_recoveries'] += 1
                    print(f"🆘 STUCK RECOVERY: Multiple stuck detections ({self.stuck_counter})")
                else:
                    self.nav_state = 'SMART_AVOIDING'
                    self.nav_start_time = current_time
                    self.turn_start_time = current_time
                    self.smart_turning = True
                    self.turn_direction = self.get_enhanced_turn_direction()  # Enhanced turn selection
                    print(f"🧠 ENHANCED RETRY: Switching to {self.turn_direction} turn (stuck: {self.stuck_counter})")
        
        elif self.nav_state == 'STUCK_RECOVERY':
            # Stuck recovery with enhanced turning direction
            recovery_backup_time = self.aggressive_backup_duration
            recovery_turn_time = self.turn_duration * 3
            
            if time_in_state < recovery_backup_time:
                backup_speed = min(255, self.reverse_speed * 1.1)
                base_left_speed = -backup_speed
                base_right_speed = -backup_speed
                if time_in_state % 2 < 1:
                    print(f"🆘 AGGRESSIVE BACKUP: {time_in_state:.1f}s at {backup_speed} speed")
            elif time_in_state < recovery_backup_time + recovery_turn_time:
                turn_speed = min(255, self.turn_speed * 1.2)
                # Use enhanced turn direction for recovery
                enhanced_direction = self.get_enhanced_turn_direction()
                if enhanced_direction == 'left':
                    base_left_speed = -turn_speed
                    base_right_speed = turn_speed
                else:
                    base_left_speed = turn_speed
                    base_right_speed = -turn_speed
                if time_in_state % 2 < 1:
                    print(f"🆘 ENHANCED RECOVERY TURN: {enhanced_direction} for {time_in_state - recovery_backup_time:.1f}s")
            else:
                self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                self.stuck_counter = 0
                print("🔄 RECOVERY COMPLETE: Resetting to search mode")
        
        # All other navigation states remain largely unchanged from V6
        # (HOLDING, APPROACHING, FORWARD_TRACKING, TURN_TESTING, SEARCHING)
        # but now benefit from enhanced obstacle detection
        elif self.nav_state == 'HOLDING' and holding_mode and display_distance:
            # Holding mode with enhanced obstacle awareness
            with self.detection_lock:
                if self.bluetooth_state['holding_start_rssi'] is None:
                    self.bluetooth_state['holding_start_rssi'] = self.bluetooth_state.get('target_rssi')
                    self.bluetooth_state['holding_start_time'] = current_time
                    print(f"🏁 HOLDING INITIALIZED: Started at {self.bluetooth_state['holding_start_rssi']:.0f}dBm")
                
                current_rssi = self.bluetooth_state.get('target_rssi')
                holding_start_rssi = self.bluetooth_state['holding_start_rssi']
                
                rssi_change_detected = False
                if current_rssi and holding_start_rssi:
                    rssi_change = abs(current_rssi - holding_start_rssi)
                    if rssi_change > 10.0:
                        rssi_change_detected = True
                        print(f"🚨 BLUECHARM MOVED: RSSI changed {rssi_change:.1f}dBm ({holding_start_rssi:.0f} → {current_rssi:.0f})")
                        print(f"🔄 RESETTING TRACKING: BlueCharm moved - clearing smoothing and returning to search")
                        
                        self.bluetooth_state['smoothed_distance'] = None
                        self.bluetooth_state['distance_history'].clear()
                        self.bluetooth_state['rssi_history'].clear()
                        self.bluetooth_state['holding_start_rssi'] = None
            
            if rssi_change_detected:
                self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                print("🔄 EXITING HOLD: BlueCharm moved - returning to search mode")
            else:
                distance_error = display_distance - self.target_bluetooth_distance
                
                if abs(distance_error) <= self.target_tolerance:
                    base_left_speed = 0
                    base_right_speed = 0
                    if time_in_state % 5 < 1:
                        print(f"🏁 TARGET PERFECT: Holding {display_distance:.1f}m distance")
                elif distance_error > 0:
                    # Enhanced caution with obstacle awareness
                    if obstacle_distance < caution_threshold:
                        adjustment_speed = min(self.hold_speed * 0.6, abs(distance_error) * 20)
                    else:
                        adjustment_speed = min(self.hold_speed, abs(distance_error) * 40)
                    base_left_speed = adjustment_speed
                    base_right_speed = adjustment_speed
                    if time_in_state % 3 < 1:
                        print(f"📏 ADJUST CLOSER: {display_distance:.1f}m → {self.target_bluetooth_distance}m")
                else:
                    adjustment_speed = min(self.hold_speed, abs(distance_error) * 40)
                    base_left_speed = -adjustment_speed
                    base_right_speed = -adjustment_speed
                    if time_in_state % 3 < 1:
                        print(f"📏 ADJUST FARTHER: {display_distance:.1f}m → {self.target_bluetooth_distance}m")
                
                if not holding_mode or (display_distance and display_distance > self.approach_threshold):
                    with self.detection_lock:
                        self.bluetooth_state['holding_start_rssi'] = None
                    self.nav_state = 'SEARCHING'
                    self.nav_start_time = current_time
                    print("📡 HOLD LOST: Signal lost or too far - returning to search")
        
        elif self.nav_state == 'APPROACHING' and approaching_mode and display_distance:
            # Approaching with enhanced obstacle awareness
            if display_distance <= self.target_bluetooth_distance + self.target_tolerance:
                with self.detection_lock:
                    self.bluetooth_state['holding_start_rssi'] = None
                self.nav_state = 'HOLDING'
                self.nav_start_time = current_time
                print(f"🎯 SWITCHING TO HOLD: {display_distance:.1f}m - target reached")
            else:
                # Enhanced speed control with sensor fusion
                if obstacle_distance < caution_threshold:
                    speed_factor = (obstacle_distance - danger_threshold) / (caution_threshold - danger_threshold)
                    speed = self.slow_speed * max(0.4, speed_factor)
                else:
                    speed = self.approach_speed
                
                base_left_speed = speed
                base_right_speed = speed
                
                if time_in_state % 4 < 1:
                    print(f"🎯 ENHANCED APPROACHING: Moving toward {display_distance:.1f}m signal at {speed:.0f} speed")
            
            if not approaching_mode:
                self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                print("📡 APPROACH LOST: Signal lost - returning to search")
        
        elif self.nav_state == 'FORWARD_TRACKING' and tracking_mode:
            # Forward tracking with enhanced obstacle detection
            with self.detection_lock:
                if self.bluetooth_state['movement_start_distance'] is None:
                    self.bluetooth_state['movement_start_distance'] = bluetooth_distance
                    self.bluetooth_state['last_distance'] = bluetooth_distance
                    self.bluetooth_state['wrong_direction_detected'] = False
                    print(f"🏁 TRACKING INITIALIZED: Starting at {bluetooth_distance:.1f}m")
                
                if bluetooth_distance and self.bluetooth_state['last_distance']:
                    distance_change = bluetooth_distance - self.bluetooth_state['last_distance']
                    total_change = bluetooth_distance - self.bluetooth_state['movement_start_distance']
                    
                    if total_change > self.course_correction_threshold:
                        self.bluetooth_state['wrong_direction_detected'] = True
                        print(f"🚨 WRONG DIRECTION: Distance increased {total_change:.1f}m ({self.bluetooth_state['movement_start_distance']:.1f}m → {bluetooth_distance:.1f}m)")
                        print(f"🔄 IMMEDIATE COURSE CORRECTION: Turning around to find BlueCharm")
                        self.nav_state = 'TURN_TESTING'
                        self.nav_start_time = current_time
                        self.stats['course_corrections'] += 1
                        self.bluetooth_state['turn_test_mode'] = True
                        self.bluetooth_state['test_direction'] = self.get_enhanced_turn_direction()  # Enhanced selection
                        self.bluetooth_state['turn_test_start_rssi'] = self.bluetooth_state['target_rssi']
                        self.bluetooth_state['movement_start_distance'] = None
                        self.bluetooth_state['last_distance'] = None
                        return 0, 0
                
                self.bluetooth_state['last_distance'] = bluetooth_distance
            
            if bluetooth_distance and bluetooth_distance <= self.approach_threshold:
                with self.detection_lock:
                    self.bluetooth_state['movement_start_distance'] = None
                self.nav_state = 'APPROACHING'
                self.nav_start_time = current_time
                print(f"🎯 SWITCHING TO APPROACH: {display_distance:.1f}m")
            elif signal_degrading and time_in_state > self.tracking_duration:
                with self.detection_lock:
                    self.bluetooth_state['movement_start_distance'] = None
                self.nav_state = 'TURN_TESTING'
                self.nav_start_time = current_time
                self.stats['course_corrections'] += 1
                self.stats['long_tracking_phases'] += 1
                self.bluetooth_state['turn_test_mode'] = True
                self.bluetooth_state['test_direction'] = self.get_enhanced_turn_direction()  # Enhanced selection
                self.bluetooth_state['turn_test_start_rssi'] = self.bluetooth_state['target_rssi']
                print(f"🔄 SIGNAL DEGRADING: Testing enhanced turn after {time_in_state:.1f}s tracking")
            else:
                # Enhanced speed control with sensor fusion
                if obstacle_distance < caution_threshold:
                    speed_factor = (obstacle_distance - danger_threshold) / (caution_threshold - danger_threshold)
                    speed = self.slow_speed * max(0.4, speed_factor)
                else:
                    speed = self.tracking_speed
                
                base_left_speed = speed
                base_right_speed = speed
                
                if time_in_state % 6 < 1:
                    signal_text = "↗improving" if signal_improving else "↘degrading" if signal_degrading else "stable"
                    print(f"🚶 ENHANCED TRACKING: {display_distance:.1f}m signal {signal_text} (t={time_in_state:.1f}s)")
            
            if not tracking_mode:
                self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                print("📡 TRACKING LOST: Signal lost - returning to search")
        
        elif self.nav_state == 'TURN_TESTING':
            # Turn testing with enhanced direction selection
            if time_in_state < self.turn_test_duration:
                with self.detection_lock:
                    test_direction = self.bluetooth_state.get('test_direction', 'left')
                
                if test_direction == 'left':
                    base_left_speed = -self.turn_speed
                    base_right_speed = self.turn_speed
                else:
                    base_left_speed = self.turn_speed
                    base_right_speed = -self.turn_speed
                
                if time_in_state % 2 < 1:
                    print(f"🔄 ENHANCED TESTING: {test_direction} turn for signal improvement")
            elif time_in_state > 3.0:
                self.nav_state = 'FORWARD_TRACKING'
                self.nav_start_time = current_time
                print(f"🔄 MAX TURN TIME: Resuming forward tracking after {time_in_state:.1f}s turn")
            else:
                # Evaluate test results (unchanged from V6)
                with self.detection_lock:
                    current_rssi = self.bluetooth_state.get('target_rssi')
                    start_rssi = self.bluetooth_state.get('turn_test_start_rssi')
                    test_direction = self.bluetooth_state.get('test_direction', 'left')
                    
                    self.stats['turn_tests'] += 1
                    
                    if current_rssi and start_rssi:
                        improvement = current_rssi - start_rssi
                        
                        if improvement >= self.signal_improvement_threshold:
                            self.nav_state = 'TURN_TESTING'
                            self.nav_start_time = current_time
                            self.bluetooth_state['signal_improving'] = True
                            self.bluetooth_state['signal_degrading'] = False
                            self.bluetooth_state['turn_test_start_rssi'] = current_rssi
                            print(f"✅ ENHANCED TEST SUCCESS: {test_direction} improved by {improvement:.1f}dBm - continuing {test_direction}")
                        elif improvement < -self.signal_degradation_threshold:
                            self.nav_state = 'FORWARD_TRACKING'
                            self.nav_start_time = current_time
                            print(f"🔄 SIGNAL DEGRADING: {test_direction} turn making signal worse ({improvement:.1f}dBm) - resuming forward")
                        elif test_direction == 'left':
                            self.bluetooth_state['test_direction'] = 'right'
                            self.nav_start_time = current_time
                            print(f"🔄 TRYING RIGHT: Left turn didn't help, testing right")
                        else:
                            self.nav_state = 'FORWARD_TRACKING'
                            self.nav_start_time = current_time
                            print(f"⚠️ NO IMPROVEMENT: Continuing forward tracking")
                    else:
                        self.nav_state = 'SEARCHING'
                        self.nav_start_time = current_time
                        print(f"📡 NO SIGNAL: Returning to search")
                    
                    self.bluetooth_state['turn_test_mode'] = False
        
        elif self.nav_state == 'SEARCHING':
            # Searching with enhanced turn selection
            mode_detected = tracking_mode or approaching_mode or holding_mode
            
            if mode_detected:
                if holding_mode:
                    with self.detection_lock:
                        self.bluetooth_state['holding_start_rssi'] = None
                    self.nav_state = 'HOLDING'
                elif approaching_mode:
                    self.nav_state = 'APPROACHING'
                elif tracking_mode:
                    with self.detection_lock:
                        self.bluetooth_state['movement_start_distance'] = None
                        self.bluetooth_state['last_distance'] = None
                        self.bluetooth_state['wrong_direction_detected'] = False
                    self.nav_state = 'FORWARD_TRACKING'
                
                self.nav_start_time = current_time
                distance_text = f"{display_distance:.1f}m" if display_distance else "unknown"
                print(f"📡 SIGNAL DETECTED: Switching to {self.nav_state.lower()} mode ({distance_text})")
            elif time_in_state > self.exploration_interval:
                # Enhanced search turns using LiDAR data
                turn_direction = self.get_enhanced_turn_direction()
                if turn_direction == 'left':
                    base_left_speed = -self.turn_speed
                    base_right_speed = self.turn_speed
                else:
                    base_left_speed = self.turn_speed
                    base_right_speed = -self.turn_speed
                
                if time_in_state > self.exploration_interval * 1.5:
                    self.nav_start_time = current_time
                    sensor_text = "Enhanced" if self.lidar_enabled else "Basic"
                    print(f"🔄 {sensor_text.upper()} SEARCH: Looking for signal...")
            else:
                # Search forward movement with enhanced obstacle awareness
                if obstacle_distance > self.safe_distance:
                    base_left_speed = self.slow_speed
                    base_right_speed = self.slow_speed
                else:
                    base_left_speed = 0
                    base_right_speed = 0
        
        # Enhanced obstacle avoidance trigger
        if obstacle_distance < caution_threshold and self.nav_state not in ['SMART_AVOIDING', 'AVOIDING', 'BACKING', 'STUCK_RECOVERY']:
            self.nav_state = 'SMART_AVOIDING'
            self.nav_start_time = current_time
            self.turn_start_time = current_time
            self.smart_turning = True
            self.turn_direction = self.get_enhanced_turn_direction()
            return 0, 0
        
        elif self.nav_state not in ['SMART_AVOIDING', 'AVOIDING', 'BACKING', 'STUCK_RECOVERY', 'HOLDING', 'APPROACHING', 'FORWARD_TRACKING', 'TURN_TESTING', 'SEARCHING']:
            self.nav_state = 'SEARCHING'
            self.nav_start_time = current_time
            print("⚠️ Unknown navigation state - defaulting to search")
        
        return base_left_speed, base_right_speed
    
    def navigate_confined_space(self, obstacle_distance, current_time, lidar_data, sensor_source):
        """
        Special navigation logic for confined spaces (corridors, tight areas)
        Uses progressive escape strategies when most sectors are blocked
        """
        time_in_confined = current_time - self.confined_space_start_time
        clear_directions = lidar_data.get('clear_directions', [])
        obstacle_sectors = lidar_data.get('obstacle_sectors', {})
        
        # Strategy progression based on time and attempts
        if time_in_confined > 15 and self.confined_space_strategy < 2:
            self.confined_space_strategy += 1
            self.confined_space_attempts = 0
            print(f"🏠 CONFINED SPACE: Escalating to strategy {self.confined_space_strategy}")
        
        if self.confined_space_strategy == 0:
            # STRATEGY 0: Turn toward ANY clear direction first - don't back up immediately
            
            # Calculate actual clearances for smarter decisions
            direction_clearances = {}
            for direction, distance_mm in obstacle_sectors.items():
                clearance_cm = distance_mm / 10.0
                direction_clearances[direction] = clearance_cm
            
            # Find the BEST direction with most clearance (even if not "clear")
            if direction_clearances:
                best_direction = max(direction_clearances, key=direction_clearances.get)
                best_clearance = direction_clearances[best_direction]
                
                print(f"🏠 CONFINED: Best direction is {best_direction} with {best_clearance:.0f}cm clearance")
                
                # If ANY direction has decent clearance, turn toward it
                if best_clearance > 50.0:  # 50cm is enough to navigate
                    if best_direction in ['left', 'front_left', 'back_left']:
                        # Turn left toward clearance
                        turn_speed = self.turn_speed * 0.8
                        left_speed = -turn_speed
                        right_speed = turn_speed
                        print(f"🏠 CONFINED: Turning LEFT toward {best_clearance:.0f}cm clearance")
                        return left_speed, right_speed
                        
                    elif best_direction in ['right', 'front_right', 'back_right']:
                        # Turn right toward clearance
                        turn_speed = self.turn_speed * 0.8
                        left_speed = turn_speed
                        right_speed = -turn_speed
                        print(f"🏠 CONFINED: Turning RIGHT toward {best_clearance:.0f}cm clearance")
                        return left_speed, right_speed
                        
                    elif best_direction == 'front' and best_clearance > 30.0:
                        # Move forward carefully if front is best
                        forward_speed = self.slow_speed * 0.5
                        print(f"🏠 CONFINED: Moving forward carefully with {best_clearance:.0f}cm clearance")
                        return forward_speed, forward_speed
            
            # Only escalate to backing up if we've tried turning for a while
            if time_in_confined > 5.0:  # Give turning a chance first
                self.confined_space_strategy = 1
                print("🏠 CONFINED: Tried turning, now escalating to back & turn strategy")
            else:
                # Keep trying to turn toward best direction
                if direction_clearances:
                    best_direction = max(direction_clearances, key=direction_clearances.get)
                    if best_direction in ['left', 'front_left', 'back_left']:
                        return -self.turn_speed * 0.5, self.turn_speed * 0.5
                    else:
                        return self.turn_speed * 0.5, -self.turn_speed * 0.5
        
        if self.confined_space_strategy == 1:
            # STRATEGY 1: Back up and turn sequence - enhanced for narrow spaces
            sequence_time = time_in_confined % 10  # 10-second cycle for better maneuvering
            
            if sequence_time < 4:
                # Extended back up phase for narrow corridors
                print(f"🏠 CONFINED: Backing up to create maneuvering space ({sequence_time:.1f}s)")
                return -self.reverse_speed * 0.9, -self.reverse_speed * 0.9
                
            elif sequence_time < 7:
                # Turn phase - enhanced direction selection
                left_dist = obstacle_sectors.get('left', 0)
                right_dist = obstacle_sectors.get('right', 0)
                back_left_dist = obstacle_sectors.get('back_left', 0)
                back_right_dist = obstacle_sectors.get('back_right', 0)
                
                # Score directions including rear sectors
                left_score = left_dist + back_left_dist
                right_score = right_dist + back_right_dist
                
                if left_score > right_score:
                    turn_direction = 'left'
                    left_speed = -self.turn_speed * 0.8
                    right_speed = self.turn_speed * 0.8
                else:
                    turn_direction = 'right'
                    left_speed = self.turn_speed * 0.8
                    right_speed = -self.turn_speed * 0.8
                
                print(f"🏠 CONFINED: Enhanced turn {turn_direction} (scores: L={left_score:.0f} R={right_score:.0f})")
                return left_speed, right_speed
                
            else:
                # Careful forward probe
                print(f"🏠 CONFINED: Careful forward probe after enhanced turn")
                return self.slow_speed * 0.4, self.slow_speed * 0.4
        
        if self.confined_space_strategy == 2:
            # STRATEGY 2: Long backup to escape tight corner
            sequence_time = time_in_confined % 12  # 12-second cycle
            
            if sequence_time < 6:
                # Extended backup
                print(f"🏠 CONFINED: Extended backup to escape corner")
                return -self.reverse_speed, -self.reverse_speed
                
            elif sequence_time < 9:
                # 180-degree turn
                print(f"🏠 CONFINED: 180-degree escape turn")
                return -self.turn_speed, self.turn_speed
                
            else:
                # Forward probe
                print(f"🏠 CONFINED: Forward probe after escape maneuver")
                return self.slow_speed * 0.6, self.slow_speed * 0.6
        
        # Fallback - shouldn't reach here
        return 0, 0
    
    def choose_turn_direction(self):
        """Choose turn direction alternating to avoid patterns (unchanged from V6)"""
        if hasattr(self, 'last_turn_direction'):
            self.last_turn_direction = 'right' if self.last_turn_direction == 'left' else 'left'
            return self.last_turn_direction
        else:
            self.last_turn_direction = 'left'
            return 'left'
    
    def send_motor_command(self, left_speed, right_speed):
        """Send motor command to Arduino gatekeeper (unchanged from V6)"""
        if not self.arduino or not self.running:
            return
        
        left_speed = max(-255, min(255, int(left_speed)))
        right_speed = max(-255, min(255, int(right_speed)))
        
        command = {
            'left': left_speed,
            'right': right_speed
        }
        
        try:
            cmd_str = json.dumps(command) + '\n'
            self.arduino.write(cmd_str.encode())
        except Exception as e:
            print(f"❌ Motor command failed: {e}")
    
    def emergency_stop(self):
        """Emergency stop (unchanged from V6)"""
        self.send_motor_command(0, 0)
        print("🛑 EMERGENCY STOP EXECUTED")
    
    def log_enhanced_status(self, obstacle_distance, sensor_source, lidar_data):
        """Log aggressive robot-aware navigation status"""
        runtime = time.time() - self.stats['start_time']
        mode_text = ['MANUAL', 'ASSISTED', 'AUTONOMOUS'][self.robot_state.get('mode', 0)]
        
        status_line = f"🤖 {mode_text} | {self.nav_state}"
        
        if self.smart_turning:
            status_line += f"🧠"
        
        if self.confined_space_mode:
            status_line += f"🏠"
        
        # Robot-aware sensor display
        if sensor_source == 'robot_aware_fusion':
            status_line += f" | AGGRESSIVE-FUSION: {obstacle_distance:.0f}cm clearance"
        elif sensor_source == 'fusion':
            status_line += f" | FUSION: {obstacle_distance:.1f}cm"
        else:
            status_line += f" | {sensor_source.upper()}: {obstacle_distance:.1f}cm"
        
        # Robot-aware LiDAR information with clearance calculations
        if lidar_data and 'obstacle_sectors' in lidar_data and not lidar_data.get('stale', False):
            clear_count = len(lidar_data.get('clear_directions', []))
            closest = lidar_data.get('closest_obstacle', {})
            
            if closest.get('distance', 8000) < 8000:
                raw_distance = closest['distance']/10
                robot_clearance = self.robot_profile.get_safe_distance(
                    closest['distance'], 'front', self.confined_space_mode
                ) / 10.0
                status_line += f" | LiDAR: {raw_distance:.0f}cm→{robot_clearance:.0f}cm@{closest['angle']:.0f}°"
            else:
                status_line += f" | LiDAR: Clear"
            
            status_line += f" | Sectors: {clear_count}/8"
            
            # Show side clearances for debugging confined space detection
            if 'obstacle_sectors' in lidar_data:
                left_raw = lidar_data['obstacle_sectors'].get('left', 8000) / 10.0
                right_raw = lidar_data['obstacle_sectors'].get('right', 8000) / 10.0
                left_clear = self.robot_profile.get_safe_distance(left_raw * 10, 'left', self.confined_space_mode) / 10.0
                right_clear = self.robot_profile.get_safe_distance(right_raw * 10, 'right', self.confined_space_mode) / 10.0
                status_line += f" | Sides: L={left_clear:.0f}cm R={right_clear:.0f}cm"
                    
        elif self.lidar_enabled:
            status_line += f" | LiDAR: No data"
        
        # Bluetooth info (unchanged from V6)
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
                        if self.bluetooth_state['signal_improving']:
                            mode_symbol += "↗"
                        elif self.bluetooth_state['signal_degrading']:
                            mode_symbol += "↘"
                    else:
                        mode_symbol = "PASS"
                    
                    status_line += f" | BT: {smoothed_distance:.1f}m ({bt_rssi:.0f}dBm {mode_symbol})"
                else:
                    status_line += f" | BT: No signal"
                
                callbacks = self.bluetooth_state['callback_detections']
                status_line += f" | Det: {callbacks}"
        
        status_line += f" | Runtime: {runtime:.0f}s"
        print(status_line)
        
        if self.stuck_counter > 0:
            print(f"   Stuck counter: {self.stuck_counter}/{self.stuck_threshold}")
        
        # Enhanced statistics every 30 seconds
        if int(runtime) % 30 == 0 and runtime > 0:
            self.print_enhanced_statistics()
    
    def print_enhanced_statistics(self):
        """Print comprehensive enhanced navigation statistics"""
        runtime = time.time() - self.stats['start_time']
        
        print("📊 === Enhanced Navigator V7 Statistics ===")
        print(f"   Runtime: {runtime:.0f} seconds")
        print(f"   Distance measurements: {self.stats['distance_measurements']}")
        print(f"   Obstacle avoidances: {self.stats['obstacle_avoidances']}")
        print(f"   Emergency stops: {self.stats['emergency_stops']}")
        print(f"   Stuck recoveries: {self.stats['stuck_recoveries']}")
        
        # Enhanced navigation statistics
        print("🎯 === Enhanced Navigation Statistics ===")
        print(f"   Smart turns: {self.stats['smart_turns']}")
        print(f"   Tracking activations: {self.stats['tracking_activations']}")
        print(f"   Approach activations: {self.stats['approach_activations']}")
        print(f"   Hold activations: {self.stats['hold_activations']}")
        print(f"   Target reaches: {self.stats['target_reaches']}")
        print(f"   Signal improvements: {self.stats['signal_improvements']}")
        print(f"   Course corrections: {self.stats['course_corrections']}")
        print(f"   Turn tests: {self.stats['turn_tests']}")
        print(f"   Long tracking phases: {self.stats['long_tracking_phases']}")
        
        # Robot-aware navigation statistics
        if self.lidar_enabled:
            print("🤖 === Robot-Aware Navigation Statistics ===")
            print(f"   LiDAR scans processed: {self.stats['lidar_scans_processed']}")
            print(f"   LiDAR obstacle detections: {self.stats['lidar_obstacle_detections']}")
            print(f"   LiDAR enhanced turns: {self.stats['lidar_enhanced_turns']}")
            print(f"   Sensor fallbacks: {self.stats['sensor_fallbacks']}")
            print(f"   Confined space detections: {self.stats['confined_space_detections']}")
            print(f"   Confined space escapes: {self.stats['confined_space_escapes']}")
            if self.stats['confined_space_time'] > 0:
                print(f"   Time in confined spaces: {self.stats['confined_space_time']:.1f}s")
            
            if runtime > 0 and self.stats['lidar_scans_processed'] > 0:
                scan_rate = self.stats['lidar_scans_processed'] / runtime
                print(f"   LiDAR scan rate: {scan_rate:.2f}/sec")
        
        # Bluetooth statistics (unchanged from V6)
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
        
        if runtime > 0:
            print(f"   Navigation rate: {self.stats['distance_measurements']/runtime:.1f} measurements/sec")
    
    def stop(self):
        """Stop the enhanced navigator and clean up all resources"""
        print("🛑 Stopping Enhanced Navigator V7...")
        self.running = False
        
        # Stop LiDAR if running
        if self.lidar_enabled and self.lidar_processor:
            self.lidar_processor.stop_scanning()
        
        time.sleep(0.5)
        
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
            self.print_enhanced_statistics()
        
        print("👋 Enhanced Navigator V7 shutdown complete")

def main():
    """Main entry point for Enhanced Bluetooth LiDAR Navigator V7"""
    print("=" * 80)
    print("🤖 BLUETOOTH LIDAR NAVIGATOR V7")
    print("=" * 80)
    print()
    print("ENHANCED FEATURES:")
    print("  ✅ All proven V6 Bluetooth navigation capabilities")
    print("  ✅ Integrated RPLidar A1 for 360° obstacle detection")
    print("  ✅ Multi-sensor fusion (ultrasonic + LiDAR)")
    print("  ✅ Enhanced obstacle avoidance with sector analysis")
    print("  ✅ Improved turn direction selection using LiDAR data")
    print("  ✅ Real-time path clearance assessment")
    print("  ✅ Fallback to ultrasonic if LiDAR fails")
    print("  ✅ Thread-safe LiDAR data processing")
    print("  ✅ Enhanced navigation decision making")
    print("  🔍 Automatic USB device detection and assignment")
    print("  🛡️ Intelligent port conflict resolution")
    print("  🔄 Robust fallback system for device connections")
    print("  📊 Real-time LiDAR visualization display")
    print()
    print("LIDAR ENHANCEMENTS:")
    print("  🔄 360° obstacle detection vs single-direction ultrasonic")
    print("  🎯 Sector-based path analysis (8 directions)")
    print("  🧠 Intelligent turn direction selection")
    print("  📏 Enhanced distance fusion for better accuracy")
    print("  🚧 Real-time obstacle mapping")
    print("  ✅ Graceful degradation if LiDAR unavailable")
    print()
    print("DEVICE DETECTION:")
    print("  🔍 Automatic USB port scanning and identification")
    print("  🤖 Arduino detection via JSON protocol recognition")
    print("  📡 LiDAR detection via RPLidar protocol handshake")
    print("  🔄 Smart fallback to common port configurations")
    print("  🛡️ Port conflict prevention and resolution")
    print("  📋 Detailed device information reporting")
    print()
    print("NAVIGATION MODES:")
    print("  🔍 SEARCHING: Enhanced search with LiDAR-guided turns")
    print("  🚶 FORWARD_TRACKING: Movement with multi-sensor obstacle awareness")
    print("  🔄 TURN_TESTING: Enhanced turn selection using LiDAR data")
    print("  🧠 SMART_AVOIDING: Sector-based obstacle avoidance")
    print("  🎯 APPROACHING: Sensor-fusion approach control")
    print("  🏁 HOLDING: Enhanced position maintenance")
    print("  🛡️ SAFETY: Multi-sensor emergency override")
    print()
    
    sensor_status = []
    if LIDAR_AVAILABLE:
        sensor_status.append("✅ LiDAR Ready")
    else:
        sensor_status.append("❌ LiDAR Unavailable (pip install rplidar)")
    
    if BLUETOOTH_AVAILABLE:
        sensor_status.append("✅ Bluetooth Ready")
    else:
        sensor_status.append("❌ Bluetooth Unavailable (pip install bleak)")
    
    if VISUALIZATION_AVAILABLE:
        sensor_status.append("✅ Visualization Ready")
    else:
        sensor_status.append("❌ Visualization Unavailable (pip install matplotlib)")
    
    print("SENSOR STATUS:")
    for status in sensor_status:
        print(f"  {status}")
    print()
    
    navigator = BluetoothLidarNavigator()
    
    try:
        if navigator.start():
            sensor_modes = []
            if navigator.lidar_enabled:
                sensor_modes.append("LiDAR")
            sensor_modes.append("Ultrasonic")
            if navigator.bluetooth_state['enabled']:
                sensor_modes.append("Bluetooth")
            
            print(f"✅ Enhanced Navigator V7 ready with {' + '.join(sensor_modes)}")
            print("📡 Switch to autonomous mode for enhanced navigation")
            print("🔄 LiDAR will provide 360° obstacle detection")
            print("🎯 Multi-sensor fusion ensures robust navigation")
            print("Press Ctrl+C to stop...")
            
            while True:
                time.sleep(1)
        else:
            print("❌ Failed to start enhanced navigator")
    
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested by user...")
    
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
    
    finally:
        navigator.stop()

if __name__ == "__main__":
    main()