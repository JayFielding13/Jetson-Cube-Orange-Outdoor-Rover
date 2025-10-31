#!/usr/bin/env python3
"""
Bluetooth LiDAR Navigator V8 - OPTIMIZED & STABLE
================================================================================
Major improvements over V7:
- Fixed LiDAR visualization using proven working method
- Reduced excessive course corrections and small turns  
- More stable Bluetooth tracking with less interference
- Simplified navigation logic for better target approach
- Fixed emergency stop false triggers

NEW IN V8:
- Working LiDAR visualization based on proven rplidar_realtime.py
- Streamlined navigation reduces fidgety behavior
- Improved Bluetooth target approach (gets closer than 4m)
- Reduced unnecessary course corrections
- More confident movement decisions

Part of the Mini Rover Development Project
Author: Developed incrementally with Jay Fielding
Version: 8.0 - Optimized Navigation & Fixed Visualization
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

# Visualization imports with working method from rplidar_realtime.py
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
        
        print(f"🤖 Robot Physical Profile Loaded:")
        print(f"   Dimensions: {self.total_length}cm × {self.total_width}cm")
        print(f"   LiDAR Position: {self.lidar_to_front}cm from front, {self.lidar_to_left}cm from left")
        print(f"   Safe Distances: F={self.min_front_distance}cm R={self.min_rear_distance}cm L={self.min_left_distance}cm R={self.min_right_distance}cm")
    
    def get_safe_distance(self, lidar_distance, sector):
        """
        Convert LiDAR reading to actual robot clearance
        Returns how much space the robot body actually has
        """
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
                    lidar.disconnect()
                    return True
                
                lidar.disconnect()
                
            except Exception:
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
        print("\\n🧪 Testing ports for device identification...")
        
        for port_info in available_ports:
            port_path = port_info['device']
            
            print(f"\\n📍 Testing {port_path}...")
            
            # Test for Arduino first
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

class WorkingLidarVisualizer:
    """
    Working LiDAR visualization based on proven rplidar_realtime.py
    Uses the exact same approach that we know works
    """
    
    def __init__(self, port='/dev/ttyUSB1'):
        self.port = port
        self.lidar = None
        self.data_queue = queue.Queue(maxsize=5)
        self.running = False
        self.fig = None
        self.ax = None
        self.line = None
        self.animation = None
        
        print("📊 Working LiDAR Visualizer initialized")
    
    def setup_plot(self):
        """Setup plot with rover-oriented display"""
        try:
            # Setup plot with rover orientation
            self.fig, self.ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(projection='polar'))
            self.ax.set_ylim(0, 6000)
            self.ax.set_title('Rover LiDAR - Real-time Navigation Data\n(0° = Forward, 90° = Right, 180° = Back, 270° = Left)')
            self.ax.grid(True)
            
            # Set 0° to point up (forward direction for rover)
            self.ax.set_theta_zero_location('N')  # North = up = forward
            self.ax.set_theta_direction(-1)  # Clockwise to match rover orientation
            
            # Add direction labels
            self.ax.set_thetagrids([0, 90, 180, 270], ['Forward', 'Right', 'Back', 'Left'])
            
            # Initialize empty plot
            self.line, = self.ax.plot([], [], 'ro', markersize=1.5, alpha=0.8)
            
            print("📊 Plot setup complete - rover oriented (0° = up)")
            return True
            
        except Exception as e:
            print(f"❌ Plot setup error: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def scan_thread_worker(self):
        """DISABLED - Using shared scanning from navigation processor"""
        # This method is now disabled - visualization uses shared scan data
        # from the navigation processor to avoid dual LiDAR connections
        pass
    
    def animate(self, frame):
        """Animation function exactly like working rplidar_realtime.py - FIXED"""
        try:
            angles, distances = self.data_queue.get_nowait()
            self.line.set_data(angles, distances)
            # Debug output to verify animation is being called
            if frame % 50 == 0:  # Every 50 frames
                print(f"🎥 Animation frame {frame}: {len(angles)} points plotted | Queue had {self.data_queue.qsize()} items")
            return self.line,
        except queue.Empty:
            # Debug empty queue
            if frame % 100 == 0:  # Every 100 frames when empty
                print(f"⚠️ Animation frame {frame}: Queue empty")
            return self.line,
    
    def start_visualization(self, lidar_instance):
        """Start visualization using SHARED scanning method"""
        try:
            self.lidar = lidar_instance  # Store reference but don't use for scanning
            
            if not self.setup_plot():
                return False
            
            print("📊 Starting working LiDAR visualization...")
            
            # Check display exactly like working version
            import os
            display_available = 'DISPLAY' in os.environ
            
            if not display_available:
                print("⚠️ No DISPLAY environment variable - visualization disabled")
                return False
            
            print(f"📊 Display detected ({os.environ.get('DISPLAY', 'unknown')}) - starting visualization")
            
            self.running = True
            
            # NO separate scan thread - use shared data from navigation processor
            
            # Start animation exactly like working version
            self.animation = animation.FuncAnimation(
                self.fig, self.animate, interval=100, blit=True, cache_frame_data=False
            )
            
            # Show plot in non-blocking mode for navigation system
            plt.ion()
            plt.show(block=False)
            plt.pause(0.1)
            
            # Force initial draw
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            
            # Start a background thread to keep matplotlib responsive
            def matplotlib_update_loop():
                while self.running:
                    try:
                        self.fig.canvas.flush_events()
                        time.sleep(0.1)
                    except:
                        break
            
            matplotlib_thread = threading.Thread(target=matplotlib_update_loop, daemon=True)
            matplotlib_thread.start()
            
            print("📊 Working visualization started successfully!")
            try:
                backend_name = plt.get_backend()
                print(f"📊 Animation interval: 100ms | Backend: {backend_name}")
            except Exception as e:
                print(f"📊 Animation interval: 100ms | Backend detection failed: {e}")
            return True
            
        except Exception as e:
            print(f"❌ Visualization start error: {e}")
            return False
    
    def stop_visualization(self):
        """Stop visualization with proper cleanup"""
        print("🛑 Stopping LiDAR visualization...")
        self.running = False
        
        # Stop animation
        if self.animation:
            try:
                self.animation.event_source.stop()
                print("✅ Animation stopped")
            except Exception as e:
                print(f"⚠️ Animation stop error: {e}")
        
        # Stop matplotlib thread
        if hasattr(self, 'matplotlib_thread'):
            try:
                # Thread will stop on next loop iteration
                print("✅ Matplotlib thread stopping")
            except Exception as e:
                print(f"⚠️ Matplotlib thread stop error: {e}")
        
        # Close figure
        if self.fig:
            try:
                plt.close(self.fig)
                print("✅ Figure closed")
            except Exception as e:
                print(f"⚠️ Figure close error: {e}")

class LidarProcessor:
    """
    Optimized LiDAR processor with working visualization
    """
    
    def __init__(self, port=None):
        self.lidar = None
        self.port = port
        self.running = False
        self.data_queue = queue.Queue(maxsize=3)
        self.scan_thread = None
        
        # Working visualization - enhanced debugging
        self.visualizer = WorkingLidarVisualizer(port) if VISUALIZATION_AVAILABLE else None
        if self.visualizer:
            print(f"📊 LiDAR Visualizer created for port {port}")
        
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
        self.sector_obstacle_threshold = 600  # mm - consider sector blocked (was 800)
        self.clear_path_threshold = 900       # mm - consider direction clear (was 1200)
        
        print("🔄 Optimized LiDAR Processor initialized")
        print(f"📏 Range: {self.min_distance}-{self.max_distance}mm")
        print(f"🚧 Obstacle threshold: {self.sector_obstacle_threshold}mm (optimized)")
        print(f"✅ Clear path threshold: {self.clear_path_threshold}mm (optimized)")
    
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
            
            return True
            
        except Exception as e:
            print(f"❌ LiDAR connection error: {e}")
            return False
    
    def start_scanning(self):
        """Start LiDAR scanning with working visualization"""
        if not self.lidar:
            print("❌ LiDAR not connected")
            return False
        
        self.running = True
        self.scan_thread = threading.Thread(target=self._scan_worker, daemon=True)
        self.scan_thread.start()
        
        # Start working visualization
        if self.visualizer:
            self.visualizer.start_visualization(self.lidar)
        
        print("📡 LiDAR scanning started with working visualization")
        return True
    
    def _scan_worker(self):
        """Background thread for continuous LiDAR scanning - SINGLE CONNECTION METHOD"""
        try:
            scan_count = 0
            for scan in self.lidar.iter_scans():
                if not self.running:
                    break
                
                scan_count += 1
                
                # Process for navigation
                processed_data = self._process_scan(scan)
                
                # Process for visualization (same scan, different format)
                vis_angles = []
                vis_distances = []
                for measurement in scan:
                    quality, angle, distance = measurement
                    if 100 < distance < 8000:  # Visualization filter
                        # Fix mirroring: flip left/right by negating angle, then convert to radians
                        corrected_angle = -angle if angle != 0 else 0  # Flip left/right
                        # Normalize to 0-360 range for visualization
                        if corrected_angle < 0:
                            corrected_angle += 360
                        vis_angles.append(math.radians(corrected_angle))
                        vis_distances.append(distance)
                
                # Update visualization queue - DEBUG VERSION
                if self.visualizer and len(vis_angles) > 20:
                    try:
                        if self.visualizer.data_queue.full():
                            try:
                                self.visualizer.data_queue.get_nowait()
                            except queue.Empty:
                                pass
                        # Debug: validate data before adding to queue
                        if len(vis_angles) != len(vis_distances):
                            print(f"❌ Data mismatch: {len(vis_angles)} angles vs {len(vis_distances)} distances")
                        elif any(math.isnan(a) or math.isinf(a) for a in vis_angles):
                            print(f"❌ Invalid angles detected in visualization data")
                        elif any(math.isnan(d) or math.isinf(d) for d in vis_distances):
                            print(f"❌ Invalid distances detected in visualization data")
                        else:
                            self.visualizer.data_queue.put((vis_angles, vis_distances), block=False)
                        
                        # Debug output every 50 scans
                        if scan_count % 50 == 0:
                            print(f"📊 Visualization queue updated: {len(vis_angles)} points | Queue size: {self.visualizer.data_queue.qsize()}")
                            print(f"📊 Animation running: {hasattr(self.visualizer, 'animation') and self.visualizer.animation is not None}")
                            
                            # Force matplotlib to process events
                            try:
                                if hasattr(self.visualizer, 'fig') and self.visualizer.fig:
                                    self.visualizer.fig.canvas.draw_idle()
                                    self.visualizer.fig.canvas.flush_events()
                                    print(f"🎥 Forced matplotlib update")
                            except Exception as e:
                                print(f"❌ Matplotlib update error: {e}")
                            
                    except queue.Full:
                        print(f"❌ Visualization queue full")
                    except Exception as e:
                        print(f"❌ Visualization queue error: {e}")
                
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
                        
                        if scan_count % 20 == 0:
                            print(f"📡 Navigation scan #{scan_count}: {len(processed_data['valid_points'])} points, {len(processed_data['clear_directions'])}/8 clear")
                            
                            # Debug sector mappings every 100 scans to verify fix
                            if scan_count % 100 == 0:
                                sectors_debug = {k: f"{v:.0f}mm" for k, v in processed_data['obstacle_sectors'].items()}
                                print(f"🧼 SECTORS: {sectors_debug}")
                            
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
        """Determine best turn direction based on LiDAR data"""
        if not self.obstacle_sectors:
            return 'left'  # Default
        
        # Score directions based on clearance
        left_score = 0
        right_score = 0
        
        # Check multiple sectors for each side
        left_sectors = ['left', 'front_left', 'back_left']
        right_sectors = ['right', 'front_right', 'back_right']
        
        for sector in left_sectors:
            if sector in self.obstacle_sectors:
                distance = self.obstacle_sectors[sector] / 10.0  # Convert to cm
                left_score += min(10, distance / 50.0)  # Score based on distance
        
        for sector in right_sectors:
            if sector in self.obstacle_sectors:
                distance = self.obstacle_sectors[sector] / 10.0  # Convert to cm
                right_score += min(10, distance / 50.0)  # Score based on distance
        
        return 'left' if left_score >= right_score else 'right'
    
    def stop_scanning(self):
        """Stop LiDAR scanning and cleanup"""
        self.running = False
        
        if self.scan_thread:
            self.scan_thread.join(timeout=2)
        
        # Stop visualization
        if self.visualizer:
            self.visualizer.stop_visualization()
        
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
                print("🔌 LiDAR disconnected")
            except:
                pass

class OptimizedBluetoothNavigator:
    """
    Optimized Bluetooth navigator with streamlined decision making
    Reduces excessive course corrections and small turns
    """
    
    def __init__(self, arduino_port=None, baud=None):
        """Initialize the optimized navigator"""
        # Initialize device detector and robot profile
        self.device_detector = USBDeviceDetector()
        self.robot_profile = RobotPhysicalProfile()
        
        # Connection setup
        self.arduino_port = arduino_port
        self.baud = baud
        self.arduino = None
        self.running = False
        
        # Initialize optimized LiDAR processor
        self.lidar_processor = LidarProcessor() if LIDAR_AVAILABLE else None
        self.lidar_enabled = False
        
        # Robot state from Arduino gatekeeper
        self.robot_state = {
            'mode': 0,
            'rc_valid': False,
            'emergency_stop': False,
            'ultrasonic_distance': 200.0,
            'last_update': time.time()
        }
        
        # Optimized Bluetooth state - reduced sensitivity
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
            'rssi_history': deque(maxlen=5),  # Reduced from 8 for faster response
            'distance_history': deque(maxlen=4),  # Reduced from 6
            'strongest_rssi': -200,
            'tracking_mode': False,
            'approaching_mode': False,
            'holding_mode': False,
            'signal_improving': False,
            'signal_degrading': False,
            'course_correction_count': 0,  # Track excessive corrections
            'last_course_correction': 0
        }
        
        # Thread-safe detection lock
        self.detection_lock = threading.Lock()
        
        # Optimized navigation state machine
        self.nav_state = 'SEARCHING'
        self.nav_start_time = time.time()
        self.turn_direction = 'left'
        self.stuck_counter = 0
        
        # Optimized motor speeds - more decisive movement
        self.cruise_speed = 120      # Increased from 100
        self.tracking_speed = 100    # Increased from 80
        self.slow_speed = 80         # Increased from 70
        self.turn_speed = 100        # Increased from 85
        self.reverse_speed = 90      # Increased from 80
        self.approach_speed = 70     # Increased from 60
        self.hold_speed = 50         # Increased from 40
        
        # Optimized navigation thresholds - less sensitive
        self.danger_threshold = 12.0   # Reduced from 15.0 for closer approach
        self.caution_threshold = 20.0  # Reduced from 25.0
        self.safe_distance = 35.0      # Reduced from 40.0
        
        # Optimized LiDAR thresholds
        self.lidar_danger_threshold = 12.0   # Closer approach
        self.lidar_caution_threshold = 20.0  # Less caution
        self.lidar_safe_threshold = 35.0     # More confident
        
        # Timing parameters - less fidgety
        self.exploration_interval = 15.0       # Increased from 10.0
        self.turn_duration = 1.5               # Reduced from 2.0
        self.backup_duration = 2.0             # Reduced from 2.5
        self.stuck_reset_time = 20.0           # Increased from 15.0
        
        # Optimized Bluetooth parameters - less course corrections
        self.bluetooth_timeout = 15.0          # Increased from 12.0
        self.target_bluetooth_distance = 0.3   # Closer target - was 0.5
        self.target_tolerance = 0.2            # Increased tolerance
        self.approach_threshold = 1.5          # Increased from 1.2
        self.tracking_threshold = 20.0         # Increased from 15.0
        self.signal_improvement_threshold = 2.0 # Increased from 1.0 - less sensitive
        self.signal_degradation_threshold = 3.0 # Increased from 2.0 - less sensitive
        self.course_correction_threshold = 3.0  # Increased from 2.0 - fewer corrections
        self.max_course_corrections_per_minute = 3  # Limit excessive corrections
        
        # Statistics
        self.stats = {
            'total_runtime': 0,
            'distance_measurements': 0,
            'obstacle_avoidances': 0,
            'exploration_turns': 0,
            'emergency_stops': 0,
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
        
        print("🚀 Optimized Bluetooth LiDAR Navigator V8")
        print(f"📊 OPTIMIZED Navigation: Danger={self.danger_threshold}cm | Caution={self.caution_threshold}cm | Safe={self.safe_distance}cm")
        print(f"🔄 OPTIMIZED LiDAR: Danger={self.lidar_danger_threshold}cm | Caution={self.lidar_caution_threshold}cm")
        print(f"📡 Bluetooth: Target {self.target_bluetooth_distance}m ±{self.target_tolerance}m")
        print(f"🎯 Optimizations: Faster speeds, closer approach, fewer corrections")
    
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
                
                # Optimized signal analysis - less sensitive
                if previous_rssi and len(self.bluetooth_state['rssi_history']) >= 3:
                    recent_avg = sum(list(self.bluetooth_state['rssi_history'])[-2:]) / 2
                    older_avg = sum(list(self.bluetooth_state['rssi_history'])[-4:-2]) / 2 if len(self.bluetooth_state['rssi_history']) >= 4 else previous_rssi
                    
                    rssi_change = recent_avg - older_avg
                    
                    if rssi_change >= self.signal_improvement_threshold:
                        if not self.bluetooth_state['signal_improving']:
                            self.bluetooth_state['signal_improving'] = True
                            self.bluetooth_state['signal_degrading'] = False
                    elif rssi_change <= -self.signal_degradation_threshold:
                        if not self.bluetooth_state['signal_degrading']:
                            self.bluetooth_state['signal_degrading'] = True
                            self.bluetooth_state['signal_improving'] = False
                
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
                
                # Reduced logging frequency
                if self.bluetooth_state['detection_count'] % 10 == 0:
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
        """Get robot-aware obstacle distance from available sensors"""
        ultrasonic_distance = self.robot_state.get('ultrasonic_distance', 200.0)
        
        # Get LiDAR front distance if available
        if self.lidar_enabled and self.lidar_processor:
            lidar_raw_distance = self.lidar_processor.get_front_distance()
            
            if lidar_raw_distance < 200.0:  # LiDAR has valid reading
                # Convert LiDAR reading to actual robot clearance
                lidar_robot_clearance = self.robot_profile.get_safe_distance(
                    lidar_raw_distance * 10, 'front'
                ) / 10.0
                
                # Use the more conservative reading
                enhanced_distance = min(ultrasonic_distance, lidar_robot_clearance)
                return enhanced_distance, 'robot_fusion'
        
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
        """Start the optimized navigator with automatic device detection"""
        print("🔍 Starting optimized device detection...")
        
        # Run device detection first
        detection_success = self.device_detector.detect_devices()
        
        if not detection_success:
            print("⚠️ No devices detected, attempting manual fallback...")
            fallback_configs = [
                ('/dev/ttyUSB0', 115200),
                ('/dev/ttyUSB1', 115200),
                ('/dev/ttyACM0', 115200)
            ]
            
            arduino_connected = False
            for port, baud in fallback_configs:
                try:
                    print(f"🔄 Trying fallback Arduino connection: {port}")
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
                self.arduino_port = '/dev/ttyUSB0'
                self.baud = 115200
        
        # Connect to Arduino
        if not self.connect_arduino():
            return False
        
        # Initialize LiDAR if available
        if LIDAR_AVAILABLE and self.lidar_processor:
            detected_lidar_port = self.device_detector.get_lidar_config()
            
            if detected_lidar_port:
                self.lidar_processor.port = detected_lidar_port
                print(f"📍 Using detected LiDAR: {detected_lidar_port}")
            else:
                lidar_fallback_ports = ['/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB0']
                if self.arduino_port in lidar_fallback_ports:
                    lidar_fallback_ports.remove(self.arduino_port)
                
                for port in lidar_fallback_ports:
                    try:
                        print(f"🔄 Trying fallback LiDAR port: {port}")
                        if self.device_detector.identify_lidar(port):
                            self.lidar_processor.port = port
                            print(f"✅ Fallback LiDAR found: {port}")
                            break
                    except Exception:
                        continue
            
            # Attempt LiDAR connection
            if self.lidar_processor.port:
                if self.lidar_processor.connect():
                    if self.lidar_processor.start_scanning():
                        self.lidar_enabled = True
                        print("✅ LiDAR navigation enhancement enabled with working visualization")
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
        
        sensor_modes = []
        if self.lidar_enabled:
            sensor_modes.append("LiDAR+Viz")
        sensor_modes.append("Ultrasonic")
        if self.bluetooth_state['enabled']:
            sensor_modes.append("Bluetooth")
        
        print(f"🚀 Optimized Navigator V8 started with {' + '.join(sensor_modes)}")
        print("🎯 Ready for optimized navigation with working visualization")
        return True
    
    def bluetooth_thread_worker(self):
        """Dedicated thread worker for Bluetooth scanning"""
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
        """Main async function for Bluetooth scanning"""
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
                                
                                self.bluetooth_state['tracking_mode'] = False
                                self.bluetooth_state['approaching_mode'] = False
                                self.bluetooth_state['holding_mode'] = False
                                self.bluetooth_state['signal_improving'] = False
                                self.bluetooth_state['signal_degrading'] = False
                
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
                            consecutive_failures += 1
                
                if time.time() - self.robot_state['last_update'] > 5.0:
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
                if current_time % 8 < 0.5:  # Was every 4s, now every 8s
                    self.log_optimized_status(obstacle_distance, sensor_source, lidar_data)
                
            except Exception as e:
                print(f"❌ Navigation error: {e}")
                self.send_motor_command(0, 0)
                time.sleep(1.0)
            
            time.sleep(0.1)  # Much faster response for autonomous mode
    
    def compute_optimized_navigation(self, obstacle_distance, current_time, lidar_data, sensor_source):
        """
        Optimized navigation with reduced fidgety behavior
        """
        time_in_state = current_time - self.nav_start_time
        base_left_speed = 0
        base_right_speed = 0
        
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
                if time_in_state % 8 < 1:
                    print(f"🏁 TARGET PERFECT: Holding {display_distance:.1f}m distance")
            elif distance_error > 0:
                # Move closer
                if obstacle_distance < self.caution_threshold:
                    adjustment_speed = min(self.hold_speed * 0.7, abs(distance_error) * 30)
                else:
                    adjustment_speed = min(self.hold_speed, abs(distance_error) * 50)
                base_left_speed = adjustment_speed
                base_right_speed = adjustment_speed
                if time_in_state % 5 < 1:
                    print(f"📏 CLOSER: {display_distance:.1f}m → {self.target_bluetooth_distance}m")
            else:
                # Move back
                adjustment_speed = min(self.hold_speed, abs(distance_error) * 50)
                base_left_speed = -adjustment_speed
                base_right_speed = -adjustment_speed
                if time_in_state % 5 < 1:
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
                    speed = self.approach_speed
                
                base_left_speed = speed
                base_right_speed = speed
                
                if time_in_state % 6 < 1:
                    print(f"🎯 APPROACHING: {display_distance:.1f}m at {speed:.0f} speed")
            
            if not approaching_mode:
                self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                print("📡 APPROACH LOST: Signal lost")
        
        elif self.nav_state == 'TRACKING' and tracking_mode:
            # Optimized tracking with reduced course corrections
            current_time_for_corrections = time.time()
            
            # Check for excessive course corrections
            if (current_time_for_corrections - self.bluetooth_state.get('last_course_correction', 0) > 60.0):
                self.bluetooth_state['course_correction_count'] = 0  # Reset counter every minute
            
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
                
                if time_in_state % 8 < 1:
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
                    base_left_speed = self.cruise_speed  # Increased search speed
                    base_right_speed = self.cruise_speed
                else:
                    base_left_speed = 0
                    base_right_speed = 0
        
        # DEBUG: Check if emergency stop is preventing movement
        if hasattr(self, 'emergency_stop_active') and self.emergency_stop_active:
            print(f"🛑 Movement blocked by emergency stop")
            return 0, 0
        
        # Final obstacle check
        if obstacle_distance < self.caution_threshold and self.nav_state not in ['AVOIDING', 'BACKING']:
            # More decisive obstacle avoidance
            base_left_speed = 0
            base_right_speed = 0
        
        # DEBUG: Show computed speeds
        if base_left_speed != 0 or base_right_speed != 0:
            print(f"🚗 Computed speeds: L={base_left_speed:.0f} R={base_right_speed:.0f} | State: {self.nav_state}")
        
        return base_left_speed, base_right_speed
    
    def send_motor_command(self, left_speed, right_speed):
        """Send motor command to Arduino gatekeeper"""
        if not self.arduino or not self.running:
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
        """Emergency stop"""
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
        """Log optimized navigation status"""
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
        
        # Statistics every 45 seconds
        if int(runtime) % 45 == 0 and runtime > 0:
            self.print_optimized_statistics()
    
    def print_optimized_statistics(self):
        """Print optimized navigation statistics"""
        runtime = time.time() - self.stats['start_time']
        
        print("📊 === Optimized Navigator V8 Statistics ===")
        print(f"   Runtime: {runtime:.0f} seconds")
        print(f"   Distance measurements: {self.stats['distance_measurements']}")
        print(f"   Obstacle avoidances: {self.stats['obstacle_avoidances']}")
        print(f"   Emergency stops: {self.stats['emergency_stops']}")
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
            print(f"   LiDAR obstacle detections: {self.stats['lidar_obstacle_detections']}")
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
    
    def stop(self):
        """Stop the optimized navigator and clean up all resources"""
        print("🛑 Stopping Optimized Navigator V8...")
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
            self.print_optimized_statistics()
        
        print("👋 Optimized Navigator V8 shutdown complete")

def main():
    """Main entry point for Optimized Bluetooth LiDAR Navigator V8"""
    print("=" * 80)
    print("🚀 BLUETOOTH LIDAR NAVIGATOR V8 - OPTIMIZED")
    print("=" * 80)
    print()
    print("NEW IN V8:")
    print("  ✅ Fixed LiDAR visualization using proven working method")
    print("  ✅ Reduced excessive course corrections and fidgety behavior")
    print("  ✅ Optimized for closer Bluetooth target approach (0.3m vs 0.5m)")
    print("  ✅ Faster navigation speeds and more decisive movement")
    print("  ✅ Streamlined navigation logic reduces unnecessary turns")
    print("  ✅ Limited course corrections to max 3 per minute")
    print("  ✅ Working LiDAR visualization based on rplidar_realtime.py")
    print()
    print("OPTIMIZATIONS:")
    print("  🎯 Closer target approach: 0.3m target vs 0.5m")
    print("  ⚡ Faster speeds: cruise 120, tracking 100, turn 100")
    print("  🧠 Smarter thresholds: danger 12cm, caution 20cm")
    print("  📡 Less sensitive signal analysis reduces false corrections")
    print("  🔄 Limited course corrections prevent excessive turning")
    print("  📊 Working visualization shows real LiDAR data")
    print()
    
    sensor_status = []
    if LIDAR_AVAILABLE:
        sensor_status.append("✅ LiDAR Ready + Working Visualization")
    else:
        sensor_status.append("❌ LiDAR Unavailable (pip install rplidar)")
    
    if BLUETOOTH_AVAILABLE:
        sensor_status.append("✅ Bluetooth Ready")
    else:
        sensor_status.append("❌ Bluetooth Unavailable (pip install bleak)")
    
    if VISUALIZATION_AVAILABLE:
        sensor_status.append("✅ Visualization Ready (Fixed)")
    else:
        sensor_status.append("❌ Visualization Unavailable (pip install matplotlib)")
    
    print("SENSOR STATUS:")
    for status in sensor_status:
        print(f"  {status}")
    print()
    
    navigator = OptimizedBluetoothNavigator()
    
    try:
        if navigator.start():
            sensor_modes = []
            if navigator.lidar_enabled:
                sensor_modes.append("LiDAR+WorkingViz")
            sensor_modes.append("Ultrasonic")
            if navigator.bluetooth_state['enabled']:
                sensor_modes.append("Bluetooth")
            
            print(f"✅ Optimized Navigator V8 ready with {' + '.join(sensor_modes)}")
            print("📡 Switch to autonomous mode for optimized navigation")
            print("🔄 LiDAR visualization should now show real data points")
            print("🎯 Optimized for closer approach and fewer corrections")
            print("Press Ctrl+C to stop...")
            
            while True:
                time.sleep(1)
        else:
            print("❌ Failed to start optimized navigator")
    
    except KeyboardInterrupt:
        print("\\n🛑 Shutdown requested by user...")
    
    except Exception as e:
        print(f"\\n❌ Unexpected error: {e}")
    
    finally:
        navigator.stop()

if __name__ == "__main__":
    main()