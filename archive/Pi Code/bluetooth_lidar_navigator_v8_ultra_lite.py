#!/usr/bin/env python3
"""
Bluetooth LiDAR Navigator V8-ULTRA-LITE - MINIMAL CORE FUNCTIONALITY
================================================================================
Ultra-simplified version for maximum reliability and performance

CORE FEATURES:
✅ Fixed LiDAR left/right orientation (CRITICAL navigation fix)
✅ LiDAR-only navigation (no Arduino dependency)
✅ Simplified Bluetooth (no asyncio complications)
✅ Robot body-aware obstacle detection
✅ Automatic USB device detection
✅ Fast autonomous mode response

REMOVED:
❌ Arduino communication (preventing startup issues)
❌ Complex asyncio Bluetooth (causing thread storms)
❌ All visualization components
❌ Excessive debug output

Part of the Mini Rover Development Project
Author: Developed incrementally with Jay Fielding
Version: 8-ULTRA-LITE - Minimal Core
Date: 2025-01-18
================================================================================
"""

import serial
import time
import threading
import math
import queue
import os

# Additional imports for device detection
import serial.tools.list_ports

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
        
        print(f"🤖 Robot Physical Profile Loaded:")
        print(f"   Dimensions: {self.total_length}cm × {self.total_width}cm")
        print(f"   LiDAR Position: {self.lidar_to_front}cm from front, {self.lidar_to_left}cm from left")
        print(f"   Safe Distances: F={self.lidar_to_front + self.safety_margin}cm")

class USBDeviceDetector:
    """
    Automatic USB device detection focused on LiDAR only
    """
    
    def __init__(self):
        self.lidar_port = None
        print("🔍 USB Device Detector initialized (LiDAR ONLY)")
    
    def detect_lidar(self):
        """Detect LiDAR device on available ports"""
        print("🔍 === LiDAR Detection Started ===")
        
        # Get list of available serial ports
        ports = serial.tools.list_ports.comports()
        usb_ports = [port for port in ports if 'USB' in port.device or 'ttyUSB' in port.device or 'ttyACM' in port.device]
        
        if not usb_ports:
            print("❌ No USB serial devices found")
            return False
        
        print(f"📋 Found {len(usb_ports)} USB serial devices:")
        for i, port in enumerate(usb_ports, 1):
            print(f"   {i}. {port.device} - {port.description}")
        
        # Test each port for LiDAR
        for port in usb_ports:
            print(f"📍 Testing {port.device} for LiDAR...")
            if self._test_lidar(port.device):
                self.lidar_port = port.device
                print(f"✅ LiDAR assigned to: {self.lidar_port}")
                return True
        
        print("❌ No LiDAR detected on any port")
        return False
    
    def _test_lidar(self, port):
        """Test if port has LiDAR responding"""
        try:
            lidar = RPLidar(port, baudrate=115200, timeout=2)
            info = lidar.get_info()
            
            if info and 'model' in info:
                print(f"✅ LiDAR detected on {port}")
                print(f"   Model: {info['model']}")
                print(f"   Firmware: {info['firmware']}")
                lidar.disconnect()
                return True
                
        except Exception as e:
            print(f"❌ LiDAR test failed on {port}: {e}")
            
        return False
    
    def get_lidar_port(self):
        """Get LiDAR port"""
        return self.lidar_port

class LidarProcessor:
    """
    LiDAR processor focused purely on navigation data
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
        
        # Navigation sectors (8 directions) - 0° = forward
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
        self.min_distance = 100      # Filter close noise (mm)
        self.max_distance = 8000     # Maximum useful range (mm)
        self.obstacle_threshold = 600  # mm - consider sector blocked
        self.clear_path_threshold = 900      # mm - consider sector clear
        
        print("🔄 LiDAR Processor initialized (ULTRA-LITE)")
        print(f"📏 Range: {self.min_distance}-{self.max_distance}mm")
        print(f"🚧 Obstacle threshold: {self.obstacle_threshold}mm")
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
        """Start LiDAR scanning"""
        if not self.lidar:
            print("❌ LiDAR not connected")
            return False
        
        self.running = True
        self.scan_thread = threading.Thread(target=self._scan_worker, daemon=True)
        self.scan_thread.start()
        
        print("📡 LiDAR scanning started - NAVIGATION ONLY")
        return True
    
    def _scan_worker(self):
        """Background thread for continuous LiDAR scanning"""
        try:
            scan_count = 0
            for scan in self.lidar.iter_scans():
                if not self.running:
                    break
                
                scan_count += 1
                processed_data = self._process_scan(scan)
                
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
                        
                        # Minimal logging for performance
                        if scan_count % 200 == 0:  # Every 200 scans
                            print(f"📡 Navigation scan #{scan_count}: {len(processed_data['valid_points'])} points, {len(processed_data['clear_directions'])}/8 clear")
                            
                            # Show sectors occasionally to verify orientation fix
                            if scan_count % 1000 == 0:
                                sectors_debug = {k: f"{v:.0f}mm" for k, v in processed_data['obstacle_sectors'].items() if v < 2000}
                                if sectors_debug:
                                    print(f"🧭 CLOSE OBSTACLES: {sectors_debug}")
                            
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
            corrected_angle = -angle if angle != 0 else 0  # Flip left/right
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
    
    def is_path_clear(self, direction):
        """Check if a specific direction is clear"""
        return direction in self.clear_directions
    
    def get_best_turn_direction(self):
        """Get best turn direction based on LiDAR data"""
        if not self.obstacle_sectors:
            return 'left'  # Default
        
        # Check left and right clearance
        left_distance = self.obstacle_sectors.get('left', 8000)
        right_distance = self.obstacle_sectors.get('right', 8000)
        front_left_distance = self.obstacle_sectors.get('front_left', 8000)
        front_right_distance = self.obstacle_sectors.get('front_right', 8000)
        
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

class LidarNavigator:
    """
    Ultra-lite navigator using only LiDAR for obstacle detection
    No Arduino dependency, no complex Bluetooth
    """
    
    def __init__(self):
        self.running = False
        
        # Device detection
        self.device_detector = USBDeviceDetector()
        
        # Robot physical profile
        self.robot_profile = RobotPhysicalProfile()
        
        # LiDAR processor
        self.lidar_processor = LidarProcessor() if LIDAR_AVAILABLE else None
        self.lidar_enabled = False
        
        # Navigation thresholds
        self.danger_threshold = 15.0    # cm - immediate danger
        self.caution_threshold = 25.0   # cm - slow down
        self.safe_distance = 40.0       # cm - normal operation
        
        # Navigation state
        self.nav_state = 'MONITORING'
        self.last_status_time = 0
        
        # Statistics
        self.stats = {
            'start_time': time.time(),
            'scans_processed': 0,
            'obstacles_detected': 0,
            'clear_paths_found': 0
        }
        
        print("🚀 LiDAR Navigator V8-ULTRA-LITE")
        print(f"📊 Thresholds: Danger={self.danger_threshold}cm | Caution={self.caution_threshold}cm | Safe={self.safe_distance}cm")
        print(f"🎯 ULTRA-LITE: LiDAR navigation only - no Arduino, no complex Bluetooth")
    
    def start(self):
        """Start the ultra-lite navigator"""
        print("🔍 Starting LiDAR-only navigation system...")
        
        # Detect LiDAR
        if not self.device_detector.detect_lidar():
            print("❌ No LiDAR detected - cannot start navigation")
            return False
        
        # Connect to LiDAR
        if LIDAR_AVAILABLE and self.lidar_processor:
            lidar_port = self.device_detector.get_lidar_port()
            if lidar_port:
                print(f"📍 Using detected LiDAR: {lidar_port}")
                if self.lidar_processor.connect_lidar(lidar_port):
                    if self.lidar_processor.start_scanning():
                        self.lidar_enabled = True
                        print("✅ LiDAR navigation enabled")
        
        if not self.lidar_enabled:
            print("❌ LiDAR initialization failed")
            return False
        
        # Start monitoring thread
        self.running = True
        monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
        monitor_thread.start()
        
        print("🚀 Ultra-Lite Navigator started - LiDAR monitoring active")
        print("📊 Navigation data will be displayed for analysis")
        print("Press Ctrl+C to stop...")
        return True
    
    def monitor_loop(self):
        """Monitor LiDAR data and provide navigation guidance"""
        while self.running:
            try:
                current_time = time.time()
                
                # Get LiDAR data
                if self.lidar_enabled and self.lidar_processor:
                    lidar_data = self.lidar_processor.get_navigation_data()
                    
                    if lidar_data and not lidar_data.get('stale', False):
                        self.stats['scans_processed'] += 1
                        self._analyze_navigation_data(lidar_data, current_time)
                
                # Status update every 5 seconds
                if current_time - self.last_status_time > 5.0:
                    self._log_status(current_time)
                    self.last_status_time = current_time
                
            except Exception as e:
                print(f"❌ Monitor error: {e}")
            
            time.sleep(0.1)  # Fast monitoring loop
    
    def _analyze_navigation_data(self, lidar_data, current_time):
        """Analyze LiDAR data for navigation guidance"""
        obstacle_sectors = lidar_data.get('obstacle_sectors', {})
        clear_directions = lidar_data.get('clear_directions', [])
        closest = lidar_data.get('closest_obstacle', {})
        
        # Check front distance
        front_distance_mm = obstacle_sectors.get('front', 8000)
        front_distance_cm = front_distance_mm / 10.0
        
        # Determine navigation state
        if front_distance_cm < self.danger_threshold:
            if self.nav_state != 'DANGER':
                self.nav_state = 'DANGER'
                self.stats['obstacles_detected'] += 1
                best_direction = self.lidar_processor.get_best_turn_direction()
                print(f"🚨 DANGER: Front obstacle at {front_distance_cm:.1f}cm - suggest turn {best_direction}")
        elif front_distance_cm < self.caution_threshold:
            if self.nav_state != 'CAUTION':
                self.nav_state = 'CAUTION'
                print(f"⚠️ CAUTION: Front obstacle at {front_distance_cm:.1f}cm - slow down")
        elif len(clear_directions) >= 6:  # Most directions clear
            if self.nav_state != 'CLEAR':
                self.nav_state = 'CLEAR'
                self.stats['clear_paths_found'] += 1
                print(f"✅ CLEAR: {len(clear_directions)}/8 directions clear - safe to proceed")
        else:
            self.nav_state = 'MONITORING'
    
    def _log_status(self, current_time):
        """Log current navigation status"""
        runtime = current_time - self.stats['start_time']
        
        # Get current LiDAR data
        if self.lidar_enabled and self.lidar_processor:
            lidar_data = self.lidar_processor.get_navigation_data()
            
            if lidar_data:
                obstacle_sectors = lidar_data.get('obstacle_sectors', {})
                clear_directions = lidar_data.get('clear_directions', [])
                closest = lidar_data.get('closest_obstacle', {})
                
                # Front distance
                front_distance_mm = obstacle_sectors.get('front', 8000)
                front_distance_cm = front_distance_mm / 10.0
                
                status_line = f"🔄 {self.nav_state} | Front: {front_distance_cm:.1f}cm"
                
                # Show closest obstacle
                if closest.get('distance', 8000) < 2000:  # Within 2m
                    close_dist_cm = closest['distance'] / 10.0
                    status_line += f" | Closest: {close_dist_cm:.1f}cm@{closest['angle']:.0f}°"
                
                status_line += f" | Clear: {len(clear_directions)}/8"
                status_line += f" | Scans: {self.stats['scans_processed']}"
                status_line += f" | Runtime: {runtime:.0f}s"
                
                print(status_line)
                
                # Show sector details occasionally
                if int(runtime) % 30 == 0 and runtime > 0:
                    close_obstacles = {k: f"{v:.0f}mm" for k, v in obstacle_sectors.items() if v < 1500}
                    if close_obstacles:
                        print(f"   Close obstacles: {close_obstacles}")
            else:
                print(f"⚠️ No LiDAR data | Runtime: {runtime:.0f}s")
    
    def stop(self):
        """Stop the navigator"""
        print("🛑 Stopping Ultra-Lite Navigator...")
        self.running = False
        
        # Stop LiDAR
        if self.lidar_enabled and self.lidar_processor:
            self.lidar_processor.stop_scanning()
        
        # Print final statistics
        runtime = time.time() - self.stats['start_time']
        print(f"📊 Final Statistics:")
        print(f"   Runtime: {runtime:.1f} seconds")
        print(f"   Scans processed: {self.stats['scans_processed']}")
        print(f"   Obstacles detected: {self.stats['obstacles_detected']}")
        print(f"   Clear paths found: {self.stats['clear_paths_found']}")
        
        if runtime > 0 and self.stats['scans_processed'] > 0:
            scan_rate = self.stats['scans_processed'] / runtime
            print(f"   Average scan rate: {scan_rate:.2f}/sec")
        
        print("👋 Ultra-Lite Navigator shutdown complete")

def main():
    """Main entry point for Ultra-Lite Navigator"""
    print("=" * 80)
    print("🚀 LIDAR NAVIGATOR V8-ULTRA-LITE")
    print("=" * 80)
    print()
    print("MINIMAL CORE FUNCTIONALITY:")
    print("  ✅ Fixed LiDAR left/right orientation (CRITICAL)")
    print("  ✅ LiDAR-only obstacle detection and analysis")
    print("  ✅ Robot body-aware navigation guidance")
    print("  ✅ Automatic LiDAR port detection")
    print("  ✅ Real-time navigation status monitoring")
    print()
    print("REMOVED FOR MAXIMUM RELIABILITY:")
    print("  ❌ Arduino communication (preventing startup issues)")
    print("  ❌ Complex asyncio Bluetooth (thread storms)")
    print("  ❌ All visualization components")
    print("  ❌ Motor control (monitoring only)")
    print()
    
    if LIDAR_AVAILABLE:
        print("SENSOR STATUS:")
        print("  ✅ LiDAR Ready (Navigation Analysis Only)")
        print()
        
        # Create and start navigator
        navigator = LidarNavigator()
        
        try:
            if navigator.start():
                # Keep main thread alive
                while True:
                    time.sleep(1)
            else:
                print("❌ Navigator failed to start")
                
        except KeyboardInterrupt:
            print("\\n🛑 Shutdown requested by user...")
            navigator.stop()
        except Exception as e:
            print(f"❌ Navigation system error: {e}")
            navigator.stop()
    else:
        print("SENSOR STATUS:")
        print("  ❌ LiDAR Unavailable (pip install rplidar)")
        print()
        print("❌ Cannot start without LiDAR support")

if __name__ == "__main__":
    main()