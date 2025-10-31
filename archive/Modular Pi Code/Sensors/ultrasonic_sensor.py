#!/usr/bin/env python3
"""
Ultrasonic Sensor Module
========================

Handles ultrasonic distance sensor data processing and filtering.
Integrates with Arduino interface to receive distance measurements.

Key Features:
- Real-time distance monitoring
- Data filtering and validation
- Obstacle detection thresholds
- Thread-safe operation
- Statistics tracking

Part of Mini Rover Development Project - Sensors Module
"""

import time
import threading
from typing import Optional, Callable, Dict, Any
from collections import deque


class UltrasonicSensor:
    """
    Ultrasonic distance sensor interface
    
    Processes distance data from Arduino and provides clean,
    filtered measurements for navigation systems.
    """
    
    def __init__(self, arduino_interface):
        """
        Initialize ultrasonic sensor
        
        Args:
            arduino_interface: Connected Arduino interface instance
        """
        self.arduino = arduino_interface
        
        # Current measurements
        self.distance = 200.0  # Current distance in cm
        self.last_update = time.time()
        self.is_valid = False
        
        # Filtering and validation
        self.max_valid_distance = 400.0  # Maximum believable distance (cm)
        self.min_valid_distance = 2.0    # Minimum believable distance (cm)
        self.filter_size = 5              # Moving average filter size
        self.distance_history = deque(maxlen=self.filter_size)
        
        # Detection thresholds
        self.obstacle_threshold = 30.0    # Obstacle detected below this (cm)
        self.warning_threshold = 60.0     # Warning zone below this (cm)
        self.safe_threshold = 100.0       # Safe distance above this (cm)
        
        # Statistics
        self.total_readings = 0
        self.valid_readings = 0
        self.invalid_readings = 0
        self.obstacle_detections = 0
        
        # Threading
        self._lock = threading.Lock()
        self._callbacks = []  # List of callback functions
        
        print("📡 Ultrasonic Sensor initialized")
        print(f"   Thresholds: Obstacle={self.obstacle_threshold}cm | Warning={self.warning_threshold}cm | Safe={self.safe_threshold}cm")
    
    def add_callback(self, callback: Callable[[float, str], None]):
        """
        Add callback function for distance updates
        
        Args:
            callback: Function(distance, status) called on updates
        """
        with self._lock:
            self._callbacks.append(callback)
    
    def remove_callback(self, callback: Callable[[float, str], None]):
        """Remove callback function"""
        with self._lock:
            if callback in self._callbacks:
                self._callbacks.remove(callback)
    
    def update_from_arduino_data(self, arduino_data: Dict[str, Any]):
        """
        Update sensor data from Arduino readings
        
        Args:
            arduino_data: Dictionary containing Arduino sensor data
        """
        raw_distance = arduino_data.get('distance', None)
        
        with self._lock:
            self.total_readings += 1
            current_time = time.time()
            self.last_update = current_time
            
            # Always update - Pi never stops due to sensor issues
            # Arduino handles safety decisions
            if raw_distance is None:
                # Arduino sent null - use safe default but keep operating
                self.distance = 200.0  # Safe default distance
                self.is_valid = False
                self.invalid_readings += 1
                status = 'UNKNOWN'
            else:
                # Validate the reading
                if self._is_valid_reading(raw_distance):
                    # Add to filter history
                    self.distance_history.append(raw_distance)
                    
                    # Apply moving average filter
                    self.distance = sum(self.distance_history) / len(self.distance_history)
                    self.is_valid = True
                    self.valid_readings += 1
                    
                    # Check for obstacle detection
                    if self.distance < self.obstacle_threshold:
                        self.obstacle_detections += 1
                    
                    status = self.get_status()
                else:
                    # Invalid reading - use safe default but keep operating
                    self.distance = 200.0  # Safe default
                    self.is_valid = False
                    self.invalid_readings += 1
                    status = 'UNKNOWN'
            
            # Always notify callbacks - Pi continues operating regardless
            for callback in self._callbacks:
                try:
                    callback(self.distance, status)
                except Exception as e:
                    print(f"⚠️ Sensor callback error: {e}")
    
    def _is_valid_reading(self, distance: float) -> bool:
        """
        Validate distance reading
        
        Args:
            distance: Raw distance reading in cm
            
        Returns:
            True if reading is valid, False otherwise
        """
        # Check basic range
        if not (self.min_valid_distance <= distance <= self.max_valid_distance):
            return False
        
        # Check for rapid changes (likely noise)
        if len(self.distance_history) > 0:
            last_distance = self.distance_history[-1]
            change = abs(distance - last_distance)
            max_change = 50.0  # Maximum believable change per reading (cm)
            
            if change > max_change:
                # Allow large changes if consistently detected
                if len(self.distance_history) >= 3:
                    recent_avg = sum(list(self.distance_history)[-3:]) / 3
                    if abs(distance - recent_avg) > max_change:
                        return False
                else:
                    return False
        
        return True
    
    def get_distance(self) -> float:
        """
        Get current filtered distance
        
        Returns:
            Current distance in centimeters
        """
        with self._lock:
            return self.distance if self.is_valid else 200.0  # Safe default
    
    def get_status(self) -> str:
        """
        Get current obstacle status
        
        Returns:
            Status string: 'OBSTACLE', 'WARNING', 'SAFE', or 'UNKNOWN'
        """
        if not self.is_valid:
            return 'UNKNOWN'
        
        if self.distance < self.obstacle_threshold:
            return 'OBSTACLE'
        elif self.distance < self.warning_threshold:
            return 'WARNING'
        elif self.distance > self.safe_threshold:
            return 'SAFE'
        else:
            return 'CAUTION'
    
    def is_obstacle_detected(self) -> bool:
        """
        Check if obstacle is detected
        
        Returns:
            True if obstacle detected, False otherwise
        """
        return self.is_valid and self.distance < self.obstacle_threshold
    
    def is_path_clear(self) -> bool:
        """
        Check if path ahead is clear for safe movement
        
        Returns:
            True if path is clear, False otherwise
        """
        return self.is_valid and self.distance > self.safe_threshold
    
    def get_sensor_info(self) -> Dict[str, Any]:
        """
        Get comprehensive sensor information
        
        Returns:
            Dictionary with sensor status and statistics
        """
        with self._lock:
            return {
                'distance': self.distance,
                'status': self.get_status(),
                'is_valid': self.is_valid,
                'last_update': self.last_update,
                'age': time.time() - self.last_update,
                'total_readings': self.total_readings,
                'valid_readings': self.valid_readings,
                'invalid_readings': self.invalid_readings,
                'obstacle_detections': self.obstacle_detections,
                'accuracy': (self.valid_readings / max(1, self.total_readings)) * 100,
                'thresholds': {
                    'obstacle': self.obstacle_threshold,
                    'warning': self.warning_threshold,
                    'safe': self.safe_threshold
                }
            }
    
    def set_thresholds(self, obstacle: Optional[float] = None, 
                      warning: Optional[float] = None, 
                      safe: Optional[float] = None):
        """
        Update detection thresholds
        
        Args:
            obstacle: New obstacle threshold (cm)
            warning: New warning threshold (cm) 
            safe: New safe threshold (cm)
        """
        with self._lock:
            if obstacle is not None:
                self.obstacle_threshold = max(2.0, obstacle)
            if warning is not None:
                self.warning_threshold = max(self.obstacle_threshold + 5.0, warning)
            if safe is not None:
                self.safe_threshold = max(self.warning_threshold + 10.0, safe)
        
        print(f"📡 Thresholds updated: Obstacle={self.obstacle_threshold}cm | Warning={self.warning_threshold}cm | Safe={self.safe_threshold}cm")
    
    def reset_statistics(self):
        """Reset sensor statistics"""
        with self._lock:
            self.total_readings = 0
            self.valid_readings = 0
            self.invalid_readings = 0
            self.obstacle_detections = 0
        
        print("📊 Sensor statistics reset")


# Example usage and testing
if __name__ == "__main__":
    import sys
    import os
    
    # Add Main directory to path for imports
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Main'))
    
    from arduino_interface import ArduinoInterface
    
    def distance_callback(distance, status):
        """Example callback for distance updates"""
        print(f"📏 Distance: {distance:.1f}cm | Status: {status}")
    
    # Create Arduino interface and sensor
    arduino = ArduinoInterface('/dev/ttyUSB0')
    sensor = UltrasonicSensor(arduino)
    sensor.add_callback(distance_callback)
    
    if arduino.connect():
        # Set up Arduino data callback to feed sensor
        arduino.set_data_callback(sensor.update_from_arduino_data)
        
        try:
            print("🧪 Testing ultrasonic sensor...")
            print("Press Ctrl+C to stop")
            
            # Monitor sensor for 30 seconds
            start_time = time.time()
            while time.time() - start_time < 30:
                # Read Arduino data (this will trigger sensor updates)
                arduino.read_data()
                time.sleep(0.1)
            
            # Print final statistics
            info = sensor.get_sensor_info()
            print("\n📊 Final Sensor Statistics:")
            for key, value in info.items():
                if key != 'thresholds':
                    print(f"   {key}: {value}")
            
        except KeyboardInterrupt:
            print("\n🛑 Test interrupted")
        
        finally:
            arduino.disconnect()
    else:
        print("❌ Could not connect to Arduino")
        
        # Simulate some test data
        print("🧪 Running with simulated data...")
        
        # Test with fake Arduino data
        test_data = [
            {'distance': 150.0},  # Safe
            {'distance': 80.0},   # Warning
            {'distance': 25.0},   # Obstacle
            {'distance': 15.0},   # Obstacle
            {'distance': 45.0},   # Warning
            {'distance': 120.0},  # Safe
        ]
        
        for data in test_data:
            sensor.update_from_arduino_data(data)
            info = sensor.get_sensor_info()
            print(f"📏 Distance: {info['distance']:.1f}cm | Status: {info['status']}")
            time.sleep(1)
        
        print("\n📊 Test Statistics:")
        info = sensor.get_sensor_info()
        print(f"   Total readings: {info['total_readings']}")
        print(f"   Valid readings: {info['valid_readings']}")
        print(f"   Obstacle detections: {info['obstacle_detections']}")
        print(f"   Accuracy: {info['accuracy']:.1f}%")