#!/usr/bin/env python3
"""
Obstacle Avoidance Navigation
============================

Basic obstacle avoidance navigation system using ultrasonic sensor.
Implements simple reactive navigation: move forward until obstacle detected,
then turn until path is clear.

Key Features:
- Reactive obstacle avoidance
- Turn direction selection (alternating)
- State machine implementation
- Configurable timing and thresholds
- Safe movement validation

Part of Mini Rover Development Project - Navigation Module
"""

import time
import random
from enum import Enum
from typing import Optional, Dict, Any


class NavigationState(Enum):
    """Navigation states for obstacle avoidance"""
    FORWARD = "FORWARD"
    TURNING = "TURNING"
    BACKING_UP = "BACKING_UP"
    STOPPED = "STOPPED"


class ObstacleAvoidance:
    """
    Simple obstacle avoidance navigator
    
    Uses ultrasonic sensor to detect obstacles and motor controller
    to navigate around them using a basic reactive approach.
    """
    
    def __init__(self, motor_controller, ultrasonic_sensor):
        """
        Initialize obstacle avoidance navigator
        
        Args:
            motor_controller: MotorController instance
            ultrasonic_sensor: UltrasonicSensor instance
        """
        self.motors = motor_controller
        self.sensor = ultrasonic_sensor
        
        # Navigation state
        self.state = NavigationState.STOPPED
        self.state_start_time = time.time()
        self.last_turn_direction = None
        
        # Navigation parameters
        self.forward_speed = 80        # Speed for forward movement
        self.turn_speed = 70           # Speed for turning
        self.backup_speed = 60         # Speed for backing up
        
        # Timing parameters
        self.min_turn_duration = 0.8   # Minimum turn time (seconds)
        self.max_turn_duration = 2.0   # Maximum turn time (seconds)
        self.backup_duration = 1.0     # Backup duration (seconds)
        self.forward_check_interval = 0.2  # How often to check while moving forward
        
        # Distance thresholds (use sensor's thresholds as defaults)
        self.obstacle_threshold = 30.0    # Stop and turn below this distance
        self.safe_threshold = 80.0        # Safe to move forward above this distance
        self.clear_path_threshold = 100.0 # Path is definitely clear above this
        
        # Navigation statistics
        self.total_obstacles_avoided = 0
        self.total_turns_made = 0
        self.total_distance_traveled = 0  # Estimated
        self.navigation_start_time = time.time()
        
        # Safety settings
        self.enabled = False
        self.last_sensor_update = time.time()
        self.sensor_timeout = 3.0  # Stop if no sensor data for this long
        
        print("🧭 Obstacle Avoidance Navigator initialized")
        print(f"   Speeds: Forward={self.forward_speed} | Turn={self.turn_speed} | Backup={self.backup_speed}")
        print(f"   Thresholds: Obstacle={self.obstacle_threshold}cm | Safe={self.safe_threshold}cm")
    
    def start(self):
        """Start autonomous navigation"""
        self.enabled = True
        self.state = NavigationState.FORWARD
        self.state_start_time = time.time()
        self.navigation_start_time = time.time()
        print("🚀 Obstacle avoidance navigation started")
    
    def stop(self):
        """Stop autonomous navigation"""
        self.enabled = False
        self.state = NavigationState.STOPPED
        self.motors.stop()
        print("🛑 Obstacle avoidance navigation stopped")
    
    def update(self):
        """
        Main navigation update loop - call this regularly (10-20 Hz)
        
        Returns:
            True if navigation is active, False if stopped
        """
        if not self.enabled:
            return False
        
        current_time = time.time()
        
        # Get current sensor readings - Pi always continues operating
        # Arduino handles all safety decisions
        distance = self.sensor.get_distance()
        sensor_valid = self.sensor.is_valid
        
        if not sensor_valid:
            # Use safe default behavior when sensor is invalid
            # Continue forward at safe speed - Arduino will stop if needed
            distance = 200.0  # Safe default distance
            print(f"⚠️ Using default distance ({distance}cm) - Arduino handles safety")
        
        # Update last sensor time
        self.last_sensor_update = current_time
        
        # Execute state machine
        self._execute_state_machine(distance, current_time)
        
        return True
    
    def _execute_state_machine(self, distance: float, current_time: float):
        """
        Execute the navigation state machine
        
        Args:
            distance: Current ultrasonic distance reading
            current_time: Current timestamp
        """
        time_in_state = current_time - self.state_start_time
        
        if self.state == NavigationState.FORWARD:
            self._handle_forward_state(distance, time_in_state)
            
        elif self.state == NavigationState.TURNING:
            self._handle_turning_state(distance, time_in_state)
            
        elif self.state == NavigationState.BACKING_UP:
            self._handle_backup_state(distance, time_in_state)
            
        elif self.state == NavigationState.STOPPED:
            self.motors.stop()
    
    def _handle_forward_state(self, distance: float, time_in_state: float):
        """Handle forward movement state"""
        if distance < self.obstacle_threshold:
            # Obstacle detected - need to avoid
            print(f"🚨 Obstacle detected at {distance:.1f}cm - avoiding")
            self._transition_to_turning()
            self.total_obstacles_avoided += 1
        else:
            # Path is clear - continue forward
            self.motors.forward(self.forward_speed)
            
            # Estimate distance traveled (very rough)
            if time_in_state > 0:
                estimated_speed = self.forward_speed * 0.01  # cm/s approximation
                self.total_distance_traveled += estimated_speed * self.forward_check_interval
    
    def _handle_turning_state(self, distance: float, time_in_state: float):
        """Handle turning to avoid obstacle state"""
        # Check if we've turned long enough and path is clear
        if (time_in_state > self.min_turn_duration and 
            distance > self.safe_threshold):
            # Path looks clear - go forward
            print(f"✅ Path clear at {distance:.1f}cm - resuming forward")
            self._transition_to_forward()
        elif time_in_state > self.max_turn_duration:
            # Turned long enough - try forward even if not ideal
            print(f"⏰ Max turn time reached - trying forward")
            self._transition_to_forward()
        else:
            # Continue turning
            turn_direction = self._get_turn_direction()
            if turn_direction == 'left':
                self.motors.turn_left(self.turn_speed)
            else:
                self.motors.turn_right(self.turn_speed)
    
    def _handle_backup_state(self, distance: float, time_in_state: float):
        """Handle backing up state"""
        if time_in_state > self.backup_duration:
            # Done backing up - start turning
            print("🔄 Backup complete - starting turn")
            self._transition_to_turning()
        else:
            # Continue backing up
            self.motors.backward(self.backup_speed)
    
    def _transition_to_forward(self):
        """Transition to forward movement state"""
        self.state = NavigationState.FORWARD
        self.state_start_time = time.time()
    
    def _transition_to_turning(self):
        """Transition to turning state"""
        self.state = NavigationState.TURNING
        self.state_start_time = time.time()
        self.total_turns_made += 1
    
    def _transition_to_backup(self):
        """Transition to backing up state"""
        self.state = NavigationState.BACKING_UP
        self.state_start_time = time.time()
    
    def _get_turn_direction(self) -> str:
        """
        Determine which direction to turn
        
        Returns:
            'left' or 'right'
        """
        # Simple strategy: alternate turn directions
        if self.last_turn_direction == 'left':
            self.last_turn_direction = 'right'
            return 'right'
        else:
            self.last_turn_direction = 'left'
            return 'left'
    
    def set_speeds(self, forward: Optional[int] = None, 
                   turn: Optional[int] = None, 
                   backup: Optional[int] = None):
        """
        Update navigation speeds
        
        Args:
            forward: Forward movement speed
            turn: Turning speed
            backup: Backup speed
        """
        if forward is not None:
            self.forward_speed = max(30, min(150, forward))
        if turn is not None:
            self.turn_speed = max(30, min(120, turn))
        if backup is not None:
            self.backup_speed = max(30, min(100, backup))
        
        print(f"🧭 Navigation speeds updated: Forward={self.forward_speed}, Turn={self.turn_speed}, Backup={self.backup_speed}")
    
    def set_thresholds(self, obstacle: Optional[float] = None, 
                      safe: Optional[float] = None):
        """
        Update navigation thresholds
        
        Args:
            obstacle: Distance below which obstacle avoidance triggers
            safe: Distance above which path is considered safe
        """
        if obstacle is not None:
            self.obstacle_threshold = max(10.0, obstacle)
        if safe is not None:
            self.safe_threshold = max(self.obstacle_threshold + 20.0, safe)
        
        print(f"🧭 Navigation thresholds updated: Obstacle={self.obstacle_threshold}cm, Safe={self.safe_threshold}cm")
    
    def get_navigation_status(self) -> Dict[str, Any]:
        """
        Get current navigation status and statistics
        
        Returns:
            Dictionary with navigation information
        """
        current_time = time.time()
        runtime = current_time - self.navigation_start_time
        
        return {
            'enabled': self.enabled,
            'state': self.state.value,
            'time_in_state': current_time - self.state_start_time,
            'current_distance': self.sensor.get_distance(),
            'sensor_status': self.sensor.get_status(),
            'last_turn_direction': self.last_turn_direction,
            'runtime': runtime,
            'obstacles_avoided': self.total_obstacles_avoided,
            'turns_made': self.total_turns_made,
            'estimated_distance': self.total_distance_traveled,
            'speeds': {
                'forward': self.forward_speed,
                'turn': self.turn_speed,
                'backup': self.backup_speed
            },
            'thresholds': {
                'obstacle': self.obstacle_threshold,
                'safe': self.safe_threshold
            }
        }
    
    def print_status(self):
        """Print current navigation status"""
        status = self.get_navigation_status()
        print(f"🧭 Navigation: {status['state']} | Distance: {status['current_distance']:.1f}cm | Sensor: {status['sensor_status']}")
        print(f"   Runtime: {status['runtime']:.1f}s | Obstacles: {status['obstacles_avoided']} | Turns: {status['turns_made']}")


# Example usage and testing
if __name__ == "__main__":
    import sys
    import os
    
    # Add other module directories to path
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Main'))
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Sensors'))
    
    from arduino_interface import ArduinoInterface
    from motor_controller import MotorController
    from ultrasonic_sensor import UltrasonicSensor
    
    # Create components
    arduino = ArduinoInterface('/dev/ttyUSB0')
    motors = MotorController(arduino)
    sensor = UltrasonicSensor(arduino)
    navigator = ObstacleAvoidance(motors, sensor)
    
    if arduino.connect():
        # Set up Arduino data callback
        arduino.set_data_callback(sensor.update_from_arduino_data)
        
        try:
            print("🧪 Testing obstacle avoidance navigation...")
            print("Press Ctrl+C to stop")
            
            # Start navigation
            navigator.start()
            
            # Main navigation loop
            while navigator.enabled:
                # Read Arduino data
                arduino.read_data()
                
                # Update navigation
                navigator.update()
                
                # Print status every 3 seconds
                if time.time() % 3 < 0.5:
                    navigator.print_status()
                
                time.sleep(0.1)  # 10Hz update rate
            
        except KeyboardInterrupt:
            print("\n🛑 Test interrupted")
            navigator.stop()
        
        finally:
            arduino.disconnect()
    else:
        print("❌ Could not connect to Arduino")
        print("🧪 Run with real hardware for full test")