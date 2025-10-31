#!/usr/bin/env python3
"""
Simple Exploration Behavior
===========================

High-level autonomous exploration behavior that combines obstacle avoidance
with exploration goals. Provides a complete autonomous navigation system
that can explore an environment safely.

Key Features:
- Integrates obstacle avoidance with exploration
- Random exploration when path is clear
- Stuck detection and recovery
- Performance monitoring and statistics
- Configurable exploration parameters

Part of Mini Rover Development Project - Autonomous Behaviors Module
"""

import time
import random
import math
from enum import Enum
from typing import Optional, Dict, Any


class ExplorationMode(Enum):
    """Exploration behavior modes"""
    EXPLORING = "EXPLORING"
    AVOIDING = "AVOIDING"
    STUCK_RECOVERY = "STUCK_RECOVERY"
    PAUSED = "PAUSED"


class SimpleExploration:
    """
    Simple autonomous exploration behavior
    
    Combines obstacle avoidance with random exploration to autonomously
    navigate and explore an environment safely.
    """
    
    def __init__(self, obstacle_avoidance_navigator):
        """
        Initialize exploration behavior
        
        Args:
            obstacle_avoidance_navigator: ObstacleAvoidance instance
        """
        self.navigator = obstacle_avoidance_navigator
        self.motors = obstacle_avoidance_navigator.motors
        self.sensor = obstacle_avoidance_navigator.sensor
        
        # Exploration state
        self.mode = ExplorationMode.PAUSED
        self.behavior_start_time = time.time()
        self.last_exploration_turn = time.time()
        
        # Exploration parameters
        self.exploration_turn_interval = 8.0    # Seconds between random turns
        self.exploration_turn_duration = 1.5    # Duration of exploration turns
        self.exploration_turn_chance = 0.7      # Probability of making turn
        
        # Stuck detection
        self.stuck_detection_enabled = True
        self.stuck_threshold = 5.0              # Seconds in same area = stuck
        self.stuck_distance_threshold = 20.0    # cm - if closer, consider stuck
        self.stuck_time_start = None
        self.stuck_recovery_turns = 0
        self.max_stuck_recovery_turns = 3
        
        # Performance tracking
        self.total_exploration_time = 0
        self.total_avoidance_time = 0
        self.total_stuck_recoveries = 0
        self.exploration_turns_made = 0
        self.areas_explored = set()  # Simple grid-based area tracking
        
        # Status tracking
        self.last_position_check = time.time()
        self.position_check_interval = 2.0
        self.last_known_area = None
        
        print("🎯 Simple Exploration Behavior initialized")
        print(f"   Turn interval: {self.exploration_turn_interval}s")
        print(f"   Stuck threshold: {self.stuck_threshold}s")
    
    def start(self):
        """Start autonomous exploration"""
        self.mode = ExplorationMode.EXPLORING
        self.behavior_start_time = time.time()
        self.last_exploration_turn = time.time()
        
        # Start the underlying obstacle avoidance
        self.navigator.start()
        
        print("🚀 Simple exploration started")
    
    def stop(self):
        """Stop autonomous exploration"""
        self.mode = ExplorationMode.PAUSED
        
        # Stop the underlying navigator
        self.navigator.stop()
        
        print("🛑 Simple exploration stopped")
    
    def update(self):
        """
        Main exploration update loop - call this regularly (10-20 Hz)
        
        Returns:
            True if exploration is active, False if stopped
        """
        if self.mode == ExplorationMode.PAUSED:
            return False
        
        current_time = time.time()
        
        # Update the underlying obstacle avoidance navigator
        # Pi always continues - Arduino handles safety
        nav_active = self.navigator.update()
        if not nav_active:
            print("⚠️ Navigator stopped - continuing exploration anyway (Arduino handles safety)")
            # Continue exploration - restart navigator if needed
            self.navigator.start()
        
        # Execute exploration behavior based on current mode
        self._execute_exploration_behavior(current_time)
        
        # Update performance tracking
        self._update_performance_tracking(current_time)
        
        return True
    
    def _execute_exploration_behavior(self, current_time: float):
        """
        Execute exploration behavior logic
        
        Args:
            current_time: Current timestamp
        """
        nav_state = self.navigator.state.value
        distance = self.sensor.get_distance()
        
        # Determine current mode based on navigator state
        if nav_state in ['TURNING', 'BACKING_UP']:
            # Navigator is handling obstacle avoidance
            if self.mode != ExplorationMode.AVOIDING:
                self.mode = ExplorationMode.AVOIDING
                print("🚨 Switched to obstacle avoidance mode")
        
        elif nav_state == 'FORWARD':
            # Check if we should switch back to exploration
            if self.mode == ExplorationMode.AVOIDING:
                self.mode = ExplorationMode.EXPLORING
                print("🎯 Switched back to exploration mode")
            
            # Handle exploration behavior
            if self.mode == ExplorationMode.EXPLORING:
                self._handle_exploration_mode(current_time, distance)
            
            # Check for stuck condition
            if self.stuck_detection_enabled:
                self._check_stuck_condition(current_time, distance)
        
        elif self.mode == ExplorationMode.STUCK_RECOVERY:
            self._handle_stuck_recovery(current_time)
    
    def _handle_exploration_mode(self, current_time: float, distance: float):
        """Handle normal exploration behavior"""
        time_since_last_turn = current_time - self.last_exploration_turn
        
        # Check if it's time for an exploration turn
        if (time_since_last_turn > self.exploration_turn_interval and 
            distance > self.navigator.safe_threshold):
            
            # Randomly decide whether to make an exploration turn
            if random.random() < self.exploration_turn_chance:
                self._make_exploration_turn()
                self.last_exploration_turn = current_time
                self.exploration_turns_made += 1
    
    def _make_exploration_turn(self):
        """Execute a random exploration turn"""
        # Choose random turn direction and duration
        turn_direction = random.choice(['left', 'right'])
        turn_duration = random.uniform(0.5, self.exploration_turn_duration)
        
        print(f"🎯 Exploration turn: {turn_direction} for {turn_duration:.1f}s")
        
        # Execute the turn
        if turn_direction == 'left':
            self.motors.turn_left(self.navigator.turn_speed)
        else:
            self.motors.turn_right(self.navigator.turn_speed)
        
        # Wait for turn to complete
        time.sleep(turn_duration)
        
        # Resume forward movement
        self.motors.forward(self.navigator.forward_speed)
    
    def _check_stuck_condition(self, current_time: float, distance: float):
        """Check if rover appears to be stuck"""
        # Simple stuck detection: too close to obstacle for too long
        if distance < self.stuck_distance_threshold:
            if self.stuck_time_start is None:
                self.stuck_time_start = current_time
            elif current_time - self.stuck_time_start > self.stuck_threshold:
                print(f"🆘 Stuck condition detected! Close to obstacle for {self.stuck_threshold}s")
                self._initiate_stuck_recovery()
        else:
            # Reset stuck timer when we have clear space
            self.stuck_time_start = None
    
    def _initiate_stuck_recovery(self):
        """Initiate stuck recovery procedure"""
        self.mode = ExplorationMode.STUCK_RECOVERY
        self.stuck_recovery_turns = 0
        self.total_stuck_recoveries += 1
        print("🆘 Starting stuck recovery procedure")
        
        # Start with backing up
        self.motors.backward(self.navigator.backup_speed)
        time.sleep(2.0)  # Back up for 2 seconds
    
    def _handle_stuck_recovery(self, current_time: float):
        """Handle stuck recovery behavior"""
        if self.stuck_recovery_turns < self.max_stuck_recovery_turns:
            # Try aggressive turning
            turn_direction = 'left' if self.stuck_recovery_turns % 2 == 0 else 'right'
            print(f"🆘 Stuck recovery turn #{self.stuck_recovery_turns + 1}: {turn_direction}")
            
            if turn_direction == 'left':
                self.motors.turn_left(self.navigator.turn_speed + 20)
            else:
                self.motors.turn_right(self.navigator.turn_speed + 20)
            
            time.sleep(2.0)  # Longer turn for stuck recovery
            self.stuck_recovery_turns += 1
            
            # Check if path is now clear
            distance = self.sensor.get_distance()
            if distance > self.navigator.safe_threshold:
                print("✅ Stuck recovery successful - resuming exploration")
                self.mode = ExplorationMode.EXPLORING
                self.stuck_time_start = None
                return
        
        else:
            # Max recovery attempts reached - pause for human intervention
            print("❌ Stuck recovery failed - pausing for assistance")
            self.stop()
    
    def _update_performance_tracking(self, current_time: float):
        """Update performance statistics"""
        # Track time spent in different modes
        if self.mode == ExplorationMode.EXPLORING:
            self.total_exploration_time += 0.1  # Approximate update interval
        elif self.mode == ExplorationMode.AVOIDING:
            self.total_avoidance_time += 0.1
        
        # Simple area tracking (divide space into 50cm grid squares)
        if current_time - self.last_position_check > self.position_check_interval:
            # This is a very simplified area tracking - in reality you'd use odometry
            distance = self.sensor.get_distance()
            # Create a rough "area" based on time and distance
            area_id = f"{int(current_time / 10)}_{int(distance / 50)}"
            self.areas_explored.add(area_id)
            self.last_position_check = current_time
    
    def get_exploration_status(self) -> Dict[str, Any]:
        """
        Get comprehensive exploration status
        
        Returns:
            Dictionary with exploration information
        """
        current_time = time.time()
        runtime = current_time - self.behavior_start_time
        
        # Get navigator status
        nav_status = self.navigator.get_navigation_status()
        
        return {
            'mode': self.mode.value,
            'runtime': runtime,
            'navigator_state': nav_status['state'],
            'current_distance': nav_status['current_distance'],
            'sensor_status': nav_status['sensor_status'],
            'exploration_turns': self.exploration_turns_made,
            'obstacles_avoided': nav_status['obstacles_avoided'],
            'stuck_recoveries': self.total_stuck_recoveries,
            'areas_explored': len(self.areas_explored),
            'time_exploring': self.total_exploration_time,
            'time_avoiding': self.total_avoidance_time,
            'exploration_efficiency': (self.total_exploration_time / max(1, runtime)) * 100,
            'stuck_detection_enabled': self.stuck_detection_enabled,
            'last_exploration_turn': current_time - self.last_exploration_turn,
            'parameters': {
                'turn_interval': self.exploration_turn_interval,
                'turn_chance': self.exploration_turn_chance,
                'stuck_threshold': self.stuck_threshold
            }
        }
    
    def print_status(self):
        """Print comprehensive exploration status"""
        status = self.get_exploration_status()
        print(f"🎯 Exploration: {status['mode']} | Nav: {status['navigator_state']} | Distance: {status['current_distance']:.1f}cm")
        print(f"   Runtime: {status['runtime']:.1f}s | Turns: {status['exploration_turns']} | Areas: {status['areas_explored']}")
        print(f"   Obstacles: {status['obstacles_avoided']} | Recoveries: {status['stuck_recoveries']} | Efficiency: {status['exploration_efficiency']:.1f}%")
    
    def set_exploration_parameters(self, turn_interval: Optional[float] = None,
                                 turn_chance: Optional[float] = None,
                                 stuck_threshold: Optional[float] = None):
        """
        Update exploration parameters
        
        Args:
            turn_interval: Time between exploration turns (seconds)
            turn_chance: Probability of making exploration turns (0.0-1.0)
            stuck_threshold: Time threshold for stuck detection (seconds)
        """
        if turn_interval is not None:
            self.exploration_turn_interval = max(2.0, turn_interval)
        if turn_chance is not None:
            self.exploration_turn_chance = max(0.0, min(1.0, turn_chance))
        if stuck_threshold is not None:
            self.stuck_threshold = max(2.0, stuck_threshold)
        
        print(f"🎯 Exploration parameters updated:")
        print(f"   Turn interval: {self.exploration_turn_interval}s")
        print(f"   Turn chance: {self.exploration_turn_chance:.1f}")
        print(f"   Stuck threshold: {self.stuck_threshold}s")


# Example usage and testing
if __name__ == "__main__":
    import sys
    import os
    
    # Add other module directories to path
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Main'))
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Sensors'))
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Navigation'))
    
    from arduino_interface import ArduinoInterface
    from motor_controller import MotorController
    from ultrasonic_sensor import UltrasonicSensor
    from obstacle_avoidance import ObstacleAvoidance
    
    # Create complete system
    arduino = ArduinoInterface('/dev/ttyUSB0')
    motors = MotorController(arduino)
    sensor = UltrasonicSensor(arduino)
    navigator = ObstacleAvoidance(motors, sensor)
    explorer = SimpleExploration(navigator)
    
    if arduino.connect():
        # Set up Arduino data callback
        arduino.set_data_callback(sensor.update_from_arduino_data)
        
        try:
            print("🧪 Testing complete exploration system...")
            print("The rover will explore autonomously, avoiding obstacles")
            print("Press Ctrl+C to stop")
            
            # Start exploration
            explorer.start()
            
            # Main exploration loop
            last_status_time = time.time()
            while explorer.mode != ExplorationMode.PAUSED:
                # Read Arduino data
                arduino.read_data()
                
                # Update exploration behavior
                explorer.update()
                
                # Print status every 5 seconds
                current_time = time.time()
                if current_time - last_status_time > 5.0:
                    explorer.print_status()
                    last_status_time = current_time
                
                time.sleep(0.1)  # 10Hz update rate
            
        except KeyboardInterrupt:
            print("\n🛑 Exploration interrupted by user")
            explorer.stop()
            
            # Print final statistics
            print("\n📊 Final Exploration Statistics:")
            status = explorer.get_exploration_status()
            for key, value in status.items():
                if key != 'parameters':
                    print(f"   {key}: {value}")
        
        finally:
            arduino.disconnect()
    else:
        print("❌ Could not connect to Arduino")
        print("🧪 Connect hardware for full autonomous exploration test")