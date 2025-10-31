#!/usr/bin/env python3
"""
Rover Exploration Test
======================

Complete test program that demonstrates the modular rover architecture
with ultrasonic sensor, obstacle avoidance, and autonomous exploration.

This program integrates all modules:
- Main/ - Core Arduino communication and motor control
- Sensors/ - Ultrasonic sensor processing
- Navigation/ - Obstacle avoidance logic
- Autonomous Behaviors/ - Exploration behavior

Usage:
    python3 rover_exploration_test.py

Part of Mini Rover Development Project - Integration Test
"""

import sys
import os
import time
import signal
from typing import Optional

# Add module directories to Python path
current_dir = os.path.dirname(__file__)
sys.path.append(os.path.join(current_dir, 'Main'))
sys.path.append(os.path.join(current_dir, 'Sensors'))
sys.path.append(os.path.join(current_dir, 'Navigation'))
sys.path.append(os.path.join(current_dir, 'Autonomous Behaviors'))

# Import all our modular components
from config import config
from arduino_interface import ArduinoInterface, ArduinoConnectionError
from motor_controller import MotorController
from ultrasonic_sensor import UltrasonicSensor
from obstacle_avoidance import ObstacleAvoidance
from simple_exploration import SimpleExploration


class RoverExplorationSystem:
    """
    Complete rover exploration system integrating all modules
    """
    
    def __init__(self):
        """Initialize the complete rover system"""
        print("=" * 60)
        print("🤖 ROVER EXPLORATION SYSTEM")
        print("=" * 60)
        print("Modular Architecture Test:")
        print("  📡 Ultrasonic Sensor Processing")
        print("  🧭 Obstacle Avoidance Navigation") 
        print("  🎯 Autonomous Exploration Behavior")
        print("  🎛️ Motor Control & Arduino Interface")
        print()
        
        # Initialize all components
        self.arduino: Optional[ArduinoInterface] = None
        self.motors: Optional[MotorController] = None
        self.sensor: Optional[UltrasonicSensor] = None
        self.navigator: Optional[ObstacleAvoidance] = None
        self.explorer: Optional[SimpleExploration] = None
        
        self.running = False
        
        # Setup signal handlers for clean shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        print(f"\n🛑 Received shutdown signal ({signum})")
        self.stop()
    
    def initialize(self) -> bool:
        """
        Initialize all rover components
        
        Returns:
            True if initialization successful, False otherwise
        """
        try:
            print("🔧 Initializing rover components...")
            
            # Initialize Arduino interface
            print("  📡 Setting up Arduino interface...")
            self.arduino = ArduinoInterface(
                port=config.hardware.arduino_port,
                baud=config.hardware.arduino_baud,
                timeout=config.hardware.arduino_timeout
            )
            
            # Connect to Arduino
            if not self.arduino.connect():
                print("❌ Failed to connect to Arduino")
                return False
            
            # Initialize motor controller
            print("  🎛️ Setting up motor controller...")
            self.motors = MotorController(self.arduino)
            self.motors.set_speeds(
                cruise=config.motor.cruise_speed,
                turn=config.motor.turn_speed,
                slow=config.motor.slow_speed
            )
            
            # Initialize ultrasonic sensor
            print("  📏 Setting up ultrasonic sensor...")
            self.sensor = UltrasonicSensor(self.arduino)
            
            # Set up Arduino data callback to feed sensor
            self.arduino.set_data_callback(self.sensor.update_from_arduino_data)
            
            # Initialize obstacle avoidance navigator
            print("  🧭 Setting up obstacle avoidance...")
            self.navigator = ObstacleAvoidance(self.motors, self.sensor)
            
            # Initialize exploration behavior
            print("  🎯 Setting up exploration behavior...")
            self.explorer = SimpleExploration(self.navigator)
            
            print("✅ All components initialized successfully")
            return True
            
        except Exception as e:
            print(f"❌ Initialization failed: {e}")
            return False
    
    def start_exploration(self):
        """Start autonomous exploration"""
        if not all([self.arduino, self.motors, self.sensor, self.navigator, self.explorer]):
            print("❌ Components not initialized")
            return False
        
        print("\n🚀 Starting autonomous exploration...")
        print("The rover will:")
        print("  • Move forward when path is clear")
        print("  • Turn to avoid obstacles")
        print("  • Make random exploration turns")
        print("  • Recover from stuck situations")
        print()
        
        # Start the exploration system
        self.explorer.start()
        self.running = True
        
        return True
    
    def run(self):
        """Main execution loop"""
        if not self.running:
            print("❌ System not started")
            return
        
        print("🤖 Rover exploration active - Press Ctrl+C to stop")
        print("=" * 50)
        
        last_status_time = time.time()
        last_sensor_check = time.time()
        status_interval = 5.0  # Print status every 5 seconds
        
        try:
            while self.running:
                current_time = time.time()
                
                # Read Arduino data (feeds sensor automatically)
                try:
                    self.arduino.read_data()
                except ArduinoConnectionError as e:
                    print(f"❌ Arduino connection lost: {e}")
                    break
                
                # Update exploration behavior
                if not self.explorer.update():
                    print("🛑 Explorer stopped")
                    break
                
                # Print status periodically
                if current_time - last_status_time > status_interval:
                    self._print_comprehensive_status()
                    last_status_time = current_time
                
                # Check sensor health
                if current_time - last_sensor_check > 2.0:
                    if not self.sensor.is_valid:
                        print("⚠️ Sensor data invalid")
                    last_sensor_check = current_time
                
                time.sleep(0.1)  # 10Hz main loop
                
        except KeyboardInterrupt:
            print("\n🛑 Exploration interrupted by user")
        
        except Exception as e:
            print(f"\n❌ Unexpected error: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            self.stop()
    
    def stop(self):
        """Stop all rover systems"""
        print("\n🛑 Stopping rover systems...")
        
        self.running = False
        
        # Stop exploration
        if self.explorer:
            self.explorer.stop()
        
        # Stop motors
        if self.motors:
            self.motors.emergency_stop()
        
        # Print final statistics
        self._print_final_statistics()
        
        # Disconnect Arduino
        if self.arduino:
            self.arduino.disconnect()
        
        print("👋 Rover exploration system stopped")
    
    def _print_comprehensive_status(self):
        """Print detailed system status"""
        if not all([self.sensor, self.navigator, self.explorer]):
            return
        
        # Get status from all components
        sensor_info = self.sensor.get_sensor_info()
        nav_status = self.navigator.get_navigation_status()
        exp_status = self.explorer.get_exploration_status()
        
        print(f"\n📊 === Rover Status ===")
        print(f"🎯 Exploration: {exp_status['mode']} | Runtime: {exp_status['runtime']:.1f}s")
        print(f"🧭 Navigation: {nav_status['state']} | Distance: {sensor_info['distance']:.1f}cm ({sensor_info['status']})")
        print(f"📈 Performance: {exp_status['exploration_turns']} turns | {nav_status['obstacles_avoided']} obstacles | {exp_status['stuck_recoveries']} recoveries")
        print(f"📊 Sensor: {sensor_info['accuracy']:.1f}% accuracy | {sensor_info['total_readings']} readings")
        print("=" * 30)
    
    def _print_final_statistics(self):
        """Print comprehensive final statistics"""
        print("\n📊 === FINAL ROVER STATISTICS ===")
        
        if self.sensor:
            sensor_info = self.sensor.get_sensor_info()
            print(f"📏 Sensor Performance:")
            print(f"   Total readings: {sensor_info['total_readings']}")
            print(f"   Valid readings: {sensor_info['valid_readings']}")
            print(f"   Accuracy: {sensor_info['accuracy']:.1f}%")
            print(f"   Obstacles detected: {sensor_info['obstacle_detections']}")
        
        if self.navigator:
            nav_status = self.navigator.get_navigation_status()
            print(f"🧭 Navigation Performance:")
            print(f"   Runtime: {nav_status['runtime']:.1f}s")
            print(f"   Obstacles avoided: {nav_status['obstacles_avoided']}")
            print(f"   Turns made: {nav_status['turns_made']}")
            print(f"   Estimated distance: {nav_status['estimated_distance']:.1f}cm")
        
        if self.explorer:
            exp_status = self.explorer.get_exploration_status()
            print(f"🎯 Exploration Performance:")
            print(f"   Runtime: {exp_status['runtime']:.1f}s")
            print(f"   Exploration turns: {exp_status['exploration_turns']}")
            print(f"   Areas explored: {exp_status['areas_explored']}")
            print(f"   Stuck recoveries: {exp_status['stuck_recoveries']}")
            print(f"   Exploration efficiency: {exp_status['exploration_efficiency']:.1f}%")
        
        print("=" * 40)


def main():
    """Main entry point for rover exploration test"""
    # Print configuration
    print("🔧 Using configuration:")
    config.print_config()
    print()
    
    # Validate configuration
    if not config.validate_config():
        print("❌ Configuration validation failed")
        return 1
    
    # Create and run rover system
    rover = RoverExplorationSystem()
    
    try:
        # Initialize all components
        if not rover.initialize():
            print("❌ Failed to initialize rover system")
            return 1
        
        # Start exploration
        if not rover.start_exploration():
            print("❌ Failed to start exploration")
            return 1
        
        # Run main loop
        rover.run()
        
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())