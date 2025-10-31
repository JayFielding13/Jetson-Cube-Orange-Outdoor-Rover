#!/usr/bin/env python3
"""
Rover Smart Startup System
===========================

Enhanced rover system with intelligent USB device management.
Provides fast startup with automatic device discovery and reconnection.

Key Features:
- Quick device validation for fast startup
- Automatic USB device discovery when needed
- Device reconnection when ports change
- Persistent device configuration
- Robust error handling and recovery

Usage:
    python3 rover_smart_startup.py

Part of Mini Rover Development Project - Enhanced Integration
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
from usb_device_manager import USBDeviceManager
from arduino_interface import ArduinoInterface, ArduinoConnectionError
from motor_controller import MotorController
from ultrasonic_sensor import UltrasonicSensor
from obstacle_avoidance import ObstacleAvoidance
from simple_exploration import SimpleExploration


class SmartRoverSystem:
    """
    Enhanced rover system with intelligent USB management
    """
    
    def __init__(self):
        """Initialize the smart rover system"""
        print("=" * 60)
        print("🤖 SMART ROVER STARTUP SYSTEM")
        print("=" * 60)
        print("Enhanced Features:")
        print("  ⚡ Fast startup with device validation")
        print("  🔌 Automatic USB device discovery")
        print("  🔄 Device reconnection on port changes")
        print("  💾 Persistent device configuration")
        print("  🧭 Complete obstacle avoidance system")
        print()
        
        # Initialize components
        self.usb_manager: Optional[USBDeviceManager] = None
        self.arduino: Optional[ArduinoInterface] = None
        self.motors: Optional[MotorController] = None
        self.sensor: Optional[UltrasonicSensor] = None
        self.navigator: Optional[ObstacleAvoidance] = None
        self.explorer: Optional[SimpleExploration] = None
        
        self.running = False
        self.startup_time = 0.0
        
        # Setup signal handlers for clean shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        print(f"\n🛑 Received shutdown signal ({signum})")
        self.stop()
    
    def smart_initialize(self) -> bool:
        """
        Smart initialization with USB device management
        
        Returns:
            True if initialization successful, False otherwise
        """
        startup_start = time.time()
        
        try:
            print("🔧 Smart rover initialization starting...")
            
            # Step 1: Initialize USB device manager
            if config.hardware.use_usb_manager:
                print("🔌 Initializing USB device manager...")
                self.usb_manager = USBDeviceManager(config.hardware.usb_config_file)
                
                # Load existing device configuration
                self.usb_manager.load_device_config()
                
                # Try quick device validation first
                if config.hardware.quick_startup and self.usb_manager.quick_device_check():
                    print("⚡ Quick startup successful!")
                else:
                    print("🔍 Performing full device discovery...")
                    self.usb_manager.discover_devices()
                    self.usb_manager.save_device_config()
                
                # Get Arduino connection details
                arduino_port = self.usb_manager.get_arduino_port()
                arduino_baud = self.usb_manager.get_arduino_baud()
                
                if not arduino_port:
                    print("❌ No Arduino found during device discovery")
                    return False
                
                print(f"🤖 Arduino detected: {arduino_port} @ {arduino_baud} baud")
                
            else:
                # Use traditional static configuration
                arduino_port = config.hardware.arduino_port
                arduino_baud = config.hardware.arduino_baud
                print(f"🔧 Using static Arduino config: {arduino_port}")
            
            # Step 2: Initialize Arduino interface
            print("📡 Connecting to Arduino...")
            self.arduino = ArduinoInterface(
                port=arduino_port,
                baud=arduino_baud,
                timeout=config.hardware.arduino_timeout
            )
            
            # Try connecting with retry logic
            connection_attempts = 3
            for attempt in range(connection_attempts):
                if self.arduino.connect():
                    break
                    
                print(f"⚠️ Connection attempt {attempt + 1} failed")
                
                # If using USB manager, try to reconnect devices
                if self.usb_manager and attempt < connection_attempts - 1:
                    print("🔄 Attempting device reconnection...")
                    if self.usb_manager.reconnect_devices():
                        # Update Arduino port in case it changed
                        new_port = self.usb_manager.get_arduino_port()
                        if new_port and new_port != arduino_port:
                            print(f"🔄 Arduino moved to new port: {new_port}")
                            self.arduino.port = new_port
                            arduino_port = new_port
                
                time.sleep(1.0)
            else:
                print("❌ Failed to connect to Arduino after all attempts")
                return False
            
            # Step 3: Initialize motor controller
            print("🎛️ Setting up motor controller...")
            self.motors = MotorController(self.arduino)
            self.motors.set_speeds(
                cruise=config.motor.cruise_speed,
                turn=config.motor.turn_speed,
                slow=config.motor.slow_speed
            )
            
            # Step 4: Initialize ultrasonic sensor
            print("📏 Setting up ultrasonic sensor...")
            self.sensor = UltrasonicSensor(self.arduino)
            
            # Set up Arduino data callback to feed sensor
            self.arduino.set_data_callback(self.sensor.update_from_arduino_data)
            
            # Step 5: Initialize navigation system
            print("🧭 Setting up obstacle avoidance...")
            self.navigator = ObstacleAvoidance(self.motors, self.sensor)
            
            # Step 6: Initialize exploration behavior
            print("🎯 Setting up exploration behavior...")
            self.explorer = SimpleExploration(self.navigator)
            
            # Calculate startup time
            self.startup_time = time.time() - startup_start
            
            print(f"✅ Smart initialization complete in {self.startup_time:.2f} seconds")
            
            # Print device status
            if self.usb_manager:
                self.usb_manager.print_device_status()
            
            return True
            
        except Exception as e:
            print(f"❌ Smart initialization failed: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def start_exploration(self):
        """Start autonomous exploration with status monitoring"""
        if not all([self.arduino, self.motors, self.sensor, self.navigator, self.explorer]):
            print("❌ Components not initialized")
            return False
        
        print(f"\n🚀 Starting autonomous exploration...")
        print("Smart Features Active:")
        print("  ⚡ Fast startup completed")
        print("  🔌 USB device monitoring")
        print("  🔄 Automatic reconnection")
        print("  🧭 Intelligent navigation")
        print()
        
        # Start the exploration system
        self.explorer.start()
        self.running = True
        
        return True
    
    def run(self):
        """Main execution loop with enhanced error handling"""
        if not self.running:
            print("❌ System not started")
            return
        
        print("🤖 Smart rover exploration active - Press Ctrl+C to stop")
        print("=" * 50)
        
        last_status_time = time.time()
        last_device_check = time.time()
        status_interval = 5.0
        device_check_interval = 30.0  # Check devices every 30 seconds
        
        try:
            while self.running:
                current_time = time.time()
                
                # Read Arduino data (feeds sensor automatically)
                try:
                    self.arduino.read_data()
                except ArduinoConnectionError as e:
                    print(f"❌ Arduino connection lost: {e}")
                    
                    # Try to reconnect using USB manager
                    if self.usb_manager:
                        print("🔄 Attempting automatic reconnection...")
                        if self._attempt_reconnection():
                            print("✅ Reconnection successful - resuming operation")
                            continue
                    
                    print("❌ Reconnection failed - stopping")
                    break
                
                # Update exploration behavior
                if not self.explorer.update():
                    print("🛑 Explorer stopped")
                    break
                
                # Print status periodically
                if current_time - last_status_time > status_interval:
                    self._print_comprehensive_status()
                    last_status_time = current_time
                
                # Periodic device health check
                if self.usb_manager and current_time - last_device_check > device_check_interval:
                    self._check_device_health()
                    last_device_check = current_time
                
                time.sleep(0.1)  # 10Hz main loop
                
        except KeyboardInterrupt:
            print("\n🛑 Exploration interrupted by user")
        
        except Exception as e:
            print(f"\n❌ Unexpected error: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            self.stop()
    
    def _attempt_reconnection(self) -> bool:
        """
        Attempt to reconnect Arduino using USB manager
        
        Returns:
            True if reconnection successful
        """
        try:
            # Disconnect current Arduino
            if self.arduino:
                self.arduino.disconnect()
            
            # Attempt device reconnection
            if self.usb_manager.reconnect_devices():
                new_port = self.usb_manager.get_arduino_port()
                new_baud = self.usb_manager.get_arduino_baud()
                
                if new_port:
                    # Update Arduino interface with new port
                    self.arduino.port = new_port
                    self.arduino.baud = new_baud
                    
                    # Try to reconnect
                    if self.arduino.connect():
                        # Re-setup data callback
                        self.arduino.set_data_callback(self.sensor.update_from_arduino_data)
                        return True
            
            return False
            
        except Exception as e:
            print(f"⚠️ Reconnection attempt failed: {e}")
            return False
    
    def _check_device_health(self):
        """Periodic device health check"""
        if self.usb_manager:
            # Quick validation of known devices
            if not self.usb_manager.quick_device_check():
                print("⚠️ Device health check detected changes")
                # Could trigger reconnection logic here if needed
    
    def stop(self):
        """Stop all rover systems with enhanced cleanup"""
        print("\n🛑 Stopping smart rover system...")
        
        self.running = False
        
        # Stop exploration
        if self.explorer:
            self.explorer.stop()
        
        # Stop motors
        if self.motors:
            self.motors.emergency_stop()
        
        # Print final statistics
        self._print_final_statistics()
        
        # Save device configuration
        if self.usb_manager:
            self.usb_manager.save_device_config()
            print("💾 Device configuration saved")
        
        # Disconnect Arduino
        if self.arduino:
            self.arduino.disconnect()
        
        print("👋 Smart rover system stopped")
    
    def _print_comprehensive_status(self):
        """Print detailed system status with device info"""
        if not all([self.sensor, self.navigator, self.explorer]):
            return
        
        # Get status from all components
        sensor_info = self.sensor.get_sensor_info()
        nav_status = self.navigator.get_navigation_status()
        exp_status = self.explorer.get_exploration_status()
        
        print(f"\n📊 === Smart Rover Status ===")
        print(f"🎯 Exploration: {exp_status['mode']} | Runtime: {exp_status['runtime']:.1f}s")
        print(f"🧭 Navigation: {nav_status['state']} | Distance: {sensor_info['distance']:.1f}cm ({sensor_info['status']})")
        print(f"📈 Performance: {exp_status['exploration_turns']} turns | {nav_status['obstacles_avoided']} obstacles")
        
        # USB device status
        if self.usb_manager:
            device_status = self.usb_manager.get_device_status()
            print(f"🔌 Devices: {device_status['connected_devices']}/{device_status['total_devices']} connected")
        
        print("=" * 40)
    
    def _print_final_statistics(self):
        """Print comprehensive final statistics"""
        print("\n📊 === FINAL SMART ROVER STATISTICS ===")
        print(f"⚡ Startup time: {self.startup_time:.2f} seconds")
        
        if self.sensor:
            sensor_info = self.sensor.get_sensor_info()
            print(f"📏 Sensor Performance:")
            print(f"   Total readings: {sensor_info['total_readings']}")
            print(f"   Accuracy: {sensor_info['accuracy']:.1f}%")
            print(f"   Obstacles detected: {sensor_info['obstacle_detections']}")
        
        if self.navigator:
            nav_status = self.navigator.get_navigation_status()
            print(f"🧭 Navigation Performance:")
            print(f"   Runtime: {nav_status['runtime']:.1f}s")
            print(f"   Obstacles avoided: {nav_status['obstacles_avoided']}")
            print(f"   Turns made: {nav_status['turns_made']}")
        
        if self.explorer:
            exp_status = self.explorer.get_exploration_status()
            print(f"🎯 Exploration Performance:")
            print(f"   Runtime: {exp_status['runtime']:.1f}s")
            print(f"   Exploration turns: {exp_status['exploration_turns']}")
            print(f"   Areas explored: {exp_status['areas_explored']}")
            print(f"   Stuck recoveries: {exp_status['stuck_recoveries']}")
        
        if self.usb_manager:
            device_status = self.usb_manager.get_device_status()
            print(f"🔌 USB Device Management:")
            print(f"   Devices managed: {device_status['total_devices']}")
            print(f"   Final connection status: {device_status['connected_devices']} connected")
        
        print("=" * 50)


def main():
    """Main entry point for smart rover system"""
    # Print configuration
    print("🔧 Smart rover configuration:")
    config.print_config()
    print()
    
    # Validate configuration
    if not config.validate_config():
        print("❌ Configuration validation failed")
        return 1
    
    # Create and run smart rover system
    rover = SmartRoverSystem()
    
    try:
        # Smart initialization with USB management
        if not rover.smart_initialize():
            print("❌ Failed to initialize smart rover system")
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