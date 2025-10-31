#!/usr/bin/env python3
"""
Rover Main Program
==================

Minimal foundational rover control program that demonstrates the modular architecture.
Provides basic remote control and manual operation capabilities.

Key Features:
- Clean modular architecture
- Arduino communication with health monitoring
- Simple motor control interface
- Configurable settings
- Graceful startup and shutdown
- Error handling and recovery

Usage:
    python3 main.py
    
Controls (when running):
    w/s - Forward/Backward
    a/d - Turn Left/Right  
    q/e - Curve Left/Right
    x   - Stop
    ESC - Emergency Stop & Exit

Part of Mini Rover Development Project - Foundational Module
"""

import time
import sys
import select
import termios
import tty
import threading
from typing import Optional

# Import our modular components
from config import config
from arduino_interface import ArduinoInterface, ArduinoConnectionError
from motor_controller import MotorController


class RoverSystem:
    """
    Main rover system coordinator
    
    Manages the integration of all rover subsystems and provides
    a clean interface for control and monitoring.
    """
    
    def __init__(self):
        """Initialize the rover system"""
        self.running = False
        self.arduino: Optional[ArduinoInterface] = None
        self.motors: Optional[MotorController] = None
        
        # System state
        self.robot_data = {
            'mode': 0,
            'rc_valid': False,
            'emergency_stop': False,
            'ultrasonic_distance': 200.0,
            'last_update': time.time()
        }
        
        # Threading
        self.data_thread: Optional[threading.Thread] = None
        self.status_thread: Optional[threading.Thread] = None
        
        print("🤖 Rover System initializing...")
        config.print_config()
    
    def arduino_data_callback(self, data: dict):
        """Handle incoming data from Arduino"""
        self.robot_data.update({
            'mode': data.get('mode', 0),
            'rc_valid': data.get('valid', False),
            'emergency_stop': data.get('emergency', False),
            'ultrasonic_distance': data.get('distance', 200.0),
            'last_update': time.time()
        })
        
        # Log emergency conditions
        if data.get('emergency', False):
            print("🚨 EMERGENCY STOP from Arduino!")
            self.emergency_stop()
    
    def start(self) -> bool:
        """
        Start the rover system
        
        Returns:
            True if startup successful, False otherwise
        """
        try:
            # Initialize Arduino interface
            self.arduino = ArduinoInterface(
                port=config.hardware.arduino_port,
                baud=config.hardware.arduino_baud,
                timeout=config.hardware.arduino_timeout
            )
            
            # Set up data callback
            self.arduino.set_data_callback(self.arduino_data_callback)
            
            # Connect to Arduino
            if not self.arduino.connect():
                print("❌ Failed to connect to Arduino")
                return False
            
            # Initialize motor controller
            self.motors = MotorController(self.arduino)
            
            # Apply config settings to motor controller
            self.motors.set_speeds(
                cruise=config.motor.cruise_speed,
                turn=config.motor.turn_speed,
                slow=config.motor.slow_speed
            )
            
            # Start background threads
            self.running = True
            self.data_thread = threading.Thread(target=self._data_loop, daemon=True)
            self.status_thread = threading.Thread(target=self._status_loop, daemon=True)
            
            self.data_thread.start()
            self.status_thread.start()
            
            print("✅ Rover system started successfully")
            print("🎮 Ready for manual control")
            return True
            
        except Exception as e:
            print(f"❌ Startup failed: {e}")
            return False
    
    def stop(self):
        """Stop the rover system and clean up"""
        print("🛑 Stopping rover system...")
        self.running = False
        
        # Stop motors
        if self.motors:
            self.motors.emergency_stop()
        
        # Give threads time to finish
        time.sleep(0.5)
        
        # Disconnect Arduino
        if self.arduino:
            self.arduino.disconnect()
        
        print("👋 Rover system stopped")
    
    def emergency_stop(self):
        """Execute emergency stop"""
        if self.motors:
            self.motors.emergency_stop()
        print("🛑 EMERGENCY STOP EXECUTED")
    
    def _data_loop(self):
        """Background thread for Arduino data reading"""
        while self.running:
            try:
                if self.arduino:
                    self.arduino.read_data()
                time.sleep(1.0 / config.system.sensor_read_rate)
                
            except ArduinoConnectionError as e:
                print(f"❌ Arduino connection lost: {e}")
                self.emergency_stop()
                break
                
            except Exception as e:
                print(f"⚠️ Data loop error: {e}")
                time.sleep(1.0)
    
    def _status_loop(self):
        """Background thread for status monitoring"""
        last_status_time = 0
        
        while self.running:
            try:
                current_time = time.time()
                
                # Periodic status updates
                if current_time - last_status_time > config.system.status_update_interval:
                    self._print_status()
                    last_status_time = current_time
                
                # Check Arduino connection health
                if self.arduino and not self.arduino.is_connected():
                    print("⚠️ Arduino connection unhealthy")
                
                time.sleep(1.0)
                
            except Exception as e:
                print(f"⚠️ Status loop error: {e}")
                time.sleep(1.0)
    
    def _print_status(self):
        """Print current system status"""
        if not self.arduino:
            return
        
        conn_status = self.arduino.get_connection_status()
        motor_state = self.motors.get_motor_state() if self.motors else {}
        
        mode_names = ['MANUAL', 'ASSISTED', 'AUTONOMOUS']
        mode = self.robot_data.get('mode', 0)
        mode_name = mode_names[mode] if 0 <= mode < len(mode_names) else 'UNKNOWN'
        
        status = (
            f"🧭 {mode_name} | "
            f"Motors: L={motor_state.get('left_speed', 0)}, R={motor_state.get('right_speed', 0)} | "
            f"Distance: {self.robot_data.get('ultrasonic_distance', 0):.1f}cm | "
            f"Arduino: {'✅' if conn_status['healthy'] else '❌'}"
        )
        
        print(status)
    
    def get_system_status(self) -> dict:
        """Get comprehensive system status"""
        return {
            'running': self.running,
            'arduino_connected': self.arduino.is_connected() if self.arduino else False,
            'robot_data': self.robot_data.copy(),
            'motor_state': self.motors.get_motor_state() if self.motors else {},
            'arduino_status': self.arduino.get_connection_status() if self.arduino else {}
        }


def setup_terminal():
    """Setup terminal for non-blocking keyboard input"""
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    return old_settings

def restore_terminal(old_settings):
    """Restore terminal to original settings"""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def get_key():
    """Get a single keypress (non-blocking)"""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

def manual_control_loop(rover: RoverSystem):
    """
    Manual keyboard control loop
    
    Controls:
        w/s - Forward/Backward
        a/d - Turn Left/Right
        q/e - Curve Left/Right
        x   - Stop
        ESC - Emergency Stop & Exit
    """
    print("\n🎮 === Manual Control Active ===")
    print("Controls:")
    print("  w/s - Forward/Backward")
    print("  a/d - Turn Left/Right")
    print("  q/e - Curve Left/Right")
    print("  x   - Stop")
    print("  ESC - Emergency Stop & Exit")
    print("=" * 40)
    
    # Setup terminal for raw input
    old_settings = setup_terminal()
    
    try:
        while rover.running:
            key = get_key()
            
            if key:
                # Movement controls
                if key == 'w':
                    rover.motors.forward()
                    print("⬆️ Forward")
                    
                elif key == 's':
                    rover.motors.backward()
                    print("⬇️ Backward")
                    
                elif key == 'a':
                    rover.motors.turn_left()
                    print("⬅️ Turn Left")
                    
                elif key == 'd':
                    rover.motors.turn_right()
                    print("➡️ Turn Right")
                    
                elif key == 'q':
                    rover.motors.curve_left()
                    print("↖️ Curve Left")
                    
                elif key == 'e':
                    rover.motors.curve_right()
                    print("↗️ Curve Right")
                    
                elif key == 'x':
                    rover.motors.stop()
                    print("🛑 Stop")
                    
                elif key == '\x1b':  # ESC key
                    print("🛑 Emergency stop - Exiting...")
                    rover.emergency_stop()
                    break
            
            time.sleep(0.1)  # Small delay to prevent CPU spinning
    
    finally:
        restore_terminal(old_settings)


def main():
    """Main entry point"""
    print("=" * 60)
    print("🤖 ROVER FOUNDATIONAL SYSTEM")
    print("=" * 60)
    print("Modular architecture demonstration")
    print("Arduino communication + Motor control + Configuration")
    print()
    
    # Validate configuration
    if not config.validate_config():
        print("❌ Configuration validation failed - exiting")
        return 1
    
    # Create and start rover system
    rover = RoverSystem()
    
    try:
        if not rover.start():
            print("❌ Failed to start rover system")
            return 1
        
        # Run manual control loop
        manual_control_loop(rover)
        
    except KeyboardInterrupt:
        print("\n🛑 Keyboard interrupt received")
    
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        rover.stop()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())