#!/usr/bin/env python3
"""
Enhanced Rover Controller with Bluetooth Navigation
Integrates RC control with Bluetooth RSSI-based autonomous navigation
"""

import serial
import json
import time
import threading
import logging
import asyncio
from typing import Dict, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import RPi.GPIO as GPIO
from bluetooth_navigator import BluetoothNavigator, NavigationCommand, NavigationState

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class RoverMode(Enum):
    MANUAL = "manual"
    AUTONOMOUS = "autonomous"
    FAILSAFE = "failsafe"

@dataclass
class RCData:
    ch1: int = 0        # Forward/Reverse (-1000 to 1000)
    ch2: int = 0        # Steering (-1000 to 1000)
    ch3: int = 0        # Mode switch (-1000 to 1000)
    valid: bool = False
    timestamp: int = 0

class RGBLEDController:
    """Controls RGB LED status indicators"""
    
    def __init__(self, red_pin: int, green_pin: int, blue_pin: int, pwm_freq: int = 1000):
        self.red_pin = red_pin
        self.green_pin = green_pin
        self.blue_pin = blue_pin
        
        # Setup GPIO pins
        GPIO.setup([self.red_pin, self.green_pin, self.blue_pin], GPIO.OUT)
        
        # Create PWM objects for smooth color control
        self.red_pwm = GPIO.PWM(self.red_pin, pwm_freq)
        self.green_pwm = GPIO.PWM(self.green_pin, pwm_freq)
        self.blue_pwm = GPIO.PWM(self.blue_pin, pwm_freq)
        
        # Start PWM with 0% duty cycle (LEDs off)
        self.red_pwm.start(0)
        self.green_pwm.start(0)
        self.blue_pwm.start(0)
        
        # Blinking control
        self.blink_state = False
        self.last_blink_time = 0
        self.blink_interval = 0.5  # seconds
        
        logger.info("RGB LED controller initialized")
    
    def set_color(self, red: int, green: int, blue: int):
        """Set RGB color (0-100 for each channel)"""
        red = max(0, min(100, red))
        green = max(0, min(100, green))
        blue = max(0, min(100, blue))
        
        self.red_pwm.ChangeDutyCycle(red)
        self.green_pwm.ChangeDutyCycle(green)
        self.blue_pwm.ChangeDutyCycle(blue)
    
    def set_mode_color(self, mode: RoverMode, nav_state: NavigationState = None, signal_strength: int = None):
        """Set LED color based on rover mode and navigation state"""
        if mode == RoverMode.FAILSAFE:
            # Solid red for failsafe
            self.set_color(100, 0, 0)
        
        elif mode == RoverMode.MANUAL:
            # Solid green for manual control
            self.set_color(0, 100, 0)
        
        elif mode == RoverMode.AUTONOMOUS:
            if nav_state == NavigationState.SEARCHING:
                # Blinking yellow for searching
                self._blink_color(100, 100, 0)
            
            elif nav_state == NavigationState.SIGNAL_LOST:
                # Blinking red for signal lost
                self._blink_color(100, 0, 0)
            
            elif nav_state in [NavigationState.TRACKING, NavigationState.APPROACHING, NavigationState.MAINTAINING]:
                # Blue intensity based on signal strength
                if signal_strength and signal_strength > -70:
                    # Strong signal - bright blue
                    self.set_color(0, 0, 100)
                elif signal_strength and signal_strength > -80:
                    # Medium signal - medium blue
                    self.set_color(0, 0, 60)
                else:
                    # Weak signal - dim blue with purple tint
                    self.set_color(30, 0, 70)
            
            else:
                # Default autonomous - solid blue
                self.set_color(0, 0, 100)
    
    def _blink_color(self, red: int, green: int, blue: int):
        """Blink specified color"""
        current_time = time.time()
        if current_time - self.last_blink_time > self.blink_interval:
            self.blink_state = not self.blink_state
            self.last_blink_time = current_time
        
        if self.blink_state:
            self.set_color(red, green, blue)
        else:
            self.set_color(0, 0, 0)
    
    def test_sequence(self):
        """Run LED test sequence"""
        logger.info("Running LED test sequence")
        
        # Red
        self.set_color(100, 0, 0)
        time.sleep(0.5)
        
        # Green
        self.set_color(0, 100, 0)
        time.sleep(0.5)
        
        # Blue
        self.set_color(0, 0, 100)
        time.sleep(0.5)
        
        # Yellow
        self.set_color(100, 100, 0)
        time.sleep(0.5)
        
        # Purple
        self.set_color(100, 0, 100)
        time.sleep(0.5)
        
        # Cyan
        self.set_color(0, 100, 100)
        time.sleep(0.5)
        
        # White
        self.set_color(100, 100, 100)
        time.sleep(0.5)
        
        # Off
        self.set_color(0, 0, 0)
        
        logger.info("LED test sequence complete")
    
    def cleanup(self):
        """Clean up LED resources"""
        self.set_color(0, 0, 0)  # Turn off LEDs
        self.red_pwm.stop()
        self.green_pwm.stop()
        self.blue_pwm.stop()

class MotorController:
    """Controls DROK motor driver via GPIO PWM"""
    
    def __init__(self, left_pins: Tuple[int, int], right_pins: Tuple[int, int], pwm_freq: int = 1000):
        # Motor driver pins (IN1, IN2 for left motor, IN3, IN4 for right motor)
        self.left_forward_pin, self.left_reverse_pin = left_pins
        self.right_forward_pin, self.right_reverse_pin = right_pins
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.left_forward_pin, self.left_reverse_pin, 
                   self.right_forward_pin, self.right_reverse_pin], GPIO.OUT)
        
        # Create PWM objects
        self.left_forward_pwm = GPIO.PWM(self.left_forward_pin, pwm_freq)
        self.left_reverse_pwm = GPIO.PWM(self.left_reverse_pin, pwm_freq)
        self.right_forward_pwm = GPIO.PWM(self.right_forward_pin, pwm_freq)
        self.right_reverse_pwm = GPIO.PWM(self.right_reverse_pin, pwm_freq)
        
        # Start PWM with 0% duty cycle
        self.left_forward_pwm.start(0)
        self.left_reverse_pwm.start(0)
        self.right_forward_pwm.start(0)
        self.right_reverse_pwm.start(0)
        
        logger.info("Motor controller initialized")
    
    def set_motor_speeds(self, left_speed: int, right_speed: int):
        """Set motor speeds (-100 to 100 percentage)"""
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        
        # Control left motor
        if left_speed >= 0:
            self.left_forward_pwm.ChangeDutyCycle(abs(left_speed))
            self.left_reverse_pwm.ChangeDutyCycle(0)
        else:
            self.left_forward_pwm.ChangeDutyCycle(0)
            self.left_reverse_pwm.ChangeDutyCycle(abs(left_speed))
        
        # Control right motor
        if right_speed >= 0:
            self.right_forward_pwm.ChangeDutyCycle(abs(right_speed))
            self.right_reverse_pwm.ChangeDutyCycle(0)
        else:
            self.right_forward_pwm.ChangeDutyCycle(0)
            self.right_reverse_pwm.ChangeDutyCycle(abs(right_speed))
    
    def stop_motors(self):
        """Stop both motors"""
        self.set_motor_speeds(0, 0)
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop_motors()
        self.left_forward_pwm.stop()
        self.left_reverse_pwm.stop()
        self.right_forward_pwm.stop()
        self.right_reverse_pwm.stop()
        GPIO.cleanup()

class SerialCommunicator:
    """Handles serial communication with Arduino"""
    
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.is_connected = False
        self.last_data_time = 0
        self.connection_timeout = 1.0  # seconds
        
    def connect(self) -> bool:
        """Connect to Arduino via serial"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                write_timeout=0.1
            )
            self.is_connected = True
            logger.info(f"Connected to Arduino on {self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Arduino: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Arduino"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.is_connected = False
        logger.info("Disconnected from Arduino")
    
    def read_rc_data(self) -> Optional[RCData]:
        """Read and parse RC data from Arduino"""
        if not self.is_connected or not self.serial_conn:
            return None
        
        try:
            if self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('utf-8').strip()
                if line:
                    data = json.loads(line)
                    rc_data = RCData(
                        ch1=data.get('ch1', 0),
                        ch2=data.get('ch2', 0),
                        ch3=data.get('ch3', 0),
                        valid=data.get('valid', False),
                        timestamp=data.get('timestamp', 0)
                    )
                    self.last_data_time = time.time()
                    return rc_data
        except (json.JSONDecodeError, UnicodeDecodeError, KeyError) as e:
            logger.warning(f"Failed to parse Arduino data: {e}")
        except Exception as e:
            logger.error(f"Serial communication error: {e}")
            self.is_connected = False
        
        return None
    
    def is_data_fresh(self) -> bool:
        """Check if we're receiving fresh data from Arduino"""
        return (time.time() - self.last_data_time) < self.connection_timeout

class EnhancedRoverController:
    """Enhanced rover controller with Bluetooth navigation"""
    
    def __init__(self, beacon_mac: str = None, target_distance: float = 2.5):
        # Initialize components
        self.serial_comm = SerialCommunicator()
        self.motor_controller = MotorController(
            left_pins=(18, 19),   # GPIO pins for left motor (IN1, IN2)
            right_pins=(20, 21)   # GPIO pins for right motor (IN3, IN4)
        )
        self.bluetooth_navigator = BluetoothNavigator(
            target_mac=beacon_mac,
            target_distance=target_distance
        )
        self.led_controller = RGBLEDController(
            red_pin=22,    # GPIO 22 for Red LED
            green_pin=23,  # GPIO 23 for Green LED
            blue_pin=24    # GPIO 24 for Blue LED
        )
        
        # State variables
        self.current_mode = RoverMode.FAILSAFE
        self.rc_data = RCData()
        self.running = False
        self.control_thread = None
        self.bluetooth_thread = None
        
        # Navigation state
        self.autonomous_command = NavigationCommand()
        self.bluetooth_active = False
        
        # RC channel thresholds for mode switching
        self.mode_switch_threshold = 500  # CH3 threshold for mode switch
        
        # Async event loop for Bluetooth
        self.bluetooth_loop = None
        
        logger.info("Enhanced rover controller initialized")
        
        # Run LED test sequence on startup
        self.led_controller.test_sequence()
    
    def start(self):
        """Start the enhanced rover controller"""
        if not self.serial_comm.connect():
            logger.error("Failed to connect to Arduino, cannot start rover")
            return False
        
        self.running = True
        
        # Start control thread
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        
        # Start Bluetooth navigation thread
        self.bluetooth_thread = threading.Thread(target=self._bluetooth_loop, daemon=True)
        self.bluetooth_thread.start()
        
        logger.info("Enhanced rover controller started")
        return True
    
    def stop(self):
        """Stop the enhanced rover controller"""
        self.running = False
        
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        
        if self.bluetooth_thread and self.bluetooth_thread.is_alive():
            self.bluetooth_thread.join(timeout=2.0)
        
        if self.bluetooth_loop:
            self.bluetooth_loop.stop()
        
        self.motor_controller.stop_motors()
        self.serial_comm.disconnect()
        logger.info("Enhanced rover controller stopped")
    
    def _bluetooth_loop(self):
        """Bluetooth navigation loop running in separate thread"""
        try:
            # Create new event loop for this thread
            self.bluetooth_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.bluetooth_loop)
            
            while self.running:
                try:
                    # Run navigation cycle
                    command = self.bluetooth_loop.run_until_complete(
                        self.bluetooth_navigator.run_navigation_cycle()
                    )
                    
                    # Update autonomous command
                    self.autonomous_command = command
                    self.bluetooth_active = True
                    
                    # Small delay
                    time.sleep(0.1)  # 10Hz navigation updates
                    
                except Exception as e:
                    logger.error(f"Bluetooth loop error: {e}")
                    self.bluetooth_active = False
                    time.sleep(1)
                    
        except Exception as e:
            logger.error(f"Bluetooth thread error: {e}")
            self.bluetooth_active = False
    
    def _control_loop(self):
        """Main control loop running in separate thread"""
        while self.running:
            try:
                # Read RC data from Arduino
                rc_data = self.serial_comm.read_rc_data()
                if rc_data:
                    self.rc_data = rc_data
                
                # Determine current mode
                self._update_mode()
                
                # Update LED status
                self._update_led_status()
                
                # Execute control based on mode
                if self.current_mode == RoverMode.MANUAL:
                    self._handle_manual_mode()
                elif self.current_mode == RoverMode.AUTONOMOUS:
                    self._handle_autonomous_mode()
                else:  # FAILSAFE
                    self._handle_failsafe_mode()
                
                time.sleep(0.01)  # 100Hz control loop
                
            except Exception as e:
                logger.error(f"Control loop error: {e}")
                self._handle_failsafe_mode()
    
    def _update_mode(self):
        """Update rover mode based on RC data and system state"""
        # Check for failsafe conditions
        if not self.rc_data.valid or not self.serial_comm.is_data_fresh():
            self.current_mode = RoverMode.FAILSAFE
            return
        
        # Mode switching based on CH3
        if self.rc_data.ch3 > self.mode_switch_threshold:
            self.current_mode = RoverMode.MANUAL
        else:
            self.current_mode = RoverMode.AUTONOMOUS
    
    def _handle_manual_mode(self):
        """Handle manual RC control"""
        # Convert RC input to motor commands
        throttle = self.rc_data.ch1 / 10  # Scale to -100 to 100
        steering = self.rc_data.ch2 / 10  # Scale to -100 to 100
        
        # Differential steering calculation
        left_speed = throttle + steering
        right_speed = throttle - steering
        
        # Normalize to prevent saturation
        max_speed = max(abs(left_speed), abs(right_speed))
        if max_speed > 100:
            left_speed = (left_speed / max_speed) * 100
            right_speed = (right_speed / max_speed) * 100
        
        self.motor_controller.set_motor_speeds(int(left_speed), int(right_speed))
    
    def _handle_autonomous_mode(self):
        """Handle autonomous navigation using Bluetooth"""
        if not self.bluetooth_active:
            # Bluetooth navigation not active, stop motors
            self.motor_controller.stop_motors()
            return
        
        # Use Bluetooth navigation command
        command = self.autonomous_command
        
        # Convert navigation command to motor speeds
        forward_speed = command.forward_speed
        turn_rate = command.turn_rate
        
        # Calculate differential steering
        left_speed = forward_speed - turn_rate
        right_speed = forward_speed + turn_rate
        
        # Normalize speeds
        max_speed = max(abs(left_speed), abs(right_speed))
        if max_speed > 100:
            left_speed = (left_speed / max_speed) * 100
            right_speed = (right_speed / max_speed) * 100
        
        self.motor_controller.set_motor_speeds(int(left_speed), int(right_speed))
        
        # Log navigation status
        if hasattr(self, '_last_log_time'):
            if time.time() - self._last_log_time > 2:  # Log every 2 seconds
                self._log_navigation_status()
        else:
            self._last_log_time = time.time()
    
    def _handle_failsafe_mode(self):
        """Handle failsafe - stop all motors"""
        self.motor_controller.stop_motors()
    
    def _update_led_status(self):
        """Update LED status based on current rover state"""
        nav_status = self.bluetooth_navigator.get_navigation_status()
        nav_state = self.bluetooth_navigator.navigation_state
        signal_strength = nav_status.get('signal_strength')
        
        self.led_controller.set_mode_color(
            self.current_mode,
            nav_state,
            signal_strength
        )
    
    def _log_navigation_status(self):
        """Log current navigation status"""
        status = self.bluetooth_navigator.get_navigation_status()
        self._last_log_time = time.time()
        
        logger.info(f"Navigation Status - State: {status['state']}, "
                   f"Distance: {status['estimated_distance']:.1f}m, "
                   f"RSSI L/R: {status['left_rssi']}/{status['right_rssi']}, "
                   f"Command: {self.autonomous_command.command_type}")
    
    def get_system_status(self) -> Dict:
        """Get comprehensive system status"""
        nav_status = self.bluetooth_navigator.get_navigation_status()
        
        return {
            "rover_mode": self.current_mode.value,
            "rc_data": {
                "ch1": self.rc_data.ch1,
                "ch2": self.rc_data.ch2,
                "ch3": self.rc_data.ch3,
                "valid": self.rc_data.valid
            },
            "navigation": nav_status,
            "autonomous_command": {
                "forward_speed": self.autonomous_command.forward_speed,
                "turn_rate": self.autonomous_command.turn_rate,
                "command_type": self.autonomous_command.command_type
            },
            "bluetooth_active": self.bluetooth_active,
            "serial_connected": self.serial_comm.is_connected
        }
    
    def cleanup(self):
        """Clean up resources"""
        self.stop()
        self.motor_controller.cleanup()
        self.led_controller.cleanup()

def main():
    """Main function"""
    # Replace with your BlueCharm beacon MAC address
    beacon_mac = "AA:BB:CC:DD:EE:FF"  # Change this to your beacon's MAC
    target_distance = 2.5  # meters
    
    rover = EnhancedRoverController(beacon_mac=beacon_mac, target_distance=target_distance)
    
    try:
        if rover.start():
            logger.info("Enhanced rover started successfully. Press Ctrl+C to stop.")
            
            while True:
                time.sleep(5)
                # Print comprehensive status every 5 seconds
                status = rover.get_system_status()
                
                logger.info(f"System Status:")
                logger.info(f"  Mode: {status['rover_mode']}")
                logger.info(f"  RC Valid: {status['rc_data']['valid']}")
                logger.info(f"  Bluetooth Active: {status['bluetooth_active']}")
                logger.info(f"  Navigation State: {status['navigation']['state']}")
                logger.info(f"  Distance: {status['navigation']['estimated_distance']:.1f}m")
                logger.info(f"  Command: {status['autonomous_command']['command_type']}")
                
        else:
            logger.error("Failed to start enhanced rover")
    
    except KeyboardInterrupt:
        logger.info("Shutdown requested by user")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        rover.cleanup()

if __name__ == "__main__":
    main()