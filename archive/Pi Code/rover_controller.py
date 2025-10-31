#!/usr/bin/env python3
"""
Rover Main Controller
Handles serial communication with Arduino and motor control
"""

import serial
import json
import time
import threading
import logging
from typing import Dict, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import RPi.GPIO as GPIO

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

@dataclass
class MotorCommand:
    left_speed: int = 0     # -100 to 100 (percentage)
    right_speed: int = 0    # -100 to 100 (percentage)

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

class RoverController:
    """Main rover controller class"""
    
    def __init__(self):
        # Initialize components
        self.serial_comm = SerialCommunicator()
        self.motor_controller = MotorController(
            left_pins=(18, 19),   # GPIO pins for left motor (IN1, IN2)
            right_pins=(20, 21)   # GPIO pins for right motor (IN3, IN4)
        )
        
        # State variables
        self.current_mode = RoverMode.FAILSAFE
        self.rc_data = RCData()
        self.running = False
        self.control_thread = None
        
        # RC channel thresholds for mode switching
        self.mode_switch_threshold = 500  # CH3 threshold for mode switch
        
        logger.info("Rover controller initialized")
    
    def start(self):
        """Start the rover controller"""
        if not self.serial_comm.connect():
            logger.error("Failed to connect to Arduino, cannot start rover")
            return False
        
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        
        logger.info("Rover controller started")
        return True
    
    def stop(self):
        """Stop the rover controller"""
        self.running = False
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        
        self.motor_controller.stop_motors()
        self.serial_comm.disconnect()
        logger.info("Rover controller stopped")
    
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
        """Handle autonomous navigation (placeholder)"""
        # TODO: Implement autonomous navigation logic
        # For now, just stop motors in autonomous mode
        self.motor_controller.stop_motors()
    
    def _handle_failsafe_mode(self):
        """Handle failsafe - stop all motors"""
        self.motor_controller.stop_motors()
    
    def cleanup(self):
        """Clean up resources"""
        self.stop()
        self.motor_controller.cleanup()

def main():
    """Main function"""
    rover = RoverController()
    
    try:
        if rover.start():
            logger.info("Rover started successfully. Press Ctrl+C to stop.")
            while True:
                time.sleep(1)
                # Print status every second
                logger.info(f"Mode: {rover.current_mode.value}, "
                           f"RC Valid: {rover.rc_data.valid}, "
                           f"CH1: {rover.rc_data.ch1}, "
                           f"CH2: {rover.rc_data.ch2}, "
                           f"CH3: {rover.rc_data.ch3}")
        else:
            logger.error("Failed to start rover")
    
    except KeyboardInterrupt:
        logger.info("Shutdown requested by user")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        rover.cleanup()

if __name__ == "__main__":
    main()