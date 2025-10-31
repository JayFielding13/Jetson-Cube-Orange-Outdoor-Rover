#!/usr/bin/env python3
"""
Basic Rover Controller - RC Control Only
For initial testing and development
"""

import serial
import json
import time
import threading
import logging
from typing import Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import RPi.GPIO as GPIO

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class RoverMode(Enum):
    FAILSAFE = "failsafe"
    MANUAL = "manual"
    AUTONOMOUS = "autonomous"

@dataclass
class RCData:
    ch1: int = 0
    ch2: int = 0  
    ch9: int = 0  # 3-position switch: -900, 0, 900
    valid: bool = False
    timestamp: int = 0

class MotorController:
    def __init__(self, left_pins: Tuple[int, int, int], right_pins: Tuple[int, int, int]):
        # Motor pins: (forward, reverse, enable)
        self.left_forward_pin, self.left_reverse_pin, self.left_enable_pin = left_pins
        self.right_forward_pin, self.right_reverse_pin, self.right_enable_pin = right_pins
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.left_forward_pin, self.left_reverse_pin, self.left_enable_pin,
                   self.right_forward_pin, self.right_reverse_pin, self.right_enable_pin], GPIO.OUT)
        
        # Direction control pins (digital HIGH/LOW)
        GPIO.output(self.left_forward_pin, GPIO.LOW)
        GPIO.output(self.left_reverse_pin, GPIO.LOW)
        GPIO.output(self.right_forward_pin, GPIO.LOW)
        GPIO.output(self.right_reverse_pin, GPIO.LOW)
        
        # Enable pins with PWM for speed control
        self.left_enable_pwm = GPIO.PWM(self.left_enable_pin, 1000)
        self.right_enable_pwm = GPIO.PWM(self.right_enable_pin, 1000)
        
        self.left_enable_pwm.start(0)
        self.right_enable_pwm.start(0)
        
        logger.info("Motor controller initialized with ENA control")
    
    def set_motor_speeds(self, left_speed: int, right_speed: int):
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        
        # Debug output for motor commands
        if left_speed != 0 or right_speed != 0:
            logger.info(f"MOTOR OUTPUT: Left={left_speed}%, Right={right_speed}%")
        
        # Control left motor
        if left_speed > 0:
            # Forward direction
            GPIO.output(self.left_forward_pin, GPIO.HIGH)
            GPIO.output(self.left_reverse_pin, GPIO.LOW)
            self.left_enable_pwm.ChangeDutyCycle(abs(left_speed))
        elif left_speed < 0:
            # Reverse direction
            GPIO.output(self.left_forward_pin, GPIO.LOW)
            GPIO.output(self.left_reverse_pin, GPIO.HIGH)
            self.left_enable_pwm.ChangeDutyCycle(abs(left_speed))
        else:
            # Stop
            GPIO.output(self.left_forward_pin, GPIO.LOW)
            GPIO.output(self.left_reverse_pin, GPIO.LOW)
            self.left_enable_pwm.ChangeDutyCycle(0)
        
        # Control right motor
        if right_speed > 0:
            # Forward direction
            GPIO.output(self.right_forward_pin, GPIO.HIGH)
            GPIO.output(self.right_reverse_pin, GPIO.LOW)
            self.right_enable_pwm.ChangeDutyCycle(abs(right_speed))
        elif right_speed < 0:
            # Reverse direction
            GPIO.output(self.right_forward_pin, GPIO.LOW)
            GPIO.output(self.right_reverse_pin, GPIO.HIGH)
            self.right_enable_pwm.ChangeDutyCycle(abs(right_speed))
        else:
            # Stop
            GPIO.output(self.right_forward_pin, GPIO.LOW)
            GPIO.output(self.right_reverse_pin, GPIO.LOW)
            self.right_enable_pwm.ChangeDutyCycle(0)
    
    def stop_motors(self):
        logger.info("MOTOR CMD: STOP")
        self.set_motor_speeds(0, 0)
    
    def cleanup(self):
        self.stop_motors()
        self.left_enable_pwm.stop()
        self.right_enable_pwm.stop()
        GPIO.cleanup()

class SerialCommunicator:
    def __init__(self, port: str = "/dev/ttyUSB0"):
        self.port = port
        self.serial_conn = None
        self.is_connected = False
        self.last_data_time = 0
        
    def connect(self) -> bool:
        try:
            self.serial_conn = serial.Serial(self.port, 115200, timeout=0.1)
            self.is_connected = True
            logger.info(f"Connected to Arduino on {self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            return False
    
    def read_rc_data(self) -> Optional[RCData]:
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
                        ch9=data.get('ch9', 0),
                        valid=data.get('valid', False),
                        timestamp=data.get('timestamp', 0)
                    )
                    self.last_data_time = time.time()
                    return rc_data
        except Exception as e:
            logger.warning(f"Data parse error: {e}")
        
        return None
    
    def is_data_fresh(self) -> bool:
        return (time.time() - self.last_data_time) < 1.0

class RoverController:
    def __init__(self):
        self.serial_comm = SerialCommunicator()
        self.motor_controller = MotorController(
            left_pins=(18, 19, 16),    # IN1, IN2, ENA1
            right_pins=(20, 21, 26)    # IN3, IN4, ENA2
        )
        self.current_mode = RoverMode.FAILSAFE
        self.rc_data = RCData()
        self.running = False
        logger.info("Rover controller initialized")
    
    def start(self):
        if not self.serial_comm.connect():
            logger.error("Failed to connect to Arduino")
            return False
        
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        logger.info("Rover started")
        return True
    
    def _control_loop(self):
        while self.running:
            try:
                rc_data = self.serial_comm.read_rc_data()
                if rc_data:
                    self.rc_data = rc_data
                
                if self.rc_data.valid and self.serial_comm.is_data_fresh():
                    # Determine mode from 3-position switch (CH9)
                    if self.rc_data.ch9 < -400:  # Position 1: -900
                        self.current_mode = RoverMode.FAILSAFE
                    elif self.rc_data.ch9 > 400:  # Position 3: +900
                        self.current_mode = RoverMode.AUTONOMOUS
                    else:  # Position 2: 0
                        self.current_mode = RoverMode.MANUAL
                    
                    # Handle motor control based on mode
                    if self.current_mode == RoverMode.MANUAL:
                        # Manual RC control - scale Arduino inputs to -100/+100
                        throttle_input = self.rc_data.ch1 / 10  # -100 to +100
                        steering_input = self.rc_data.ch2 / 10  # -100 to +100
                        
                        # Calculate differential steering
                        left_speed = throttle_input + steering_input
                        right_speed = throttle_input - steering_input
                        
                        # Normalize to prevent saturation
                        max_speed = max(abs(left_speed), abs(right_speed))
                        if max_speed > 100:
                            left_speed = (left_speed / max_speed) * 100
                            right_speed = (right_speed / max_speed) * 100
                        
                        # Debug output showing signal flow
                        logger.info(f"RC INPUT: Throttle={throttle_input:.1f}, Steering={steering_input:.1f}")
                        logger.info(f"CALC OUTPUT: Left={left_speed:.1f}, Right={right_speed:.1f}")
                        
                        self.motor_controller.set_motor_speeds(int(left_speed), int(right_speed))
                    elif self.current_mode == RoverMode.AUTONOMOUS:
                        # Autonomous mode - stop motors for now (placeholder)
                        self.motor_controller.stop_motors()
                    else:
                        # Failsafe mode - stop motors
                        self.motor_controller.stop_motors()
                else:
                    self.current_mode = RoverMode.FAILSAFE
                    self.motor_controller.stop_motors()
                
                time.sleep(0.01)
            except Exception as e:
                logger.error(f"Control loop error: {e}")
                self.motor_controller.stop_motors()
    
    def cleanup(self):
        self.running = False
        self.motor_controller.cleanup()

def main():
    rover = RoverController()
    try:
        if rover.start():
            logger.info("Rover running. Press Ctrl+C to stop.")
            while True:
                time.sleep(1)
                logger.info(f"Mode: {rover.current_mode.value}, RC Valid: {rover.rc_data.valid}, CH1: {rover.rc_data.ch1}, CH2: {rover.rc_data.ch2}, CH9: {rover.rc_data.ch9}")
        else:
            logger.error("Failed to start rover")
    except KeyboardInterrupt:
        logger.info("Stopping rover")
    finally:
        rover.cleanup()

if __name__ == "__main__":
    main()