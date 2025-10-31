#!/usr/bin/env python3
"""
Motor Controller Module
=======================

High-level motor control interface for rover movement commands.
Provides intuitive movement functions and safety controls built on top
of the Arduino interface.

Key Features:
- Simple movement commands (forward, backward, turn_left, turn_right, stop)
- Speed control with safety limits
- Smooth acceleration/deceleration
- Emergency stop capability
- Movement validation and bounds checking

Part of Mini Rover Development Project - Foundational Module
"""

import time
from typing import Optional, Tuple
from arduino_interface import ArduinoInterface


class MotorController:
    """
    High-level motor control for rover movement
    
    Provides intuitive movement commands and safety controls
    built on top of the Arduino interface.
    """
    
    def __init__(self, arduino_interface: ArduinoInterface):
        """
        Initialize motor controller
        
        Args:
            arduino_interface: Connected Arduino interface instance
        """
        self.arduino = arduino_interface
        
        # Speed settings
        self.max_speed = 255        # Maximum motor speed
        self.cruise_speed = 100     # Normal forward speed
        self.turn_speed = 80        # Speed for turning
        self.slow_speed = 60        # Slow/careful movement speed
        
        # Safety settings
        self.min_speed = 30         # Minimum effective speed
        self.speed_limit = 200      # Safety speed limit
        
        # Current motor state
        self.current_left_speed = 0
        self.current_right_speed = 0
        self.is_moving = False
        
        print("🎛️ Motor Controller initialized")
        print(f"   Cruise: {self.cruise_speed} | Turn: {self.turn_speed} | Limit: {self.speed_limit}")
    
    def _validate_speed(self, speed: int) -> int:
        """
        Validate and clamp speed to safe range
        
        Args:
            speed: Raw speed value
            
        Returns:
            Clamped speed within safe limits
        """
        # Apply speed limit
        speed = max(-self.speed_limit, min(self.speed_limit, speed))
        
        # Apply minimum threshold (avoid stalling)
        if 0 < abs(speed) < self.min_speed:
            speed = self.min_speed if speed > 0 else -self.min_speed
        
        return int(speed)
    
    def _send_speeds(self, left_speed: int, right_speed: int) -> bool:
        """
        Send validated motor speeds to Arduino
        
        Args:
            left_speed: Left motor speed
            right_speed: Right motor speed
            
        Returns:
            True if command sent successfully
        """
        # Validate speeds
        left_speed = self._validate_speed(left_speed)
        right_speed = self._validate_speed(right_speed)
        
        # Send to Arduino
        success = self.arduino.send_motor_command(left_speed, right_speed)
        
        if success:
            self.current_left_speed = left_speed
            self.current_right_speed = right_speed
            self.is_moving = (left_speed != 0 or right_speed != 0)
        
        return success
    
    def stop(self) -> bool:
        """
        Stop all motor movement
        
        Returns:
            True if stop command sent successfully
        """
        success = self._send_speeds(0, 0)
        if success:
            print("🛑 Motors stopped")
        return success
    
    def emergency_stop(self) -> bool:
        """
        Emergency stop - direct call to Arduino
        
        Returns:
            True if emergency stop successful
        """
        success = self.arduino.emergency_stop()
        if success:
            self.current_left_speed = 0
            self.current_right_speed = 0
            self.is_moving = False
        return success
    
    def forward(self, speed: Optional[int] = None) -> bool:
        """
        Move forward at specified speed
        
        Args:
            speed: Forward speed (uses cruise_speed if None)
            
        Returns:
            True if command sent successfully
        """
        if speed is None:
            speed = self.cruise_speed
        
        return self._send_speeds(speed, speed)
    
    def backward(self, speed: Optional[int] = None) -> bool:
        """
        Move backward at specified speed
        
        Args:
            speed: Backward speed (uses cruise_speed if None)
            
        Returns:
            True if command sent successfully
        """
        if speed is None:
            speed = self.cruise_speed
        
        return self._send_speeds(-speed, -speed)
    
    def turn_left(self, speed: Optional[int] = None) -> bool:
        """
        Turn left in place
        
        Args:
            speed: Turn speed (uses turn_speed if None)
            
        Returns:
            True if command sent successfully
        """
        if speed is None:
            speed = self.turn_speed
        
        return self._send_speeds(-speed, speed)
    
    def turn_right(self, speed: Optional[int] = None) -> bool:
        """
        Turn right in place
        
        Args:
            speed: Turn speed (uses turn_speed if None)
            
        Returns:
            True if command sent successfully
        """
        if speed is None:
            speed = self.turn_speed
        
        return self._send_speeds(speed, -speed)
    
    def curve_left(self, speed: Optional[int] = None, turn_ratio: float = 0.5) -> bool:
        """
        Curve left while moving forward
        
        Args:
            speed: Forward speed (uses cruise_speed if None)
            turn_ratio: How much to slow the left wheel (0.0 to 1.0)
            
        Returns:
            True if command sent successfully
        """
        if speed is None:
            speed = self.cruise_speed
        
        turn_ratio = max(0.0, min(1.0, turn_ratio))
        left_speed = int(speed * (1.0 - turn_ratio))
        right_speed = speed
        
        return self._send_speeds(left_speed, right_speed)
    
    def curve_right(self, speed: Optional[int] = None, turn_ratio: float = 0.5) -> bool:
        """
        Curve right while moving forward
        
        Args:
            speed: Forward speed (uses cruise_speed if None)
            turn_ratio: How much to slow the right wheel (0.0 to 1.0)
            
        Returns:
            True if command sent successfully
        """
        if speed is None:
            speed = self.cruise_speed
        
        turn_ratio = max(0.0, min(1.0, turn_ratio))
        left_speed = speed
        right_speed = int(speed * (1.0 - turn_ratio))
        
        return self._send_speeds(left_speed, right_speed)
    
    def custom_speeds(self, left_speed: int, right_speed: int) -> bool:
        """
        Set custom motor speeds directly
        
        Args:
            left_speed: Left motor speed (-255 to 255)
            right_speed: Right motor speed (-255 to 255)
            
        Returns:
            True if command sent successfully
        """
        return self._send_speeds(left_speed, right_speed)
    
    def timed_movement(self, left_speed: int, right_speed: int, duration: float) -> bool:
        """
        Execute movement for a specific duration
        
        Args:
            left_speed: Left motor speed
            right_speed: Right motor speed  
            duration: Movement duration in seconds
            
        Returns:
            True if movement completed successfully
        """
        # Start movement
        if not self._send_speeds(left_speed, right_speed):
            return False
        
        # Wait for duration
        time.sleep(duration)
        
        # Stop movement
        return self.stop()
    
    def get_motor_state(self) -> dict:
        """
        Get current motor state information
        
        Returns:
            Dictionary with current motor state
        """
        return {
            'left_speed': self.current_left_speed,
            'right_speed': self.current_right_speed,
            'is_moving': self.is_moving,
            'cruise_speed': self.cruise_speed,
            'turn_speed': self.turn_speed,
            'speed_limit': self.speed_limit
        }
    
    def set_speeds(self, cruise: Optional[int] = None, turn: Optional[int] = None, 
                   slow: Optional[int] = None):
        """
        Update speed settings
        
        Args:
            cruise: New cruise speed
            turn: New turn speed
            slow: New slow speed
        """
        if cruise is not None:
            self.cruise_speed = max(self.min_speed, min(self.speed_limit, cruise))
        
        if turn is not None:
            self.turn_speed = max(self.min_speed, min(self.speed_limit, turn))
        
        if slow is not None:
            self.slow_speed = max(self.min_speed, min(self.speed_limit, slow))
        
        print(f"🎛️ Speed settings updated: Cruise={self.cruise_speed}, Turn={self.turn_speed}, Slow={self.slow_speed}")


# Example usage and testing
if __name__ == "__main__":
    from arduino_interface import ArduinoInterface
    
    # Create Arduino interface and motor controller
    arduino = ArduinoInterface('/dev/ttyUSB0')
    motors = MotorController(arduino)
    
    if arduino.connect():
        try:
            print("🧪 Testing motor controller...")
            
            # Test basic movements
            print("🔄 Forward...")
            motors.forward()
            time.sleep(2)
            
            print("🔄 Turn left...")
            motors.turn_left()
            time.sleep(1)
            
            print("🔄 Turn right...")
            motors.turn_right()
            time.sleep(1)
            
            print("🔄 Backward...")
            motors.backward()
            time.sleep(1)
            
            print("🔄 Curve left...")
            motors.curve_left(turn_ratio=0.7)
            time.sleep(2)
            
            print("🔄 Timed movement test...")
            motors.timed_movement(80, -80, 1.5)  # Turn for 1.5 seconds
            
            print("🛑 Stop")
            motors.stop()
            
            # Print final state
            state = motors.get_motor_state()
            print(f"📊 Final state: {state}")
            
        except KeyboardInterrupt:
            print("\n🛑 Test interrupted")
            motors.emergency_stop()
        
        finally:
            arduino.disconnect()
    else:
        print("❌ Could not connect to Arduino")