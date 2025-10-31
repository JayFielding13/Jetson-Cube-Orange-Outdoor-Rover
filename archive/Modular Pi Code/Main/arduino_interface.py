#!/usr/bin/env python3
"""
Arduino Interface Module
========================

Core communication module for Raspberry Pi to Arduino serial communication.
Handles JSON-based protocol for sending motor commands and receiving sensor data.

Key Features:
- Reliable serial communication with timeout handling
- JSON protocol for structured data exchange
- Thread-safe operations
- Connection health monitoring
- Clean error handling and recovery

Protocol:
- Outgoing: {"motor": {"left": speed, "right": speed}} 
- Incoming: {"mode": int, "valid": bool, "emergency": bool, "distance": float}

Part of Mini Rover Development Project - Foundational Module
"""

import serial
import json
import time
import threading
from typing import Optional, Dict, Any, Callable


class ArduinoConnectionError(Exception):
    """Raised when Arduino connection fails or is lost"""
    pass


class ArduinoInterface:
    """
    Handles serial communication with Arduino gatekeeper
    
    Provides reliable JSON-based communication for motor control
    and sensor data reception with proper error handling.
    """
    
    def __init__(self, port: str = '/dev/ttyUSB0', baud: int = 115200, timeout: float = 1.0):
        """
        Initialize Arduino interface
        
        Args:
            port: Serial port path (e.g., '/dev/ttyUSB0', 'COM3')
            baud: Baud rate for serial communication
            timeout: Serial read timeout in seconds
        """
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.connection: Optional[serial.Serial] = None
        self.connected = False
        self.last_data_time = 0.0
        
        # Thread safety
        self._lock = threading.Lock()
        
        # Data callback
        self._data_callback: Optional[Callable[[Dict[str, Any]], None]] = None
        
        # Connection monitoring
        self.connection_timeout = 5.0  # Seconds before considering connection lost
        self.consecutive_failures = 0
        self.max_failures = 10
        
        print(f"🔧 Arduino Interface initialized: {port} @ {baud} baud")
    
    def connect(self) -> bool:
        """
        Establish connection to Arduino
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            with self._lock:
                if self.connection:
                    self.connection.close()
                
                self.connection = serial.Serial(
                    port=self.port,
                    baudrate=self.baud,
                    timeout=self.timeout,
                    write_timeout=self.timeout
                )
                
                # Allow Arduino to reset
                time.sleep(2.0)
                
                # Clear any buffered data
                self.connection.flushInput()
                self.connection.flushOutput()
                
                self.connected = True
                self.last_data_time = time.time()
                self.consecutive_failures = 0
                
                print(f"✅ Arduino connected on {self.port}")
                return True
                
        except Exception as e:
            print(f"❌ Arduino connection failed: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Close Arduino connection"""
        with self._lock:
            if self.connection:
                try:
                    self.connection.close()
                except:
                    pass
                finally:
                    self.connection = None
                    self.connected = False
            
            print("🔌 Arduino disconnected")
    
    def set_data_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """
        Set callback function for incoming Arduino data
        
        Args:
            callback: Function to call with parsed JSON data
        """
        self._data_callback = callback
    
    def send_motor_command(self, left_speed: int, right_speed: int) -> bool:
        """
        Send motor speed command to Arduino
        
        Args:
            left_speed: Left motor speed (-255 to 255)
            right_speed: Right motor speed (-255 to 255)
            
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.connected or not self.connection:
            return False
        
        # Clamp speeds to valid range
        left_speed = max(-255, min(255, int(left_speed)))
        right_speed = max(-255, min(255, int(right_speed)))
        
        # Create command structure
        command = {
            'motor': {
                'left': left_speed,
                'right': right_speed
            }
        }
        
        try:
            with self._lock:
                cmd_str = json.dumps(command) + '\n'
                self.connection.write(cmd_str.encode())
                return True
                
        except Exception as e:
            print(f"❌ Motor command failed: {e}")
            self.consecutive_failures += 1
            
            if self.consecutive_failures >= self.max_failures:
                self.connected = False
                raise ArduinoConnectionError("Too many consecutive command failures")
            
            return False
    
    def emergency_stop(self) -> bool:
        """
        Send emergency stop command (zero speeds)
        
        Returns:
            True if stop command sent successfully
        """
        success = self.send_motor_command(0, 0)
        if success:
            print("🛑 Emergency stop sent to Arduino")
        return success
    
    def read_data(self) -> Optional[Dict[str, Any]]:
        """
        Read and parse data from Arduino
        
        Returns:
            Parsed JSON data if available, None otherwise
        """
        if not self.connected or not self.connection:
            return None
        
        try:
            with self._lock:
                if self.connection.in_waiting > 0:
                    line = self.connection.readline().decode().strip()
                    
                    if line and line.startswith('{'):
                        data = json.loads(line)
                        self.last_data_time = time.time()
                        self.consecutive_failures = 0
                        
                        # Call data callback if set
                        if self._data_callback:
                            self._data_callback(data)
                        
                        return data
                        
        except json.JSONDecodeError as e:
            print(f"⚠️ JSON decode error: {e}")
            self.consecutive_failures += 1
            
        except Exception as e:
            print(f"❌ Arduino read error: {e}")
            self.consecutive_failures += 1
        
        # Check for connection timeout
        if time.time() - self.last_data_time > self.connection_timeout:
            print("⚠️ Arduino communication timeout")
            self.consecutive_failures += 1
        
        # Handle persistent failures
        if self.consecutive_failures >= self.max_failures:
            self.connected = False
            raise ArduinoConnectionError("Too many consecutive communication failures")
        
        return None
    
    def is_connected(self) -> bool:
        """Check if Arduino connection is healthy"""
        if not self.connected:
            return False
        
        # Check for communication timeout
        time_since_data = time.time() - self.last_data_time
        return time_since_data < self.connection_timeout
    
    def get_connection_status(self) -> Dict[str, Any]:
        """
        Get detailed connection status
        
        Returns:
            Dictionary with connection health information
        """
        return {
            'connected': self.connected,
            'port': self.port,
            'baud': self.baud,
            'last_data_age': time.time() - self.last_data_time,
            'consecutive_failures': self.consecutive_failures,
            'healthy': self.is_connected()
        }


# Example usage
if __name__ == "__main__":
    def data_handler(data):
        """Example data callback"""
        print(f"📊 Arduino data: {data}")
    
    # Create interface
    arduino = ArduinoInterface()
    arduino.set_data_callback(data_handler)
    
    # Connect and test
    if arduino.connect():
        try:
            print("🧪 Testing motor commands...")
            
            # Test forward
            arduino.send_motor_command(100, 100)
            time.sleep(1)
            
            # Test turn
            arduino.send_motor_command(-80, 80)
            time.sleep(1)
            
            # Test stop
            arduino.emergency_stop()
            
            # Read data for a few seconds
            print("📡 Reading data...")
            for _ in range(10):
                data = arduino.read_data()
                time.sleep(0.5)
            
        except KeyboardInterrupt:
            print("\n🛑 Test interrupted")
        
        except ArduinoConnectionError as e:
            print(f"❌ Connection error: {e}")
        
        finally:
            arduino.disconnect()
    else:
        print("❌ Could not connect to Arduino")