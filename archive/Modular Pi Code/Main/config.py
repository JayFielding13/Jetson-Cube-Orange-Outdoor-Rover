#!/usr/bin/env python3
"""
Configuration Module
====================

Centralized configuration management for the rover system.
Contains hardware settings, movement parameters, and system constants.

Key Features:
- Hardware port and connection settings
- Motor speed and movement parameters
- Safety thresholds and limits
- Easy parameter tuning
- Environment variable support

Part of Mini Rover Development Project - Foundational Module
"""

import os
from dataclasses import dataclass
from typing import Optional


@dataclass
class HardwareConfig:
    """Hardware connection settings"""
    # Arduino connection (will be auto-detected by USB manager)
    arduino_port: str = '/dev/ttyUSB0'  # Default fallback
    arduino_baud: int = 115200
    arduino_timeout: float = 1.0
    connection_timeout: float = 5.0
    max_connection_failures: int = 10
    
    # USB device management
    use_usb_manager: bool = True        # Enable smart USB management
    quick_startup: bool = True          # Try quick validation first
    usb_config_file: str = "usb_devices.json"  # Device configuration file
    
    @classmethod
    def from_env(cls) -> 'HardwareConfig':
        """Create hardware config from environment variables"""
        return cls(
            arduino_port=os.getenv('ROVER_ARDUINO_PORT', cls.arduino_port),
            arduino_baud=int(os.getenv('ROVER_ARDUINO_BAUD', str(cls.arduino_baud))),
            arduino_timeout=float(os.getenv('ROVER_ARDUINO_TIMEOUT', str(cls.arduino_timeout))),
            connection_timeout=float(os.getenv('ROVER_CONNECTION_TIMEOUT', str(cls.connection_timeout))),
            max_connection_failures=int(os.getenv('ROVER_MAX_FAILURES', str(cls.max_connection_failures)))
        )


@dataclass
class MotorConfig:
    """Motor control parameters"""
    # Speed settings
    max_speed: int = 255           # Maximum possible motor speed
    cruise_speed: int = 100        # Normal forward movement speed
    turn_speed: int = 80           # Speed for turning in place
    slow_speed: int = 60           # Careful/slow movement speed
    reverse_speed: int = 70        # Speed for backing up
    
    # Safety limits
    min_speed: int = 30            # Minimum effective speed (prevent stalling)
    speed_limit: int = 200         # Safety speed limit
    
    # Movement parameters
    turn_duration: float = 1.5     # Default turn duration (seconds)
    backup_duration: float = 2.0   # Default backup duration (seconds)
    
    @classmethod
    def from_env(cls) -> 'MotorConfig':
        """Create motor config from environment variables"""
        return cls(
            max_speed=int(os.getenv('ROVER_MAX_SPEED', str(cls.max_speed))),
            cruise_speed=int(os.getenv('ROVER_CRUISE_SPEED', str(cls.cruise_speed))),
            turn_speed=int(os.getenv('ROVER_TURN_SPEED', str(cls.turn_speed))),
            slow_speed=int(os.getenv('ROVER_SLOW_SPEED', str(cls.slow_speed))),
            reverse_speed=int(os.getenv('ROVER_REVERSE_SPEED', str(cls.reverse_speed))),
            min_speed=int(os.getenv('ROVER_MIN_SPEED', str(cls.min_speed))),
            speed_limit=int(os.getenv('ROVER_SPEED_LIMIT', str(cls.speed_limit))),
            turn_duration=float(os.getenv('ROVER_TURN_DURATION', str(cls.turn_duration))),
            backup_duration=float(os.getenv('ROVER_BACKUP_DURATION', str(cls.backup_duration)))
        )


@dataclass
class SafetyConfig:
    """Safety and emergency settings"""
    # Emergency thresholds
    emergency_stop_enabled: bool = True
    communication_timeout: float = 5.0    # Arduino comm timeout
    
    # Startup delays
    arduino_reset_delay: float = 2.0      # Time for Arduino to reset
    startup_delay: float = 1.0            # General startup delay
    
    @classmethod
    def from_env(cls) -> 'SafetyConfig':
        """Create safety config from environment variables"""
        return cls(
            emergency_stop_enabled=os.getenv('ROVER_EMERGENCY_STOP', 'true').lower() == 'true',
            communication_timeout=float(os.getenv('ROVER_COMM_TIMEOUT', str(cls.communication_timeout))),
            arduino_reset_delay=float(os.getenv('ROVER_RESET_DELAY', str(cls.arduino_reset_delay))),
            startup_delay=float(os.getenv('ROVER_STARTUP_DELAY', str(cls.startup_delay)))
        )


@dataclass
class SystemConfig:
    """System-wide configuration"""
    # Logging and debugging
    debug_mode: bool = False
    log_level: str = 'INFO'            # DEBUG, INFO, WARNING, ERROR
    status_update_interval: float = 3.0 # Seconds between status updates
    
    # Loop timing
    main_loop_rate: float = 10.0       # Hz - main control loop rate
    sensor_read_rate: float = 50.0     # Hz - sensor reading rate
    
    @classmethod
    def from_env(cls) -> 'SystemConfig':
        """Create system config from environment variables"""
        return cls(
            debug_mode=os.getenv('ROVER_DEBUG', 'false').lower() == 'true',
            log_level=os.getenv('ROVER_LOG_LEVEL', cls.log_level),
            status_update_interval=float(os.getenv('ROVER_STATUS_INTERVAL', str(cls.status_update_interval))),
            main_loop_rate=float(os.getenv('ROVER_MAIN_RATE', str(cls.main_loop_rate))),
            sensor_read_rate=float(os.getenv('ROVER_SENSOR_RATE', str(cls.sensor_read_rate)))
        )


class RoverConfig:
    """Main configuration class combining all config sections"""
    
    def __init__(self, load_from_env: bool = True):
        """
        Initialize rover configuration
        
        Args:
            load_from_env: Whether to load settings from environment variables
        """
        if load_from_env:
            self.hardware = HardwareConfig.from_env()
            self.motor = MotorConfig.from_env()
            self.safety = SafetyConfig.from_env()
            self.system = SystemConfig.from_env()
        else:
            self.hardware = HardwareConfig()
            self.motor = MotorConfig()
            self.safety = SafetyConfig()
            self.system = SystemConfig()
    
    def print_config(self):
        """Print current configuration settings"""
        print("🔧 === Rover Configuration ===")
        print(f"Hardware:")
        print(f"  Arduino: {self.hardware.arduino_port} @ {self.hardware.arduino_baud} baud")
        print(f"  Timeout: {self.hardware.arduino_timeout}s | Max failures: {self.hardware.max_connection_failures}")
        print()
        print(f"Motor:")
        print(f"  Speeds: Cruise={self.motor.cruise_speed} | Turn={self.motor.turn_speed} | Slow={self.motor.slow_speed}")
        print(f"  Limits: Min={self.motor.min_speed} | Max={self.motor.speed_limit}")
        print(f"  Timing: Turn={self.motor.turn_duration}s | Backup={self.motor.backup_duration}s")
        print()
        print(f"Safety:")
        print(f"  Emergency stop: {self.safety.emergency_stop_enabled}")
        print(f"  Communication timeout: {self.safety.communication_timeout}s")
        print()
        print(f"System:")
        print(f"  Debug: {self.system.debug_mode} | Log level: {self.system.log_level}")
        print(f"  Rates: Main={self.system.main_loop_rate}Hz | Sensor={self.system.sensor_read_rate}Hz")
        print("=" * 40)
    
    def validate_config(self) -> bool:
        """
        Validate configuration settings
        
        Returns:
            True if configuration is valid, False otherwise
        """
        valid = True
        issues = []
        
        # Validate hardware settings
        if self.hardware.arduino_baud <= 0:
            issues.append("Arduino baud rate must be positive")
            valid = False
        
        if self.hardware.arduino_timeout <= 0:
            issues.append("Arduino timeout must be positive")
            valid = False
        
        # Validate motor settings
        if self.motor.min_speed >= self.motor.speed_limit:
            issues.append("Minimum speed must be less than speed limit")
            valid = False
        
        if self.motor.cruise_speed > self.motor.speed_limit:
            issues.append("Cruise speed exceeds speed limit")
            valid = False
        
        if self.motor.turn_speed > self.motor.speed_limit:
            issues.append("Turn speed exceeds speed limit")
            valid = False
        
        # Validate system settings
        if self.system.main_loop_rate <= 0:
            issues.append("Main loop rate must be positive")
            valid = False
        
        if not valid:
            print("❌ Configuration validation failed:")
            for issue in issues:
                print(f"   - {issue}")
        else:
            print("✅ Configuration validation passed")
        
        return valid


# Create default global configuration instance
config = RoverConfig()


# Convenience functions for common config access
def get_arduino_port() -> str:
    """Get Arduino port setting"""
    return config.hardware.arduino_port

def get_arduino_baud() -> int:
    """Get Arduino baud rate setting"""
    return config.hardware.arduino_baud

def get_cruise_speed() -> int:
    """Get cruise speed setting"""
    return config.motor.cruise_speed

def get_turn_speed() -> int:
    """Get turn speed setting"""
    return config.motor.turn_speed

def get_main_loop_rate() -> float:
    """Get main loop rate setting"""
    return config.system.main_loop_rate


# Example usage and testing
if __name__ == "__main__":
    print("🧪 Testing configuration system...")
    
    # Test default configuration
    cfg = RoverConfig(load_from_env=False)
    cfg.print_config()
    cfg.validate_config()
    
    print("\n" + "="*50)
    print("🌍 Testing environment variable loading...")
    
    # Set some environment variables for testing
    os.environ['ROVER_CRUISE_SPEED'] = '120'
    os.environ['ROVER_ARDUINO_PORT'] = '/dev/ttyACM0'
    os.environ['ROVER_DEBUG'] = 'true'
    
    # Load config from environment
    cfg_env = RoverConfig(load_from_env=True)
    cfg_env.print_config()
    cfg_env.validate_config()
    
    # Test convenience functions
    print(f"\n📍 Convenience functions:")
    print(f"  Arduino port: {get_arduino_port()}")
    print(f"  Cruise speed: {get_cruise_speed()}")
    print(f"  Main loop rate: {get_main_loop_rate()}")