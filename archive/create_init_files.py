#!/usr/bin/env python3
"""
Create Missing __init__.py Files
===============================

Creates the missing __init__.py files needed for Python package structure.
These files make the modular folders importable as Python packages.
"""

import os

# __init__.py content for each package
INIT_FILES = {
    'Main/__init__.py': '''"""
Main Module Package
==================

Core rover functionality including Arduino interface, motor control, 
USB device management, and system configuration.
"""

from .arduino_interface import ArduinoInterface, ArduinoConnectionError
from .motor_controller import MotorController  
from .config import config
from .usb_device_manager import USBDeviceManager

__all__ = [
    'ArduinoInterface',
    'ArduinoConnectionError', 
    'MotorController',
    'config',
    'USBDeviceManager'
]
''',

    'Sensors/__init__.py': '''"""
Sensors Module Package
=====================

Sensor processing modules for ultrasonic distance measurement
and other sensor integrations.
"""

from .ultrasonic_sensor import UltrasonicSensor

__all__ = [
    'UltrasonicSensor'
]
''',

    'Navigation/__init__.py': '''"""
Navigation Module Package
========================

Navigation and pathfinding modules including obstacle avoidance
and movement control algorithms.
"""

from .obstacle_avoidance import ObstacleAvoidance

__all__ = [
    'ObstacleAvoidance'
]
''',

    'Autonomous Behaviors/__init__.py': '''"""
Autonomous Behaviors Module Package
==================================

High-level autonomous behavior modules for exploration,
following, and other complex rover behaviors.
"""

from .simple_exploration import SimpleExploration

__all__ = [
    'SimpleExploration'
]
'''
}

def main():
    print("📦 CREATING MISSING __init__.py FILES")
    print("=" * 40)
    
    created_count = 0
    
    for file_path, content in INIT_FILES.items():
        try:
            # Create directory if it doesn't exist
            directory = os.path.dirname(file_path)
            if directory and not os.path.exists(directory):
                os.makedirs(directory)
                print(f"📁 Created directory: {directory}")
            
            # Create the __init__.py file
            with open(file_path, 'w') as f:
                f.write(content)
            
            print(f"✅ Created: {file_path}")
            created_count += 1
            
        except Exception as e:
            print(f"❌ Failed to create {file_path}: {e}")
    
    print()
    print(f"📊 SUMMARY:")
    print(f"✅ Created {created_count}/{len(INIT_FILES)} __init__.py files")
    
    if created_count == len(INIT_FILES):
        print("🎉 All Python packages are now properly initialized!")
        print("📋 Next step: Run the verification script again")
    else:
        print("⚠️  Some files failed to create - check permissions")
    
    print("=" * 40)

if __name__ == "__main__":
    main()