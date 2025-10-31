#!/usr/bin/env python3
"""
Rover Cleanup Verification Script
=================================

This script helps verify that your rover Pi has only the necessary modular files
and removes any remaining unwanted files from the previous cleanup process.

Run this on your rover Pi to check the current state and finish cleanup if needed.
"""

import os
import subprocess
from typing import List, Set

# Files and folders that SHOULD exist on the rover
REQUIRED_FILES = {
    # Main modules
    'Main/arduino_interface.py',
    'Main/motor_controller.py', 
    'Main/config.py',
    'Main/usb_device_manager.py',
    'Main/__init__.py',
    
    # Sensor modules
    'Sensors/ultrasonic_sensor.py',
    'Sensors/__init__.py',
    
    # Navigation modules
    'Navigation/obstacle_avoidance.py',
    'Navigation/__init__.py',
    
    # Autonomous behavior modules
    'Autonomous Behaviors/simple_exploration.py',
    'Autonomous Behaviors/__init__.py',
    
    # Main execution files
    'rover_smart_startup.py',
    'rover_exploration_test.py',
    
    # Documentation and setup
    'ROVER_EXPLORATION_README.md',
    'requirements.txt',
    
    # Virtual environment (directory)
    'venv/'
}

# File patterns that should be REMOVED (old monolithic files)
UNWANTED_PATTERNS = [
    'stable_bluetooth_ranging_v*.py',
    'bluetooth_ranging*.py',
    'rover_test*.py',
    'simple_rover*.py',
    'motor_test*.py',
    'serial_test*.py',
    'ultrasonic_test*.py',
    'obstacle_test*.py',
    'exploration_test*.py',
    'BLE_*.py',
    'main.py',  # If it's the old main.py, not our modular one
]

# Directories that might contain old files
CHECK_DIRECTORIES = [
    '.',
    'old_files/',
    'backup/',
    'archive/',
    'tests/',
]

def get_current_directory_contents() -> List[str]:
    """Get all files and directories in current location"""
    try:
        result = subprocess.run(['find', '.', '-type', 'f'], 
                              capture_output=True, text=True)
        files = [f.lstrip('./') for f in result.stdout.strip().split('\n') if f.strip()]
        return files
    except Exception as e:
        print(f"Error getting directory contents: {e}")
        return []

def get_directory_size() -> str:
    """Get total size of current directory"""
    try:
        result = subprocess.run(['du', '-sh', '.'], 
                              capture_output=True, text=True)
        return result.stdout.strip().split()[0]
    except:
        return "Unknown"

def check_required_files(all_files: List[str]) -> tuple:
    """Check which required files are present"""
    present = []
    missing = []
    
    for required_file in REQUIRED_FILES:
        # Handle directories
        if required_file.endswith('/'):
            if any(f.startswith(required_file) for f in all_files) or os.path.isdir(required_file):
                present.append(required_file)
            else:
                missing.append(required_file)
        else:
            if required_file in all_files:
                present.append(required_file)
            else:
                missing.append(required_file)
    
    return present, missing

def find_unwanted_files(all_files: List[str]) -> List[str]:
    """Find files that match unwanted patterns"""
    unwanted = []
    
    for file in all_files:
        filename = os.path.basename(file)
        
        # Check against unwanted patterns
        for pattern in UNWANTED_PATTERNS:
            if '*' in pattern:
                # Simple wildcard matching
                pattern_start = pattern.split('*')[0]
                pattern_end = pattern.split('*')[-1] if pattern.split('*')[-1] else ''
                if filename.startswith(pattern_start) and filename.endswith(pattern_end):
                    unwanted.append(file)
                    break
            else:
                if filename == pattern:
                    unwanted.append(file)
                    break
    
    return unwanted

def suggest_cleanup_commands(unwanted_files: List[str]) -> List[str]:
    """Generate cleanup commands for unwanted files"""
    if not unwanted_files:
        return []
    
    commands = []
    for file in unwanted_files:
        commands.append(f"rm '{file}'")
    
    return commands

def main():
    print("🔍 ROVER PI CLEANUP VERIFICATION")
    print("=" * 50)
    
    # Get current directory info
    current_dir = os.getcwd()
    total_size = get_directory_size()
    
    print(f"📍 Current directory: {current_dir}")
    print(f"📦 Total size: {total_size}")
    print()
    
    # Get all files
    print("📋 Scanning directory contents...")
    all_files = get_current_directory_contents()
    print(f"📄 Total files found: {len(all_files)}")
    print()
    
    # Check required files
    print("✅ CHECKING REQUIRED FILES:")
    print("-" * 30)
    present_files, missing_files = check_required_files(all_files)
    
    print(f"Present ({len(present_files)}/{len(REQUIRED_FILES)}):")
    for file in sorted(present_files):
        print(f"  ✅ {file}")
    
    if missing_files:
        print(f"\nMissing ({len(missing_files)}):")
        for file in sorted(missing_files):
            print(f"  ❌ {file}")
    
    print()
    
    # Check for unwanted files
    print("🗑️  CHECKING FOR UNWANTED FILES:")
    print("-" * 35)
    unwanted_files = find_unwanted_files(all_files)
    
    if unwanted_files:
        print(f"❌ Found {len(unwanted_files)} unwanted files:")
        for file in sorted(unwanted_files):
            print(f"  🗑️  {file}")
        
        print("\n🧹 CLEANUP COMMANDS:")
        print("-" * 20)
        cleanup_commands = suggest_cleanup_commands(unwanted_files)
        for cmd in cleanup_commands:
            print(f"  {cmd}")
        
        print(f"\n💡 Or run all at once:")
        quoted_files = [f"'{f}'" for f in unwanted_files]
        print(f"  rm {' '.join(quoted_files)}")
        
    else:
        print("✅ No unwanted files found!")
    
    print()
    
    # Summary
    print("📊 CLEANUP VERIFICATION SUMMARY:")
    print("-" * 35)
    print(f"✅ Required files present: {len(present_files)}/{len(REQUIRED_FILES)}")
    print(f"❌ Required files missing: {len(missing_files)}")
    print(f"🗑️  Unwanted files found: {len(unwanted_files)}")
    print(f"📦 Directory size: {total_size}")
    
    # Overall status
    if missing_files:
        print("\n⚠️  STATUS: INCOMPLETE - Missing required files")
        print("   Upload missing files to complete modular setup")
    elif unwanted_files:
        print("\n🧹 STATUS: NEEDS CLEANUP - Remove unwanted files")
        print("   Run the suggested cleanup commands above")
    else:
        print("\n🎉 STATUS: CLEAN - Ready for rover operations!")
        print("   Run: source venv/bin/activate && python3 rover_smart_startup.py")
    
    print("=" * 50)

if __name__ == "__main__":
    main()