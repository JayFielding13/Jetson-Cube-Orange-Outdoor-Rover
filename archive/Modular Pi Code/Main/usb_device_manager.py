#!/usr/bin/env python3
"""
USB Device Manager
==================

Smart USB device discovery and management for Raspberry Pi robotics.
Provides fast startup with automatic device reconnection when ports change.

Key Features:
- Quick validation of known devices first
- Full port discovery only when needed
- Device identification by hardware characteristics
- Port change detection and automatic reconnection
- Persistent device configuration
- Arduino-specific detection methods

Part of Mini Rover Development Project - Main Module
"""

import os
import time
import json
import serial
import subprocess
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, asdict
import threading


@dataclass
class USBDevice:
    """USB device information"""
    device_type: str        # 'arduino', 'sensor', 'camera', etc.
    port: str              # '/dev/ttyUSB0', '/dev/ttyACM0', etc.
    vendor_id: str         # USB vendor ID
    product_id: str        # USB product ID
    description: str       # Human readable description
    baud_rate: int         # For serial devices
    last_seen: float       # Timestamp when last detected
    is_connected: bool     # Current connection status


class USBDeviceManager:
    """
    Smart USB device manager for Raspberry Pi
    
    Handles automatic detection, validation, and reconnection
    of USB devices with fast startup and robust recovery.
    """
    
    def __init__(self, config_file: str = "usb_devices.json"):
        """
        Initialize USB device manager
        
        Args:
            config_file: Path to device configuration file
        """
        self.config_file = config_file
        self.known_devices: Dict[str, USBDevice] = {}
        self.arduino_device: Optional[USBDevice] = None
        
        # Detection settings
        self.quick_validation_timeout = 0.5    # Fast check timeout
        self.full_discovery_timeout = 2.0      # Full check timeout
        self.arduino_detection_patterns = [
            "Arduino",
            "USB2.0-Serial",
            "ch340",
            "CH340",
            "FT232",
            "FTDI"
        ]
        
        # Port patterns to check
        self.serial_port_patterns = [
            "/dev/ttyUSB*",
            "/dev/ttyACM*",
            "/dev/ttyS*"
        ]
        
        print("🔌 USB Device Manager initialized")
    
    def quick_device_check(self) -> bool:
        """
        Quick validation of known devices
        
        Returns:
            True if all known devices are still connected, False otherwise
        """
        if not self.known_devices:
            print("📋 No known devices - performing full discovery")
            return False
        
        print("⚡ Quick device validation...")
        all_devices_ok = True
        
        for device_id, device in self.known_devices.items():
            if self._quick_validate_device(device):
                device.is_connected = True
                device.last_seen = time.time()
                print(f"✅ {device.device_type} found at {device.port}")
            else:
                device.is_connected = False
                all_devices_ok = False
                print(f"❌ {device.device_type} missing from {device.port}")
        
        if all_devices_ok:
            print("🚀 All devices validated - quick startup complete")
        else:
            print("🔍 Some devices missing - will perform full discovery")
        
        return all_devices_ok
    
    def _quick_validate_device(self, device: USBDevice) -> bool:
        """
        Quickly validate a specific device
        
        Args:
            device: Device to validate
            
        Returns:
            True if device responds at expected port
        """
        if not os.path.exists(device.port):
            return False
        
        if device.device_type == 'arduino':
            return self._quick_arduino_check(device.port, device.baud_rate)
        
        # For other device types, just check if port exists and is accessible
        try:
            with serial.Serial(device.port, device.baud_rate, timeout=0.1):
                return True
        except:
            return False
    
    def _quick_arduino_check(self, port: str, baud_rate: int) -> bool:
        """
        Quick Arduino connectivity check
        
        Args:
            port: Serial port to check
            baud_rate: Baud rate to use
            
        Returns:
            True if Arduino responds
        """
        try:
            with serial.Serial(port, baud_rate, timeout=self.quick_validation_timeout) as ser:
                # Wait for Arduino to settle
                time.sleep(0.1)
                
                # Clear any buffered data
                ser.flushInput()
                ser.flushOutput()
                
                # Try to read any data (Arduino should be sending status)
                start_time = time.time()
                while time.time() - start_time < self.quick_validation_timeout:
                    if ser.in_waiting > 0:
                        data = ser.readline().decode('utf-8', errors='ignore').strip()
                        if data and ('{' in data or 'arduino' in data.lower()):
                            return True
                    time.sleep(0.01)
                
                return False
                
        except Exception:
            return False
    
    def discover_devices(self) -> Dict[str, USBDevice]:
        """
        Perform full USB device discovery
        
        Returns:
            Dictionary of discovered devices
        """
        print("🔍 Starting full USB device discovery...")
        discovered_devices = {}
        
        # Get all available serial ports
        available_ports = self._get_available_serial_ports()
        print(f"📍 Found {len(available_ports)} serial ports: {available_ports}")
        
        # Check each port
        for port in available_ports:
            device = self._identify_device_at_port(port)
            if device:
                device_id = f"{device.device_type}_{port.replace('/', '_')}"
                discovered_devices[device_id] = device
                print(f"✅ Identified {device.device_type} at {port}: {device.description}")
        
        # Update known devices
        self.known_devices.update(discovered_devices)
        
        # Find Arduino device
        self._update_arduino_reference()
        
        print(f"🎉 Discovery complete - found {len(discovered_devices)} devices")
        return discovered_devices
    
    def _get_available_serial_ports(self) -> List[str]:
        """Get list of available serial ports"""
        ports = []
        
        # Check common serial port locations
        for pattern in self.serial_port_patterns:
            import glob
            ports.extend(glob.glob(pattern))
        
        # Sort ports for consistent ordering
        ports.sort()
        return ports
    
    def _identify_device_at_port(self, port: str) -> Optional[USBDevice]:
        """
        Identify device type at a specific port
        
        Args:
            port: Serial port to check
            
        Returns:
            USBDevice if identified, None otherwise
        """
        # Get USB device info
        vendor_id, product_id, description = self._get_usb_device_info(port)
        
        # Try different baud rates for Arduino detection
        arduino_baud_rates = [115200, 9600, 57600, 38400]
        
        for baud_rate in arduino_baud_rates:
            if self._test_arduino_at_port(port, baud_rate):
                return USBDevice(
                    device_type='arduino',
                    port=port,
                    vendor_id=vendor_id,
                    product_id=product_id,
                    description=description,
                    baud_rate=baud_rate,
                    last_seen=time.time(),
                    is_connected=True
                )
        
        # Check for other device types (cameras, sensors, etc.)
        # This can be extended for other device types
        
        return None
    
    def _get_usb_device_info(self, port: str) -> Tuple[str, str, str]:
        """
        Get USB device information using system commands
        
        Args:
            port: Serial port path
            
        Returns:
            Tuple of (vendor_id, product_id, description)
        """
        try:
            # Use udevadm to get device info
            result = subprocess.run(
                ['udevadm', 'info', '--name=' + port, '--attribute-walk'],
                capture_output=True, text=True, timeout=2
            )
            
            vendor_id = ""
            product_id = ""
            description = ""
            
            for line in result.stdout.split('\n'):
                if 'ATTRS{idVendor}' in line:
                    vendor_id = line.split('"')[1]
                elif 'ATTRS{idProduct}' in line:
                    product_id = line.split('"')[1]
                elif 'ATTRS{product}' in line:
                    description = line.split('"')[1]
            
            return vendor_id, product_id, description
            
        except Exception as e:
            # Fallback - try to get basic info from /sys
            try:
                # Extract device path from port
                import re
                match = re.search(r'tty(USB|ACM)(\d+)', port)
                if match:
                    return "unknown", "unknown", f"Serial device at {port}"
            except:
                pass
            
            return "unknown", "unknown", f"Unknown device at {port}"
    
    def _test_arduino_at_port(self, port: str, baud_rate: int) -> bool:
        """
        Test if an Arduino is present at the given port and baud rate
        
        Args:
            port: Serial port to test
            baud_rate: Baud rate to test
            
        Returns:
            True if Arduino detected
        """
        try:
            with serial.Serial(port, baud_rate, timeout=self.full_discovery_timeout) as ser:
                # Wait for Arduino to reset and settle
                time.sleep(1.0)
                
                # Clear buffers
                ser.flushInput()
                ser.flushOutput()
                
                # Look for Arduino-like data
                start_time = time.time()
                while time.time() - start_time < self.full_discovery_timeout:
                    if ser.in_waiting > 0:
                        try:
                            data = ser.readline().decode('utf-8', errors='ignore').strip()
                            
                            # Check for JSON data (our Arduino protocol)
                            if data.startswith('{') and data.endswith('}'):
                                try:
                                    json.loads(data)
                                    return True
                                except json.JSONDecodeError:
                                    pass
                            
                            # Check for Arduino-specific strings
                            data_lower = data.lower()
                            for pattern in self.arduino_detection_patterns:
                                if pattern.lower() in data_lower:
                                    return True
                                    
                        except Exception:
                            continue
                    
                    time.sleep(0.01)
                
                return False
                
        except Exception:
            return False
    
    def _update_arduino_reference(self):
        """Update the arduino_device reference"""
        for device in self.known_devices.values():
            if device.device_type == 'arduino' and device.is_connected:
                self.arduino_device = device
                return
        
        self.arduino_device = None
    
    def get_arduino_port(self) -> Optional[str]:
        """
        Get the current Arduino port
        
        Returns:
            Arduino port string if available, None otherwise
        """
        if self.arduino_device and self.arduino_device.is_connected:
            return self.arduino_device.port
        return None
    
    def get_arduino_baud(self) -> int:
        """
        Get the Arduino baud rate
        
        Returns:
            Arduino baud rate, default 115200 if not found
        """
        if self.arduino_device:
            return self.arduino_device.baud_rate
        return 115200
    
    def reconnect_devices(self) -> bool:
        """
        Attempt to reconnect missing devices
        
        Returns:
            True if all devices reconnected successfully
        """
        print("🔄 Attempting device reconnection...")
        
        # Perform full discovery
        self.discover_devices()
        
        # Check if critical devices are found
        if self.arduino_device and self.arduino_device.is_connected:
            print("✅ Arduino reconnected successfully")
            return True
        else:
            print("❌ Arduino not found after reconnection attempt")
            return False
    
    def save_device_config(self):
        """Save current device configuration to file"""
        try:
            config_data = {}
            for device_id, device in self.known_devices.items():
                config_data[device_id] = asdict(device)
            
            with open(self.config_file, 'w') as f:
                json.dump(config_data, f, indent=2)
            
            print(f"💾 Device configuration saved to {self.config_file}")
            
        except Exception as e:
            print(f"⚠️ Failed to save device config: {e}")
    
    def load_device_config(self) -> bool:
        """
        Load device configuration from file
        
        Returns:
            True if config loaded successfully
        """
        try:
            if not os.path.exists(self.config_file):
                print("📋 No existing device configuration found")
                return False
            
            with open(self.config_file, 'r') as f:
                config_data = json.load(f)
            
            self.known_devices = {}
            for device_id, device_data in config_data.items():
                device = USBDevice(**device_data)
                device.is_connected = False  # Will be validated
                self.known_devices[device_id] = device
            
            self._update_arduino_reference()
            print(f"📋 Loaded {len(self.known_devices)} devices from config")
            return True
            
        except Exception as e:
            print(f"⚠️ Failed to load device config: {e}")
            return False
    
    def get_device_status(self) -> Dict[str, any]:
        """
        Get comprehensive device status
        
        Returns:
            Dictionary with device status information
        """
        status = {
            'total_devices': len(self.known_devices),
            'connected_devices': sum(1 for d in self.known_devices.values() if d.is_connected),
            'arduino_connected': self.arduino_device is not None and self.arduino_device.is_connected,
            'arduino_port': self.get_arduino_port(),
            'arduino_baud': self.get_arduino_baud(),
            'devices': {}
        }
        
        for device_id, device in self.known_devices.items():
            status['devices'][device_id] = {
                'type': device.device_type,
                'port': device.port,
                'connected': device.is_connected,
                'description': device.description,
                'last_seen': device.last_seen
            }
        
        return status
    
    def print_device_status(self):
        """Print current device status"""
        status = self.get_device_status()
        
        print("🔌 === USB Device Status ===")
        print(f"   Total devices: {status['total_devices']}")
        print(f"   Connected: {status['connected_devices']}")
        print(f"   Arduino: {'✅' if status['arduino_connected'] else '❌'} ({status['arduino_port']})")
        
        for device_id, device_info in status['devices'].items():
            status_icon = '✅' if device_info['connected'] else '❌'
            print(f"   {status_icon} {device_info['type']}: {device_info['port']} - {device_info['description']}")
        
        print("=" * 30)


# Example usage and testing
if __name__ == "__main__":
    print("🧪 Testing USB Device Manager...")
    
    # Create device manager
    manager = USBDeviceManager()
    
    # Load existing configuration
    manager.load_device_config()
    
    # Try quick validation first
    if not manager.quick_device_check():
        # Quick check failed - do full discovery
        manager.discover_devices()
        
        # Save the new configuration
        manager.save_device_config()
    
    # Print status
    manager.print_device_status()
    
    # Test Arduino connection if found
    arduino_port = manager.get_arduino_port()
    if arduino_port:
        print(f"🤖 Arduino found at {arduino_port}")
        print("   Ready for rover operations!")
    else:
        print("❌ No Arduino found")
        print("   Please check connections and try again")
    
    print("\n🎯 USB Device Manager test complete")