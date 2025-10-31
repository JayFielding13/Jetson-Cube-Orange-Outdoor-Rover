#!/usr/bin/env python3
"""
Bluetooth RSSI-based Navigator for Rover
Tracks BlueCharm BLE beacon using dual USB Bluetooth dongles
"""

import asyncio
import time
import logging
import statistics
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum
import math
from collections import deque
import subprocess
import re

try:
    from bleak import BleakScanner, BleakClient
    from bleak.backends.device import BLEDevice
    from bleak.backends.scanner import AdvertisementData
except ImportError:
    print("Warning: bleak not installed. Install with: pip install bleak")

# Configure logging
logger = logging.getLogger(__name__)

class NavigationState(Enum):
    SEARCHING = "searching"
    TRACKING = "tracking"
    APPROACHING = "approaching"
    MAINTAINING = "maintaining"
    SIGNAL_LOST = "signal_lost"
    STOPPED = "stopped"

@dataclass
class RSSIReading:
    rssi: int
    timestamp: float
    adapter_id: str
    device_address: str

@dataclass
class BeaconData:
    left_rssi: Optional[int] = None
    right_rssi: Optional[int] = None
    left_readings: deque = field(default_factory=lambda: deque(maxlen=10))
    right_readings: deque = field(default_factory=lambda: deque(maxlen=10))
    last_seen: float = 0.0
    estimated_distance: Optional[float] = None
    signal_strength: float = 0.0

@dataclass
class NavigationCommand:
    forward_speed: int = 0      # -100 to 100
    turn_rate: int = 0          # -100 to 100 (negative = left, positive = right)
    command_type: str = "stop"  # "stop", "approach", "maintain", "turn", "search"

class BluetoothNavigator:
    """Bluetooth RSSI-based navigation system"""
    
    def __init__(self, target_mac: str = None, target_distance: float = 2.5):
        # Target beacon configuration
        self.target_mac = target_mac.lower() if target_mac else None
        self.target_distance = target_distance  # meters
        self.distance_tolerance = 0.75  # +/- meters
        
        # Distance thresholds
        self.min_distance = self.target_distance - self.distance_tolerance
        self.max_distance = self.target_distance + self.distance_tolerance
        
        # RSSI to distance calibration (needs calibration for your beacon)
        self.rssi_reference = -59  # RSSI at 1 meter
        self.path_loss_exponent = 2.0  # Environment dependent (2.0 for free space)
        
        # Navigation parameters
        self.min_rssi_threshold = -85  # Below this, consider signal lost
        self.rssi_difference_threshold = 5  # Minimum difference to turn
        self.signal_timeout = 3.0  # Seconds before signal considered lost
        self.weak_signal_threshold = -75  # Below this, consider signal weak
        
        # State tracking
        self.navigation_state = NavigationState.SEARCHING
        self.beacon_data = BeaconData()
        self.last_command = NavigationCommand()
        
        # Bluetooth adapters
        self.left_adapter = "hci0"   # First USB dongle
        self.right_adapter = "hci1"  # Second USB dongle
        self.scanners = {}
        
        # Movement smoothing
        self.command_history = deque(maxlen=5)
        self.gentle_stop_steps = 10  # Steps to gradually reduce speed
        self.current_stop_step = 0
        
        logger.info(f"Bluetooth Navigator initialized - Target: {target_mac}, Distance: {target_distance}m")
    
    def estimate_distance(self, rssi: int) -> float:
        """Estimate distance from RSSI using log-distance path loss model"""
        if rssi >= self.rssi_reference:
            return 0.1  # Very close
        
        # Distance = 10^((RSSI_ref - RSSI) / (10 * n))
        distance = math.pow(10, (self.rssi_reference - rssi) / (10 * self.path_loss_exponent))
        return max(0.1, min(50.0, distance))  # Clamp between 0.1m and 50m
    
    def update_beacon_data(self, rssi: int, adapter_id: str, device_address: str):
        """Update beacon data with new RSSI reading"""
        current_time = time.time()
        reading = RSSIReading(rssi, current_time, adapter_id, device_address)
        
        # Update appropriate adapter readings
        if adapter_id == self.left_adapter:
            self.beacon_data.left_rssi = rssi
            self.beacon_data.left_readings.append(reading)
        elif adapter_id == self.right_adapter:
            self.beacon_data.right_rssi = rssi
            self.beacon_data.right_readings.append(reading)
        
        # Update last seen time
        self.beacon_data.last_seen = current_time
        
        # Calculate estimated distance using strongest signal
        max_rssi = max(
            self.beacon_data.left_rssi or -100,
            self.beacon_data.right_rssi or -100
        )
        
        if max_rssi > -100:
            self.beacon_data.estimated_distance = self.estimate_distance(max_rssi)
            self.beacon_data.signal_strength = max_rssi
    
    def get_smoothed_rssi(self, readings: deque) -> Optional[int]:
        """Get smoothed RSSI from recent readings"""
        if not readings:
            return None
        
        # Use recent readings (last 3 seconds)
        current_time = time.time()
        recent_readings = [r for r in readings if current_time - r.timestamp < 3.0]
        
        if not recent_readings:
            return None
        
        # Return median of recent readings for noise reduction
        rssi_values = [r.rssi for r in recent_readings]
        return int(statistics.median(rssi_values))
    
    def calculate_navigation_command(self) -> NavigationCommand:
        """Calculate navigation command based on current beacon data"""
        current_time = time.time()
        
        # Check for signal loss
        if current_time - self.beacon_data.last_seen > self.signal_timeout:
            self.navigation_state = NavigationState.SIGNAL_LOST
            return self._handle_signal_lost()
        
        # Get smoothed RSSI values
        left_rssi = self.get_smoothed_rssi(self.beacon_data.left_readings)
        right_rssi = self.get_smoothed_rssi(self.beacon_data.right_readings)
        
        # Check if we have any signal
        if left_rssi is None and right_rssi is None:
            self.navigation_state = NavigationState.SEARCHING
            return self._handle_searching()
        
        # Check signal strength
        max_rssi = max(left_rssi or -100, right_rssi or -100)
        if max_rssi < self.min_rssi_threshold:
            self.navigation_state = NavigationState.SIGNAL_LOST
            return self._handle_signal_lost()
        
        # We have signal - determine navigation action
        distance = self.beacon_data.estimated_distance
        
        if distance is None:
            self.navigation_state = NavigationState.SEARCHING
            return self._handle_searching()
        
        # Distance-based navigation
        if distance < self.min_distance:
            self.navigation_state = NavigationState.MAINTAINING
            return self._handle_too_close(left_rssi, right_rssi)
        elif distance > self.max_distance:
            self.navigation_state = NavigationState.APPROACHING
            return self._handle_approach(left_rssi, right_rssi)
        else:
            self.navigation_state = NavigationState.MAINTAINING
            return self._handle_maintain_distance(left_rssi, right_rssi)
    
    def _handle_signal_lost(self) -> NavigationCommand:
        """Handle signal loss with gentle stop"""
        if self.current_stop_step < self.gentle_stop_steps:
            # Gradually reduce speed
            remaining_ratio = (self.gentle_stop_steps - self.current_stop_step) / self.gentle_stop_steps
            forward_speed = int(self.last_command.forward_speed * remaining_ratio * 0.5)
            self.current_stop_step += 1
            
            command = NavigationCommand(
                forward_speed=forward_speed,
                turn_rate=0,
                command_type="gentle_stop"
            )
        else:
            # Complete stop
            command = NavigationCommand(
                forward_speed=0,
                turn_rate=0,
                command_type="stop"
            )
        
        logger.warning("Signal lost - executing gentle stop")
        return command
    
    def _handle_searching(self) -> NavigationCommand:
        """Handle searching for beacon"""
        # Slow rotation to find beacon
        command = NavigationCommand(
            forward_speed=0,
            turn_rate=30,  # Slow right turn
            command_type="search"
        )
        
        logger.info("Searching for beacon...")
        return command
    
    def _handle_approach(self, left_rssi: Optional[int], right_rssi: Optional[int]) -> NavigationCommand:
        """Handle approaching the beacon"""
        self.current_stop_step = 0  # Reset gentle stop counter
        
        # Determine turning based on RSSI difference
        turn_rate = self._calculate_turn_rate(left_rssi, right_rssi)
        
        # Calculate forward speed based on distance
        distance = self.beacon_data.estimated_distance
        if distance > 10:
            forward_speed = 60  # Fast approach for far distances
        elif distance > 5:
            forward_speed = 40  # Medium speed
        else:
            forward_speed = 20  # Slow approach
        
        # Reduce forward speed when turning
        if abs(turn_rate) > 20:
            forward_speed = int(forward_speed * 0.6)
        
        command = NavigationCommand(
            forward_speed=forward_speed,
            turn_rate=turn_rate,
            command_type="approach"
        )
        
        logger.info(f"Approaching beacon - Distance: {distance:.1f}m, Turn: {turn_rate}")
        return command
    
    def _handle_maintain_distance(self, left_rssi: Optional[int], right_rssi: Optional[int]) -> NavigationCommand:
        """Handle maintaining target distance"""
        self.current_stop_step = 0  # Reset gentle stop counter
        
        # Just turn to face beacon, minimal forward movement
        turn_rate = self._calculate_turn_rate(left_rssi, right_rssi)
        
        command = NavigationCommand(
            forward_speed=0,
            turn_rate=turn_rate,
            command_type="maintain"
        )
        
        logger.info(f"Maintaining distance - Turn: {turn_rate}")
        return command
    
    def _handle_too_close(self, left_rssi: Optional[int], right_rssi: Optional[int]) -> NavigationCommand:
        """Handle being too close to beacon"""
        self.current_stop_step = 0  # Reset gentle stop counter
        
        # Turn to face beacon but move backward slowly
        turn_rate = self._calculate_turn_rate(left_rssi, right_rssi)
        
        command = NavigationCommand(
            forward_speed=-15,  # Slow backward movement
            turn_rate=turn_rate,
            command_type="back_away"
        )
        
        logger.info(f"Too close - backing away, Turn: {turn_rate}")
        return command
    
    def _calculate_turn_rate(self, left_rssi: Optional[int], right_rssi: Optional[int]) -> int:
        """Calculate turn rate based on RSSI difference"""
        if left_rssi is None and right_rssi is None:
            return 0
        
        if left_rssi is None:
            return 40  # Turn right toward signal
        if right_rssi is None:
            return -40  # Turn left toward signal
        
        # Calculate difference
        rssi_diff = right_rssi - left_rssi
        
        # Only turn if difference is significant
        if abs(rssi_diff) < self.rssi_difference_threshold:
            return 0
        
        # Turn toward stronger signal
        if rssi_diff > 0:
            # Right side stronger, turn right
            turn_rate = min(50, max(20, abs(rssi_diff) * 2))
        else:
            # Left side stronger, turn left
            turn_rate = -min(50, max(20, abs(rssi_diff) * 2))
        
        return turn_rate
    
    def smooth_command(self, command: NavigationCommand) -> NavigationCommand:
        """Apply smoothing to navigation commands"""
        self.command_history.append(command)
        
        if len(self.command_history) < 3:
            return command
        
        # Average recent commands for smoothing
        recent_commands = list(self.command_history)[-3:]
        
        avg_forward = sum(cmd.forward_speed for cmd in recent_commands) // len(recent_commands)
        avg_turn = sum(cmd.turn_rate for cmd in recent_commands) // len(recent_commands)
        
        smoothed_command = NavigationCommand(
            forward_speed=avg_forward,
            turn_rate=avg_turn,
            command_type=command.command_type
        )
        
        self.last_command = smoothed_command
        return smoothed_command
    
    async def scan_for_beacon(self, adapter_id: str, duration: float = 1.0):
        """Scan for beacon on specific adapter"""
        try:
            # Set HCI device for scanning
            scanner = BleakScanner(adapter=adapter_id)
            
            def detection_callback(device: BLEDevice, advertisement_data: AdvertisementData):
                if self.target_mac is None or device.address.lower() == self.target_mac:
                    rssi = advertisement_data.rssi
                    self.update_beacon_data(rssi, adapter_id, device.address)
                    logger.debug(f"Beacon detected on {adapter_id}: RSSI={rssi}")
            
            scanner.register_detection_callback(detection_callback)
            
            await scanner.start()
            await asyncio.sleep(duration)
            await scanner.stop()
            
        except Exception as e:
            logger.error(f"Scanning error on {adapter_id}: {e}")
    
    async def run_navigation_cycle(self):
        """Run one navigation cycle"""
        try:
            # Scan both adapters simultaneously
            await asyncio.gather(
                self.scan_for_beacon(self.left_adapter, 0.5),
                self.scan_for_beacon(self.right_adapter, 0.5)
            )
            
            # Calculate navigation command
            command = self.calculate_navigation_command()
            
            # Apply smoothing
            smoothed_command = self.smooth_command(command)
            
            return smoothed_command
            
        except Exception as e:
            logger.error(f"Navigation cycle error: {e}")
            return NavigationCommand(command_type="error")
    
    def get_navigation_status(self) -> Dict:
        """Get current navigation status"""
        return {
            "state": self.navigation_state.value,
            "left_rssi": self.beacon_data.left_rssi,
            "right_rssi": self.beacon_data.right_rssi,
            "estimated_distance": self.beacon_data.estimated_distance,
            "signal_strength": self.beacon_data.signal_strength,
            "last_seen": self.beacon_data.last_seen,
            "target_distance": self.target_distance,
            "distance_tolerance": self.distance_tolerance
        }

# Test function
async def test_bluetooth_navigator():
    """Test the Bluetooth navigator"""
    navigator = BluetoothNavigator(target_mac="AA:BB:CC:DD:EE:FF")  # Replace with your beacon MAC
    
    print("Testing Bluetooth Navigator...")
    
    for i in range(20):
        command = await navigator.run_navigation_cycle()
        status = navigator.get_navigation_status()
        
        print(f"Cycle {i+1}: {command.command_type} - "
              f"Forward: {command.forward_speed}, Turn: {command.turn_rate}")
        print(f"  State: {status['state']}, Distance: {status['estimated_distance']}")
        
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(test_bluetooth_navigator())