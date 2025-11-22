#!/usr/bin/env python3
"""
Test MAVLink connection to Cube Orange
This script will try to connect and receive a heartbeat
"""

from pymavlink import mavutil
import time
import sys

def test_connection(port, baud):
    """Test MAVLink connection on specified port"""
    print(f"Testing connection on {port} at {baud} baud...")

    try:
        # Create connection
        mav = mavutil.mavlink_connection(port, baud=baud)
        print(f"✓ Serial port opened: {port}")

        # Wait for heartbeat with timeout
        print("Waiting for heartbeat (timeout: 10 seconds)...")
        msg = mav.wait_heartbeat(timeout=10)

        if msg:
            print(f"✓ Heartbeat received!")
            print(f"  System ID: {mav.target_system}")
            print(f"  Component ID: {mav.target_component}")
            print(f"  MAVLink version: {msg.mavlink_version}")
            print(f"  Autopilot: {msg.autopilot}")
            print(f"  Vehicle type: {msg.type}")
            print(f"  Base mode: {msg.base_mode}")
            print(f"  System status: {msg.system_status}")

            # Check if armed
            armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            print(f"  Armed: {armed}")

            return True
        else:
            print("✗ No heartbeat received")
            return False

    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def main():
    print("=" * 60)
    print("Cube Orange MAVLink Connection Test")
    print("=" * 60)

    # Common ports and baud rates to try
    ports_to_test = [
        ('/dev/ttyACM0', 57600),
        ('/dev/ttyACM0', 115200),
        ('/dev/ttyACM1', 57600),
        ('/dev/ttyACM1', 115200),
    ]

    for port, baud in ports_to_test:
        print()
        if test_connection(port, baud):
            print(f"\n✓ SUCCESS! Use: {port} at {baud} baud")
            return
        print(f"✗ Failed on {port} at {baud}")

    print("\n✗ Could not establish connection on any port")
    print("\nTroubleshooting:")
    print("  1. Ensure Cube Orange is powered on")
    print("  2. Check USB cable connection")
    print("  3. Verify user is in dialout group: groups")
    print("  4. Check available ports: ls -la /dev/ttyACM*")
    print("  5. Try logging out and back in after adding to dialout group")


if __name__ == '__main__':
    main()
