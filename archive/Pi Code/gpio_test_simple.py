#!/usr/bin/env python3
"""
Simple GPIO Test - Sets GPIO pins to fixed values for testing
This bypasses all RC control and just sets motor pins to known states
"""

import RPi.GPIO as GPIO
import time

print("=== GPIO Test Program ===")
print("This will set motor control pins to fixed values")
print("Use multimeter to verify voltages:")
print("- GPIO 18 (Pin 12) should read 3.3V")
print("- GPIO 19 (Pin 35) should read 0V") 
print("- GPIO 20 (Pin 38) should read 3.3V")
print("- GPIO 21 (Pin 40) should read 0V")
print("- GPIO 16 (Pin 36) should read 3.3V")
print("- GPIO 26 (Pin 37) should read 3.3V")
print()

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup([18, 19, 20, 21, 16, 26], GPIO.OUT)

# Set motor pins for "both motors forward"
GPIO.output(18, GPIO.HIGH)  # Left motor forward
GPIO.output(19, GPIO.LOW)   # Left motor reverse (off)
GPIO.output(20, GPIO.HIGH)  # Right motor forward  
GPIO.output(21, GPIO.LOW)   # Right motor reverse (off)
GPIO.output(16, GPIO.HIGH)  # Left motor enable
GPIO.output(26, GPIO.HIGH)  # Right motor enable

print("GPIO pins set! Motors should be spinning forward.")
print("If motors don't spin, check:")
print("1. Motor controller has both 12V power AND 5V logic power")
print("2. All GPIO wires are connected properly")
print("3. Common ground between Pi and motor controller")
print()
print("Press Ctrl+C to stop and clean up GPIO")

try:
    while True:
        print("GPIO 18=HIGH, 19=LOW, 20=HIGH, 21=LOW, 16=HIGH, 26=HIGH")
        time.sleep(2)
except KeyboardInterrupt:
    print("\nStopping GPIO test...")
    GPIO.cleanup()
    print("GPIO cleaned up. Test complete.")