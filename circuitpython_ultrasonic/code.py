"""
CircuitPython Ultrasonic Sensor Array Firmware
ESP32 DevKit with 6x AJ-SR04M Waterproof Ultrasonic Sensors

This is a CircuitPython version of the Arduino ultrasonic firmware.
Outputs the same JSON format for compatibility with visualization tools.

Pin Assignments (after level shifter for ECHO pins):
- Front:        GPIO 15 (TRIG), GPIO 2 (ECHO)
- Corner Left:  GPIO 13 (TRIG), GPIO 12 (ECHO)
- Corner Right: GPIO 5 (TRIG), GPIO 27 (ECHO)
- Side Left:    GPIO 14 (TRIG), GPIO 4 (ECHO)
- Side Right:   GPIO 32 (TRIG), GPIO 33 (ECHO)
- Rear:         GPIO 25 (TRIG), GPIO 26 (ECHO)

Author: Jay
Date: November 9, 2025
"""

import board
import digitalio
import time
import json
import pulseio

# Configuration
DEBUG_MODE = False  # Set True for human-readable output
READ_INTERVAL = 0.1  # 10 Hz (100ms between readings)
ULTRASONIC_TIMEOUT = 0.038  # 38ms timeout (6.5m max range)
MAX_DISTANCE = 6.0  # meters
SPEED_OF_SOUND = 343.0  # m/s at 20°C

# Sensor definitions
class UltrasonicSensor:
    def __init__(self, name, angle, trig_pin, echo_pin):
        self.name = name
        self.angle = angle
        self.distance = 0.0
        self.valid = False

        # Setup trigger pin (output)
        self.trig = digitalio.DigitalInOut(trig_pin)
        self.trig.direction = digitalio.Direction.OUTPUT
        self.trig.value = False

        # Setup echo pin (input)
        self.echo = digitalio.DigitalInOut(echo_pin)
        self.echo.direction = digitalio.Direction.INPUT

    def read(self):
        """Read distance from sensor"""
        # Send 10us trigger pulse
        self.trig.value = False
        time.sleep(0.000002)  # 2us low
        self.trig.value = True
        time.sleep(0.00001)  # 10us high
        self.trig.value = False

        # Wait for echo to go high (with timeout)
        timeout_start = time.monotonic()
        while not self.echo.value:
            if (time.monotonic() - timeout_start) > ULTRASONIC_TIMEOUT:
                self.distance = MAX_DISTANCE
                self.valid = False
                return

        # Measure pulse width
        pulse_start = time.monotonic()

        while self.echo.value:
            if (time.monotonic() - pulse_start) > ULTRASONIC_TIMEOUT:
                self.distance = MAX_DISTANCE
                self.valid = False
                return

        pulse_end = time.monotonic()
        pulse_duration = pulse_end - pulse_start

        # Calculate distance in meters
        # Distance = (time * speed_of_sound) / 2
        distance_m = (pulse_duration * SPEED_OF_SOUND) / 2

        # Validate range
        if distance_m < 0.02 or distance_m > MAX_DISTANCE:
            self.distance = MAX_DISTANCE
            self.valid = False
        else:
            self.distance = distance_m
            self.valid = True

    def to_dict(self):
        """Convert to dictionary for JSON output"""
        return {
            'name': self.name,
            'angle': self.angle,
            'distance': round(self.distance, 3),
            'valid': self.valid
        }


# Initialize sensors
sensors = [
    UltrasonicSensor('front', 0, board.IO15, board.IO2),
    UltrasonicSensor('corner_left', -45, board.IO13, board.IO12),
    UltrasonicSensor('corner_right', 45, board.IO5, board.IO27),
    UltrasonicSensor('side_left', -90, board.IO14, board.IO4),
    UltrasonicSensor('side_right', 90, board.IO33, board.IO32),  # SWAPPED to test wiring
    UltrasonicSensor('rear', 180, board.IO25, board.IO26),
]

# Startup message
if DEBUG_MODE:
    print("=" * 60)
    print("CircuitPython Ultrasonic Sensor Array")
    print("DEBUG MODE - Hardware Validation")
    print("=" * 60)
    print("Sensors initialized:")
    for sensor in sensors:
        print(f"  {sensor.name} ({sensor.angle:+4d}°): TRIG={sensor.trig}, ECHO={sensor.echo}")
    print("=" * 60)
    print()
else:
    status = {
        'status': 'ready',
        'sensors': len(sensors),
        'mode': 'raw',
        'firmware': 'circuitpython'
    }
    print(json.dumps(status))

# Small delay for sensor stabilization
time.sleep(0.1)

# Main loop
last_read_time = 0

while True:
    current_time = time.monotonic()

    # Read sensors at specified interval
    if current_time - last_read_time >= READ_INTERVAL:
        last_read_time = current_time

        # Read all sensors
        for sensor in sensors:
            sensor.read()
            time.sleep(0.01)  # Small delay to avoid cross-talk

        # Output results
        if DEBUG_MODE:
            # Human-readable output
            print("-" * 60)
            print(f"Time: {current_time:.1f} s")
            print()

            for sensor in sensors:
                status_str = f"{sensor.distance:.2f} m" if sensor.valid else "TIMEOUT / OUT OF RANGE"
                print(f"  {sensor.name:15} ({sensor.angle:+4d}°): {status_str}")

            print()

        else:
            # JSON output for Jetson
            output = {
                'timestamp': int(current_time * 1000),  # milliseconds
                'sensors': [sensor.to_dict() for sensor in sensors]
            }
            print(json.dumps(output))

    # Small delay to prevent busy-waiting
    time.sleep(0.001)
