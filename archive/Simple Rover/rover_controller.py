#!/usr/bin/env python3
"""
Obstacle-Avoidance Rover - Raspberry Pi Controller

Hardware:
- Raspberry Pi 4
- 2× HC-SR04 Ultrasonic Sensors (30° outward angle, 5" separation)
- Arduino Nano (motor controller via USB)

GPIO Connections:
- Left Sensor: TRIG=GPIO23, ECHO=GPIO24 (with voltage divider)
- Right Sensor: TRIG=GPIO17, ECHO=GPIO27 (with voltage divider)

Behavior:
- Autonomous wandering with obstacle avoidance
- Reads sensors continuously
- Makes decisions based on obstacle proximity
- Sends commands to Arduino via serial
"""

import RPi.GPIO as GPIO
import serial
import time
import sys

# GPIO Pin Assignments
LEFT_TRIG = 23
LEFT_ECHO = 24
RIGHT_TRIG = 17
RIGHT_ECHO = 27

# Serial Configuration
SERIAL_PORT = '/dev/ttyUSB0'  # Adjust if needed (may be /dev/ttyACM0)
BAUD_RATE = 115200

# Movement Parameters
DEFAULT_SPEED = 150      # Default speed (0-255)
TURN_SPEED = 120         # Speed for turning maneuvers
BACKUP_SPEED = 120       # Speed when backing up

# Distance Thresholds (in cm)
CRITICAL_DISTANCE = 20   # Stop and turn away immediately
WARNING_DISTANCE = 40    # Slow down and prepare to turn
SAFE_DISTANCE = 60       # Can continue forward

# Timing Parameters
SENSOR_INTERVAL = 0.1    # Time between sensor readings (seconds)
BACKUP_DURATION = 0.5    # How long to backup before turning (seconds)
TURN_DURATION = 0.7      # How long to turn (seconds)


class UltrasonicSensor:
    """Handles HC-SR04 ultrasonic sensor readings"""

    def __init__(self, trig_pin, echo_pin, name="Sensor"):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.name = name
        self.max_distance = 400  # Maximum measurable distance in cm
        self.timeout = 0.05      # Timeout for echo (50ms)

    def get_distance(self):
        """Read distance from sensor in centimeters"""
        # Send 10us pulse to trigger
        GPIO.output(self.trig_pin, GPIO.HIGH)
        time.sleep(0.00001)  # 10 microseconds
        GPIO.output(self.trig_pin, GPIO.LOW)

        # Wait for echo to start
        pulse_start = time.time()
        timeout_start = pulse_start
        while GPIO.input(self.echo_pin) == GPIO.LOW:
            pulse_start = time.time()
            if pulse_start - timeout_start > self.timeout:
                return self.max_distance  # Timeout = no obstacle detected

        # Wait for echo to end
        pulse_end = time.time()
        timeout_start = pulse_end
        while GPIO.input(self.echo_pin) == GPIO.HIGH:
            pulse_end = time.time()
            if pulse_end - timeout_start > self.timeout:
                return self.max_distance  # Timeout = no obstacle detected

        # Calculate distance
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # Speed of sound / 2
        distance = round(distance, 2)

        # Validate range
        if distance > self.max_distance or distance < 2:
            return self.max_distance

        return distance


class RoverController:
    """Main controller for obstacle-avoidance rover"""

    def __init__(self):
        self.serial_conn = None
        self.left_sensor = None
        self.right_sensor = None
        self.running = False

    def setup(self):
        """Initialize GPIO and serial communication"""
        print("Initializing Rover Controller...")

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Configure sensor pins
        GPIO.setup(LEFT_TRIG, GPIO.OUT)
        GPIO.setup(LEFT_ECHO, GPIO.IN)
        GPIO.setup(RIGHT_TRIG, GPIO.OUT)
        GPIO.setup(RIGHT_ECHO, GPIO.IN)

        # Initialize sensors
        self.left_sensor = UltrasonicSensor(LEFT_TRIG, LEFT_ECHO, "Left")
        self.right_sensor = UltrasonicSensor(RIGHT_TRIG, RIGHT_ECHO, "Right")

        # Connect to Arduino
        try:
            self.serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            print(f"Connected to Arduino on {SERIAL_PORT}")
        except serial.SerialException as e:
            print(f"ERROR: Could not connect to Arduino: {e}")
            print("Check that Arduino is connected and SERIAL_PORT is correct")
            sys.exit(1)

        print("Initialization complete!")

    def send_command(self, command):
        """Send command to Arduino and wait for acknowledgment"""
        try:
            self.serial_conn.write(f"{command}\n".encode())
            self.serial_conn.flush()

            # Wait for acknowledgment (with timeout)
            response = self.serial_conn.readline().decode().strip()
            if response.startswith("OK"):
                return True
            elif response.startswith("ERROR"):
                print(f"Arduino error: {response}")
                return False
        except Exception as e:
            print(f"Serial communication error: {e}")
            return False

        return True

    def read_sensors(self):
        """Read both sensors and return distances"""
        left_dist = self.left_sensor.get_distance()
        right_dist = self.right_sensor.get_distance()
        return left_dist, right_dist

    def avoid_obstacle(self, left_dist, right_dist):
        """Execute obstacle avoidance maneuver"""
        print(f"OBSTACLE! Left: {left_dist}cm, Right: {right_dist}cm")

        # Stop
        self.send_command("S")
        time.sleep(0.2)

        # Backup
        print("Backing up...")
        self.send_command(f"B{BACKUP_SPEED}")
        time.sleep(BACKUP_DURATION)

        # Decide turn direction (turn away from closer obstacle)
        if left_dist < right_dist:
            print("Turning right...")
            self.send_command(f"R{TURN_SPEED}")
        else:
            print("Turning left...")
            self.send_command(f"L{TURN_SPEED}")

        time.sleep(TURN_DURATION)

        # Stop after turn
        self.send_command("S")
        time.sleep(0.2)

    def run(self):
        """Main control loop"""
        self.running = True
        print("\nStarting autonomous navigation...")
        print("Press Ctrl+C to stop\n")

        try:
            while self.running:
                # Read sensors
                left_dist, right_dist = self.read_sensors()
                min_dist = min(left_dist, right_dist)

                # Display sensor readings
                print(f"L: {left_dist:6.1f}cm | R: {right_dist:6.1f}cm | Min: {min_dist:6.1f}cm", end='\r')

                # Decision making based on closest obstacle
                if min_dist < CRITICAL_DISTANCE:
                    # Critical: Stop and avoid
                    self.avoid_obstacle(left_dist, right_dist)

                elif min_dist < WARNING_DISTANCE:
                    # Warning: Slow down
                    reduced_speed = int(DEFAULT_SPEED * 0.5)
                    self.send_command(f"F{reduced_speed}")

                elif min_dist < SAFE_DISTANCE:
                    # Caution: Continue at moderate speed
                    moderate_speed = int(DEFAULT_SPEED * 0.75)
                    self.send_command(f"F{moderate_speed}")

                else:
                    # All clear: Full speed ahead
                    self.send_command(f"F{DEFAULT_SPEED}")

                # Wait before next reading
                time.sleep(SENSOR_INTERVAL)

        except KeyboardInterrupt:
            print("\n\nShutdown requested...")
            self.running = False

    def cleanup(self):
        """Clean up resources"""
        print("Stopping motors...")
        if self.serial_conn:
            self.send_command("S")
            time.sleep(0.5)
            self.serial_conn.close()

        print("Cleaning up GPIO...")
        GPIO.cleanup()
        print("Shutdown complete.")


def main():
    """Entry point"""
    rover = RoverController()

    try:
        rover.setup()
        rover.run()
    except Exception as e:
        print(f"\nUnexpected error: {e}")
    finally:
        rover.cleanup()


if __name__ == "__main__":
    main()
