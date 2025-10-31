#!/usr/bin/env python3
"""
Obstacle-Avoidance Rover with LoRa Cutoff Switch - Raspberry Pi Controller

Hardware:
- Raspberry Pi 4
- 2× HC-SR04 Ultrasonic Sensors (30° outward angle, 5" separation)
- Arduino Nano (motor controller via USB)
- Heltec LoRa WiFi 32 V3 (receiver for remote cutoff switch)

GPIO Connections:
- Left Sensor: TRIG=GPIO23, ECHO=GPIO24 (with voltage divider)
- Right Sensor: TRIG=GPIO17, ECHO=GPIO27 (with voltage divider)

Serial Connections:
- Arduino: /dev/ttyUSB0 or /dev/ttyACM0 (motor control)
- Heltec: /dev/ttyUSB1 or /dev/ttyACM1 (LoRa commands)

Behavior:
- Listens for LoRa commands (STOP/AUTONOMOUS)
- STOP mode: Motors disabled, no movement
- AUTONOMOUS mode: Full obstacle avoidance navigation
- Safety: Always starts in STOP mode
"""

import RPi.GPIO as GPIO
import serial
import time
import sys
import glob
from threading import Thread, Event
import json

# GPIO Pin Assignments
LEFT_TRIG = 23
LEFT_ECHO = 24
RIGHT_TRIG = 17
RIGHT_ECHO = 27

# Serial Configuration
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


class LoRaListener:
    """Listens for LoRa commands from Heltec receiver"""

    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.current_mode = "STOP"  # Safe default
        self.running = Event()
        self.mode_callback = None
        self.last_rssi = 0

    def connect(self):
        """Connect to Heltec receiver"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1
            )
            time.sleep(2)  # Wait for board to initialize
            print(f"✓ Connected to Heltec LoRa receiver on {self.port}")
            return True
        except serial.SerialException as e:
            print(f"✗ ERROR: Could not connect to Heltec: {e}")
            return False

    def set_mode_callback(self, callback):
        """Set callback function for mode changes"""
        self.mode_callback = callback

    def parse_message(self, line):
        """Parse incoming LoRa message"""
        line = line.strip()

        # Try JSON format first
        if line.startswith('{'):
            try:
                data = json.loads(line)
                if 'mode' in data:
                    if 'rssi' in data:
                        self.last_rssi = data['rssi']
                    return data['mode']
            except json.JSONDecodeError:
                pass

        # Try simple format: MODE:value
        if line.startswith('MODE:'):
            mode = line.split(':', 1)[1].strip()
            return mode

        return None

    def listen_loop(self):
        """Background thread listening for LoRa commands"""
        print("LoRa listener started...")

        while self.running.is_set():
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore')

                    # Parse for mode commands
                    mode = self.parse_message(line)
                    if mode and mode in ["STOP", "AUTONOMOUS"]:
                        if mode != self.current_mode:
                            self.current_mode = mode
                            print(f"\n{'='*50}")
                            print(f"   LoRa MODE CHANGE: {mode}")
                            print(f"   Signal: {self.last_rssi} dBm")
                            print(f"{'='*50}\n")

                            # Call callback if set
                            if self.mode_callback:
                                self.mode_callback(mode)

                time.sleep(0.01)

            except Exception as e:
                print(f"LoRa listener error: {e}")
                time.sleep(0.1)

    def start(self):
        """Start listening in background thread"""
        self.running.set()
        thread = Thread(target=self.listen_loop, daemon=True)
        thread.start()
        return thread

    def stop(self):
        """Stop listening"""
        self.running.clear()
        if self.serial_conn:
            self.serial_conn.close()


class RoverController:
    """Main controller for obstacle-avoidance rover with LoRa cutoff"""

    def __init__(self):
        self.motor_serial = None
        self.lora_listener = None
        self.left_sensor = None
        self.right_sensor = None
        self.running = False
        self.autonomous_enabled = False  # Controlled by LoRa

    def find_serial_ports(self):
        """Auto-detect Arduino and Heltec serial ports"""
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')

        if len(ports) < 2:
            print(f"ERROR: Need 2 serial devices, found {len(ports)}")
            print("Make sure both Arduino and Heltec are connected via USB")
            return None, None

        print(f"\nFound {len(ports)} serial devices:")
        for port in ports:
            print(f"  - {port}")

        # Ask user to identify which is which
        print("\nWhich device is the Arduino (motor controller)?")
        for i, port in enumerate(ports):
            print(f"  {i+1}. {port}")

        try:
            choice = int(input("Enter number (1 or 2): "))
            arduino_port = ports[choice - 1]
            heltec_port = ports[1 - (choice - 1)]  # The other one

            print(f"\n✓ Arduino (motors): {arduino_port}")
            print(f"✓ Heltec (LoRa): {heltec_port}")
            return arduino_port, heltec_port

        except (ValueError, IndexError):
            print("Invalid selection")
            return None, None

    def setup(self):
        """Initialize GPIO, serial, and LoRa listener"""
        print("="*60)
        print("  OBSTACLE-AVOIDANCE ROVER WITH LoRa CUTOFF SWITCH")
        print("="*60)
        print("\nInitializing...")

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
        print("✓ Ultrasonic sensors initialized")

        # Find and connect to serial devices
        arduino_port, heltec_port = self.find_serial_ports()

        if not arduino_port or not heltec_port:
            print("\nERROR: Could not identify serial ports")
            sys.exit(1)

        # Connect to Arduino
        try:
            self.motor_serial = serial.Serial(arduino_port, BAUD_RATE, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            print(f"✓ Connected to Arduino on {arduino_port}")
        except serial.SerialException as e:
            print(f"✗ ERROR: Could not connect to Arduino: {e}")
            sys.exit(1)

        # Connect to Heltec LoRa receiver
        self.lora_listener = LoRaListener(heltec_port, BAUD_RATE)
        if not self.lora_listener.connect():
            print("✗ ERROR: Could not connect to Heltec receiver")
            sys.exit(1)

        # Set callback for mode changes
        self.lora_listener.set_mode_callback(self.on_mode_change)

        # Start LoRa listener
        self.lora_listener.start()

        print("\n" + "="*60)
        print("  Initialization Complete!")
        print("="*60)
        print("\nSAFETY: Rover starts in STOP mode")
        print("Press button on transmitter to enable AUTONOMOUS mode")
        print("\nPress Ctrl+C to exit\n")

    def on_mode_change(self, mode):
        """Called when LoRa mode changes"""
        if mode == "AUTONOMOUS":
            self.autonomous_enabled = True
            print(">>> AUTONOMOUS MODE ENABLED - Rover can move <<<")
        else:
            self.autonomous_enabled = False
            print(">>> STOP MODE - Rover disabled <<<")
            # Immediately stop motors
            self.send_command("S")

    def send_command(self, command):
        """Send command to Arduino"""
        try:
            self.motor_serial.write(f"{command}\n".encode())
            self.motor_serial.flush()

            # Wait for acknowledgment
            response = self.motor_serial.readline().decode().strip()
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
        if not self.autonomous_enabled:
            return  # Safety check

        print(f"\nOBSTACLE! Left: {left_dist}cm, Right: {right_dist}cm")

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

        try:
            while self.running:
                # Check if autonomous mode is enabled
                if not self.autonomous_enabled:
                    # In STOP mode - just wait and show status
                    mode = self.lora_listener.current_mode
                    print(f"Mode: {mode:12s} | Waiting for AUTONOMOUS command...", end='\r')
                    time.sleep(0.5)
                    continue

                # AUTONOMOUS MODE - Run normal navigation
                # Read sensors
                left_dist, right_dist = self.read_sensors()
                min_dist = min(left_dist, right_dist)

                # Display sensor readings
                print(f"AUTONOMOUS | L: {left_dist:6.1f}cm | R: {right_dist:6.1f}cm | Min: {min_dist:6.1f}cm", end='\r')

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
        print("\nStopping motors...")
        if self.motor_serial:
            self.send_command("S")
            time.sleep(0.5)
            self.motor_serial.close()

        print("Stopping LoRa listener...")
        if self.lora_listener:
            self.lora_listener.stop()

        print("Cleaning up GPIO...")
        GPIO.cleanup()

        print("\n" + "="*60)
        print("  Shutdown Complete")
        print("="*60)


def main():
    """Entry point"""
    rover = RoverController()

    try:
        rover.setup()
        rover.run()
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rover.cleanup()


if __name__ == "__main__":
    main()
