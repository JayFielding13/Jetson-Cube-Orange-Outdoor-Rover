#!/usr/bin/env python3
"""
Configuration file for Jetson Simple Rover
Centralized parameter tuning and hardware configuration
"""

# ============================================================================
# GPIO PIN ASSIGNMENTS (Jetson 40-pin header, BOARD numbering)
# ============================================================================
# IMPORTANT: Use GPIO.setmode(GPIO.BOARD) - these are PHYSICAL pin numbers!
#
# Front sensors (angled 30° outward from center)
LEFT_FRONT_TRIG = 26    # Physical Pin 26 (GPIO 7)
LEFT_FRONT_ECHO = 24    # Physical Pin 24 (GPIO 8) - MUST use voltage divider!
RIGHT_FRONT_TRIG = 21   # Physical Pin 21 (GPIO 9)
RIGHT_FRONT_ECHO = 19   # Physical Pin 19 (GPIO 10) - MUST use voltage divider!

# Side sensors (perpendicular to rover body)
LEFT_SIDE_TRIG = 23     # Physical Pin 23 (GPIO 11)
LEFT_SIDE_ECHO = 32     # Physical Pin 32 (GPIO 12) - MUST use voltage divider!
RIGHT_SIDE_TRIG = 33    # Physical Pin 33 (GPIO 13)
RIGHT_SIDE_ECHO = 37    # Physical Pin 37 (GPIO 26) - MUST use voltage divider!

# ============================================================================
# SERIAL PORT CONFIGURATION
# ============================================================================

BAUD_RATE = 115200

# Device identification hints
ARDUINO_MODEL_ID = "CH340"  # Arduino Nano with CH340 USB chip
HELTEC_MODEL_ID = "CP210"   # Heltec with CP2102 USB chip

# ============================================================================
# NAVIGATION PARAMETERS (TUNED FROM SIMPLE ROVER)
# ============================================================================

# Distance Thresholds (centimeters)
CRITICAL_THRESHOLD = 10         # Emergency stop - immediate avoidance required
OBSTACLE_THRESHOLD = 50         # Start reacting to obstacle
SENSOR_FUSION_DIFF = 8          # Max difference for center obstacle detection
SIDE_WARNING_THRESHOLD = 10     # Distance to trigger side wall steering
SAFE_DISTANCE = 100             # Distance considered "clear"

# Movement Speeds (0-255 PWM range)
DEFAULT_SPEED = 150             # Normal forward cruising speed
SLOW_SPEED = 75                 # Cautious forward speed near obstacles
TURN_SPEED = 120                # Speed during turning maneuvers
BACKUP_SPEED = 120              # Speed when reversing
GENTLE_TURN_SPEED = 90          # Speed for gentle side-wall steering

# Timing Parameters (seconds)
SENSOR_INTERVAL = 0.1           # Time between sensor readings
BACKUP_DURATION = 0.5           # How long to backup before turning
TURN_DURATION = 0.7             # How long to turn during avoidance
GENTLE_STEER_DURATION = 0.3     # Duration of gentle steering corrections

# ============================================================================
# SENSOR CONFIGURATION
# ============================================================================

# Ultrasonic sensor parameters
ULTRASONIC_MAX_DISTANCE = 400   # Maximum measurable distance (cm)
ULTRASONIC_TIMEOUT = 0.05       # Sensor timeout (seconds)
ULTRASONIC_MIN_DISTANCE = 2     # Minimum valid distance (cm)

# Sensor averaging (for noise reduction)
SENSOR_SAMPLES = 1              # Number of samples to average (1 = no averaging)

# ============================================================================
# LORA CONTROL CONFIGURATION
# ============================================================================

LORA_MODES = ["STOP", "AUTONOMOUS"]
DEFAULT_MODE = "STOP"           # Always start in safe mode

# ============================================================================
# ARDUINO MOTOR CONTROLLER COMMANDS
# ============================================================================

# Command format sent to Arduino
MOTOR_CMD_STOP = "S"            # Stop all motors
MOTOR_CMD_FORWARD = "F"         # Forward (append speed)
MOTOR_CMD_BACKWARD = "B"        # Backward (append speed)
MOTOR_CMD_LEFT = "L"            # Turn left (append speed)
MOTOR_CMD_RIGHT = "R"           # Turn right (append speed)

# Command timeout
MOTOR_CMD_TIMEOUT = 1.0         # Seconds to wait for Arduino response

# ============================================================================
# VISION CONFIGURATION (PHASE 2 - FUTURE)
# ============================================================================

# Camera settings
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# Object detection
YOLO_MODEL = "yolov8n.pt"       # Nano model for speed
YOLO_CONFIDENCE = 0.5           # Detection confidence threshold
YOLO_IOU = 0.45                 # NMS IoU threshold

# Vision processing
VISION_ENABLED = False          # Enable vision processing (Phase 2)
VISION_PRIORITY = "safety"      # "safety" or "exploration"

# ============================================================================
# DEBUGGING AND LOGGING
# ============================================================================

DEBUG_MODE = False              # Enable verbose debug output
LOG_SENSOR_DATA = False         # Log all sensor readings to file
DISPLAY_REFRESH_RATE = 0.5      # Seconds between status updates

# Console output
SHOW_SENSOR_READINGS = True     # Display sensor values in console
SHOW_NAVIGATION_STATE = True    # Display current navigation decision
SHOW_MOTOR_COMMANDS = False     # Display motor commands sent

# ============================================================================
# SAFETY PARAMETERS
# ============================================================================

# Emergency stop conditions
ENABLE_EMERGENCY_STOP = True
EMERGENCY_DISTANCE = 5          # Distance for absolute emergency stop (cm)

# Watchdog timers
ARDUINO_TIMEOUT = 5.0           # Max time without Arduino response (seconds)
LORA_TIMEOUT = 30.0             # Max time without LoRa message (seconds)

# Automatic safety shutdown
AUTO_STOP_ON_ERROR = True       # Stop motors if critical error occurs
MAX_CONSECUTIVE_ERRORS = 10     # Stop after this many sensor errors

# ============================================================================
# PERFORMANCE TUNING
# ============================================================================

# Navigation loop timing
NAVIGATION_LOOP_RATE = 10       # Hz (10 = 100ms per loop)

# Threading
USE_THREADED_SENSORS = False    # Read sensors in parallel (future optimization)

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def get_gpio_pins():
    """Return dictionary of all GPIO pin assignments"""
    return {
        'left_front': {'trig': LEFT_FRONT_TRIG, 'echo': LEFT_FRONT_ECHO},
        'right_front': {'trig': RIGHT_FRONT_TRIG, 'echo': RIGHT_FRONT_ECHO},
        'left_side': {'trig': LEFT_SIDE_TRIG, 'echo': LEFT_SIDE_ECHO},
        'right_side': {'trig': RIGHT_SIDE_TRIG, 'echo': RIGHT_SIDE_ECHO}
    }

def get_thresholds():
    """Return dictionary of navigation thresholds"""
    return {
        'critical': CRITICAL_THRESHOLD,
        'obstacle': OBSTACLE_THRESHOLD,
        'fusion_diff': SENSOR_FUSION_DIFF,
        'side_warning': SIDE_WARNING_THRESHOLD,
        'safe': SAFE_DISTANCE
    }

def get_speeds():
    """Return dictionary of movement speeds"""
    return {
        'default': DEFAULT_SPEED,
        'slow': SLOW_SPEED,
        'turn': TURN_SPEED,
        'backup': BACKUP_SPEED,
        'gentle_turn': GENTLE_TURN_SPEED
    }

def print_config():
    """Print current configuration for verification"""
    print("="*60)
    print("JETSON SIMPLE ROVER - CONFIGURATION")
    print("="*60)
    print("\nGPIO Pins:")
    for sensor, pins in get_gpio_pins().items():
        print(f"  {sensor:12s}: TRIG=GPIO{pins['trig']}, ECHO=GPIO{pins['echo']}")

    print("\nNavigation Thresholds:")
    for name, value in get_thresholds().items():
        print(f"  {name:12s}: {value}cm")

    print("\nMovement Speeds:")
    for name, value in get_speeds().items():
        print(f"  {name:12s}: {value}")

    print("\nSerial Configuration:")
    print(f"  Baud Rate: {BAUD_RATE}")

    print("\nSafety Settings:")
    print(f"  Emergency Stop: {ENABLE_EMERGENCY_STOP}")
    print(f"  Emergency Distance: {EMERGENCY_DISTANCE}cm")
    print(f"  Default Mode: {DEFAULT_MODE}")

    print("\nVision System:")
    print(f"  Enabled: {VISION_ENABLED}")
    if VISION_ENABLED:
        print(f"  Camera: {CAMERA_WIDTH}x{CAMERA_HEIGHT} @ {CAMERA_FPS}fps")
        print(f"  Model: {YOLO_MODEL}")

    print("="*60)

if __name__ == "__main__":
    # Print configuration when run directly
    print_config()
