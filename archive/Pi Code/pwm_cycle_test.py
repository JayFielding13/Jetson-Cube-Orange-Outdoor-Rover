#!/usr/bin/env python3
"""
PWM Motor Cycle Test - Hardware PWM Speed Control
Tests motor speed control from 0-100% in both directions
Uses hardware PWM pins for smooth speed control
"""

import RPi.GPIO as GPIO
import time

print("=== PWM Motor Cycle Test ===")
print("This will cycle motors through:")
print("1. Forward: 0% → 100% → 0%")
print("2. Reverse: 0% → 100% → 0%")
print("3. Repeat for both motors")
print()

# Updated GPIO Pin Assignments (using hardware PWM)
# Left Motor
LEFT_FORWARD = 18   # IN1 (direction)
LEFT_REVERSE = 23   # IN2 (direction) 
LEFT_ENABLE = 19    # ENA1 (Hardware PWM speed)

# Right Motor  
RIGHT_FORWARD = 20  # IN3 (direction)
RIGHT_REVERSE = 21  # IN4 (direction)
RIGHT_ENABLE = 12   # ENA2 (Hardware PWM speed)

print("Updated GPIO Wiring:")
print(f"Left Motor:  IN1=GPIO{LEFT_FORWARD} (Pin 12), IN2=GPIO{LEFT_REVERSE} (Pin 16), ENA1=GPIO{LEFT_ENABLE} (Pin 35)")
print(f"Right Motor: IN3=GPIO{RIGHT_FORWARD} (Pin 38), IN4=GPIO{RIGHT_REVERSE} (Pin 40), ENA2=GPIO{RIGHT_ENABLE} (Pin 32)")
print("IMPORTANT: You'll need to rewire ENA pins to use hardware PWM pins!")
print()

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup([LEFT_FORWARD, LEFT_REVERSE, LEFT_ENABLE, RIGHT_FORWARD, RIGHT_REVERSE, RIGHT_ENABLE], GPIO.OUT)

# Create hardware PWM objects
left_pwm = GPIO.PWM(LEFT_ENABLE, 1000)   # 1kHz frequency
right_pwm = GPIO.PWM(RIGHT_ENABLE, 1000) # 1kHz frequency

# Start PWM at 0%
left_pwm.start(0)
right_pwm.start(0)

def set_motor_direction(motor, direction):
    """Set motor direction: 'forward', 'reverse', or 'stop'"""
    if motor == 'left':
        if direction == 'forward':
            GPIO.output(LEFT_FORWARD, GPIO.HIGH)
            GPIO.output(LEFT_REVERSE, GPIO.LOW)
        elif direction == 'reverse':
            GPIO.output(LEFT_FORWARD, GPIO.LOW)
            GPIO.output(LEFT_REVERSE, GPIO.HIGH)
        else:  # stop
            GPIO.output(LEFT_FORWARD, GPIO.LOW)
            GPIO.output(LEFT_REVERSE, GPIO.LOW)
    
    elif motor == 'right':
        if direction == 'forward':
            GPIO.output(RIGHT_FORWARD, GPIO.HIGH)
            GPIO.output(RIGHT_REVERSE, GPIO.LOW)
        elif direction == 'reverse':
            GPIO.output(RIGHT_FORWARD, GPIO.LOW)
            GPIO.output(RIGHT_REVERSE, GPIO.HIGH)
        else:  # stop
            GPIO.output(RIGHT_FORWARD, GPIO.LOW)
            GPIO.output(RIGHT_REVERSE, GPIO.LOW)

def cycle_motor_speed(motor, direction, duration=5):
    """Cycle motor speed from 0% → 100% → 0% over duration seconds"""
    print(f"  {motor.upper()} motor {direction}: 0% → 100% → 0%")
    
    # Set direction
    set_motor_direction(motor, direction)
    
    # Ramp up 0% → 100%
    for speed in range(0, 101, 2):
        if motor == 'left':
            left_pwm.ChangeDutyCycle(speed)
        else:
            right_pwm.ChangeDutyCycle(speed)
        print(f"    {motor} {direction}: {speed}%", end='\r')
        time.sleep(duration / 100)
    
    time.sleep(0.5)  # Hold at 100% briefly
    
    # Ramp down 100% → 0%
    for speed in range(100, -1, -2):
        if motor == 'left':
            left_pwm.ChangeDutyCycle(speed)
        else:
            right_pwm.ChangeDutyCycle(speed)
        print(f"    {motor} {direction}: {speed}%", end='\r')
        time.sleep(duration / 100)
    
    print(f"    {motor} {direction}: Complete      ")
    
    # Stop direction
    set_motor_direction(motor, 'stop')
    time.sleep(1)

print("Starting motor cycle test...")
print("Press Ctrl+C to stop at any time")
print()

try:
    cycle = 1
    while True:
        print(f"=== CYCLE {cycle} ===")
        
        # Test Left Motor
        print("Testing LEFT motor:")
        cycle_motor_speed('left', 'forward', 3)
        cycle_motor_speed('left', 'reverse', 3)
        
        # Test Right Motor  
        print("Testing RIGHT motor:")
        cycle_motor_speed('right', 'forward', 3)
        cycle_motor_speed('right', 'reverse', 3)
        
        print(f"Cycle {cycle} complete. Waiting 2 seconds...\n")
        time.sleep(2)
        cycle += 1

except KeyboardInterrupt:
    print("\nStopping PWM test...")
    
    # Clean shutdown
    left_pwm.stop()
    right_pwm.stop()
    GPIO.cleanup()
    print("GPIO cleaned up. Test complete.")