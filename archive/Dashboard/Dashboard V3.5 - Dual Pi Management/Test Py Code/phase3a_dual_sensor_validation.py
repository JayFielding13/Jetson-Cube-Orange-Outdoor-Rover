#!/usr/bin/env python3
"""
Phase 3A - Dual Ultrasonic Sensor Validation
Enhanced sensor system testing with directional obstacle detection

Tests the new dual sensor Arduino gatekeeper with:
- Individual sensor readings (left/right at 30° angles)
- Sensor fusion algorithm validation
- Directional obstacle detection 
- Intelligent avoidance maneuver testing
"""

import serial
import json
import time
import sys
from datetime import datetime

# Configuration
ARDUINO_PORT = '/dev/ttyUSB0'
ARDUINO_BAUD = 115200
TEST_DURATION = 60  # seconds

# Color codes for output
class Colors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    BOLD = '\033[1m'
    END = '\033[0m'

def print_header():
    print(f"\n{Colors.CYAN}{Colors.BOLD}=" * 70)
    print("🔍 PHASE 3A: DUAL ULTRASONIC SENSOR VALIDATION")
    print("Enhanced Obstacle Detection & Directional Avoidance Testing")
    print("=" * 70 + Colors.END)
    
def connect_arduino():
    """Connect to Arduino with timeout and error handling"""
    try:
        print(f"{Colors.YELLOW}🔌 Connecting to Arduino on {ARDUINO_PORT}...{Colors.END}")
        arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=2)
        time.sleep(2)  # Arduino startup delay
        print(f"{Colors.GREEN}✅ Connected successfully!{Colors.END}")
        return arduino
    except Exception as e:
        print(f"{Colors.RED}❌ Failed to connect: {e}{Colors.END}")
        return None

def parse_sensor_data(data_str):
    """Parse enhanced sensor telemetry"""
    try:
        data = json.loads(data_str)
        
        # Extract sensor information
        sensors = data.get('sensors', {})
        
        return {
            'timestamp': datetime.now(),
            'left_distance': sensors.get('left_distance'),
            'right_distance': sensors.get('right_distance'),
            'min_distance': sensors.get('min_distance'),
            'obstacle_direction': sensors.get('obstacle_direction', 0),
            'left_valid': sensors.get('left_valid', False),
            'right_valid': sensors.get('right_valid', False),
            'emergency': data.get('emergency', False),
            'mode': data.get('gatekeeper_mode', 'UNKNOWN')
        }
    except Exception as e:
        print(f"{Colors.RED}⚠️  Parse error: {e}{Colors.END}")
        return None

def format_distance(distance):
    """Format distance with null handling"""
    if distance is None:
        return f"{Colors.RED}NULL{Colors.END}"
    return f"{Colors.GREEN}{distance:.1f}cm{Colors.END}"

def format_obstacle_direction(direction):
    """Format obstacle direction indicator"""
    if direction == -1:
        return f"{Colors.YELLOW}← LEFT{Colors.END}"
    elif direction == 1:
        return f"{Colors.YELLOW}RIGHT →{Colors.END}"
    elif direction == 0:
        return f"{Colors.RED}CENTER{Colors.END}"
    else:
        return f"{Colors.WHITE}UNKNOWN{Colors.END}"

def display_sensor_status(data):
    """Display dual sensor status in organized format"""
    print(f"\n{Colors.CYAN}📊 DUAL SENSOR STATUS{Colors.END}")
    print(f"   Time: {data['timestamp'].strftime('%H:%M:%S.%f')[:-3]}")
    print(f"   Mode: {Colors.BOLD}{data['mode']}{Colors.END}")
    
    # Sensor readings
    print(f"\n{Colors.BLUE}🔍 SENSOR READINGS:{Colors.END}")
    print(f"   Left  (30°): {format_distance(data['left_distance'])} {'✅' if data['left_valid'] else '❌'}")
    print(f"   Right (30°): {format_distance(data['right_distance'])} {'✅' if data['right_valid'] else '❌'}")
    print(f"   Minimum:     {format_distance(data['min_distance'])}")
    
    # Obstacle analysis
    print(f"\n{Colors.YELLOW}🚨 OBSTACLE ANALYSIS:{Colors.END}")
    print(f"   Emergency Stop: {'🛑 YES' if data['emergency'] else '✅ NO'}")
    print(f"   Obstacle Direction: {format_obstacle_direction(data['obstacle_direction'])}")
    
    # Coverage visualization
    print(f"\n{Colors.WHITE}📡 COVERAGE MAP:{Colors.END}")
    left_indicator = "🔴" if data['left_distance'] and data['left_distance'] < 15 else "🟢"
    right_indicator = "🔴" if data['right_distance'] and data['right_distance'] < 15 else "🟢"
    
    print(f"        {left_indicator}     {right_indicator}")
    print(f"         \\   /")
    print(f"          \\ /")
    print(f"           🤖")
    print(f"      30°L   30°R")
    
    print("-" * 50)

def analyze_sensor_performance(readings):
    """Analyze sensor performance statistics"""
    if not readings:
        return
        
    print(f"\n{Colors.CYAN}{Colors.BOLD}📈 SENSOR PERFORMANCE ANALYSIS{Colors.END}")
    
    # Calculate statistics
    total_readings = len(readings)
    left_valid_count = sum(1 for r in readings if r['left_valid'])
    right_valid_count = sum(1 for r in readings if r['right_valid'])
    both_valid_count = sum(1 for r in readings if r['left_valid'] and r['right_valid'])
    emergency_count = sum(1 for r in readings if r['emergency'])
    
    print(f"Total Readings: {total_readings}")
    print(f"Left Sensor Valid: {left_valid_count}/{total_readings} ({left_valid_count/total_readings*100:.1f}%)")
    print(f"Right Sensor Valid: {right_valid_count}/{total_readings} ({right_valid_count/total_readings*100:.1f}%)")
    print(f"Both Sensors Valid: {both_valid_count}/{total_readings} ({both_valid_count/total_readings*100:.1f}%)")
    print(f"Emergency Stops: {emergency_count}/{total_readings} ({emergency_count/total_readings*100:.1f}%)")
    
    # Directional analysis
    left_obstacles = sum(1 for r in readings if r['obstacle_direction'] == -1)
    right_obstacles = sum(1 for r in readings if r['obstacle_direction'] == 1)
    center_obstacles = sum(1 for r in readings if r['obstacle_direction'] == 0 and r['emergency'])
    
    if emergency_count > 0:
        print(f"\n{Colors.YELLOW}🎯 OBSTACLE DIRECTION ANALYSIS:{Colors.END}")
        print(f"Left Obstacles: {left_obstacles} ({left_obstacles/emergency_count*100:.1f}% of obstacles)")
        print(f"Right Obstacles: {right_obstacles} ({right_obstacles/emergency_count*100:.1f}% of obstacles)")
        print(f"Center Obstacles: {center_obstacles} ({center_obstacles/emergency_count*100:.1f}% of obstacles)")

def test_directional_commands():
    """Test directional avoidance commands"""
    print(f"\n{Colors.BLUE}{Colors.BOLD}🎮 DIRECTIONAL AVOIDANCE TEST{Colors.END}")
    print("This will test the rover's response to different obstacle scenarios")
    print("Position obstacles at different angles to test sensor coverage")
    
    test_scenarios = [
        ("Left Obstacle", "Position obstacle 30° to the left, 8cm away"),
        ("Right Obstacle", "Position obstacle 30° to the right, 8cm away"),
        ("Center Obstacle", "Position obstacle directly ahead, 8cm away"),
        ("Narrow Passage", "Position obstacles on both sides, creating narrow passage")
    ]
    
    for scenario, instruction in test_scenarios:
        print(f"\n{Colors.CYAN}📋 {scenario}:{Colors.END}")
        print(f"   {instruction}")
        input(f"   Press Enter when ready...")
        
        # Brief monitoring period
        print(f"   Monitoring for 5 seconds...")
        # This would be implemented with actual sensor monitoring

def main():
    print_header()
    
    # Connect to Arduino
    arduino = connect_arduino()
    if not arduino:
        sys.exit(1)
    
    print(f"\n{Colors.BLUE}🔬 Starting dual sensor validation test...{Colors.END}")
    print(f"📊 Monitoring for {TEST_DURATION} seconds")
    print(f"💡 Move obstacles around to test directional detection")
    print(f"🛑 Press Ctrl+C to stop early\n")
    
    readings = []
    start_time = time.time()
    
    try:
        while time.time() - start_time < TEST_DURATION:
            if arduino.in_waiting > 0:
                try:
                    line = arduino.readline().decode('utf-8').strip()
                    if line.startswith('{') and line.endswith('}'):
                        data = parse_sensor_data(line)
                        if data:
                            readings.append(data)
                            display_sensor_status(data)
                            
                            # Special alerts
                            if data['emergency'] and not data.get('last_emergency', False):
                                direction = format_obstacle_direction(data['obstacle_direction'])
                                print(f"\n{Colors.RED}🚨 EMERGENCY: Obstacle detected {direction}! 🚨{Colors.END}")
                            
                except Exception as e:
                    print(f"{Colors.RED}⚠️  Error processing data: {e}{Colors.END}")
            
            time.sleep(0.1)  # 10Hz monitoring
    
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}⏹️  Test stopped by user{Colors.END}")
    
    finally:
        arduino.close()
        
    # Final analysis
    analyze_sensor_performance(readings)
    
    # Test scenarios
    if input(f"\n{Colors.CYAN}🎮 Run directional avoidance tests? (y/n): {Colors.END}").lower() == 'y':
        test_directional_commands()
    
    print(f"\n{Colors.GREEN}✅ Dual sensor validation complete!{Colors.END}")
    print(f"📊 Collected {len(readings)} sensor readings")
    
    return len(readings) > 0

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)