#include <PinChangeInterrupt.h>

// RC Pin definitions
#define CH1_PIN 2  // Forward/Reverse (Throttle)
#define CH2_PIN 3  // Steering
#define CH9_PIN 4  // Mode Switch (3-position)
#define LED_PIN 13 // Status LED

// Motor control pins (as specified by user)
#define LEFT_IN1 5    // Left motor direction 1 (D5)
#define LEFT_IN2 6    // Left motor direction 2 (D6)
#define LEFT_ENA 9    // Left motor enable/speed (D9) - PWM capable
#define RIGHT_IN1 7   // Right motor direction 1 (D7)
#define RIGHT_IN2 10  // Right motor direction 2 (D10)
#define RIGHT_ENA 11  // Right motor enable/speed (D11) - PWM capable

// RC signal parameters
#define RC_MIN_PULSE 1000
#define RC_MAX_PULSE 2000
#define RC_CENTER_PULSE 1500
#define RC_DEADBAND 50
#define RC_TIMEOUT 100  // milliseconds

// Serial communication
#define SERIAL_BAUD 115200
#define SERIAL_TIMEOUT 10

// Motor control parameters
#define MOTOR_PWM_FREQ 1000  // 1kHz PWM frequency

// Ultrasonic sensor pins
#define FRONT_TRIG_PIN 8   // Front ultrasonic trigger
#define FRONT_ECHO_PIN 12  // Front ultrasonic echo

// Collision avoidance parameters
#define COLLISION_DISTANCE 15.0    // Stop distance in cm
#define WARNING_DISTANCE 30.0      // Slow down distance in cm
#define ULTRASONIC_TIMEOUT 10000   // 10ms timeout for pulseIn (was 30ms!)
#define ULTRASONIC_INTERVAL 200    // Read every 200ms (was 100ms)

// Global variables for PWM reading
volatile unsigned long ch1_start_time = 0;
volatile unsigned long ch2_start_time = 0;
volatile unsigned long ch9_start_time = 0;
volatile unsigned long ch1_pulse_width = 1500;
volatile unsigned long ch2_pulse_width = 1500;
volatile unsigned long ch9_pulse_width = 1500;
volatile unsigned long ch1_last_time = 0;
volatile unsigned long ch2_last_time = 0;
volatile unsigned long ch9_last_time = 0;

// Signal validation
unsigned long last_valid_signal = 0;
bool signal_valid = false;

// Motor control variables
int left_motor_speed = 0;   // -255 to +255
int right_motor_speed = 0;  // -255 to +255

// Ultrasonic sensor variables
unsigned long last_ultrasonic_reading = 0;
float front_distance = 999.0;  // Distance in cm, default to "far"
bool collision_detected = false;

// Simplified autonomous roaming variables
enum RoamState {
  ROAM_STRAIGHT,        // Moving straight forward (default state)
  ROAM_SLOWING,         // Detected obstacle - slowing down to confirm
  ROAM_CONFIRMING,      // Stopped - confirming obstacle is real
  ROAM_AVOIDING         // Turning to avoid confirmed obstacle
};
RoamState roam_state = ROAM_STRAIGHT;
unsigned long roam_state_start = 0;
unsigned long last_direction_change = 0;

// Simple movement parameters
int cruise_speed = 180;        // Normal cruising speed
int slow_speed = 80;           // Speed when approaching obstacle
int turn_speed = 140;          // Speed when turning to avoid obstacle

// Obstacle detection confidence
float obstacle_distances[5] = {999.0, 999.0, 999.0, 999.0, 999.0};  // Last 5 readings
int distance_index = 0;
int consecutive_obstacles = 0;  // Count of consecutive obstacle detections
bool obstacle_confirmed = false;

// Navigation parameters
float slow_down_distance = 35.0;   // Start slowing at 35cm
float confirm_distance = 20.0;     // Confirm obstacle at 20cm
float critical_distance = 15.0;    // Emergency stop distance
unsigned long min_straight_time = 3000;    // Minimum 3 seconds straight before reacting
unsigned long confirm_time = 1000;         // 1 second to confirm obstacle
unsigned long turn_time = 2000;            // 2 seconds to turn away

// Control mode variables
enum ControlMode {
  MODE_FAILSAFE,    // CH9 < -500
  MODE_MANUAL,      // CH9 -500 to +500  
  MODE_AUTONOMOUS   // CH9 > +500
};
ControlMode current_mode = MODE_FAILSAFE;

// Data structure for RC communication
struct RCData {
  int ch1_value;
  int ch2_value;
  int ch9_value;
  bool valid;
  unsigned long timestamp;
};

// Data structure for motor commands from Pi
struct MotorCommand {
  int left_speed;   // -255 to +255
  int right_speed;  // -255 to +255
};

void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(SERIAL_TIMEOUT);
  
  // Configure RC input pins
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH9_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Configure motor output pins
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_ENA, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_ENA, OUTPUT);
  
  // Configure ultrasonic sensor pins
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  
  // Initialize motors to stopped state
  stopMotors();
  
  // Attach pin change interrupts for RC signals
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CH1_PIN), ch1_interrupt, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CH2_PIN), ch2_interrupt, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CH9_PIN), ch9_interrupt, CHANGE);
  
  // Initialize timing
  last_valid_signal = millis();
  
  // Status indication
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("Arduino Enhanced Motor Controller v2.0 Initialized");
  Serial.println("Pins: LEFT(D5,D6,D9) RIGHT(D7,D10,D11)");
  Serial.println("Front Ultrasonic: TRIG=D8, ECHO=D12");
}

void loop() {
  static unsigned long last_rc_update = 0;
  static unsigned long last_serial_update = 0;
  unsigned long current_time = millis();
  
  // High-frequency tasks (every loop) - motor control is time-critical
  checkSignalValidity();
  updateControlMode();
  
  // Handle motor control based on mode (time-critical)
  if (current_mode == MODE_MANUAL) {
    processRCMotorControl();
  } else if (current_mode == MODE_AUTONOMOUS) {
    // Check for Pi commands first, then default to roaming
    if (Serial.available() > 0) {
      processMotorCommands();
    } else {
      processAutonomousRoaming();
    }
  } else {
    // Failsafe mode - stop motors
    stopMotors();
  }
  
  // Medium-frequency tasks (every 50ms) - reduce processing load
  if (current_time - last_rc_update >= 50) {
    processRCData();  // RC data updates
    updateStatusLED();
    last_rc_update = current_time;
  }
  
  // Low-frequency tasks (every 200ms) - heavy processing
  if (current_time - last_serial_update >= 200) {
    updateUltrasonicSensor();  // This includes the blocking pulseIn()
    last_serial_update = current_time;
  }
  
  // Minimal delay - keep loop fast for motor control
  delay(5);  // Reduced from 10ms to 5ms
}

// RC Interrupt handlers
void ch1_interrupt() {
  if (digitalRead(CH1_PIN) == HIGH) {
    ch1_start_time = micros();
  } else {
    unsigned long pulse_time = micros() - ch1_start_time;
    if (pulse_time > RC_MIN_PULSE && pulse_time < RC_MAX_PULSE) {
      ch1_pulse_width = pulse_time;
      ch1_last_time = millis();
    }
  }
}

void ch2_interrupt() {
  if (digitalRead(CH2_PIN) == HIGH) {
    ch2_start_time = micros();
  } else {
    unsigned long pulse_time = micros() - ch2_start_time;
    if (pulse_time > RC_MIN_PULSE && pulse_time < RC_MAX_PULSE) {
      ch2_pulse_width = pulse_time;
      ch2_last_time = millis();
    }
  }
}

void ch9_interrupt() {
  if (digitalRead(CH9_PIN) == HIGH) {
    ch9_start_time = micros();
  } else {
    unsigned long pulse_time = micros() - ch9_start_time;
    if (pulse_time > RC_MIN_PULSE && pulse_time < RC_MAX_PULSE) {
      ch9_pulse_width = pulse_time;
      ch9_last_time = millis();
    }
  }
}

void checkSignalValidity() {
  unsigned long current_time = millis();
  
  // Check if all channels have recent updates
  bool ch1_fresh = (current_time - ch1_last_time) < RC_TIMEOUT;
  bool ch2_fresh = (current_time - ch2_last_time) < RC_TIMEOUT;
  bool ch9_fresh = (current_time - ch9_last_time) < RC_TIMEOUT;
  
  signal_valid = ch1_fresh && ch2_fresh && ch9_fresh;
  
  if (signal_valid) {
    last_valid_signal = current_time;
  }
}

void processRCData() {
  RCData rc_data;
  
  // Convert pulse widths to standardized values (-1000 to 1000)
  rc_data.ch1_value = map(constrain(ch1_pulse_width, RC_MIN_PULSE, RC_MAX_PULSE), 
                         RC_MIN_PULSE, RC_MAX_PULSE, -1000, 1000);
  rc_data.ch2_value = map(constrain(ch2_pulse_width, RC_MIN_PULSE, RC_MAX_PULSE), 
                         RC_MIN_PULSE, RC_MAX_PULSE, -1000, 1000);
  rc_data.ch9_value = map(constrain(ch9_pulse_width, RC_MIN_PULSE, RC_MAX_PULSE), 
                         RC_MIN_PULSE, RC_MAX_PULSE, -1000, 1000);
  
  // Apply deadband to center channels (but not ch9 - it's a 3-position switch)
  rc_data.ch1_value = applyDeadband(rc_data.ch1_value, RC_DEADBAND);
  rc_data.ch2_value = applyDeadband(rc_data.ch2_value, RC_DEADBAND);
  
  // Set validity and timestamp
  rc_data.valid = signal_valid;
  rc_data.timestamp = millis();
  
  // Send data via serial
  sendRCData(rc_data);
}

int applyDeadband(int value, int deadband) {
  if (abs(value) < deadband) {
    return 0;
  }
  return value;
}

void sendRCData(RCData data) {
  // Send structured data as JSON for easy parsing
  Serial.print("{\"ch1\":");
  Serial.print(data.ch1_value);
  Serial.print(",\"ch2\":");
  Serial.print(data.ch2_value);
  Serial.print(",\"ch9\":");
  Serial.print(data.ch9_value);
  Serial.print(",\"valid\":");
  Serial.print(data.valid ? "true" : "false");
  Serial.print(",\"timestamp\":");
  Serial.print(data.timestamp);
  Serial.println("}");
}

void processMotorCommands() {
  // Check if Pi has sent motor commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Parse motor command JSON: {"left":speed,"right":speed}
    if (command.startsWith("{") && command.endsWith("}")) {
      int leftStart = command.indexOf("\"left\":");
      int rightStart = command.indexOf("\"right\":");
      
      if (leftStart != -1 && rightStart != -1) {
        // Extract left motor speed
        leftStart += 7; // Skip "left":
        int leftEnd = command.indexOf(",", leftStart);
        if (leftEnd == -1) leftEnd = command.indexOf("}", leftStart);
        int leftSpeed = command.substring(leftStart, leftEnd).toInt();
        
        // Extract right motor speed
        rightStart += 8; // Skip "right":
        int rightEnd = command.indexOf("}", rightStart);
        int rightSpeed = command.substring(rightStart, rightEnd).toInt();
        
        // Apply motor commands
        setMotorSpeeds(leftSpeed, rightSpeed);
        
        // Send acknowledgment
        Serial.print("{\"motor_ack\":{\"left\":");
        Serial.print(leftSpeed);
        Serial.print(",\"right\":");
        Serial.print(rightSpeed);
        Serial.println("}}");
      }
    }
  }
}

void setMotorSpeeds(int left_speed, int right_speed) {
  // Constrain speeds to valid range
  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);
  
  // Apply collision avoidance for forward movement
  bool left_forward = left_speed > 0;
  bool right_forward = right_speed > 0;
  bool is_moving_forward = left_forward && right_forward;
  
  if (is_moving_forward) {
    left_speed = applyCollisionAvoidance(left_speed, true);
    right_speed = applyCollisionAvoidance(right_speed, true);
  }
  
  // Store current speeds (after collision avoidance)
  left_motor_speed = left_speed;
  right_motor_speed = right_speed;
  
  // Control left motor
  setMotor(LEFT_IN1, LEFT_IN2, LEFT_ENA, left_speed);
  
  // Control right motor (inverted direction)
  setMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_ENA, -right_speed);
}

void setMotor(int in1_pin, int in2_pin, int ena_pin, int speed) {
  // Determine direction and PWM value
  if (speed > 0) {
    // Forward direction
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
    analogWrite(ena_pin, speed);
  } else if (speed < 0) {
    // Reverse direction
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
    analogWrite(ena_pin, -speed);  // Use absolute value for PWM
  } else {
    // Stop motor
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, LOW);
    analogWrite(ena_pin, 0);
  }
}

void stopMotors() {
  setMotorSpeeds(0, 0);
}

void updateControlMode() {
  // Update control mode based on CH9 (mode switch)
  RCData rc_data;
  rc_data.ch9_value = map(constrain(ch9_pulse_width, RC_MIN_PULSE, RC_MAX_PULSE), 
                         RC_MIN_PULSE, RC_MAX_PULSE, -1000, 1000);
  
  ControlMode new_mode;
  if (!signal_valid) {
    new_mode = MODE_FAILSAFE;
  } else if (rc_data.ch9_value < -500) {
    new_mode = MODE_FAILSAFE;
  } else if (rc_data.ch9_value > 500) {
    new_mode = MODE_AUTONOMOUS;
  } else {
    new_mode = MODE_MANUAL;
  }
  
  // Print mode changes
  if (new_mode != current_mode) {
    current_mode = new_mode;
    Serial.print("{\"mode_change\":\"");
    switch(current_mode) {
      case MODE_FAILSAFE: Serial.print("FAILSAFE"); break;
      case MODE_MANUAL: Serial.print("MANUAL"); break;
      case MODE_AUTONOMOUS: Serial.print("AUTONOMOUS"); break;
    }
    Serial.println("\"}");
  }
}

void processRCMotorControl() {
  // Convert RC inputs to motor commands (tank/differential steering)
  int ch1_value = map(constrain(ch1_pulse_width, RC_MIN_PULSE, RC_MAX_PULSE), 
                     RC_MIN_PULSE, RC_MAX_PULSE, -1000, 1000);
  int ch2_value = map(constrain(ch2_pulse_width, RC_MIN_PULSE, RC_MAX_PULSE), 
                     RC_MIN_PULSE, RC_MAX_PULSE, -1000, 1000);
  
  // Apply deadband
  ch1_value = applyDeadband(ch1_value, RC_DEADBAND);
  ch2_value = applyDeadband(ch2_value, RC_DEADBAND);
  
  // Calculate differential steering
  // CH1 = forward/reverse (throttle)
  // CH2 = left/right (steering)
  
  int throttle = ch1_value;  // Forward/reverse
  int steering = ch2_value;  // Left/right steering
  
  // Improved differential steering mixing with scaling
  // Scale inputs to percentage first (-100 to +100)
  int throttle_pct = map(throttle, -1000, 1000, -100, 100);
  int steering_pct = map(steering, -1000, 1000, -100, 100);
  
  // Calculate motor percentages with corrected steering direction
  int left_pct = throttle_pct - steering_pct;   // Subtract steering from left (corrected)
  int right_pct = throttle_pct + steering_pct;  // Add steering to right (corrected)
  
  // Scale down if either motor exceeds 100% to prevent clipping
  int max_abs = max(abs(left_pct), abs(right_pct));
  if (max_abs > 100) {
    left_pct = (left_pct * 100) / max_abs;
    right_pct = (right_pct * 100) / max_abs;
  }
  
  // Convert to motor PWM values (-255 to +255)
  int left_speed = map(left_pct, -100, 100, -255, 255);
  int right_speed = map(right_pct, -100, 100, -255, 255);
  
  // Apply motor commands (collision avoidance handled in setMotorSpeeds)
  setMotorSpeeds(left_speed, right_speed);
}

void updateStatusLED() {
  static unsigned long last_blink = 0;
  static bool led_state = false;
  unsigned long current_time = millis();
  
  if (collision_detected) {
    // Fast blink when collision detected
    if (current_time - last_blink > 150) {
      led_state = !led_state;
      digitalWrite(LED_PIN, led_state);
      last_blink = current_time;
    }
  } else if (signal_valid) {
    // Solid LED when signal is valid
    digitalWrite(LED_PIN, HIGH);
  } else {
    // Slow blink LED when signal is invalid
    if (current_time - last_blink > 500) {
      led_state = !led_state;
      digitalWrite(LED_PIN, led_state);
      last_blink = current_time;
    }
  }
}

void updateUltrasonicSensor() {
  unsigned long current_time = millis();
  
  // Read sensor at specified interval
  if (current_time - last_ultrasonic_reading >= ULTRASONIC_INTERVAL) {
    float new_reading = readUltrasonicDistance();
    last_ultrasonic_reading = current_time;
    
    // Add reading to obstacle history
    if (new_reading > 0) {
      obstacle_distances[distance_index] = new_reading;
      distance_index = (distance_index + 1) % 5;
      
      // Use most recent reading as current distance
      front_distance = new_reading;
      
      // Count consecutive obstacle detections for confidence
      if (new_reading < confirm_distance) {
        consecutive_obstacles++;
      } else {
        consecutive_obstacles = 0; // Reset if no obstacle
      }
      
      // Confirm obstacle only after 3 consecutive detections
      obstacle_confirmed = (consecutive_obstacles >= 3);
      
      // Emergency collision detection (immediate stop)
      collision_detected = (new_reading < critical_distance);
    }
    
    // Send sensor data less frequently to reduce processing
    static unsigned long last_sensor_report = 0;
    if (current_time - last_sensor_report >= 1000) { // Report every 1000ms (was 500ms)
      sendSensorData();
      last_sensor_report = current_time;
    }
  }
}

float readUltrasonicDistance() {
  // Clear trigger pin
  digitalWrite(FRONT_TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Send 10us pulse
  digitalWrite(FRONT_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(FRONT_TRIG_PIN, LOW);
  
  // Read echo pulse duration
  unsigned long duration = pulseIn(FRONT_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT);
  
  if (duration == 0) {
    return -1; // No echo received (timeout)
  }
  
  // Calculate distance in cm (speed of sound = 343 m/s)
  float distance = (duration * 0.0343) / 2;
  
  // Validate reading (HC-SR04 reliable range: 2-400cm)
  if (distance < 2 || distance > 400) {
    return -1; // Invalid reading
  }
  
  return distance;
}

void sendSensorData() {
  Serial.print("{\"sensors\":{");
  Serial.print("\"front_distance\":");
  
  if (front_distance > 0) {
    Serial.print(front_distance, 1);
  } else {
    Serial.print("null");
  }
  
  Serial.print(",\"collision_detected\":");
  Serial.print(collision_detected ? "true" : "false");
  Serial.print(",\"timestamp\":");
  Serial.print(millis());
  Serial.println("}}");
}

int applyCollisionAvoidance(int motor_speed, bool is_forward_movement) {
  // Only apply collision avoidance to forward movement
  if (!is_forward_movement || front_distance < 0) {
    return motor_speed; // No restriction for reverse or invalid readings
  }
  
  // Complete stop if collision detected
  if (collision_detected) {
    return 0;
  }
  
  // Gradual speed reduction in warning zone
  if (front_distance < WARNING_DISTANCE) {
    float speed_factor = (front_distance - COLLISION_DISTANCE) / (WARNING_DISTANCE - COLLISION_DISTANCE);
    speed_factor = constrain(speed_factor, 0.0, 1.0);
    
    // Reduce speed proportionally (minimum 30% speed to maintain steering)
    speed_factor = max(speed_factor, 0.3);
    motor_speed = (int)(motor_speed * speed_factor);
  }
  
  return motor_speed;
}

void processAutonomousRoaming() {
  unsigned long current_time = millis();
  unsigned long time_in_state = current_time - roam_state_start;
  
  // Simple, confident state machine
  switch (roam_state) {
    
    case ROAM_STRAIGHT:
      // Default state: move straight with confidence
      setMotorSpeeds(cruise_speed, cruise_speed);
      
      // Ensure minimum straight time before reacting (prevents twitching)
      if (time_in_state < min_straight_time) {
        break; // Keep going straight regardless of sensors
      }
      
      // Emergency stop for immediate danger
      if (collision_detected) {
        changeRoamState(ROAM_CONFIRMING);
        break;
      }
      
      // Start slowing if obstacle detected in slow-down zone
      if (front_distance < slow_down_distance && front_distance > 0) {
        changeRoamState(ROAM_SLOWING);
        break;
      }
      
      // Occasional random direction change for exploration (less frequent)
      if (current_time - last_direction_change > random(15000, 30000)) { // 15-30 seconds
        changeRoamState(ROAM_AVOIDING);
        last_direction_change = current_time;
      }
      break;
      
    case ROAM_SLOWING:
      // Slow down to confirm obstacle
      setMotorSpeeds(slow_speed, slow_speed);
      
      // If obstacle disappears while slowing, resume straight movement
      if (front_distance > slow_down_distance) {
        changeRoamState(ROAM_STRAIGHT);
        break;
      }
      
      // If we get close enough, stop to confirm
      if (front_distance < confirm_distance || collision_detected) {
        changeRoamState(ROAM_CONFIRMING);
      }
      break;
      
    case ROAM_CONFIRMING:
      // Stop and confirm obstacle is real
      setMotorSpeeds(0, 0);
      
      // Wait for confirmation time
      if (time_in_state > confirm_time) {
        if (obstacle_confirmed || collision_detected) {
          // Obstacle confirmed - avoid it
          changeRoamState(ROAM_AVOIDING);
        } else {
          // False alarm - resume straight movement
          changeRoamState(ROAM_STRAIGHT);
        }
      }
      break;
      
    case ROAM_AVOIDING:
      // Turn to avoid obstacle (or random exploration)
      static bool avoid_left = true;
      
      // Choose turn direction on entry
      if (time_in_state == 0) {
        avoid_left = random(2); // Random turn direction
      }
      
      // Execute turn
      if (avoid_left) {
        setMotorSpeeds(-turn_speed, turn_speed); // Turn left
      } else {
        setMotorSpeeds(turn_speed, -turn_speed); // Turn right
      }
      
      // Turn for specified duration
      if (time_in_state > turn_time) {
        changeRoamState(ROAM_STRAIGHT);
        last_direction_change = current_time;
      }
      break;
  }
  
  // Send roaming status less frequently to reduce processing
  static unsigned long last_roam_report = 0;
  if (current_time - last_roam_report >= 2000) { // Every 2 seconds (was 1 second)
    sendRoamingStatus();
    last_roam_report = current_time;
  }
}

void changeRoamState(RoamState new_state) {
  roam_state = new_state;
  roam_state_start = millis();
  
  // Reset obstacle detection when changing states
  if (new_state == ROAM_STRAIGHT) {
    consecutive_obstacles = 0;
  }
}

void sendRoamingStatus() {
  Serial.print("{\"roaming\":{");
  Serial.print("\"state\":\"");
  
  switch (roam_state) {
    case ROAM_STRAIGHT: Serial.print("STRAIGHT"); break;
    case ROAM_SLOWING: Serial.print("SLOWING"); break;
    case ROAM_CONFIRMING: Serial.print("CONFIRMING"); break;
    case ROAM_AVOIDING: Serial.print("AVOIDING"); break;
  }
  
  Serial.print("\",\"speed\":");
  Serial.print(cruise_speed);
  Serial.print(",\"front_distance\":");
  Serial.print(front_distance, 1);
  Serial.print(",\"collision_detected\":");
  Serial.print(collision_detected ? "true" : "false");
  Serial.print(",\"timestamp\":");
  Serial.print(millis());
  Serial.println("}}");
}

