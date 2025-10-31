#include <PinChangeInterrupt.h>

// RC Pin definitions
#define CH1_PIN 2  // Forward/Reverse (Throttle)
#define CH2_PIN 3  // Steering
#define CH9_PIN 4  // Mode Switch (3-position)
#define LED_PIN 13 // Status LED

// Motor control pins
#define LEFT_IN1 5    // Left motor direction 1 (D5)
#define LEFT_IN2 6    // Left motor direction 2 (D6)
#define LEFT_ENA 9    // Left motor enable/speed (D9) - PWM capable
#define RIGHT_IN1 7   // Right motor direction 1 (D7)
#define RIGHT_IN2 10  // Right motor direction 2 (D10)
#define RIGHT_ENA 11  // Right motor enable/speed (D11) - PWM capable

// Dual Ultrasonic sensor pins (angled 30° left and right for overlapping coverage)
#define LEFT_SENSOR_TRIG_PIN 8   // Left sensor trigger (angled 30° left)
#define LEFT_SENSOR_ECHO_PIN 12  // Left sensor echo
#define RIGHT_SENSOR_TRIG_PIN A0 // Right sensor trigger (angled 30° right) - using analog pin as digital
#define RIGHT_SENSOR_ECHO_PIN A1 // Right sensor echo - using analog pin as digital

// RC signal parameters
#define RC_MIN_PULSE 1000
#define RC_MAX_PULSE 2000
#define RC_DEADBAND 50
#define RC_TIMEOUT 250  // milliseconds

// Serial communication
#define SERIAL_BAUD 115200

// Emergency stop parameters
#define EMERGENCY_DISTANCE 10.0  // Emergency stop at 10cm
#define ULTRASONIC_TIMEOUT 30000  // 30ms timeout for improved reliability

// Sensor fusion parameters
#define SENSOR_SEPARATION_CM 11.43  // 4.5 inches = 11.43 cm
#define SENSOR_ANGLE_DEG 30         // 30 degrees from center
#define MIN_RELIABLE_DISTANCE 2.0   // Minimum reliable distance in cm
#define MAX_RELIABLE_DISTANCE 400.0 // Maximum reliable distance in cm

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

// Control mode with stability
enum ControlMode {
  MODE_FAILSAFE,    // CH9 < -500
  MODE_MANUAL,      // CH9 -500 to +500  
  MODE_AUTONOMOUS   // CH9 > +500
};
 ControlMode current_mode = MODE_FAILSAFE;

// Emergency stop with directional awareness
bool emergency_stop = false;
unsigned long last_emergency_check = 0;

// Sensor readings structure
struct SensorReadings {
  float left_distance;
  float right_distance;
  float min_distance;
  bool left_valid;
  bool right_valid;
  bool obstacle_detected;
  int obstacle_direction;  // -1 = left, 0 = center, 1 = right
};
SensorReadings sensors = {0, 0, 0, false, false, false, 0};

// Pi command storage
struct PiCommand {
  int left_speed;
  int right_speed;
  unsigned long timestamp;
  bool valid;
};
PiCommand pi_command = {0, 0, 0, false};
#define PI_COMMAND_TIMEOUT 500  // 500ms timeout for Pi commands

// Pi telemetry storage
String pi_telemetry = "";
unsigned long last_telemetry_update = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD);
  
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
  
  // Configure dual ultrasonic sensor pins
  pinMode(LEFT_SENSOR_TRIG_PIN, OUTPUT);
  pinMode(LEFT_SENSOR_ECHO_PIN, INPUT);
  pinMode(RIGHT_SENSOR_TRIG_PIN, OUTPUT);  // A0 as digital output
  pinMode(RIGHT_SENSOR_ECHO_PIN, INPUT);   // A1 as digital input
  
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
  
  Serial.println("Arduino Gatekeeper v2.0 - Dual Ultrasonic Initialized");
  Serial.println("🛡️ Safety Authority: Arduino holds the keys");
  Serial.println("🧠 Always listening to Pi, applying commands only in AUTONOMOUS mode");
  Serial.println("⚠️ MANUAL mode: Full operator control - NO safety overrides");
  Serial.println("🛡️ AUTONOMOUS mode: Full safety systems active");
  Serial.println("🔍 NEW: Dual ultrasonic sensors with 30° angled coverage");
  Serial.println("📊 NEW: Directional obstacle detection and avoidance");
}

void loop() {
  // High frequency tasks - safety first
  checkSignalValidity();
  updateControlMode();
  
  // Emergency stop check (every 100ms for safety)
  if (millis() - last_emergency_check >= 100) {
    checkEmergencyStop();
    last_emergency_check = millis();
  }
  
  // Always listen for Pi commands (even in manual mode)
  processPiCommands();
  
  // GATEKEEPER LOGIC: Decide which commands to actually apply
  applyMotorCommands();
  
  // Send data to Pi (every 200ms)
  static unsigned long last_data_send = 0;
  if (millis() - last_data_send >= 200) {
    sendDataToPi();
    last_data_send = millis();
  }
  
  // Update status LED
  updateStatusLED();
  
  // Fast loop for responsive control
  delay(5);
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
  
  // Only require CH9 (mode switch) to be fresh for basic validity
  bool ch9_fresh = (current_time - ch9_last_time) < RC_TIMEOUT;
  signal_valid = ch9_fresh;
  
  if (signal_valid) {
    last_valid_signal = current_time;
  }
}

void updateControlMode() {
  // Simple, reliable mode detection
  int ch9_value = map(constrain(ch9_pulse_width, RC_MIN_PULSE, RC_MAX_PULSE), 
                     RC_MIN_PULSE, RC_MAX_PULSE, -1000, 1000);
  
  ControlMode new_mode;
  
  // Determine mode based on CH9 value
  if (ch9_value < -500) {
    new_mode = MODE_FAILSAFE;
  } else if (ch9_value > 500) {
    new_mode = MODE_AUTONOMOUS;
  } else {
    new_mode = MODE_MANUAL;
  }
  
  // Simple stability - require 3 consecutive identical readings
  static ControlMode last_readings[3] = {MODE_FAILSAFE, MODE_FAILSAFE, MODE_FAILSAFE};
  static int reading_index = 0;
  
  last_readings[reading_index] = new_mode;
  reading_index = (reading_index + 1) % 3;
  
  // Check if all 3 readings are the same
  if (last_readings[0] == last_readings[1] && last_readings[1] == last_readings[2]) {
    current_mode = last_readings[0];
  }
}

void processPiCommands() {
  // Always listen for Pi commands, regardless of mode
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("{") && command.endsWith("}")) {
      // Check if this is the new extended format with telemetry
      if (command.indexOf("\"motor\"") != -1 && command.indexOf("\"telemetry\"") != -1) {
        parseExtendedCommand(command);
      } else if (command.indexOf("\"left\"") != -1 && command.indexOf("\"right\"") != -1) {
        // Legacy simple format: {"left":speed,"right":speed}
        parseSimpleCommand(command);
      }
    }
  }
  
  // Check if Pi command is still fresh
  if (pi_command.valid && (millis() - pi_command.timestamp > PI_COMMAND_TIMEOUT)) {
    pi_command.valid = false;  // Pi command timed out
  }
}

void parseSimpleCommand(String command) {
  // Parse legacy format: {"left":speed,"right":speed}
  int leftStart = command.indexOf("\"left\":");
  int rightStart = command.indexOf("\"right\":");
  
  if (leftStart != -1 && rightStart != -1) {
    // Extract speeds
    leftStart += 7;
    int leftEnd = command.indexOf(",", leftStart);
    if (leftEnd == -1) leftEnd = command.indexOf("}", leftStart);
    int leftSpeed = command.substring(leftStart, leftEnd).toInt();
    
    rightStart += 8;
    int rightEnd = command.indexOf("}", rightStart);
    int rightSpeed = command.substring(rightStart, rightEnd).toInt();
    
    // Store Pi command
    pi_command.left_speed = constrain(leftSpeed, -255, 255);
    pi_command.right_speed = constrain(rightSpeed, -255, 255);
    pi_command.timestamp = millis();
    pi_command.valid = true;
    
    sendPiAcknowledgment();
  }
}

void parseExtendedCommand(String command) {
  // Parse extended format: {"motor":{"left":100,"right":-50},"telemetry":{...}}
  
  // Extract motor commands
  int motorStart = command.indexOf("\"motor\":{");
  if (motorStart != -1) {
    motorStart += 9;  // Skip "motor":{
    int motorEnd = command.indexOf("}", motorStart);
    String motorSection = command.substring(motorStart, motorEnd);
    
    // Parse left and right from motor section
    int leftStart = motorSection.indexOf("\"left\":");
    int rightStart = motorSection.indexOf("\"right\":");
    
    if (leftStart != -1 && rightStart != -1) {
      leftStart += 7;
      int leftEnd = motorSection.indexOf(",", leftStart);
      if (leftEnd == -1) leftEnd = motorSection.length();
      int leftSpeed = motorSection.substring(leftStart, leftEnd).toInt();
      
      rightStart += 8;
      int rightEnd = motorSection.length();
      int rightSpeed = motorSection.substring(rightStart, rightEnd).toInt();
      
      // Store Pi command
      pi_command.left_speed = constrain(leftSpeed, -255, 255);
      pi_command.right_speed = constrain(rightSpeed, -255, 255);
      pi_command.timestamp = millis();
      pi_command.valid = true;
    }
  }
  
  // Extract and store telemetry data
  int telemetryStart = command.indexOf("\"telemetry\":");
  if (telemetryStart != -1) {
    telemetryStart += 12;  // Skip "telemetry":
    int telemetryEnd = command.lastIndexOf("}");
    
    // Find the matching closing brace for telemetry object
    int braceCount = 0;
    int actualEnd = telemetryStart;
    for (int i = telemetryStart; i < command.length(); i++) {
      if (command.charAt(i) == '{') braceCount++;
      else if (command.charAt(i) == '}') {
        braceCount--;
        if (braceCount == 0) {
          actualEnd = i + 1;
          break;
        }
      }
    }
    
    pi_telemetry = command.substring(telemetryStart, actualEnd);
    last_telemetry_update = millis();
  }
  
  sendPiAcknowledgment();
}

void sendPiAcknowledgment() {
  // Send acknowledgment to Pi
  Serial.print("{\"pi_ack\":{\"left\":");
  Serial.print(pi_command.left_speed);
  Serial.print(",\"right\":");
  Serial.print(pi_command.right_speed);
  Serial.print(",\"applied\":");
  Serial.print((current_mode == MODE_AUTONOMOUS && !emergency_stop) ? "true" : "false");
  Serial.println("}}");
}

void applyMotorCommands() {
  // GATEKEEPER LOGIC: Decide which commands to actually apply
  
  switch (current_mode) {
    case MODE_AUTONOMOUS:
      // AUTONOMOUS MODE: Full safety systems active with directional awareness
      if (emergency_stop) {
        // Apply intelligent obstacle avoidance based on sensor readings
        applyDirectionalAvoidance();
        return;
      }
      
      // Apply Pi commands if available and fresh
      if (pi_command.valid) {
        setMotorSpeeds(pi_command.left_speed, pi_command.right_speed);
      } else {
        // No valid Pi command - stop for safety
        setMotorSpeeds(0, 0);
      }
      break;
      
    case MODE_MANUAL:
      // MANUAL MODE: Full operator control, NO safety overrides
      // Operator has complete authority - can drive over/through anything
      applyRCCommands();
      break;
      
    case MODE_FAILSAFE:
    default:
      // FAILSAFE MODE: Emergency stop applies
      if (emergency_stop) {
        setMotorSpeeds(0, 0);
        return;
      }
      // Stop motors in failsafe (no RC signal)
      setMotorSpeeds(0, 0);
      break;
  }
}

void applyDirectionalAvoidance() {
  // Smart directional avoidance based on sensor fusion
  if (sensors.obstacle_direction == -1) {
    // Obstacle on left - turn right gently
    setMotorSpeeds(-50, 50);  // Slow turn right
  } else if (sensors.obstacle_direction == 1) {
    // Obstacle on right - turn left gently
    setMotorSpeeds(50, -50);  // Slow turn left
  } else {
    // Obstacle in center or both sides - full stop
    setMotorSpeeds(0, 0);
  }
}

void applyRCCommands() {
  // Convert RC inputs to motor commands
  int ch1_value = map(constrain(ch1_pulse_width, RC_MIN_PULSE, RC_MAX_PULSE), 
                     RC_MIN_PULSE, RC_MAX_PULSE, -1000, 1000);
  int ch2_value = map(constrain(ch2_pulse_width, RC_MIN_PULSE, RC_MAX_PULSE), 
                     RC_MIN_PULSE, RC_MAX_PULSE, -1000, 1000);
  
  // Apply deadband
  ch1_value = applyDeadband(ch1_value, RC_DEADBAND);
  ch2_value = applyDeadband(ch2_value, RC_DEADBAND);
  
  // Calculate differential steering
  int throttle_pct = map(ch1_value, -1000, 1000, -100, 100);
  int steering_pct = map(ch2_value, -1000, 1000, -100, 100);
  
  int left_pct = throttle_pct - steering_pct;
  int right_pct = throttle_pct + steering_pct;
  
  // Scale down if needed
  int max_abs = max(abs(left_pct), abs(right_pct));
  if (max_abs > 100) {
    left_pct = (left_pct * 100) / max_abs;
    right_pct = (right_pct * 100) / max_abs;
  }
  
  // Convert to motor values
  int left_speed = map(left_pct, -100, 100, -255, 255);
  int right_speed = map(right_pct, -100, 100, -255, 255);
  
  // Apply motor commands
  setMotorSpeeds(left_speed, right_speed);
}

void setMotorSpeeds(int left_speed, int right_speed) {
  // Final motor control - constrain and apply
  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);
  
  // Control left motor
  setMotor(LEFT_IN1, LEFT_IN2, LEFT_ENA, left_speed);
  
  // Control right motor (inverted direction)
  setMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_ENA, -right_speed);
}

void setMotor(int in1_pin, int in2_pin, int ena_pin, int speed) {
  if (speed > 0) {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
    analogWrite(ena_pin, speed);
  } else if (speed < 0) {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
    analogWrite(ena_pin, -speed);
  } else {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, LOW);
    analogWrite(ena_pin, 0);
  }
}

void stopMotors() {
  setMotorSpeeds(0, 0);
}

void checkEmergencyStop() {
  // Read both sensors and perform sensor fusion
  readDualSensors();
  
  // Determine if emergency stop is needed
  emergency_stop = sensors.obstacle_detected;
}

void readDualSensors() {
  // Read left sensor (angled 30° left)
  sensors.left_distance = readUltrasonicSensor(LEFT_SENSOR_TRIG_PIN, LEFT_SENSOR_ECHO_PIN);
  sensors.left_valid = (sensors.left_distance > MIN_RELIABLE_DISTANCE && 
                       sensors.left_distance < MAX_RELIABLE_DISTANCE);
  
  // Small delay between sensor readings to avoid interference
  delayMicroseconds(100);
  
  // Read right sensor (angled 30° right)
  sensors.right_distance = readUltrasonicSensor(RIGHT_SENSOR_TRIG_PIN, RIGHT_SENSOR_ECHO_PIN);
  sensors.right_valid = (sensors.right_distance > MIN_RELIABLE_DISTANCE && 
                        sensors.right_distance < MAX_RELIABLE_DISTANCE);
  
  // Sensor fusion logic
  fuseSensorData();
}

float readUltrasonicSensor(int trig_pin, int echo_pin) {
  // Send ultrasonic pulse
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  
  // Read echo
  unsigned long duration = pulseIn(echo_pin, HIGH, ULTRASONIC_TIMEOUT);
  
  if (duration == 0) {
    return -1; // Timeout
  }
  
  // Calculate distance
  float distance = (duration * 0.0343) / 2;
  
  // Validate range
  if (distance < MIN_RELIABLE_DISTANCE || distance > MAX_RELIABLE_DISTANCE) {
    return -1;
  }
  
  return distance;
}

void fuseSensorData() {
  // Initialize sensor fusion results
  sensors.obstacle_detected = false;
  sensors.obstacle_direction = 0;
  sensors.min_distance = 999.0;
  
  // Check for obstacles and determine direction
  bool left_obstacle = sensors.left_valid && (sensors.left_distance < EMERGENCY_DISTANCE);
  bool right_obstacle = sensors.right_valid && (sensors.right_distance < EMERGENCY_DISTANCE);
  
  if (left_obstacle && right_obstacle) {
    // Obstacles on both sides - center obstacle or narrow passage
    sensors.obstacle_detected = true;
    sensors.obstacle_direction = 0;  // Center
    sensors.min_distance = min(sensors.left_distance, sensors.right_distance);
  } else if (left_obstacle) {
    // Obstacle on left side
    sensors.obstacle_detected = true;
    sensors.obstacle_direction = -1;  // Left
    sensors.min_distance = sensors.left_distance;
  } else if (right_obstacle) {
    // Obstacle on right side
    sensors.obstacle_detected = true;
    sensors.obstacle_direction = 1;   // Right
    sensors.min_distance = sensors.right_distance;
  } else {
    // No obstacles detected
    sensors.obstacle_detected = false;
    sensors.obstacle_direction = 0;
    
    // Set minimum valid distance for reporting
    if (sensors.left_valid && sensors.right_valid) {
      sensors.min_distance = min(sensors.left_distance, sensors.right_distance);
    } else if (sensors.left_valid) {
      sensors.min_distance = sensors.left_distance;
    } else if (sensors.right_valid) {
      sensors.min_distance = sensors.right_distance;
    }
  }
}

void sendDataToPi() {
  // Send status data to Pi including dual sensor readings
  int ch1_value = map(constrain(ch1_pulse_width, RC_MIN_PULSE, RC_MAX_PULSE), 
                     RC_MIN_PULSE, RC_MAX_PULSE, -1000, 1000);
  int ch2_value = map(constrain(ch2_pulse_width, RC_MIN_PULSE, RC_MAX_PULSE), 
                     RC_MIN_PULSE, RC_MAX_PULSE, -1000, 1000);
  int ch9_value = map(constrain(ch9_pulse_width, RC_MIN_PULSE, RC_MAX_PULSE), 
                     RC_MIN_PULSE, RC_MAX_PULSE, -1000, 1000);
  
  // Read current sensor data
  readDualSensors();
  
  Serial.print("{\"ch1\":");
  Serial.print(applyDeadband(ch1_value, RC_DEADBAND));
  Serial.print(",\"ch2\":");
  Serial.print(applyDeadband(ch2_value, RC_DEADBAND));
  Serial.print(",\"ch9\":");
  Serial.print(ch9_value);
  Serial.print(",\"valid\":");
  Serial.print(signal_valid ? "true" : "false");
  Serial.print(",\"mode\":");
  Serial.print(current_mode);
  Serial.print(",\"emergency\":");
  Serial.print(emergency_stop ? "true" : "false");
  
  // Enhanced sensor data reporting
  Serial.print(",\"sensors\":{");
  Serial.print("\"left_distance\":");
  if (sensors.left_valid) {
    Serial.print(sensors.left_distance, 1);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"right_distance\":");
  if (sensors.right_valid) {
    Serial.print(sensors.right_distance, 1);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"min_distance\":");
  if (sensors.min_distance < 999.0) {
    Serial.print(sensors.min_distance, 1);
  } else {
    Serial.print("null");
  }
  Serial.print(",\"obstacle_direction\":");
  Serial.print(sensors.obstacle_direction);
  Serial.print(",\"left_valid\":");
  Serial.print(sensors.left_valid ? "true" : "false");
  Serial.print(",\"right_valid\":");
  Serial.print(sensors.right_valid ? "true" : "false");
  Serial.print("}");
  
  // Legacy distance field for backward compatibility
  Serial.print(",\"distance\":");
  if (sensors.min_distance < 999.0) {
    Serial.print(sensors.min_distance, 1);
  } else {
    Serial.print("null");
  }
  
  Serial.print(",\"pi_command_valid\":");
  Serial.print(pi_command.valid ? "true" : "false");
  Serial.print(",\"gatekeeper_mode\":\"");
  switch(current_mode) {
    case MODE_FAILSAFE: Serial.print("FAILSAFE"); break;
    case MODE_MANUAL: Serial.print("MANUAL"); break;
    case MODE_AUTONOMOUS: Serial.print("AUTONOMOUS"); break;
  }
  Serial.print("\"");
  
  // Include Pi telemetry data if available
  if (pi_telemetry.length() > 0 && (millis() - last_telemetry_update < 2000)) {
    Serial.print(",\"pi_telemetry\":");
    Serial.print(pi_telemetry);
  }
  
  Serial.println("}");
}

int applyDeadband(int value, int deadband) {
  if (abs(value) < deadband) {
    return 0;
  }
  return value;
}

void updateStatusLED() {
  static unsigned long last_blink = 0;
  static bool led_state = false;
  unsigned long current_time = millis();
  
  if (emergency_stop && current_mode == MODE_AUTONOMOUS) {
    // Variable blink rate based on obstacle direction
    int blink_rate = 100;  // Default fast blink
    if (sensors.obstacle_direction == -1) {
      blink_rate = 150;  // Slower blink for left obstacle
    } else if (sensors.obstacle_direction == 1) {
      blink_rate = 80;   // Faster blink for right obstacle
    }
    
    if (current_time - last_blink > blink_rate) {
      led_state = !led_state;
      digitalWrite(LED_PIN, led_state);
      last_blink = current_time;
    }
  } else if (current_mode == MODE_AUTONOMOUS && pi_command.valid) {
    // Solid when applying Pi commands (safety systems active)
    digitalWrite(LED_PIN, HIGH);
  } else if (current_mode == MODE_MANUAL) {
    // Double blink pattern for manual mode (safety systems disabled)
    unsigned long pattern = current_time % 1000;
    if (pattern < 100 || (pattern >= 200 && pattern < 300)) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
  } else if (signal_valid) {
    // Slow blink when RC valid but not autonomous
    if (current_time - last_blink > 500) {
      led_state = !led_state;
      digitalWrite(LED_PIN, led_state);
      last_blink = current_time;
    }
  } else {
    // Very slow blink when no signal
    if (current_time - last_blink > 1000) {
      led_state = !led_state;
      digitalWrite(LED_PIN, led_state);
      last_blink = current_time;
    }
  }
}