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

// Ultrasonic sensor pins (for emergency stop only)
#define FRONT_TRIG_PIN 8   // Front ultrasonic trigger
#define FRONT_ECHO_PIN 12  // Front ultrasonic echo

// RC signal parameters (more lenient for stability)
#define RC_MIN_PULSE 1000
#define RC_MAX_PULSE 2000
#define RC_DEADBAND 50
#define RC_TIMEOUT 250  // Increased from 100ms to 250ms for stability

// Serial communication
#define SERIAL_BAUD 115200

// Emergency stop parameters
#define EMERGENCY_DISTANCE 10.0  // Emergency stop at 10cm
#define ULTRASONIC_TIMEOUT 5000  // 5ms timeout (fast!)

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
ControlMode stable_mode = MODE_FAILSAFE;
unsigned long mode_change_time = 0;
#define MODE_STABILITY_TIME 300  // 300ms stability before mode change

// Emergency stop
bool emergency_stop = false;
unsigned long last_emergency_check = 0;

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
  
  Serial.println("Arduino Simple Bridge v3.0 Initialized");
}

void loop() {
  // High frequency tasks - keep these fast and simple
  checkSignalValidity();
  updateControlMode();
  
  // Emergency stop check (every 100ms for safety)
  if (millis() - last_emergency_check >= 100) {
    checkEmergencyStop();
    last_emergency_check = millis();
  }
  
  // Handle control modes
  if (current_mode == MODE_MANUAL) {
    processRCControl();
  } else if (current_mode == MODE_AUTONOMOUS) {
    processPiCommands();
  } else {
    stopMotors(); // Failsafe
  }
  
  // Send data to Pi (every 200ms to reduce sensor noise)
  static unsigned long last_data_send = 0;
  if (millis() - last_data_send >= 200) {
    sendDataToPi();
    last_data_send = millis();
  }
  
  // Update status LED
  updateStatusLED();
  
  // Fast loop for responsive motor control
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
  
  // Check if all channels have recent updates
  bool ch1_fresh = (current_time - ch1_last_time) < RC_TIMEOUT;
  bool ch2_fresh = (current_time - ch2_last_time) < RC_TIMEOUT;
  bool ch9_fresh = (current_time - ch9_last_time) < RC_TIMEOUT;
  
  // More lenient validation - only require CH9 (mode switch) to be fresh
  // CH1 and CH2 can be stale if we're not moving the sticks
  signal_valid = ch9_fresh;
  
  if (signal_valid) {
    last_valid_signal = current_time;
  }
}

void updateControlMode() {
  // Simplified, more reliable mode detection
  int ch9_value = map(constrain(ch9_pulse_width, RC_MIN_PULSE, RC_MAX_PULSE), 
                     RC_MIN_PULSE, RC_MAX_PULSE, -1000, 1000);
  
  ControlMode new_mode;
  
  // Determine mode based purely on CH9 value, ignore signal_valid for mode detection
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

void processRCControl() {
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
  
  // Apply motor commands (with emergency stop check)
  setMotorSpeeds(left_speed, right_speed);
}

void processPiCommands() {
  // Check for motor commands from Pi
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Parse motor command JSON: {"left":speed,"right":speed}
    if (command.startsWith("{") && command.endsWith("}")) {
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
        
        // Apply motor commands (with emergency stop check)
        setMotorSpeeds(leftSpeed, rightSpeed);
      }
    }
  }
}

void setMotorSpeeds(int left_speed, int right_speed) {
  // Emergency stop override
  if (emergency_stop) {
    left_speed = 0;
    right_speed = 0;
  }
  
  // Constrain speeds
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
  // Quick emergency stop check
  float distance = readUltrasonicDistance();
  
  if (distance > 0) {
    emergency_stop = (distance < EMERGENCY_DISTANCE);
  }
}

float readUltrasonicDistance() {
  // Send ultrasonic pulse
  digitalWrite(FRONT_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(FRONT_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(FRONT_TRIG_PIN, LOW);
  
  // Read echo
  unsigned long duration = pulseIn(FRONT_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT);
  
  if (duration == 0) {
    return -1; // Timeout
  }
  
  // Calculate distance
  float distance = (duration * 0.0343) / 2;
  
  // Validate range
  if (distance < 2 || distance > 400) {
    return -1;
  }
  
  return distance;
}

void sendDataToPi() {
  // Send compact data to Pi including ultrasonic distance
  int ch1_value = map(constrain(ch1_pulse_width, RC_MIN_PULSE, RC_MAX_PULSE), 
                     RC_MIN_PULSE, RC_MAX_PULSE, -1000, 1000);
  int ch2_value = map(constrain(ch2_pulse_width, RC_MIN_PULSE, RC_MAX_PULSE), 
                     RC_MIN_PULSE, RC_MAX_PULSE, -1000, 1000);
  int ch9_value = map(constrain(ch9_pulse_width, RC_MIN_PULSE, RC_MAX_PULSE), 
                     RC_MIN_PULSE, RC_MAX_PULSE, -1000, 1000);
  
  // Read current distance
  float current_distance = readUltrasonicDistance();
  
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
  Serial.print(",\"distance\":");
  if (current_distance > 0) {
    Serial.print(current_distance, 1);
  } else {
    Serial.print("null");
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
  
  if (emergency_stop) {
    // Fast blink for emergency
    if (current_time - last_blink > 100) {
      led_state = !led_state;
      digitalWrite(LED_PIN, led_state);
      last_blink = current_time;
    }
  } else if (signal_valid) {
    // Solid when signal valid
    digitalWrite(LED_PIN, HIGH);
  } else {
    // Slow blink when no signal
    if (current_time - last_blink > 500) {
      led_state = !led_state;
      digitalWrite(LED_PIN, led_state);
      last_blink = current_time;
    }
  }
}