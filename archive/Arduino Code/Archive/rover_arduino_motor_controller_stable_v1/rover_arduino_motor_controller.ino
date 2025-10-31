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
  
  Serial.println("Arduino Motor Controller Initialized");
  Serial.println("Pins: LEFT(D5,D6,D9) RIGHT(D7,D10,D11)");
}

void loop() {
  // Check signal validity
  checkSignalValidity();
  
  // Process and send RC data
  processRCData();
  
  // Determine control mode
  updateControlMode();
  
  // Handle motor control based on mode
  if (current_mode == MODE_MANUAL) {
    processRCMotorControl();
  } else if (current_mode == MODE_AUTONOMOUS) {
    // Process incoming motor commands from Pi
    processMotorCommands();
  } else {
    // Failsafe mode - stop motors
    stopMotors();
  }
  
  // Update status LED
  updateStatusLED();
  
  // Small delay for stability
  delay(10);
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
  
  // Store current speeds
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
  
  // Apply motor commands
  setMotorSpeeds(left_speed, right_speed);
}

void updateStatusLED() {
  static unsigned long last_blink = 0;
  static bool led_state = false;
  unsigned long current_time = millis();
  
  if (signal_valid) {
    // Solid LED when signal is valid
    digitalWrite(LED_PIN, HIGH);
  } else {
    // Blink LED when signal is invalid
    if (current_time - last_blink > 500) {
      led_state = !led_state;
      digitalWrite(LED_PIN, led_state);
      last_blink = current_time;
    }
  }
}