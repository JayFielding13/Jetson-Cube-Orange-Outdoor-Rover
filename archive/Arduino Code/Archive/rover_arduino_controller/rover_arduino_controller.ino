#include <PinChangeInterrupt.h>

// Pin definitions
#define CH1_PIN 2  // Forward/Reverse (Throttle)
#define CH2_PIN 3  // Steering
#define CH9_PIN 4  // Mode Switch (3-position)
#define LED_PIN 13 // Status LED

// RC signal parameters
#define RC_MIN_PULSE 1000
#define RC_MAX_PULSE 2000
#define RC_CENTER_PULSE 1500
#define RC_DEADBAND 50
#define RC_TIMEOUT 100  // milliseconds

// Serial communication
#define SERIAL_BAUD 115200
#define SERIAL_TIMEOUT 10

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

// Data structure for serial communication
struct RCData {
  int ch1_value;
  int ch2_value;
  int ch9_value;
  bool valid;
  unsigned long timestamp;
};

void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(SERIAL_TIMEOUT);
  
  // Configure pins
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH9_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Attach pin change interrupts
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CH1_PIN), ch1_interrupt, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CH2_PIN), ch2_interrupt, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CH9_PIN), ch9_interrupt, CHANGE);
  
  // Initialize timing
  last_valid_signal = millis();
  
  // Status indication
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("Arduino RC Controller Initialized");
}

void loop() {
  // Check signal validity
  checkSignalValidity();
  
  // Process and send RC data
  processRCData();
  
  // Update status LED
  updateStatusLED();
  
  // Small delay for stability
  delay(10);
}

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