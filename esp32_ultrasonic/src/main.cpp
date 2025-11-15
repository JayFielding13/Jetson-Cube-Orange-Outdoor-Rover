/*
 * ESP32 Ultrasonic Sensor Array - Raw Data Output
 *
 * Hardware: ESP32 DevKit (38-pin, Micro-USB)
 * Sensors: 6x AJ-SR04M waterproof ultrasonic sensors
 *
 * This firmware reads 6 ultrasonic sensors and sends raw distance data
 * to the Jetson Orin Nano via USB serial. No filtering or smoothing is
 * performed on the ESP32 - all processing happens on the Jetson.
 *
 * Pin Assignments (after level shifter for ECHO pins):
 * - Front:        GPIO 15 (TRIG), GPIO 2 (ECHO)
 * - Corner Left:  GPIO 13 (TRIG), GPIO 12 (ECHO)
 * - Corner Right: GPIO 5 (TRIG), GPIO 27 (ECHO)
 * - Side Left:    GPIO 14 (TRIG), GPIO 4 (ECHO)
 * - Side Right:   GPIO 9 (TRIG), GPIO 10 (ECHO)
 * - Rear:         GPIO 25 (TRIG), GPIO 26 (ECHO)
 *
 * Modes:
 * - DEBUG_MODE = true:  Human-readable output for hardware validation
 * - DEBUG_MODE = false: JSON output for Jetson consumption
 */

#include <Arduino.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

// Set to true for standalone hardware debugging, false for production
#define DEBUG_MODE false

// Serial baud rate (must match Jetson configuration)
#define SERIAL_BAUD 115200

// Sensor reading interval (milliseconds)
#define READ_INTERVAL_MS 100  // 10 Hz update rate

// Ultrasonic sensor timeout (microseconds) - 38ms for 6.5m max range
#define ULTRASONIC_TIMEOUT 38000

// Maximum valid distance (meters)
#define MAX_DISTANCE 6.0

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Front sensor (0°)
#define FRONT_TRIG_PIN 15
#define FRONT_ECHO_PIN 2

// Corner Left sensor (-45°)
#define CORNER_LEFT_TRIG_PIN 13
#define CORNER_LEFT_ECHO_PIN 12

// Corner Right sensor (+45°)
#define CORNER_RIGHT_TRIG_PIN 5
#define CORNER_RIGHT_ECHO_PIN 27

// Side Left sensor (-90°)
#define SIDE_LEFT_TRIG_PIN 14
#define SIDE_LEFT_ECHO_PIN 4

// Side Right sensor (+90°)
#define SIDE_RIGHT_TRIG_PIN 9
#define SIDE_RIGHT_ECHO_PIN 10

// Rear sensor (180°)
#define REAR_TRIG_PIN 25
#define REAR_ECHO_PIN 26

// ============================================================================
// SENSOR STRUCTURE
// ============================================================================

struct UltrasonicSensor {
  const char* name;
  int angle;        // Degrees from front (0° = front, ±180° = rear)
  int trigPin;
  int echoPin;
  float distance;   // Last measured distance in meters
  bool valid;       // Is the reading valid?
};

// Sensor array
UltrasonicSensor sensors[] = {
  {"front",        0,    FRONT_TRIG_PIN,        FRONT_ECHO_PIN,        0.0, false},
  {"corner_left",  -45,  CORNER_LEFT_TRIG_PIN,  CORNER_LEFT_ECHO_PIN,  0.0, false},
  {"corner_right", 45,   CORNER_RIGHT_TRIG_PIN, CORNER_RIGHT_ECHO_PIN, 0.0, false},
  {"side_left",    -90,  SIDE_LEFT_TRIG_PIN,    SIDE_LEFT_ECHO_PIN,    0.0, false},
  {"side_right",   90,   SIDE_RIGHT_TRIG_PIN,   SIDE_RIGHT_ECHO_PIN,   0.0, false},
  {"rear",         180,  REAR_TRIG_PIN,         REAR_ECHO_PIN,         0.0, false}
};

const int NUM_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

void readSensor(UltrasonicSensor* sensor);
void outputDebug();
void outputJSON();

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD);

  // Wait for serial port to initialize
  delay(1000);

  // Initialize all sensor pins
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensors[i].trigPin, OUTPUT);
    pinMode(sensors[i].echoPin, INPUT);
    digitalWrite(sensors[i].trigPin, LOW);
  }

  // Initial stabilization delay
  delay(100);

  // Print startup message
  if (DEBUG_MODE) {
    Serial.println("========================================");
    Serial.println("ESP32 Ultrasonic Sensor Array");
    Serial.println("DEBUG MODE - Hardware Validation");
    Serial.println("========================================");
    Serial.println("Sensors initialized:");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print("  ");
      Serial.print(sensors[i].name);
      Serial.print(" (");
      Serial.print(sensors[i].angle);
      Serial.print("°): TRIG=GPIO");
      Serial.print(sensors[i].trigPin);
      Serial.print(", ECHO=GPIO");
      Serial.println(sensors[i].echoPin);
    }
    Serial.println("========================================");
    Serial.println();
  } else {
    Serial.println("{\"status\":\"ready\",\"sensors\":6,\"mode\":\"raw\"}");
  }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  static unsigned long lastReadTime = 0;
  unsigned long currentTime = millis();

  // Read sensors at specified interval
  if (currentTime - lastReadTime >= READ_INTERVAL_MS) {
    lastReadTime = currentTime;

    // Read all sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
      readSensor(&sensors[i]);
    }

    // Output results
    if (DEBUG_MODE) {
      outputDebug();
    } else {
      outputJSON();
    }
  }
}

// ============================================================================
// SENSOR READING
// ============================================================================

void readSensor(UltrasonicSensor* sensor) {
  // Ensure trigger is low
  digitalWrite(sensor->trigPin, LOW);
  delayMicroseconds(2);

  // Send 10us trigger pulse
  digitalWrite(sensor->trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor->trigPin, LOW);

  // Measure echo pulse duration
  long duration = pulseIn(sensor->echoPin, HIGH, ULTRASONIC_TIMEOUT);

  // Calculate distance in meters
  // Speed of sound = 343 m/s = 0.0343 cm/us
  // Distance = (duration / 2) * 0.0343 cm/us = duration * 0.01715 cm/us
  // Convert to meters: distance_m = duration * 0.0001715

  if (duration == 0) {
    // Timeout - no echo received
    sensor->distance = MAX_DISTANCE;
    sensor->valid = false;
  } else {
    float distance_m = duration * 0.0001715;

    // Validate range
    if (distance_m < 0.02 || distance_m > MAX_DISTANCE) {
      sensor->distance = MAX_DISTANCE;
      sensor->valid = false;
    } else {
      sensor->distance = distance_m;
      sensor->valid = true;
    }
  }

  // Small delay between sensor readings to avoid cross-talk
  delay(10);
}

// ============================================================================
// DEBUG OUTPUT (Human-readable)
// ============================================================================

void outputDebug() {
  Serial.println("----------------------------------------");
  Serial.print("Time: ");
  Serial.print(millis() / 1000.0, 1);
  Serial.println(" s");
  Serial.println();

  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("  ");

    // Sensor name (padded for alignment)
    Serial.print(sensors[i].name);
    for (int j = strlen(sensors[i].name); j < 15; j++) {
      Serial.print(" ");
    }

    // Angle
    Serial.print("(");
    if (sensors[i].angle >= 0) Serial.print(" ");
    Serial.print(sensors[i].angle);
    Serial.print("°): ");

    // Distance
    if (sensors[i].valid) {
      Serial.print(sensors[i].distance, 2);
      Serial.println(" m");
    } else {
      Serial.println("TIMEOUT / OUT OF RANGE");
    }
  }

  Serial.println();
}

// ============================================================================
// JSON OUTPUT (Machine-readable for Jetson)
// ============================================================================

void outputJSON() {
  Serial.print("{\"timestamp\":");
  Serial.print(millis());
  Serial.print(",\"sensors\":[");

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (i > 0) Serial.print(",");

    Serial.print("{\"name\":\"");
    Serial.print(sensors[i].name);
    Serial.print("\",\"angle\":");
    Serial.print(sensors[i].angle);
    Serial.print(",\"distance\":");
    Serial.print(sensors[i].distance, 3);
    Serial.print(",\"valid\":");
    Serial.print(sensors[i].valid ? "true" : "false");
    Serial.print("}");
  }

  Serial.println("]}");
}

// ============================================================================
// END OF FILE
// ============================================================================
