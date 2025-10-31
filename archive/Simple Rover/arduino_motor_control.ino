/*
 * Obstacle-Avoidance Rover - Arduino Motor Controller (MIXED MODE)
 *
 * The MDDS30 is in MIXED R/C mode where:
 * - Channel 1 (D6 - Left): THROTTLE (forward/backward)
 * - Channel 2 (D5 - Right): STEERING (left/right)
 *
 * The driver automatically mixes these to control both motors
 *
 * Based on testing:
 * - 5F (D5=1700): Both motors forward
 * - 5B (D5=1300): Both motors backward
 * - 6F (D6=1700): Turn pattern (left back, right forward)
 * - 6B (D6=1300): Turn pattern (left forward, right back)
 */

#include <Servo.h>

const int STEERING_PIN = 5;  // D5 - Channel 2 (Steering)
const int THROTTLE_PIN = 6;  // D6 - Channel 1 (Throttle)

Servo throttle;  // Channel 1
Servo steering;  // Channel 2

const int RC_CENTER = 1500;
const int RC_FORWARD = 1700;
const int RC_BACKWARD = 1300;

void setup() {
  Serial.begin(115200);
  delay(100);

  throttle.attach(THROTTLE_PIN);
  steering.attach(STEERING_PIN);
  delay(50);

  // Initialize to center
  throttle.writeMicroseconds(RC_CENTER);
  steering.writeMicroseconds(RC_CENTER);

  Serial.println("READY: Mixed mode motor controller");
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() == 0) return;

    char command = cmd.charAt(0);
    int speed = 0;

    if (cmd.length() > 1) {
      speed = cmd.substring(1).toInt();
      speed = constrain(speed, 0, 255);
    }

    // Map speed to pulse width
    int pulse = map(speed, 0, 255, RC_CENTER, RC_FORWARD);
    int reversePulse = map(speed, 0, 255, RC_CENTER, RC_BACKWARD);

    if (command == 'F' || command == 'f') {
      // FORWARD: Steering forward, throttle center
      steering.writeMicroseconds(pulse);
      throttle.writeMicroseconds(RC_CENTER);
      Serial.print("OK: ");
      Serial.println(cmd);
    }
    else if (command == 'B' || command == 'b') {
      // BACKWARD: Steering backward, throttle center
      steering.writeMicroseconds(reversePulse);
      throttle.writeMicroseconds(RC_CENTER);
      Serial.print("OK: ");
      Serial.println(cmd);
    }
    else if (command == 'L' || command == 'l') {
      // TURN LEFT: Based on observation (6F pattern)
      throttle.writeMicroseconds(pulse);
      steering.writeMicroseconds(RC_CENTER);
      Serial.print("OK: ");
      Serial.println(cmd);
    }
    else if (command == 'R' || command == 'r') {
      // TURN RIGHT: Based on observation (6B pattern)
      throttle.writeMicroseconds(reversePulse);
      steering.writeMicroseconds(RC_CENTER);
      Serial.print("OK: ");
      Serial.println(cmd);
    }
    else if (command == 'S' || command == 's') {
      // STOP: Both center
      steering.writeMicroseconds(RC_CENTER);
      throttle.writeMicroseconds(RC_CENTER);
      Serial.print("OK: ");
      Serial.println(cmd);
    }
    else {
      Serial.print("Unknown: ");
      Serial.println(cmd);
    }
  }
}
