/*
 * LoRa Kill Switch - TRANSMITTER with Channel Separation
 * For Heltec LoRa WiFi 32 V3
 *
 * Uses momentary button to toggle between STOP and AUTONOMOUS modes
 * Implements sync word for channel separation (won't interfere with other rovers)
 */

#include <RadioLib.h>
#include <Wire.h>
#include "HT_SSD1306Wire.h"

// OLED display setup using Heltec library
SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// LoRa module pins for Heltec LoRa WiFi 32 V3
#define LORA_SCK    9
#define LORA_MISO   11
#define LORA_MOSI   10
#define LORA_CS     8
#define LORA_RST    12
#define LORA_DIO1   14
#define LORA_BUSY   13

// Momentary button pin (using safe GPIO that doesn't conflict with OLED)
#define BUTTON_PIN  47  // GPIO 47 - button switch

// LED pin (for illuminated button)
#define LED_PIN     48  // GPIO 48 - button LED

// LoRa frequency (915.0 for US, 868.0 for EU, 433.0 for Asia)
#define LORA_FREQ   915.0

// ===== CHANNEL CONFIGURATION =====
// Change this sync word to create different "channels"
// Each rover should have a unique sync word
// Transmitter and receiver MUST have matching sync words
// Examples:
//   Rover 1: 0x12
//   Rover 2: 0x34
//   Rover 3: 0x56
#define SYNC_WORD   0x12  // <-- CHANGE THIS for each rover pair

SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);

// State variables
bool currentMode = false;  // false = STOP, true = AUTONOMOUS
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
int messageCount = 0;
bool lastSendSuccess = false;

// LED blinking variables
unsigned long lastBlinkTime = 0;
bool ledBlinkState = false;
#define BLINK_INTERVAL 500  // Blink every 500ms in STOP mode

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("LoRa Kill Switch - Transmitter (Channel Mode)");
  Serial.print("Sync Word (Channel): 0x");
  Serial.println(SYNC_WORD, HEX);

  // Turn on Vext power for display (critical!)
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW); // LOW = ON
  delay(100);

  // Initialize display
  display.init();
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "LoRa Transmitter");
  display.drawString(0, 12, "Channel Mode");
  display.drawString(0, 24, "Initializing...");
  display.display();

  delay(1000);

  // Setup button pin with internal pull-up
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Setup LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Start with LED off

  // Initialize LoRa
  Serial.print("Initializing LoRa... ");
  int state = radio.begin(LORA_FREQ);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("success!");

    // Set LoRa parameters for better range
    radio.setSpreadingFactor(10);
    radio.setBandwidth(125.0);
    radio.setCodingRate(8);
    radio.setOutputPower(22); // Max power

    // Set sync word for channel separation
    radio.setSyncWord(SYNC_WORD);

    Serial.print("Sync word set to: 0x");
    Serial.println(SYNC_WORD, HEX);

    updateDisplay("STOP", "Ready", true);
  } else {
    Serial.print("failed, code ");
    Serial.println(state);
    updateDisplay("LoRa FAILED", "Check wiring", false);
    while (true);
  }

  Serial.println("Ready. Press button to toggle STOP/AUTONOMOUS");

  // Send initial STOP command
  sendCommand(false);
}

void loop() {
  // Update LED based on current mode
  updateLED();

  // Read button with debouncing
  bool reading = digitalRead(BUTTON_PIN);

  // If button state changed, reset debounce timer
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  // Check if button state has been stable for debounce period
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If button is pressed (LOW because of pull-up)
    if (reading == LOW && lastButtonState == HIGH) {
      // Toggle mode
      currentMode = !currentMode;
      sendCommand(currentMode);

      Serial.print("Button pressed - Mode: ");
      Serial.println(currentMode ? "AUTONOMOUS" : "STOP");

      // Wait for button release to avoid multiple triggers
      while (digitalRead(BUTTON_PIN) == LOW) {
        delay(10);
      }
    }
  }

  lastButtonState = reading;
  delay(10);
}

void sendCommand(bool autonomous) {
  String message;
  String displayStatus;

  if (autonomous) {
    message = "AUTONOMOUS";
    displayStatus = "AUTONOMOUS";
    Serial.println("Sending: AUTONOMOUS");
  } else {
    message = "STOP";
    displayStatus = "STOP";
    Serial.println("Sending: STOP");
  }

  // Send message
  int state = radio.transmit(message);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Message sent successfully!");
    lastSendSuccess = true;
    messageCount++;
    updateDisplay(displayStatus, "Sent OK", true);
  } else {
    Serial.print("Send failed, code ");
    Serial.println(state);
    lastSendSuccess = false;
    updateDisplay(displayStatus, "SEND FAILED", false);
  }
}

void updateLED() {
  if (currentMode) {
    // AUTONOMOUS mode - LED solid ON
    digitalWrite(LED_PIN, HIGH);
  } else {
    // STOP mode - LED blinks
    unsigned long currentTime = millis();
    if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
      lastBlinkTime = currentTime;
      ledBlinkState = !ledBlinkState;
      digitalWrite(LED_PIN, ledBlinkState ? HIGH : LOW);
    }
  }
}

void updateDisplay(String mode, String status, bool success) {
  display.clear();

  // Title with channel indicator
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "TX - Ch:0x" + String(SYNC_WORD, HEX));

  // Current mode (larger text)
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 15, mode);

  // Status
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 35, status);

  // Message count and success indicator
  String countStr = "Msgs: " + String(messageCount);
  display.drawString(0, 50, countStr);

  // Success/Fail indicator
  if (messageCount > 0) {
    if (success) {
      display.drawString(90, 50, "[OK]");
    } else {
      display.drawString(85, 50, "[FAIL]");
    }
  }

  display.display();
}
