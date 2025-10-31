/*

 * LoRa Kill Switch - TRANSMITTER
 * For Heltec HT-WB32LAF
 *
 * Reads toggle switch and sends enable/disable commands to receiver
 */

#include <RadioLib.h>
#include <Wire.h>
#include "HT_SSD1306Wire.h"

// OLED display setup using Heltec library
SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// LoRa module pins for HT-WB32LAF
#define LORA_SCK    9
#define LORA_MISO   11
#define LORA_MOSI   10
#define LORA_CS     8
#define LORA_RST    12
#define LORA_DIO1   14
#define LORA_BUSY   13

// Toggle switch pin (change to your actual pin)
#define TOGGLE_PIN  21

// LoRa frequency (adjust based on your region)
// 915.0 for US, 868.0 for EU, 433.0 for Asia
#define LORA_FREQ   915.0

SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);

bool lastSwitchState = LOW;
bool lastSendSuccess = false;
int messageCount = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("LoRa Kill Switch - Transmitter");

  // Turn on Vext power for display (critical!)
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW); // LOW = ON
  delay(100);

  // Initialize display
  display.init();
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "LoRa Kill Switch");
  display.drawString(0, 12, "TRANSMITTER");
  display.drawString(0, 24, "Initializing...");
  display.display();

  delay(1000);

  // Setup toggle switch pin
  pinMode(TOGGLE_PIN, INPUT_PULLUP);

  // Initialize LoRa
  Serial.print("Initializing LoRa... ");
  int state = radio.begin(LORA_FREQ);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("success!");
    updateDisplay("LoRa Ready", "Waiting...", false);
  } else {
    Serial.print("failed, code ");
    Serial.println(state);
    updateDisplay("LoRa FAILED", "Check wiring", false);
    while (true);
  }

  // Set LoRa parameters for better range
  radio.setSpreadingFactor(10);
  radio.setBandwidth(125.0);
  radio.setCodingRate(8);
  radio.setOutputPower(22); // Max power

  Serial.println("Ready. Toggle switch to send commands.");

  // Send initial state
  lastSwitchState = digitalRead(TOGGLE_PIN);
  sendCommand(lastSwitchState);
}

void loop() {
  bool currentState = digitalRead(TOGGLE_PIN);

  // Check if switch state changed
  if (currentState != lastSwitchState) {
    delay(50); // Debounce
    currentState = digitalRead(TOGGLE_PIN);

    if (currentState != lastSwitchState) {
      lastSwitchState = currentState;
      sendCommand(currentState);
    }
  }

  delay(100);
}

void sendCommand(bool switchState) {
  String message;
  String status;

  if (switchState == HIGH) {
    message = "ENABLE";
    status = "ROVER ON";
    Serial.println("Sending: ENABLE (Rover ON)");
  } else {
    message = "DISABLE";
    status = "ROVER OFF";
    Serial.println("Sending: DISABLE (Rover OFF)");
  }

  // Send message
  int state = radio.transmit(message);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Message sent successfully!");
    lastSendSuccess = true;
    messageCount++;
    updateDisplay(status, "Sent OK", true);
  } else {
    Serial.print("Send failed, code ");
    Serial.println(state);
    lastSendSuccess = false;
    updateDisplay(status, "SEND FAILED", false);
  }
}

void updateDisplay(String line1, String line2, bool success) {
  display.clear();

  // Title
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "TRANSMITTER");

  // Status (larger text)
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 15, line1);

  // Secondary info
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 35, line2);

  // Message count and status indicator
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
