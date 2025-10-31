/*
 * LoRa Kill Switch - RECEIVER
 * For Heltec HT-WB32LAF
 *
 * Receives enable/disable commands and controls relay
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

// Relay control pin (change to your actual pin)
#define RELAY_PIN   21

// LoRa frequency (must match transmitter)
// 915.0 for US, 868.0 for EU, 433.0 for Asia
#define LORA_FREQ   915.0

SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);

bool roverEnabled = false;
int lastRSSI = 0;
String lastCommand = "NONE";
unsigned long lastMessageTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("LoRa Kill Switch - Receiver");

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
  display.drawString(0, 12, "RECEIVER");
  display.drawString(0, 24, "Initializing...");
  display.display();

  delay(1000);

  // Setup relay pin
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Start with rover OFF (safe default)

  // Initialize LoRa
  Serial.print("Initializing LoRa... ");
  int state = radio.begin(LORA_FREQ);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("success!");
  } else {
    Serial.print("failed, code ");
    Serial.println(state);
    lastCommand = "INIT FAIL";
    updateDisplay();
    while (true);
  }

  // Set LoRa parameters (must match transmitter)
  radio.setSpreadingFactor(10);
  radio.setBandwidth(125.0);
  radio.setCodingRate(8);
  radio.setOutputPower(22);

  // Start listening
  radio.startReceive();

  Serial.println("Ready. Waiting for commands...");
  Serial.println("Rover is OFF (safe mode)");
}

void loop() {
  String message;

  // Check if message received
  int state = radio.receive(message);

  if (state == RADIOLIB_ERR_NONE) {
    // Message received successfully
    lastRSSI = radio.getRSSI();
    lastMessageTime = millis();

    Serial.print("Received: ");
    Serial.print(message);
    Serial.print(" | RSSI: ");
    Serial.print(lastRSSI);
    Serial.println(" dBm");

    // Process command
    if (message == "ENABLE") {
      digitalWrite(RELAY_PIN, HIGH);
      roverEnabled = true;
      lastCommand = "ENABLE";
      Serial.println(">>> ROVER ENABLED <<<");
    } else if (message == "DISABLE") {
      digitalWrite(RELAY_PIN, LOW);
      roverEnabled = false;
      lastCommand = "DISABLE";
      Serial.println(">>> ROVER DISABLED <<<");
    } else {
      lastCommand = "UNKNOWN";
      Serial.println("Unknown command");
    }

    updateDisplay();

    // Start listening again
    radio.startReceive();

  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    // Timeout is normal, just keep listening
    radio.startReceive();
  } else {
    // Some other error
    Serial.print("Receive failed, code ");
    Serial.println(state);
    radio.startReceive();
  }

  delay(10);
}

void updateDisplay() {
  display.clear();

  // Title
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "RECEIVER");

  // Rover status (large text)
  display.setFont(ArialMT_Plain_16);
  if (roverEnabled) {
    display.drawString(0, 15, "ROVER ON");
  } else {
    display.drawString(0, 15, "ROVER OFF");
  }

  // Last command
  display.setFont(ArialMT_Plain_10);
  String cmdStr = "Cmd: " + lastCommand;
  display.drawString(0, 35, cmdStr);

  // RSSI
  if (lastMessageTime > 0) {
    String rssiStr = "RSSI: " + String(lastRSSI) + " dBm";
    display.drawString(0, 50, rssiStr);
  } else {
    display.drawString(0, 50, "Waiting...");
  }

  display.display();
}
