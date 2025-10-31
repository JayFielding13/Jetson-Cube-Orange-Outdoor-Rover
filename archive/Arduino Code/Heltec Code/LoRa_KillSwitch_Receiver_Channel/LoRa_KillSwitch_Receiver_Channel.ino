/*
 * LoRa Kill Switch - RECEIVER with Channel Separation
 * For Heltec LoRa WiFi 32 V3
 *
 * Receives STOP/AUTONOMOUS commands and forwards to Raspberry Pi via Serial
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

// Optional: LED or relay pin for visual status indicator
#define STATUS_LED_PIN  21  // Optional - set to -1 to disable

// LoRa frequency (must match transmitter)
#define LORA_FREQ   915.0

// ===== CHANNEL CONFIGURATION =====
// MUST MATCH the transmitter's sync word
// Change this to match your rover's transmitter
#define SYNC_WORD   0x12  // <-- CHANGE THIS to match transmitter

SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);

// State variables
String currentMode = "STOP";
int lastRSSI = 0;
String lastCommand = "NONE";
unsigned long lastMessageTime = 0;
int messageCount = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("LoRa Kill Switch - Receiver (Channel Mode)");
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
  display.drawString(0, 0, "LoRa Receiver");
  display.drawString(0, 12, "Channel Mode");
  display.drawString(0, 24, "Initializing...");
  display.display();

  delay(1000);

  // Setup optional status LED
  if (STATUS_LED_PIN > 0) {
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
  }

  // Initialize LoRa
  Serial.print("Initializing LoRa... ");
  int state = radio.begin(LORA_FREQ);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("success!");

    // Set LoRa parameters (must match transmitter)
    radio.setSpreadingFactor(10);
    radio.setBandwidth(125.0);
    radio.setCodingRate(8);
    radio.setOutputPower(22);

    // Set sync word for channel separation - MUST MATCH TRANSMITTER
    radio.setSyncWord(SYNC_WORD);

    Serial.print("Sync word set to: 0x");
    Serial.println(SYNC_WORD, HEX);

    updateDisplay();
  } else {
    Serial.print("failed, code ");
    Serial.println(state);
    lastCommand = "INIT FAIL";
    updateDisplay();
    while (true);
  }

  // Start listening
  radio.startReceive();

  Serial.println("Ready. Waiting for commands...");
  Serial.println("Rover mode: STOP (safe default)");

  // Send initial status to Pi
  sendToPi("STOP");
}

void loop() {
  String message;

  // Check if message received
  int state = radio.receive(message);

  if (state == RADIOLIB_ERR_NONE) {
    // Message received successfully
    lastRSSI = radio.getRSSI();
    lastMessageTime = millis();
    messageCount++;

    Serial.print("Received [");
    Serial.print(messageCount);
    Serial.print("]: ");
    Serial.print(message);
    Serial.print(" | RSSI: ");
    Serial.print(lastRSSI);
    Serial.println(" dBm");

    // Process command
    if (message == "AUTONOMOUS") {
      currentMode = "AUTONOMOUS";
      lastCommand = "AUTONOMOUS";
      if (STATUS_LED_PIN > 0) digitalWrite(STATUS_LED_PIN, HIGH);
      Serial.println(">>> MODE: AUTONOMOUS <<<");
      sendToPi("AUTONOMOUS");

    } else if (message == "STOP") {
      currentMode = "STOP";
      lastCommand = "STOP";
      if (STATUS_LED_PIN > 0) digitalWrite(STATUS_LED_PIN, LOW);
      Serial.println(">>> MODE: STOP <<<");
      sendToPi("STOP");

    } else {
      lastCommand = "UNKNOWN";
      Serial.println("Unknown command received");
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

void sendToPi(String command) {
  // Send formatted command to Raspberry Pi via Serial
  // Format: MODE:command
  Serial.print("MODE:");
  Serial.println(command);

  // Also send as JSON for easier parsing (optional)
  Serial.print("{\"mode\":\"");
  Serial.print(command);
  Serial.print("\",\"rssi\":");
  Serial.print(lastRSSI);
  Serial.print(",\"time\":");
  Serial.print(millis());
  Serial.println("}");
}

void updateDisplay() {
  display.clear();

  // Title with channel indicator
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "RX - Ch:0x" + String(SYNC_WORD, HEX));

  // Current mode (large text)
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 15, currentMode);

  // Last command received
  display.setFont(ArialMT_Plain_10);
  String cmdStr = "Cmd: " + lastCommand;
  display.drawString(0, 35, cmdStr);

  // RSSI and message count
  if (lastMessageTime > 0) {
    String rssiStr = "RSSI:" + String(lastRSSI) + " Msg:" + String(messageCount);
    display.drawString(0, 50, rssiStr);
  } else {
    display.drawString(0, 50, "Waiting...");
  }

  display.display();
}
