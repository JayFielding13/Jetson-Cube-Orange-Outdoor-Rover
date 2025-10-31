/*
 * OLED Display Test for HT-WB32LAF
 * Tests multiple pin configurations to find the correct one
 */

#include <Wire.h>

// Pin combinations to test
struct PinConfig {
  int sda;
  int scl;
  const char* name;
};

PinConfig configs[] = {
  {17, 18, "SDA=17, SCL=18"},
  {18, 17, "SDA=18, SCL=17"},
  {41, 42, "SDA=41, SCL=42"},
  {21, 22, "SDA=21, SCL=22"},
  {4, 15, "SDA=4, SCL=15"},
  {5, 4, "SDA=5, SCL=4"}
};

void scanI2C(int sda, int scl) {
  Wire.end();
  Wire.begin(sda, scl);
  delay(100);

  byte count = 0;
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("  -> Found I2C device at address 0x");
      if (i < 16) Serial.print("0");
      Serial.println(i, HEX);
      count++;
    }
  }

  if (count == 0) {
    Serial.println("  -> No devices found");
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n\n=================================");
  Serial.println("OLED Display Pin Scanner");
  Serial.println("HT-WB32LAF");
  Serial.println("=================================\n");

  // Test all pin combinations
  int numConfigs = sizeof(configs) / sizeof(configs[0]);

  for (int i = 0; i < numConfigs; i++) {
    Serial.print("Testing ");
    Serial.println(configs[i].name);
    scanI2C(configs[i].sda, configs[i].scl);
    delay(200);
  }

  Serial.println("\n=================================");
  Serial.println("Scan complete!");
  Serial.println("=================================");
  Serial.println("\nIf a device was found at 0x3C,");
  Serial.println("that's your OLED display!");
  Serial.println("\nIf no devices found, the display");
  Serial.println("may be using SPI instead of I2C.");
}

void loop() {
  // Nothing to do
  delay(1000);
}
