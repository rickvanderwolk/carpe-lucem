// Carpe Lucem - I2C-zoeker
// Zoekt uit op welke pinnen de sensoren zitten. Twee stappen:
// 1. Per pin kijken of er een pull-up aan hangt. Een gevoed sensorbordje trekt
//    zijn SDA en SCL hoog, dus zo zie je of er überhaupt iets aan zit.
// 2. Alle combinaties van die pinnen proberen als SDA/SCL en scannen.

#include <Wire.h>

const uint8_t PINS[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 21};
const uint8_t PIN_COUNT = sizeof(PINS);

uint8_t pulled[16];
uint8_t pulledCount = 0;

const char *note(uint8_t pin) {
#if CONFIG_IDF_TARGET_ESP32C3
  if (pin == 8) return "  (let op: hier zit het blauwe ledje van de C3 op)";
  if (pin == 9) return "  (let op: hier zit de BOOT-knop op)";
#endif
  return "";
}

const char *chipName(uint8_t addr) {
  if (addr == 0x10) return "VEML7700";
  if (addr == 0x39) return "AS7341";
  return "onbekend";
}

void findPullups() {
  Serial.println("\n1. Pinnen met een pull-up (daar hangt iets aan):");
  pulledCount = 0;
  for (uint8_t i = 0; i < PIN_COUNT; i++) {
    pinMode(PINS[i], INPUT_PULLDOWN);
    delay(2);
    if (digitalRead(PINS[i]) == HIGH) {
      Serial.printf("   GPIO %u%s\n", PINS[i], note(PINS[i]));
      if (pulledCount < sizeof(pulled)) pulled[pulledCount++] = PINS[i];
    }
    pinMode(PINS[i], INPUT);
  }
  if (pulledCount == 0) {
    Serial.println("   geen enkele. De sensoren krijgen waarschijnlijk geen stroom,");
    Serial.println("   of hun GND zit niet aan de GND van de C3.");
  }
}

// Scant één combinatie; geeft het aantal gevonden adressen terug.
uint8_t scanPair(uint8_t sda, uint8_t scl) {
  Wire.end();
  if (!Wire.begin(sda, scl, 100000)) return 0;
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) continue;
    if (found == 0) Serial.printf("   SDA %u, SCL %u:", sda, scl);
    Serial.printf("  0x%02X %s", addr, chipName(addr));
    found++;
  }
  if (found) Serial.println();
  return found;
}

void tryCombinations() {
  Serial.println("2. Combinaties proberen:");
  uint8_t hits = 0;

  for (uint8_t a = 0; a < pulledCount; a++) {
    for (uint8_t b = 0; b < pulledCount; b++) {
      if (a == b) continue;
      hits += scanPair(pulled[a], pulled[b]);
    }
  }

  if (hits == 0 && pulledCount < 2) {
    // Toch de gebruikelijke paren proberen, voor het geval de pull-up-test niets zag.
    const uint8_t PAIRS[][2] = {{6, 7}, {7, 6}, {8, 9}, {9, 8}, {4, 5}, {5, 4},
                                {0, 1}, {1, 0}, {2, 3}, {3, 2}, {10, 20}, {20, 10}};
    for (uint8_t i = 0; i < sizeof(PAIRS) / 2; i++) {
      hits += scanPair(PAIRS[i][0], PAIRS[i][1]);
    }
  }

  if (hits == 0) Serial.println("   niets gevonden");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  Serial.println("\nCarpe Lucem I2C-zoeker");
}

void loop() {
  findPullups();
  tryCombinations();
  delay(5000);
}
