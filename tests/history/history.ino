// Carpe Lucem - opschuiftest
// Gebruikt precies dezelfde manier van opschuiven als de displaymodule, maar
// met goed zichtbare kleuren. Elke halve seconde komt er vooraan een nieuwe
// kleur bij en schuift de rest een plek op.

#include <Adafruit_NeoPixel.h>

#define LED_PIN 6
#define LED_COUNT 60

const unsigned long STEP_MS = 500;

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

unsigned long lastStep = 0;
uint16_t step = 0;

void shiftHistory() {
  for (int i = LED_COUNT - 1; i > 0; i--) {
    strip.setPixelColor(i, strip.getPixelColor(i - 1));
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  Serial.println("\nCarpe Lucem opschuiftest");
  strip.begin();
  strip.clear();
  strip.show();
}

void loop() {
  if (millis() - lastStep < STEP_MS) return;
  lastStep = millis();

  // Elke vijfde stap een felle kleur, daartussen donker. Zo zie je losse
  // blokjes over de strip lopen.
  uint32_t color = 0;
  switch (step % 5) {
    case 0: color = strip.Color(60, 0, 0); break;
    case 1: color = strip.Color(0, 60, 0); break;
    case 2: color = strip.Color(0, 0, 60); break;
  }
  step++;

  shiftHistory();
  strip.setPixelColor(0, color);
  strip.show();

  Serial.print("strip:");
  for (uint8_t i = 0; i < 10; i++) {
    uint32_t c = strip.getPixelColor(i);
    Serial.printf(" %u,%u,%u", (uint8_t)(c >> 16), (uint8_t)(c >> 8), (uint8_t)c);
  }
  Serial.println();
}
