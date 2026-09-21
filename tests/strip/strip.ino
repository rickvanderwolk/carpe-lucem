// Carpe Lucem - strip-test
// Loopt een paar testpatronen door om te zien of de ledstrip goed werkt op de ESP32.
// Helderheid staat laag zodat de strip op USB-stroom kan draaien.

#include <Adafruit_NeoPixel.h>

#define LED_PIN 6
#define LED_COUNT 60
#define BRIGHTNESS 25   // van 255; wit is dan ~350 mA voor 60 LEDs

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void fill(uint32_t color, const char *name) {
  Serial.println(name);
  strip.fill(color);
  strip.show();
  delay(1500);
}

// Eén lichtje loopt de hele strip af: elke LED moet één keer oplichten.
void chase() {
  Serial.println("loopt van begin naar eind");
  for (int i = 0; i < LED_COUNT; i++) {
    strip.clear();
    strip.setPixelColor(i, strip.Color(255, 255, 255));
    strip.show();
    delay(30);
  }
}

void rainbow() {
  Serial.println("regenboog");
  for (uint16_t offset = 0; offset < 65535 / 2; offset += 256) {
    for (int i = 0; i < LED_COUNT; i++) {
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(offset + i * 65536L / LED_COUNT)));
    }
    strip.show();
    delay(15);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  Serial.println();
  Serial.println("Carpe Lucem strip-test");

  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.show();
}

void loop() {
  fill(strip.Color(255, 0, 0), "rood");
  fill(strip.Color(0, 255, 0), "groen");
  fill(strip.Color(0, 0, 255), "blauw");
  fill(strip.Color(255, 255, 255), "wit");
  chase();
  rainbow();
  fill(0, "uit");
}
