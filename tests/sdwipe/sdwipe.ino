// Carpe Lucem - logbestanden wissen
// Eenmalig draaien om met een schone log te beginnen. Daarna weer de
// display-sketch flashen.

#include <SPI.h>
#include <SD.h>

#define SD_CS_PIN 10
#define SD_MOSI_PIN 11
#define SD_SCK_PIN 12
#define SD_MISO_PIN 13

const char *PATHS[] = {"/RUW.CSV", "/LEDS.CSV"};

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  Serial.println("\nCarpe Lucem logbestanden wissen");

  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, SPI)) {
    Serial.println("geen SD-kaart gevonden");
    return;
  }

  for (uint8_t i = 0; i < 2; i++) {
    if (!SD.exists(PATHS[i])) {
      Serial.printf("%s bestond niet\n", PATHS[i]);
    } else if (SD.remove(PATHS[i])) {
      Serial.printf("%s gewist\n", PATHS[i]);
    } else {
      Serial.printf("%s wissen mislukt\n", PATHS[i]);
    }
  }
  Serial.println("klaar, nu weer de display-sketch flashen");
}

void loop() {
  delay(1000);
}
