// Carpe Lucem - logbestanden uitlezen
// Print de logbestanden van de SD-kaart naar de seriële monitor, zodat je ze
// op je computer kunt opslaan of bekijken zonder het kaartje eruit te halen.

#include <SPI.h>
#include <SD.h>

#define SD_CS_PIN 10
#define SD_MOSI_PIN 11
#define SD_SCK_PIN 12
#define SD_MISO_PIN 13

// Zet op false om alleen de LED-regels te printen; RUW.CSV is veel groter.
const bool DUMP_RAW = true;

void dump(const char *path) {
  Serial.printf("\n===== %s =====\n", path);
  File f = SD.open(path, FILE_READ);
  if (!f) {
    Serial.println("(bestaat niet)");
    return;
  }
  Serial.printf("(%lu bytes)\n", (unsigned long)f.size());
  while (f.available()) {
    Serial.write(f.read());
  }
  f.close();
  Serial.printf("\n===== einde %s =====\n", path);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  delay(500);
  Serial.println("\nCarpe Lucem logbestanden uitlezen");

  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, SPI)) {
    Serial.println("geen SD-kaart gevonden");
    return;
  }

  dump("/LEDS.CSV");
  dump("/LEDS.OUD");
  if (DUMP_RAW) {
    dump("/RUW.CSV");
    dump("/RUW.OUD");
  }
  Serial.println("\nklaar");
}

void loop() {
  delay(1000);
}
