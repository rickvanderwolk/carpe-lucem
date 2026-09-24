// Carpe Lucem - SD-kaart testen
// Probeert de kaart op vier snelheden te starten en herhaalt dat elke 5
// seconden, zodat je tijdens het kijken aan de bedrading kunt voelen.

#include <SPI.h>
#include <SD.h>

#define SD_CS_PIN 10
#define SD_MOSI_PIN 11
#define SD_SCK_PIN 12
#define SD_MISO_PIN 13

const uint32_t SPEEDS[] = {20000000UL, 4000000UL, 1000000UL, 400000UL};

const char *cardType() {
  switch (SD.cardType()) {
    case CARD_NONE: return "geen";
    case CARD_MMC:  return "MMC";
    case CARD_SD:   return "SD";
    case CARD_SDHC: return "SDHC";
    default:        return "onbekend";
  }
}

void listFiles() {
  File root = SD.open("/");
  if (!root) return;
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    Serial.printf("      %s  %lu bytes\n", f.name(), (unsigned long)f.size());
    f.close();
  }
  root.close();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  Serial.println("\nCarpe Lucem SD-test");
}

void loop() {
  Serial.println("\n--- poging ---");
  for (uint8_t i = 0; i < 4; i++) {
    SD.end();
    SPI.end();
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    bool ok = SD.begin(SD_CS_PIN, SPI, SPEEDS[i]);
    Serial.printf("  %8lu Hz: %s", (unsigned long)SPEEDS[i], ok ? "OK" : "mislukt");
    if (ok) {
      Serial.printf("   kaart: %s, %llu MB\n", cardType(), SD.cardSize() / (1024ULL * 1024ULL));
      listFiles();
      Serial.println("  gelukt, verder hoeft niet");
      break;
    }
    Serial.println();
  }
  delay(5000);
}
