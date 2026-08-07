#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_AS7341.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 6
#define LED_COUNT 60
#define SD_CS_PIN 10

// Hoe lang doet de strip over één volledige cyclus (alle LEDs vol).
//   60000UL * 2           ->  2 min   (snel testen)
//   60000UL * 60          ->  1 uur
//   60000UL * 60 * 24     -> 24 uur
//   60000UL * 60 * 24 * 7 ->  1 week
const unsigned long TOTAL_TIME_MS = 60000UL * 60 * 24;

const unsigned long intervalMs = TOTAL_TIME_MS / LED_COUNT;

const uint8_t SATURATION_PCT = 80;

// Logbestand groeit ~80 bytes per meting. Boven deze grens: roteren (oud weg, nieuw begin).
const uint32_t MAX_FILE_BYTES = 150000UL;
const char CSV_NAME[] = "LICHT.CSV";

Adafruit_VEML7700 veml = Adafruit_VEML7700();
Adafruit_AS7341 as7341;
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

unsigned long lastRead = 0;
uint32_t nextSeq = 1;
File logFile;
bool sdActive = false;

int clamp255(int v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return v;
}

void computeColor(float lux,
                  uint16_t ch415, uint16_t ch445, uint16_t ch480, uint16_t ch515,
                  uint16_t ch555, uint16_t ch590, uint16_t ch630, uint16_t ch680,
                  uint8_t *outR, uint8_t *outG, uint8_t *outB) {
  long rawR = (long)ch630 + (long)ch680 + ((long)ch590 / 2);
  long rawG = (long)ch515 + (long)ch555;
  long rawB = (long)ch445 + (long)ch480 + ((long)ch415 / 2);

  long minRGB = rawR;
  if (rawG < minRGB) minRGB = rawG;
  if (rawB < minRGB) minRGB = rawB;
  long maxRGB = rawR;
  if (rawG > maxRGB) maxRGB = rawG;
  if (rawB > maxRGB) maxRGB = rawB;

  long stretchMin = (minRGB * SATURATION_PCT) / 100;
  long range = maxRGB - stretchMin;
  if (range <= 0) range = 1;

  int r = clamp255(((rawR - stretchMin) * 255L) / range);
  int g = clamp255(((rawG - stretchMin) * 255L) / range);
  int b = clamp255(((rawB - stretchMin) * 255L) / range);

  int brightness = clamp255((int)(lux / 4.0));
  *outR = (uint8_t)(((long)r * brightness) / 255);
  *outG = (uint8_t)(((long)g * brightness) / 255);
  *outB = (uint8_t)(((long)b * brightness) / 255);
}

void shiftHistory() {
  for (int i = LED_COUNT - 1; i > 0; i--) {
    strip.setPixelColor(i, strip.getPixelColor(i - 1));
  }
}

bool parseAndPush(const char *line) {
  if (line[0] < '0' || line[0] > '9') return false;
  const char *p = line;
  while (*p && *p != ',') p++;
  if (!*p) return false; p++;
  while (*p && *p != ',') p++;
  if (!*p) return false; p++;

  float lux = atof(p);
  while (*p && *p != ',') p++;
  if (!*p) return false; p++;

  uint16_t ch[8];
  for (uint8_t i = 0; i < 8; i++) {
    ch[i] = (uint16_t)atoi(p);
    while (*p && *p != ',') p++;
    if (i < 7) {
      if (!*p) return false;
      p++;
    }
  }

  uint8_t r, g, b;
  computeColor(lux, ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7], &r, &g, &b);
  shiftHistory();
  strip.setPixelColor(0, strip.Color(r, g, b));
  return true;
}

uint32_t findLastSeqInLogFile() {
  uint32_t size = logFile.size();
  if (size < 2) return 0;
  uint32_t pos = size;
  while (pos > 0) {
    pos--;
    logFile.seek(pos);
    int c = logFile.read();
    if (c == '\n' && pos + 1 < size) {
      logFile.seek(pos + 1);
      int d = logFile.read();
      if (d >= '0' && d <= '9') {
        uint32_t seq = (uint32_t)(d - '0');
        while (true) {
          int e = logFile.read();
          if (e < '0' || e > '9') break;
          seq = seq * 10 + (uint32_t)(e - '0');
        }
        return seq;
      }
    }
  }
  return 0;
}

void restoreStripFromLog() {
  uint32_t size = logFile.size();
  if (size < 70) return;

  // Lees ruim genoeg om alle LED-regels te dekken (~120 bytes per regel max).
  uint32_t window = (uint32_t)LED_COUNT * 120UL;
  uint32_t seekPos = (size > window) ? size - window : 0;
  logFile.seek(seekPos);

  if (seekPos > 0) {
    while (logFile.available()) {
      int c = logFile.read();
      if (c == '\n') break;
      if (c == -1) return;
    }
  }

  char buf[100];
  uint8_t bufIdx = 0;
  uint16_t restored = 0;

  while (logFile.available()) {
    int c = logFile.read();
    if (c == '\n' || c == '\r') {
      if (bufIdx > 5) {
        buf[bufIdx] = 0;
        if (parseAndPush(buf)) restored++;
      }
      bufIdx = 0;
    } else if (bufIdx < sizeof(buf) - 1) {
      buf[bufIdx++] = (char)c;
    }
  }
  Serial.print(F("RESTORED "));
  Serial.println(restored);
}

void writeHeader() {
  logFile.println(F("seq,millis,lux,ch415,ch445,ch480,ch515,ch555,ch590,ch630,ch680"));
  logFile.flush();
}

bool isHeaderValid() {
  if (logFile.size() < 4) return false;
  logFile.seek(0);
  char hdr[4];
  for (uint8_t i = 0; i < 4; i++) hdr[i] = (char)logFile.read();
  return (hdr[0] == 's' && hdr[1] == 'e' && hdr[2] == 'q' && hdr[3] == ',');
}

// Probeer log te openen. Bij falen: sdActive blijft false en alles SD-gerelateerd
// wordt overgeslagen. Strip + sensor blijven werken zonder log/herstel.
void tryOpenLog() {
  for (uint8_t attempt = 1; attempt <= 3; attempt++) {
    logFile = SD.open(CSV_NAME, FILE_WRITE);
    if (logFile) { sdActive = true; return; }
    delay(100);
  }
  sdActive = false;
}

void resetLogFile() {
  if (logFile) logFile.close();
  SD.remove(CSV_NAME);
  tryOpenLog();
}

void initLog() {
  tryOpenLog();
  if (!sdActive) {
    Serial.println(F("LOG NIET BESCHIKBAAR - draait zonder log"));
    nextSeq = 1;
    return;
  }

  bool needFresh = false;
  if (logFile.size() == 0) {
    needFresh = true;
    Serial.println(F("LOG NIEUW"));
  } else if (!isHeaderValid()) {
    resetLogFile();
    if (!sdActive) { Serial.println(F("LOG NIET BESCHIKBAAR")); nextSeq = 1; return; }
    needFresh = true;
    Serial.println(F("LOG ONGELDIG -> NIEUW"));
  } else if (logFile.size() > MAX_FILE_BYTES) {
    resetLogFile();
    if (!sdActive) { Serial.println(F("LOG NIET BESCHIKBAAR")); nextSeq = 1; return; }
    needFresh = true;
    Serial.println(F("LOG GEROTEERD"));
  }

  if (needFresh) {
    writeHeader();
    nextSeq = 1;
  } else {
    nextSeq = findLastSeqInLogFile() + 1;
    Serial.print(F("LOG HERVAT seq="));
    Serial.println(nextSeq);
  }
}

void logEntry(uint32_t seq, float lux,
              uint16_t ch415, uint16_t ch445, uint16_t ch480, uint16_t ch515,
              uint16_t ch555, uint16_t ch590, uint16_t ch630, uint16_t ch680) {
  if (!sdActive) return;
  logFile.seek(logFile.size());
  logFile.print(seq); logFile.print(',');
  logFile.print(millis()); logFile.print(',');
  logFile.print(lux, 2); logFile.print(',');
  logFile.print(ch415); logFile.print(',');
  logFile.print(ch445); logFile.print(',');
  logFile.print(ch480); logFile.print(',');
  logFile.print(ch515); logFile.print(',');
  logFile.print(ch555); logFile.print(',');
  logFile.print(ch590); logFile.print(',');
  logFile.print(ch630); logFile.print(',');
  logFile.println(ch680);
  logFile.flush();
}

void bootAnimation() {
  for (int pos = 0; pos < LED_COUNT + 5; pos++) {
    for (int i = 0; i < LED_COUNT; i++) {
      int dist = pos - i;
      if (dist < 0 || dist > 4) {
        strip.setPixelColor(i, 0);
      } else {
        uint8_t b = 80 >> dist;
        strip.setPixelColor(i, strip.Color(b, b, b));
      }
    }
    strip.show();
    delay(18);
  }
  for (int i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, 0);
  strip.show();
}

void takeSample() {
  float lux = veml.readLux();

  uint16_t readings[12];
  if (!as7341.readAllChannels(readings)) {
    Serial.println(F("AS7341 LEESFOUT"));
    return;
  }

  uint16_t ch415 = readings[0];
  uint16_t ch445 = readings[1];
  uint16_t ch480 = readings[2];
  uint16_t ch515 = readings[3];
  uint16_t ch555 = readings[6];
  uint16_t ch590 = readings[7];
  uint16_t ch630 = readings[8];
  uint16_t ch680 = readings[9];

  logEntry(nextSeq, lux, ch415, ch445, ch480, ch515, ch555, ch590, ch630, ch680);
  Serial.print(sdActive ? F("LOG OK seq=") : F("LOG SKIP seq="));
  Serial.println(nextSeq);
  nextSeq++;

  uint8_t r, g, b;
  computeColor(lux, ch415, ch445, ch480, ch515, ch555, ch590, ch630, ch680, &r, &g, &b);

  shiftHistory();
  strip.setPixelColor(0, strip.Color(r, g, b));
  strip.show();

  Serial.print(F("Lux: "));
  Serial.print(lux, 2);
  Serial.print(F(" | RGB: "));
  Serial.print(r);
  Serial.print(',');
  Serial.print(g);
  Serial.print(',');
  Serial.println(b);
}

void setup() {
  Serial.begin(9600);
  delay(1500);
  Serial.println(F("BOOT"));

  if (!veml.begin()) { Serial.println(F("VEML7700 NIET GEVONDEN")); while (1); }
  Serial.println(F("VEML7700 OK"));

  if (!as7341.begin()) { Serial.println(F("AS7341 NIET GEVONDEN")); while (1); }
  Serial.println(F("AS7341 OK"));

  pinMode(SD_CS_PIN, OUTPUT);
  if (!SD.begin((uint32_t)SPI_QUARTER_SPEED, SD_CS_PIN)) {
    Serial.println(F("SD KAART NIET GEVONDEN"));
    while (1);
  }
  Serial.println(F("SD OK (quarter speed)"));
  delay(150);

  // DIAGNOSE: kan Arduino het bestand lezen? Als ja, communicatie is OK
  // en is het probleem specifiek schrijven (module level-shifter? wiring?).
  File diag = SD.open(CSV_NAME, FILE_READ);
  if (diag) {
    Serial.print(F("DIAG READ ok, size="));
    Serial.print(diag.size());
    Serial.print(F(", eerste byte="));
    Serial.println((char)diag.read());
    diag.close();
  } else {
    Serial.println(F("DIAG READ FAIL - kan zelfs niet lezen"));
  }

  initLog();

  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_256X);

  strip.begin();
  strip.show();

  bootAnimation();

  restoreStripFromLog();
  strip.show();

  takeSample();
  lastRead = millis();
  Serial.println(F("GESTART"));
}

void loop() {
  if (millis() - lastRead < intervalMs) return;
  lastRead = millis();
  takeSample();
}
