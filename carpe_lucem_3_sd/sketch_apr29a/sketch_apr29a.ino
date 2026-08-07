#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_AS7341.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 6
#define LED_COUNT 60
#define SD_CS_PIN 10

#define SLOT_SIZE 96
#define SLOT_COUNT LED_COUNT

// 60000UL = 1h strip (1 LED/min) -- standaard
// 2000UL = 2 min strip (snel testen)
// 60000UL * 24 = 24h strip
const unsigned long intervalMs = 2000UL;

const char CSV_NAME[] = "licht.csv";

Adafruit_VEML7700 veml = Adafruit_VEML7700();
Adafruit_AS7341 as7341;
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

unsigned long lastRead = 0;
uint16_t nextSlot = 0;
uint32_t nextSeq = 1;

int clamp255(int value) {
  if (value < 0) return 0;
  if (value > 255) return 255;
  return value;
}

void computeColor(float lux,
                  uint16_t ch415, uint16_t ch445, uint16_t ch480, uint16_t ch515,
                  uint16_t ch555, uint16_t ch590, uint16_t ch630, uint16_t ch680,
                  int *outR, int *outG, int *outB) {
  long rawR = (long)ch630 + (long)ch680 + ((long)ch590 / 2);
  long rawG = (long)ch515 + (long)ch555;
  long rawB = (long)ch445 + (long)ch480 + ((long)ch415 / 2);
  long maxRGB = rawR;
  if (rawG > maxRGB) maxRGB = rawG;
  if (rawB > maxRGB) maxRGB = rawB;
  if (maxRGB == 0) maxRGB = 1;
  int r = clamp255((rawR * 255L) / maxRGB);
  int g = clamp255((rawG * 255L) / maxRGB);
  int b = clamp255((rawB * 255L) / maxRGB);
  int brightness = clamp255((int)(lux / 4.0));
  *outR = (r * brightness) / 255;
  *outG = (g * brightness) / 255;
  *outB = (b * brightness) / 255;
}

void shiftHistory() {
  for (int i = LED_COUNT - 1; i > 0; i--) {
    strip.setPixelColor(i, strip.getPixelColor(i - 1));
  }
}

bool parseSeqFromFile(File &f, uint32_t *seq) {
  char buf[12];
  uint8_t i = 0;
  int c = f.read();
  while (c != ',' && c != ' ' && c != -1 && i < 11) {
    if (c < '0' || c > '9') return false;
    buf[i++] = (char)c;
    c = f.read();
  }
  if (i == 0) return false;
  buf[i] = 0;
  *seq = strtoul(buf, NULL, 10);
  return true;
}

bool parseSlotFromFile(File &f, float *lux,
                       uint16_t *ch415, uint16_t *ch445, uint16_t *ch480, uint16_t *ch515,
                       uint16_t *ch555, uint16_t *ch590, uint16_t *ch630, uint16_t *ch680) {
  int c = f.read();
  if (c == ' ' || c == -1 || c < '0' || c > '9') return false;

  while (c != ',' && c != -1) c = f.read();
  if (c == -1) return false;

  c = f.read();
  while (c != ',' && c != -1) c = f.read();
  if (c == -1) return false;

  char buf[12];
  uint8_t i;

  i = 0;
  c = f.read();
  while (c != ',' && c != -1 && i < 11) { buf[i++] = (char)c; c = f.read(); }
  buf[i] = 0;
  *lux = atof(buf);
  if (c == -1) return false;

  uint16_t* channels[] = {ch415, ch445, ch480, ch515, ch555, ch590, ch630, ch680};
  for (uint8_t j = 0; j < 8; j++) {
    i = 0;
    c = f.read();
    while (c != ',' && c != ' ' && c != '\r' && c != '\n' && c != -1 && i < 11) {
      buf[i++] = (char)c;
      c = f.read();
    }
    buf[i] = 0;
    *channels[j] = (uint16_t)atoi(buf);
    if (c == -1 && j < 7) return false;
  }
  return true;
}

void initRing() {
  bool needCreate = true;

  if (SD.exists(CSV_NAME)) {
    File f = SD.open(CSV_NAME, FILE_READ);
    if (f && f.size() == (uint32_t)SLOT_COUNT * SLOT_SIZE) {
      needCreate = false;
      uint32_t bestSeq = 0;
      int bestSlot = -1;
      for (uint16_t s = 0; s < SLOT_COUNT; s++) {
        f.seek((uint32_t)s * SLOT_SIZE);
        uint32_t seq;
        if (parseSeqFromFile(f, &seq)) {
          if (seq > bestSeq) { bestSeq = seq; bestSlot = (int)s; }
        }
      }
      if (bestSlot >= 0) {
        nextSlot = (bestSlot + 1) % SLOT_COUNT;
        nextSeq = bestSeq + 1;
        Serial.print(F("RING HERVAT seq="));
        Serial.print(bestSeq);
        Serial.print(F(" slot="));
        Serial.println(bestSlot);
      } else {
        nextSlot = 0;
        nextSeq = 1;
        Serial.println(F("RING LEEG"));
      }
      f.close();
    } else {
      if (f) f.close();
      SD.remove(CSV_NAME);
    }
  }

  if (needCreate) {
    Serial.println(F("RING NIEUW"));
    File f = SD.open(CSV_NAME, FILE_WRITE);
    if (!f) {
      Serial.println(F("RING CREATE FOUT"));
      return;
    }
    for (uint16_t s = 0; s < SLOT_COUNT; s++) {
      for (uint16_t i = 0; i < SLOT_SIZE - 2; i++) f.write((uint8_t)' ');
      f.write((uint8_t)'\r');
      f.write((uint8_t)'\n');
    }
    f.close();
    nextSlot = 0;
    nextSeq = 1;
  }
}

bool writeSlot(uint16_t slot, uint32_t seq, float lux,
               uint16_t ch415, uint16_t ch445, uint16_t ch480, uint16_t ch515,
               uint16_t ch555, uint16_t ch590, uint16_t ch630, uint16_t ch680) {
  File f = SD.open(CSV_NAME, FILE_WRITE);
  if (!f) {
    Serial.println(F("SD OPEN FOUT"));
    return false;
  }
  uint32_t startPos = (uint32_t)slot * SLOT_SIZE;
  f.seek(startPos);
  f.print(seq); f.print(',');
  f.print(millis()); f.print(',');
  f.print(lux, 2); f.print(',');
  f.print(ch415); f.print(',');
  f.print(ch445); f.print(',');
  f.print(ch480); f.print(',');
  f.print(ch515); f.print(',');
  f.print(ch555); f.print(',');
  f.print(ch590); f.print(',');
  f.print(ch630); f.print(',');
  f.print(ch680);
  uint32_t written = f.position() - startPos;
  while (written < SLOT_SIZE - 2) { f.write((uint8_t)' '); written++; }
  f.write((uint8_t)'\r');
  f.write((uint8_t)'\n');
  f.close();
  return true;
}

void restoreStrip() {
  File f = SD.open(CSV_NAME, FILE_READ);
  if (!f) return;

  for (int pixel = 0; pixel < LED_COUNT; pixel++) {
    uint16_t s = (nextSlot + SLOT_COUNT - 1 - pixel) % SLOT_COUNT;
    f.seek((uint32_t)s * SLOT_SIZE);

    float lux;
    uint16_t ch415, ch445, ch480, ch515, ch555, ch590, ch630, ch680;
    if (parseSlotFromFile(f, &lux, &ch415, &ch445, &ch480, &ch515, &ch555, &ch590, &ch630, &ch680)) {
      int r, g, b;
      computeColor(lux, ch415, ch445, ch480, ch515, ch555, ch590, ch630, ch680, &r, &g, &b);
      strip.setPixelColor(pixel, strip.Color(r, g, b));
    } else {
      strip.setPixelColor(pixel, 0);
    }
  }

  f.close();
  strip.show();
}

void setup() {
  Serial.begin(9600);
  delay(1500);
  Serial.println(F("BOOT"));

  if (!veml.begin()) {
    Serial.println(F("VEML7700 NIET GEVONDEN"));
    while (1);
  }
  Serial.println(F("VEML7700 OK"));

  if (!as7341.begin()) {
    Serial.println(F("AS7341 NIET GEVONDEN"));
    while (1);
  }
  Serial.println(F("AS7341 OK"));

  pinMode(SD_CS_PIN, OUTPUT);
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD KAART NIET GEVONDEN"));
    while (1);
  }
  Serial.println(F("SD OK"));

  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_256X);

  strip.begin();
  strip.show();

  initRing();
  restoreStrip();

  lastRead = millis() - intervalMs;
  Serial.println(F("GESTART"));
}

void loop() {
  if (millis() - lastRead < intervalMs) return;
  lastRead = millis();

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

  bool ok = writeSlot(nextSlot, nextSeq, lux, ch415, ch445, ch480, ch515, ch555, ch590, ch630, ch680);
  Serial.print(F("LOG "));
  Serial.print(ok ? F("OK") : F("FOUT"));
  Serial.print(F(" seq="));
  Serial.print(nextSeq);
  Serial.print(F(" slot="));
  Serial.println(nextSlot);

  if (ok) {
    nextSlot = (nextSlot + 1) % SLOT_COUNT;
    nextSeq++;
  }

  int r, g, b;
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
  Serial.print(b);
  Serial.print(F(" | CH: "));
  Serial.print(ch415);
  Serial.print(',');
  Serial.print(ch445);
  Serial.print(',');
  Serial.print(ch480);
  Serial.print(',');
  Serial.print(ch515);
  Serial.print(',');
  Serial.print(ch555);
  Serial.print(',');
  Serial.print(ch590);
  Serial.print(',');
  Serial.print(ch630);
  Serial.print(',');
  Serial.println(ch680);
}
