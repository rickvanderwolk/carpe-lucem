#include <Wire.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_AS7341.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 6
#define LED_COUNT 60

Adafruit_VEML7700 veml = Adafruit_VEML7700();
Adafruit_AS7341 as7341;
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

unsigned long lastRead = 0;
const unsigned long intervalMs = 10000;

uint8_t ledR[LED_COUNT];
uint8_t ledG[LED_COUNT];
uint8_t ledB[LED_COUNT];

int clamp255(int value) {
if (value < 0) return 0;
if (value > 255) return 255;
return value;
}

void shiftHistory() {
for (int i = LED_COUNT - 1; i > 0; i--) {
ledR[i] = ledR[i - 1];
ledG[i] = ledG[i - 1];
ledB[i] = ledB[i - 1];
}
}

void drawStrip() {
for (int i = 0; i < LED_COUNT; i++) {
strip.setPixelColor(i, strip.Color(ledR[i], ledG[i], ledB[i]));
}
strip.show();
}

void setup() {
Serial.begin(9600);

if (!veml.begin()) {
Serial.println("VEML7700 niet gevonden");
while (1);
}

if (!as7341.begin()) {
Serial.println("AS7341 niet gevonden");
while (1);
}

as7341.setATIME(100);
as7341.setASTEP(999);
as7341.setGain(AS7341_GAIN_256X);

strip.begin();
strip.show();

for (int i = 0; i < LED_COUNT; i++) {
ledR[i] = 0;
ledG[i] = 0;
ledB[i] = 0;
}

Serial.println("Gestart");
}

void loop() {
if (millis() - lastRead < intervalMs) {
return;
}

lastRead = millis();

float lux = veml.readLux();

uint16_t readings[12];
if (!as7341.readAllChannels(readings)) {
Serial.println("AS7341 leesfout");
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

long rawR = (long)ch630 + (long)ch680 + ((long)ch590 / 2);
long rawG = (long)ch515 + (long)ch555;
long rawB = (long)ch445 + (long)ch480 + ((long)ch415 / 2);

long maxRGB = rawR;
if (rawG > maxRGB) maxRGB = rawG;
if (rawB > maxRGB) maxRGB = rawB;
if (maxRGB == 0) maxRGB = 1;

int rgbR = clamp255((rawR * 255L) / maxRGB);
int rgbG = clamp255((rawG * 255L) / maxRGB);
int rgbB = clamp255((rawB * 255L) / maxRGB);

int brightness = clamp255((int)(lux / 4.0));

rgbR = (rgbR * brightness) / 255;
rgbG = (rgbG * brightness) / 255;
rgbB = (rgbB * brightness) / 255;

shiftHistory();

ledR[0] = rgbR;
ledG[0] = rgbG;
ledB[0] = rgbB;

drawStrip();

Serial.print("Lux: ");
Serial.print(lux, 2);
Serial.print(" | RGB: ");
Serial.print(rgbR);
Serial.print(",");
Serial.print(rgbG);
Serial.print(",");
Serial.println(rgbB);
}
