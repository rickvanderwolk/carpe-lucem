#include <Wire.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_AS7341.h>

Adafruit_VEML7700 veml = Adafruit_VEML7700();
Adafruit_AS7341 as7341;

unsigned long lastRead = 0;
const unsigned long intervalMs = 10000;

int clamp255(int value) {
if (value < 0) return 0;
if (value > 255) return 255;
return value;
}

void setup() {
Serial.begin(9600);
while (!Serial) {}

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

Serial.println("Beide sensoren gestart");
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

int brightness = (int)(lux / 4.0);
brightness = clamp255(brightness);

Serial.println("-----");
Serial.print("Lux: ");
Serial.println(lux, 2);

Serial.print("415nm: ");
Serial.print(ch415);
Serial.print("  445nm: ");
Serial.print(ch445);
Serial.print("  480nm: ");
Serial.print(ch480);
Serial.print("  515nm: ");
Serial.print(ch515);
Serial.print("  555nm: ");
Serial.print(ch555);
Serial.print("  590nm: ");
Serial.print(ch590);
Serial.print("  630nm: ");
Serial.print(ch630);
Serial.print("  680nm: ");
Serial.println(ch680);

Serial.print("RGB approx: ");
Serial.print(rgbR);
Serial.print(", ");
Serial.print(rgbG);
Serial.print(", ");
Serial.println(rgbB);

Serial.print("Brightness 0-255: ");
Serial.println(brightness);
}
