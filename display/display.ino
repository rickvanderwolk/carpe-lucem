// Carpe Lucem - displaymodule
// Ontvangt metingen van de sensormodule via ESP-NOW, middelt ze en zet elke
// LED_INTERVAL_MS een nieuwe LED vooraan op de strip; de rest schuift een plek op.

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 6
#define LED_COUNT 60

// Moet gelijk zijn aan sensor/sensor.ino.
#define ESPNOW_CHANNEL 1

// Hoe lang doet de strip over één volledige cyclus (alle LEDs vol).
//   60000UL * 5           ->  5 min   (demo: elke 5 s een LED)
//   60000UL * 60          ->  1 uur
//   60000UL * 60 * 24     -> 24 uur
//   60000UL * 60 * 24 * 7 ->  1 week
const unsigned long TOTAL_TIME_MS = 60000UL * 5;

const unsigned long LED_INTERVAL_MS = TOTAL_TIME_MS / LED_COUNT;

// true: elke LED is het gemiddelde licht van alle metingen in zijn interval.
// false: elke LED is de laatste meting (momentopname).
const bool AVERAGE = true;

const uint8_t SATURATION_PCT = 80;

// Na zoveel stilte een melding in de seriële monitor.
const unsigned long SILENCE_WARN_MS = 15000;

// Moet gelijk zijn aan sensor/sensor.ino.
const uint8_t PACKET_VERSION = 1;
struct __attribute__((packed)) Measurement {
  uint8_t  version;
  uint32_t seq;
  float    lux;
  uint8_t  gain;      // AS7341-gainstand, ter info
  uint32_t ch[10];    // 415, 445, 480, 515, 555, 590, 630, 680, clear, nir; omgerekend naar 512x
};

// Licht zoals het display ermee rekent: één meting of een gemiddelde.
struct Light {
  float lux;
  float ch[10];
};

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Van de ESP-NOW-callback naar loop().
QueueHandle_t inbox;

bool receivedAny = false;
uint32_t lastSeq = 0;
unsigned long lastReceived = 0;
unsigned long lastWarned = 0;

// Opgeteld licht sinds de vorige LED.
double sumLux = 0;
double sumCh[10] = {0};
uint32_t sampleCount = 0;
Light latest;

bool shownOnce = false;
unsigned long lastShown = 0;

uint8_t clamp255(float v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return (uint8_t)v;
}

// Kleur uit de verhouding tussen de kanalen, helderheid uit lux.
void computeColor(const Light &m, uint8_t *outR, uint8_t *outG, uint8_t *outB) {
  float rawR = m.ch[6] + m.ch[7] + m.ch[5] / 2.0f;   // 630 + 680 + ½ 590
  float rawG = m.ch[3] + m.ch[4];                    // 515 + 555
  float rawB = m.ch[1] + m.ch[2] + m.ch[0] / 2.0f;   // 445 + 480 + ½ 415

  float minRGB = min(rawR, min(rawG, rawB));
  float maxRGB = max(rawR, max(rawG, rawB));

  float stretchMin = minRGB * SATURATION_PCT / 100.0f;
  float range = maxRGB - stretchMin;
  if (range <= 0) range = 1;

  float brightness = clamp255(m.lux / 4.0f) / 255.0f;
  *outR = clamp255((rawR - stretchMin) * 255.0f / range * brightness);
  *outG = clamp255((rawG - stretchMin) * 255.0f / range * brightness);
  *outB = clamp255((rawB - stretchMin) * 255.0f / range * brightness);
}

void shiftHistory() {
  for (int i = LED_COUNT - 1; i > 0; i--) {
    strip.setPixelColor(i, strip.getPixelColor(i - 1));
  }
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
  strip.clear();
  strip.show();
}

// Draait in de wifi-taak: alleen controleren en doorgeven, de rest gebeurt in loop().
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if (len != sizeof(Measurement)) return;
  Measurement m;
  memcpy(&m, data, sizeof(m));
  if (m.version != PACKET_VERSION) return;
  xQueueSend(inbox, &m, 0);
}

void initRadio() {
  inbox = xQueueCreate(4, sizeof(Measurement));
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW start mislukt");
    return;
  }
  esp_now_register_recv_cb(onReceive);
  Serial.println("ESP-NOW OK, wacht op sensor");
}

void collect(const Measurement &m) {
  if (receivedAny && m.seq <= lastSeq) {
    Serial.println("sensor opnieuw opgestart");
  } else if (receivedAny && m.seq > lastSeq + 1) {
    Serial.printf("%lu meting(en) gemist\n", (unsigned long)(m.seq - lastSeq - 1));
  }
  receivedAny = true;
  lastSeq = m.seq;

  latest.lux = m.lux;
  sumLux += m.lux;
  for (uint8_t i = 0; i < 10; i++) {
    latest.ch[i] = m.ch[i];
    sumCh[i] += m.ch[i];
  }
  sampleCount++;
}

void showNextLed() {
  Light l = latest;
  if (AVERAGE) {
    l.lux = sumLux / sampleCount;
    for (uint8_t i = 0; i < 10; i++) l.ch[i] = sumCh[i] / sampleCount;
  }

  uint8_t r, g, b;
  computeColor(l, &r, &g, &b);
  shiftHistory();
  strip.setPixelColor(0, strip.Color(r, g, b));
  strip.show();

  Serial.printf("LED uit %lu meting(en)\tlux %.1f\tRGB %u,%u,%u\n",
                (unsigned long)sampleCount, l.lux, r, g, b);

  sumLux = 0;
  for (uint8_t i = 0; i < 10; i++) sumCh[i] = 0;
  sampleCount = 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  Serial.println();
  Serial.println("Carpe Lucem display");

  strip.begin();
  strip.show();
  bootAnimation();

  initRadio();
}

void loop() {
  Measurement m;
  if (xQueueReceive(inbox, &m, pdMS_TO_TICKS(50)) == pdTRUE) {
    lastReceived = millis();
    collect(m);
  }

  // Eerste LED meteen bij de eerste meting, daarna elke LED_INTERVAL_MS.
  bool due = shownOnce ? millis() - lastShown >= LED_INTERVAL_MS : sampleCount > 0;
  if (due) {
    lastShown = millis();
    if (sampleCount > 0) {
      showNextLed();
      shownOnce = true;
    } else {
      Serial.println("geen metingen in dit interval, LED overgeslagen");
    }
  }

  if (millis() - lastReceived > SILENCE_WARN_MS && millis() - lastWarned > SILENCE_WARN_MS) {
    Serial.println("geen metingen van de sensor");
    lastWarned = millis();
  }
}
