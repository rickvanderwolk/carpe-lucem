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

// Lange-afstandsmodus van ESP-NOW: langzamer zenden, maar twee tot vier keer
// meer bereik. Beide modules moeten hierin hetzelfde staan, anders horen ze
// elkaar niet meer.
const bool LONG_RANGE = true;

// Hoe lang doet de strip over één volledige cyclus (alle LEDs vol).
//   60000UL * 5           ->  5 min   (elke 5 s een LED)
//   60000UL * 15          -> 15 min   (elke 15 s een LED)
//   60000UL * 60          ->  1 uur
//   60000UL * 60 * 24     -> 24 uur
//   60000UL * 60 * 24 * 7 ->  1 week
const unsigned long TOTAL_TIME_MS = 60000UL * 15;

const unsigned long LED_INTERVAL_MS = TOTAL_TIME_MS / LED_COUNT;

// true: elke LED is het gemiddelde licht van alle metingen in zijn interval.
// false: elke LED is de laatste meting (momentopname).
const bool AVERAGE = true;

// Hoe lux wordt omgerekend naar helderheid.
//   BRIGHT_LOG    : logaritmisch, zoals je ogen licht ervaren
//   BRIGHT_POWER  : tussenweg tussen logaritmisch en recht evenredig
//   BRIGHT_LINEAR : lux / 4, zoals de eerste versie
enum BrightnessMode { BRIGHT_LOG, BRIGHT_POWER, BRIGHT_LINEAR };
const BrightnessMode BRIGHTNESS_MODE = BRIGHT_LOG;

const float LUX_MIN = 1.0;        // hieronder blijft de LED uit
const float LUX_MAX = 50000.0;    // hierboven staat hij vol
const float POWER_GAMMA = 2.5;    // alleen voor BRIGHT_POWER

const uint8_t SATURATION_PCT = 80;

// Print bij elke nieuwe LED de eerste acht LEDs van de strip, om te controleren
// dat de geschiedenis opschuift.
const bool DEBUG_PIXELS = false;

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

// Cijfers over de verbinding, om te zien hoe ver de modules uit elkaar kunnen.
// Alles sinds het opstarten, en alles sinds de vorige LED.
uint32_t statReceived = 0;   // metingen die aankwamen
uint32_t statMissed = 0;     // metingen waarvan geen enkele kopie aankwam
uint32_t statCopies = 0;     // extra kopieën van metingen die er al waren
uint32_t ledReceived = 0;
uint32_t ledMissed = 0;
uint32_t ledCopies = 0;

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

// Helderheid van 0 tot 1 voor een gemeten lux-waarde.
float brightnessFor(float lux) {
  if (lux <= LUX_MIN) return 0;
  if (BRIGHTNESS_MODE == BRIGHT_LINEAR) return min(lux / 4.0f, 255.0f) / 255.0f;
  if (lux >= LUX_MAX) return 1;
  if (BRIGHTNESS_MODE == BRIGHT_POWER) return powf(lux / LUX_MAX, 1.0f / POWER_GAMMA);
  return logf(lux / LUX_MIN) / logf(LUX_MAX / LUX_MIN);
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

  float brightness = brightnessFor(m.lux);
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
  if (LONG_RANGE) esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW start mislukt");
    return;
  }
  esp_now_register_recv_cb(onReceive);
  Serial.println("ESP-NOW OK, wacht op sensor");
}

const char *resetReason() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "stroom aan";
    case ESP_RST_SW:       return "software (bijv. na flashen)";
    case ESP_RST_PANIC:    return "crash";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "watchdog";
    case ESP_RST_BROWNOUT: return "brownout: spanning zakte te ver weg";
    default:               return "anders";
  }
}

void collect(const Measurement &m) {
  if (receivedAny && m.seq == lastSeq) {
    statCopies++;   // kopie van een meting die we al hebben
    ledCopies++;
    return;
  }
  if (receivedAny && m.seq < lastSeq) {
    Serial.println("sensor opnieuw opgestart");
    statReceived = statMissed = statCopies = 0;
  } else if (receivedAny && m.seq > lastSeq + 1) {
    statMissed += m.seq - lastSeq - 1;
    ledMissed += m.seq - lastSeq - 1;
  }
  receivedAny = true;
  statReceived++;
  ledReceived++;
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

  uint32_t ledTotal = ledReceived + ledMissed;
  uint32_t total = statReceived + statMissed;
  Serial.printf("LED uit %lu meting(en)\tlux %.1f\tRGB %u,%u,%u"
                "\t| nu: gemist %lu van %lu, kopieën %.1f"
                "\t| totaal: gemist %.0f%% van %lu\n",
                (unsigned long)sampleCount, l.lux, r, g, b,
                (unsigned long)ledMissed, (unsigned long)ledTotal,
                ledReceived ? 1.0 + (double)ledCopies / ledReceived : 0.0,
                total ? 100.0 * statMissed / total : 0.0, (unsigned long)total);
  ledReceived = ledMissed = ledCopies = 0;

  if (DEBUG_PIXELS) {
    Serial.print("   strip:");
    for (uint8_t i = 0; i < 8 && i < LED_COUNT; i++) {
      uint32_t c = strip.getPixelColor(i);
      Serial.printf(" %u,%u,%u", (uint8_t)(c >> 16), (uint8_t)(c >> 8), (uint8_t)c);
    }
    Serial.println();
  }

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
  Serial.printf("opgestart door: %s (%d)\n", resetReason(), esp_reset_reason());
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
