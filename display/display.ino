// Carpe Lucem - displaymodule
// Ontvangt metingen van de sensormodule via ESP-NOW, middelt ze en zet elke
// LED_INTERVAL_MS een nieuwe LED vooraan op de strip; de rest schuift een plek op.

#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 6
#define LED_COUNT 60

// microSD-module (HW-125), gevoed met 5V van de ESP.
#define SD_CS_PIN 10
#define SD_MOSI_PIN 11
#define SD_SCK_PIN 12
#define SD_MISO_PIN 13

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
const unsigned long TOTAL_TIME_MS = 60000UL * 60 * 24;

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

// Volumeknop voor de hele strip: 1.0 is vol, 0.5 is halve sterkte. Alle LEDs
// worden even veel zachter, dus de onderlinge verschillen blijven hetzelfde.
const float MAX_BRIGHTNESS = 1.0;

const float LUX_MIN = 0.2;       // hieronder blijft de LED uit
const float LUX_MAX = 50000.0;    // hierboven staat hij vol
const float POWER_GAMMA = 2.5;    // alleen voor BRIGHT_POWER

// IJking van de sensor, gemeten aan een hele dag daglicht uit de eigen log:
// daglicht komt er binnen in de verhouding rood 2,46 : groen 1,56 : blauw 1,
// doordat silicium veel gevoeliger is voor rood dan voor blauw. Door daarvoor
// terug te rekenen is gemiddeld daglicht wit, en blijft licht dat echt warmer
// of koeler is dat ook. Verhuist de sensor naar een andere plek, dan hoort
// deze ijking opnieuw uit de log bepaald te worden.
const float WB_R = 0.41;
const float WB_G = 0.64;
const float WB_B = 1.00;

// IJking van de ledstrip zelf. Gelijke waarden voor rood, groen en blauw zien
// er koelwit uit, want de blauwe LED is relatief sterk. Met deze factoren ziet
// neutraal licht er ook neutraal uit. Stel ze gerust bij op het oog.
const float STRIP_R = 1.00;
const float STRIP_G = 0.92;
const float STRIP_B = 0.78;

// Hoe ver kleurverschillen worden uitvergroot. Lager is subtieler, 0 geeft
// precies de gemeten verhoudingen.
const uint8_t SATURATION_PCT = 90;

// Print bij elke nieuwe LED de eerste acht LEDs van de strip, om te controleren
// dat de geschiedenis opschuift.
const bool DEBUG_PIXELS = false;

// Twee logbestanden op de SD-kaart. RUW bevat elke meting, LEDS één regel per
// LED. Bij het opstarten wordt de strip uit LEDS hersteld. Boven de grens
// begint een bestand opnieuw, zodat de kaart niet volloopt.
const char RAW_PATH[] = "/RUW.CSV";
const char LED_PATH[] = "/LEDS.CSV";
const char RAW_OLD_PATH[] = "/RUW.OUD";
const char LED_OLD_PATH[] = "/LEDS.OUD";
const char RAW_HEADER[] = "seq,ms,lux,gain,ch415,ch445,ch480,ch515,ch555,ch590,ch630,ch680,clear,nir";
const char LED_HEADER[] = "seq,ms,metingen,lux,ch415,ch445,ch480,ch515,ch555,ch590,ch630,ch680,clear,nir";
const uint32_t RAW_MAX_BYTES = 200000000UL;  // ~2 jaar bij een meting per 30 s
const uint32_t LED_MAX_BYTES = 4000000UL;    // ~600 dagen bij 60 LEDs per dag

// Na zoveel stilte een melding in de seriële monitor.
const unsigned long SILENCE_WARN_MS = 180000;

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

bool sdActive = false;
uint32_t ledSeq = 1;

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
  float rawR = (m.ch[6] + m.ch[7] + m.ch[5] / 2.0f) * WB_R;   // 630 + 680 + ½ 590
  float rawG = (m.ch[3] + m.ch[4]) * WB_G;                    // 515 + 555
  float rawB = (m.ch[1] + m.ch[2] + m.ch[0] / 2.0f) * WB_B;   // 445 + 480 + ½ 415

  float minRGB = min(rawR, min(rawG, rawB));
  float maxRGB = max(rawR, max(rawG, rawB));

  float stretchMin = minRGB * SATURATION_PCT / 100.0f;
  float range = maxRGB - stretchMin;
  if (range <= 0) range = 1;

  float brightness = brightnessFor(m.lux) * MAX_BRIGHTNESS;
  *outR = clamp255((rawR - stretchMin) * 255.0f / range * brightness * STRIP_R);
  *outG = clamp255((rawG - stretchMin) * 255.0f / range * brightness * STRIP_G);
  *outB = clamp255((rawB - stretchMin) * 255.0f / range * brightness * STRIP_B);
}

void shiftHistory() {
  for (int i = LED_COUNT - 1; i > 0; i--) {
    strip.setPixelColor(i, strip.getPixelColor(i - 1));
  }
}

// Zet het licht als kleur vooraan op de strip en schuift de rest een plek op.
void pushColor(const Light &l, uint8_t *outR, uint8_t *outG, uint8_t *outB) {
  computeColor(l, outR, outG, outB);
  shiftHistory();
  strip.setPixelColor(0, strip.Color(*outR, *outG, *outB));
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

// ---- SD-kaart -------------------------------------------------------------

// Zorgt dat een bestand bestaat met de juiste kop. Is het vol, dan schuift het
// door naar het .OUD-bestand en begint een nieuw bestand, zodat je altijd de
// laatste twee blokken hebt. Let op: op de ESP32 maakt FILE_WRITE een bestand
// leeg, toevoegen gaat met FILE_APPEND.
bool ensureFile(const char *path, const char *oldPath, const char *header, uint32_t maxBytes) {
  bool fresh = !SD.exists(path);
  if (!fresh) {
    File f = SD.open(path, FILE_READ);
    if (!f) return false;
    uint32_t size = f.size();
    char head[5] = {0};
    for (uint8_t i = 0; i < 4 && f.available(); i++) head[i] = (char)f.read();
    f.close();

    if (strncmp(head, "seq,", 4) != 0) {
      SD.remove(path);            // onbruikbaar bestand
      fresh = true;
    } else if (size > maxBytes) {
      SD.remove(oldPath);
      SD.rename(path, oldPath);   // doorschuiven
      Serial.printf("%s was vol en is %s geworden\n", path, oldPath);
      fresh = true;
    }
  }
  if (!fresh) return true;

  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;
  f.println(header);
  f.close();
  return true;
}

void initSD() {
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  // Een paar pogingen: de module heeft soms even nodig, en een wat slapper
  // contact lukt bij de tweede poging vaak wel.
  bool mounted = false;
  for (uint8_t attempt = 1; attempt <= 4 && !mounted; attempt++) {
    mounted = SD.begin(SD_CS_PIN, SPI);
    if (!mounted) {
      SD.end();
      delay(250);
    }
  }
  if (!mounted) {
    Serial.println("geen SD-kaart gevonden, draait zonder log");
    return;
  }
  sdActive = ensureFile(RAW_PATH, RAW_OLD_PATH, RAW_HEADER, RAW_MAX_BYTES) &&
             ensureFile(LED_PATH, LED_OLD_PATH, LED_HEADER, LED_MAX_BYTES);
  Serial.println(sdActive ? "SD OK" : "SD-kaart gevonden, maar schrijven lukt niet");
  if (sdActive) {
    File raw = SD.open(RAW_PATH, FILE_READ);
    File led = SD.open(LED_PATH, FILE_READ);
    Serial.printf("   %s %lu bytes, %s %lu bytes\n",
                  RAW_PATH, (unsigned long)(raw ? raw.size() : 0),
                  LED_PATH, (unsigned long)(led ? led.size() : 0));
    if (raw) raw.close();
    if (led) led.close();
  }
}

void appendLine(const char *path, const char *line) {
  if (!sdActive) return;
  File f = SD.open(path, FILE_APPEND);
  if (!f) {
    Serial.println("schrijven naar SD mislukt");
    return;
  }
  f.println(line);
  f.close();
}

void logRaw(const Measurement &m) {
  if (!sdActive) return;
  char line[180];
  int n = snprintf(line, sizeof(line), "%lu,%lu,%.2f,%u",
                   (unsigned long)m.seq, (unsigned long)millis(), m.lux, m.gain);
  for (uint8_t i = 0; i < 10 && n > 0 && n < (int)sizeof(line); i++) {
    n += snprintf(line + n, sizeof(line) - n, ",%lu", (unsigned long)m.ch[i]);
  }
  appendLine(RAW_PATH, line);
}

void logLed(const Light &l, uint32_t samples) {
  if (!sdActive) return;
  char line[200];
  int n = snprintf(line, sizeof(line), "%lu,%lu,%lu,%.2f", (unsigned long)ledSeq,
                   (unsigned long)millis(), (unsigned long)samples, l.lux);
  for (uint8_t i = 0; i < 10 && n > 0 && n < (int)sizeof(line); i++) {
    n += snprintf(line + n, sizeof(line) - n, ",%.0f", l.ch[i]);
  }
  appendLine(LED_PATH, line);
}

// Leest één regel uit LEDS.CSV: seq,ms,metingen,lux en tien kanalen.
bool parseLedLine(const char *line, Light *l) {
  if (line[0] < '0' || line[0] > '9') return false;
  const char *p = line;
  float v[14];
  uint8_t field = 0;
  while (field < 14) {
    v[field++] = atof(p);
    while (*p && *p != ',') p++;
    if (*p != ',') break;
    p++;
  }
  if (field < 14) return false;
  l->lux = v[3];
  for (uint8_t i = 0; i < 10; i++) l->ch[i] = v[4 + i];
  return true;
}

// Zet de laatste LED_COUNT regels uit de log terug op de strip, zodat een
// herstart of een nieuwe upload je dag niet wist. De kleur wordt opnieuw
// berekend, dus met andere instellingen ziet dezelfde dag er anders uit.
void restoreFromLog() {
  if (!sdActive) return;
  File f = SD.open(LED_PATH, FILE_READ);
  if (!f) return;

  uint32_t size = f.size();
  uint32_t window = (uint32_t)LED_COUNT * 160UL;
  uint32_t start = size > window ? size - window : 0;
  f.seek(start);
  if (start > 0) {
    while (f.available() && f.read() != '\n') {}   // halve regel overslaan
  }

  char buf[220];
  uint16_t idx = 0;
  uint16_t restored = 0;
  while (f.available()) {
    int c = f.read();
    if (c == '\n' || c == '\r') {
      buf[idx] = 0;
      Light l;
      if (idx > 10 && parseLedLine(buf, &l)) {
        uint8_t r, g, b;
        pushColor(l, &r, &g, &b);
        ledSeq = strtoul(buf, NULL, 10) + 1;
        restored++;
      }
      idx = 0;
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = (char)c;
    }
  }
  f.close();
  strip.show();
  if (restored) {
    shownOnce = true;
    lastShown = millis();
  }
  Serial.printf("hersteld uit de log: %u LED(s), volgende seq %lu\n",
                restored, (unsigned long)ledSeq);
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

  logRaw(m);

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
  pushColor(l, &r, &g, &b);
  strip.show();
  logLed(l, sampleCount);
  ledSeq++;

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

  initSD();
  restoreFromLog();

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
