// Carpe Lucem - sensormodule
// Leest beide sensoren uit, print de meting naar de seriële monitor en
// verstuurt hem via ESP-NOW naar de displaymodule.

#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_AS7341.h>

// I2C-pinnen. Dezelfde op de C3 en de S3, zodat je van bordje kunt wisselen
// zonder te herbedraden. Op de C3 niet 8/9: daar zit het blauwe ledje op en
// het zijn bovendien opstartpinnen.
#define SDA_PIN 6
#define SCL_PIN 7

// Moet gelijk zijn aan display/display.ino.
#define ESPNOW_CHANNEL 1

// Lange-afstandsmodus van ESP-NOW: langzamer zenden, maar twee tot vier keer
// meer bereik. Beide modules moeten hierin hetzelfde staan, anders horen ze
// elkaar niet meer.
const bool LONG_RANGE = true;

// Zo vaak meten en versturen. Het display middelt alle metingen per LED.
// Sneller dan ~1 s kan niet: één meting kost de AS7341 al ~0,6 s.
const unsigned long INTERVAL_MS = 30000;

// Elke meting een paar keer versturen. Een broadcast wordt niet bevestigd, dus
// dit is de manier om afstand en een zwakke antenne op te vangen. Het display
// herkent kopieën aan het seq-nummer en gooit ze weg.
const uint8_t SEND_COPIES = 3;
const uint8_t SEND_GAP_MS = 20;

// AS7341 kiest zelf zijn gain: omlaag als hij bijna verzadigt, omhoog als hij
// weinig telt. ATIME/ASTEP blijven vast, dus de maximale telling is 65535.
const uint16_t AS_FULL_SCALE = 65535;
const uint16_t AS_TOO_HIGH = AS_FULL_SCALE * 0.9;
const uint16_t AS_TOO_LOW = AS_FULL_SCALE * 0.4;   // na verdubbelen nog < 80%
const uint8_t AS_MAX_TRIES = 12;

// Moet gelijk zijn aan display/display.ino.
const uint8_t PACKET_VERSION = 1;
struct __attribute__((packed)) Measurement {
  uint8_t  version;
  uint32_t seq;
  float    lux;
  uint8_t  gain;      // AS7341-gainstand, ter info
  uint32_t ch[10];    // 415, 445, 480, 515, 555, 590, 630, 680, clear, nir; omgerekend naar 512x
};

const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

Adafruit_VEML7700 veml;
Adafruit_AS7341 as7341;

bool vemlOk = false;
bool asOk = false;
bool radioOk = false;
unsigned long lastRead = 0;
uint8_t gain = AS7341_GAIN_512X;
uint32_t nextSeq = 1;
uint16_t linesPrinted = 0;

void scanI2C() {
  Serial.println("I2C-scan:");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) continue;
    found++;
    Serial.printf("  0x%02X", addr);
    if (addr == 0x10) Serial.print("  VEML7700");
    if (addr == 0x39) Serial.print("  AS7341");
    Serial.println();
  }
  if (found == 0) Serial.println("  niets gevonden, check de bedrading");
}

// Probeert sensoren die nog niet gevonden zijn opnieuw, zodat je tijdens het
// testen draadjes kunt verhangen zonder te resetten.
void initSensors() {
  if (!vemlOk) {
    vemlOk = veml.begin();
    Serial.println(vemlOk ? "VEML7700 OK" : "VEML7700 niet gevonden");
  }
  if (!asOk) {
    asOk = as7341.begin();
    if (asOk) {
      as7341.setATIME(100);
      as7341.setASTEP(999);
      as7341.setGain((as7341_gain_t)gain);
    }
    Serial.println(asOk ? "AS7341 OK" : "AS7341 niet gevonden");
  }
}

void initRadio() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (LONG_RANGE) esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW start mislukt");
    return;
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  radioOk = esp_now_add_peer(&peer) == ESP_OK;
  Serial.println(radioOk ? "ESP-NOW OK" : "ESP-NOW peer mislukt");
}

void printHeader() {
  Serial.println("t_s\tseq\tlux\tgain\t415\t445\t480\t515\t555\t590\t630\t680\tclear\tnir");
}

void printGain(uint8_t g) {
  if (g == AS7341_GAIN_0_5X) Serial.print("0.5");
  else Serial.print(1u << (g - 1));
}

// Meet opnieuw met een andere gain tot de hoogste telling tussen AS_TOO_LOW en
// AS_TOO_HIGH ligt, of tot de gain op zijn grens zit.
bool readSpectrum(uint16_t *r, uint16_t *peak) {
  for (uint8_t attempt = 1; ; attempt++) {
    if (!as7341.readAllChannels(r)) return false;

    *peak = 0;
    for (uint8_t i = 0; i < 12; i++) {
      if (r[i] > *peak) *peak = r[i];
    }

    uint8_t next = gain;
    if (*peak > AS_TOO_HIGH && gain > AS7341_GAIN_0_5X) next = gain - 1;
    else if (*peak < AS_TOO_LOW && gain < AS7341_GAIN_512X) next = gain + 1;
    if (next == gain || attempt >= AS_MAX_TRIES) return true;

    gain = next;
    as7341.setGain((as7341_gain_t)gain);
  }
}

void measureAndSend() {
  float lux = veml.readLux(VEML_LUX_AUTO);

  uint16_t r[12];
  uint16_t peak;
  if (!readSpectrum(r, &peak)) {
    Serial.println("AS7341 leesfout");
    return;
  }

  // readAllChannels geeft F1-F4, clear, nir, F5-F8, clear, nir.
  const uint8_t order[10] = {0, 1, 2, 3, 6, 7, 8, 9, 4, 5};

  Measurement m;
  m.version = PACKET_VERSION;
  m.seq = nextSeq++;
  m.lux = lux;
  m.gain = gain;
  for (uint8_t i = 0; i < 10; i++) {
    m.ch[i] = (uint32_t)r[order[i]] << (AS7341_GAIN_512X - gain);
  }

  bool sent = false;
  for (uint8_t i = 0; radioOk && i < SEND_COPIES; i++) {
    if (i) delay(SEND_GAP_MS);
    if (esp_now_send(BROADCAST_MAC, (const uint8_t *)&m, sizeof(m)) == ESP_OK) sent = true;
  }

  if (linesPrinted % 20 == 0) printHeader();
  linesPrinted++;

  Serial.printf("%lu\t%lu\t%.1f\t", millis() / 1000, (unsigned long)m.seq, lux);
  printGain(gain);
  for (uint8_t i = 0; i < 10; i++) {
    Serial.printf("\t%lu", (unsigned long)m.ch[i]);
  }
  if (peak >= AS_FULL_SCALE) Serial.print("\tVERZADIGD");
  if (!sent) Serial.print("\tNIET VERZONDEN");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  Serial.println();
  Serial.println("Carpe Lucem sensor");

  Wire.begin(SDA_PIN, SCL_PIN);
  scanI2C();
  initSensors();
  initRadio();
}

void loop() {
  delay(10);
  if (millis() - lastRead < INTERVAL_MS) return;
  lastRead = millis();

  if (!vemlOk || !asOk) {
    initSensors();
    return;
  }
  measureAndSend();
}
