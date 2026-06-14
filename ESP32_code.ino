// ============================================================
//  OutReady ESP32 Firmware
//  Senzori: AHT20 (temp+umiditate) | BMP280 (presiune) | VEML7700 (lumina)
//  Comunicare: Firebase Realtime Database (HTTP REST)
//  Clock: NTP sincronizat
//  PM2.5: simulat
// ============================================================

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_VEML7700.h>
#include <time.h>

// ─────────────────────────────────────────────
//  CONFIGURARE
// ─────────────────────────────────────────────
const char* WIFI_SSID     = "DIGI-2j9C";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
// Firebase Realtime Database
const char* FIREBASE_HOST = "outready-af38a-default-rtdb.europe-west1.firebasedatabase.app";
const char* FIREBASE_PATH = "/sensor_data/latest.json";

const char* NTP_SERVER          = "pool.ntp.org";
const long  GMT_OFFSET_SEC      = 7200;
const int   DAYLIGHT_OFFSET_SEC = 3600;

const unsigned long SENSOR_INTERVAL_MS = 2000;
const int           LED_PIN            = 2;

const float PRESSURE_OFFSET = 11.1f;
const float TEMP_OFFSET     = -2.6f;

// ─────────────────────────────────────────────
//  Obiecte globale
// ─────────────────────────────────────────────
Adafruit_AHTX0    aht;
Adafruit_BMP280   bmp;
Adafruit_VEML7700 veml;
WiFiClientSecure  wifiClient;

unsigned long lastSensorRead = 0;
bool          ntpSynced      = false;
bool          vemlOk         = false;
bool          bmpOk          = false;

struct SensorReading {
  float  temperature;
  float  humidity;
  float  pressure;
  float  lightLux;
  float  pm25;
  String timestamp;
  bool   valid;
};

void initBMP();
void initSensors();
void scanI2C();
void connectWiFi();
void syncNTP();
String getTimestamp();
SensorReading readSensors();
String buildJson(const SensorReading& r);
void printReading(const SensorReading& r);
bool sendToFirebase(const String& json);

// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========== OutReady ESP32 Firmware ==========");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin();
  initSensors();
  connectWiFi();
  syncNTP();

  wifiClient.setInsecure();

  digitalWrite(LED_PIN, HIGH);
  Serial.println("============================================\n");
}

// ─────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────
void loop() {
  unsigned long now = millis();
  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = now;
    SensorReading reading = readSensors();
    if (reading.valid) {
      String json = buildJson(reading);
      bool ok = sendToFirebase(json);
      Serial.printf("[Firebase] %s\n", ok ? "OK" : "EROARE");
      printReading(reading);
    } else {
      Serial.println("[SENSOR] Citire invalida — skip");
    }
  }
}

// ─────────────────────────────────────────────
//  TRIMITE DATE LA FIREBASE
// ─────────────────────────────────────────────
bool sendToFirebase(const String& json) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Firebase] WiFi disconnected!!");
    connectWiFi();
    return false;
  }

  HTTPClient http;
  String url = String("https://") + FIREBASE_HOST + FIREBASE_PATH;

  http.begin(wifiClient, url);
  http.addHeader("Content-Type", "application/json");

  int code = http.PUT(json);

  if (code == 200 || code == 204) {
    http.end();
    return true;
  } else {
    Serial.printf("[Firebase] HTTP error: %d\n", code);
    http.end();
    return false;
  }
}

// ─────────────────────────────────────────────
//  INIT BMP280
// ─────────────────────────────────────────────
void initBMP() {
  bmpOk = false;

  Wire.beginTransmission(0x77);
  if (Wire.endTransmission() == 0) {
    if (bmp.begin(0x77)) {
      bmpOk = true;
      Serial.println("[BMP280] OK (0x77)");
    }
  } else {
    Wire.beginTransmission(0x76);
    if (Wire.endTransmission() == 0) {
      if (bmp.begin(0x76)) {
        bmpOk = true;
        Serial.println("[BMP280] OK (0x76)");
      }
    }
  }

  if (!bmpOk) {
    Serial.println("[BMP280] EROARE: negasit!");
    return;
  }
  Serial.println("[BMP280] Gata");
}

// ─────────────────────────────────────────────
//  INITIALIZARE SENZORI
// ─────────────────────────────────────────────
void initSensors() {
  scanI2C();

  initBMP();
  delay(200);

  if (!aht.begin()) {
    Serial.println("[AHT20] ERROR!");
  } else {
    Serial.println("[AHT20] OK");
  }

  if (veml.begin()) {
    vemlOk = true;
    Serial.println("[VEML7700] OK");
  } else {
    vemlOk = false;
    Serial.println("[VEML7700] ERROR: not found!");
  }
}

// ─────────────────────────────────────────────
//  SCANARE I2C
// ─────────────────────────────────────────────
void scanI2C() {
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Gasit I2C la 0x%02X\n", addr);
      count++;
    }
  }
  Serial.printf("  Total: %d dispozitiv(e)\n", count);
}

// ─────────────────────────────────────────────
//  CITIRE SENZORI
// ─────────────────────────────────────────────
SensorReading readSensors() {
  SensorReading r;
  r.valid = false;

  // --- AHT20 ---
  sensors_event_t ahtHumidity, ahtTemp;
  bool ahtValid  = aht.getEvent(&ahtHumidity, &ahtTemp);
  float tempAHT  = ahtValid ? ahtTemp.temperature : -999.0f;
  float humidity = ahtValid ? ahtHumidity.relative_humidity : 50.0f;

  if (ahtValid && (isnan(tempAHT) || isnan(humidity) ||
      humidity < 0 || humidity > 100)) {
    ahtValid = false;
    humidity = 50.0f;
  }

  // --- BMP280 ---
  static float lastGoodTemp = 24.0f;
  float tempBMP  = -999.0f;
  float pressure = 1013.25f;

  if (bmpOk) {
    float rawPressure = bmp.readPressure();
    float rawTemp     = bmp.readTemperature();

    if (!isnan(rawPressure) && rawPressure > 50000) {
      pressure = (rawPressure / 100.0F) + PRESSURE_OFFSET;
    }

    if (!isnan(rawTemp) && rawTemp > 0 && rawTemp < 85) {
      float calibrat = rawTemp + TEMP_OFFSET;
      if (calibrat > 0 && calibrat < 50) {
        tempBMP      = calibrat;
        lastGoodTemp = tempBMP;
      } else {
        tempBMP = lastGoodTemp;
      }
    } else {
      tempBMP = lastGoodTemp;
    }

    Serial.printf("[BMP280] raw=%.2f cal=%.2f pres=%.2f hPa\n",
                  rawTemp, tempBMP, pressure);
  } else {
    initBMP();
    tempBMP = lastGoodTemp;
  }

  // --- Temperatura finala ---
  float finalTemp;
  if (tempBMP > 0 && tempBMP < 50) {
    finalTemp = tempBMP;
  } else if (ahtValid && tempAHT > -50 && tempAHT < 60) {
    finalTemp = tempAHT;
  } else {
    finalTemp = lastGoodTemp;
  }

  if (!ahtValid) humidity = 50.0f;

  // --- Lux simulat --- DEZACTIVAT pentru test hardware
  // float lux = 50.0f;
  // struct tm t;
  // if (getLocalTime(&t)) {
  //   int h = t.tm_hour;
  //   if (h >= 7 && h <= 19) {
  //     lux = 10000.0f - abs(h - 13) * 1500.0f;
  //     lux = constrain(lux, 500.0f, 10000.0f);
  //   }
  // }

  // --- Lux real VEML7700 ---
  float lux = 0.0f;
  if (veml.begin()) {
    vemlOk = true;
    lux = veml.readLux();
    Serial.printf("[VEML7700] Lux: %.1f\n", lux);
  } else {
    vemlOk = false;
    Serial.println("[VEML7700] EROARE: negasit!");
    lux = 0.0f;
  }

  // --- PM2.5 simulat ---
  float pm25Base  = 5.0f + (humidity > 60 ? (humidity - 60) * 0.15f : 0);
  float pm25Noise = ((float)(esp_random() % 100) / 100.0f - 0.5f) * 2.0f;
  float pm25      = constrain(pm25Base + pm25Noise, 0.0f, 500.0f);

  r.temperature = roundf(finalTemp * 10) / 10.0f;
  r.humidity    = roundf(humidity  * 10) / 10.0f;
  r.pressure    = roundf(pressure  * 10) / 10.0f;
  r.lightLux    = roundf(lux);
  r.pm25        = roundf(pm25      * 10) / 10.0f;
  r.timestamp   = getTimestamp();
  r.valid       = true;

  return r;
}

// ─────────────────────────────────────────────
//  JSON
// ─────────────────────────────────────────────
String buildJson(const SensorReading& r) {
  StaticJsonDocument<256> doc;
  doc["temperature"] = r.temperature;
  doc["humidity"]    = r.humidity;
  doc["pressure"]    = r.pressure;
  doc["light_lux"]   = r.lightLux;
  doc["pm25"]        = r.pm25;
  doc["timestamp"]   = r.timestamp;
  doc["ntp_synced"]  = ntpSynced;
  String out;
  serializeJson(doc, out);
  return out;
}

// ─────────────────────────────────────────────
//  WIFI cu IP static
// ─────────────────────────────────────────────
void connectWiFi() {
  IPAddress local_IP(192, 168, 100, 97);
  IPAddress gateway(192, 168, 100, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress primaryDNS(8, 8, 8, 8);
  WiFi.config(local_IP, gateway, subnet, primaryDNS);

  Serial.printf("[WiFi] Conectare la \"%s\"", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Conectat!");
    Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WiFi] RSSI: %d dBm\n", WiFi.RSSI());
  } else {
    Serial.println("\n[WiFi] EROARE — restart in 5s...");
    delay(5000);
    ESP.restart();
  }
}

// ─────────────────────────────────────────────
//  NTP
// ─────────────────────────────────────────────
void syncNTP() {
  Serial.print("[NTP] Synchronising...");
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER,
             "time.google.com", "time.cloudflare.com");

  struct tm timeinfo;
  int attempts = 0;
  while (!getLocalTime(&timeinfo) && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (getLocalTime(&timeinfo)) {
    ntpSynced = true;
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    Serial.printf("\n[NTP] OK — %s\n", buf);
  } else {
    ntpSynced = false;
    Serial.println("\n[NTP] Failed — using millis()");
  }
}

// ─────────────────────────────────────────────
//  TIMESTAMP ISO 8601
// ─────────────────────────────────────────────
String getTimestamp() {
  if (!ntpSynced) {
    unsigned long ms = millis();
    char buf[32];
    snprintf(buf, sizeof(buf), "1970-01-01T00:%02lu:%02lu.%03luZ",
             (ms / 60000) % 60, (ms / 1000) % 60, ms % 1000);
    return String(buf);
  }
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "1970-01-01T00:00:00Z";
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &timeinfo);
  return String(buf);
}

// ─────────────────────────────────────────────
//  DEBUG SERIAL
// ─────────────────────────────────────────────
void printReading(const SensorReading& r) {
  Serial.println("─────────────────────────────");
  Serial.printf("  Timestamp:  %s\n",        r.timestamp.c_str());
  Serial.printf("  Temp:       %.1f C\n",    r.temperature);
  Serial.printf("  Umiditate:  %.1f %%\n",   r.humidity);
  Serial.printf("  Presiune:   %.1f hPa\n",  r.pressure);
  Serial.printf("  Lumina:     %.0f lux\n",  r.lightLux);
  Serial.printf("  PM2.5:      %.1f ug/m3\n", r.pm25);
}