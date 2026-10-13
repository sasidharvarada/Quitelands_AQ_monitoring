// =====================================================
//  AQ Node - Simple Hardware Test
//  Checks: AHT10, DS3231 RTC, SD card, SDS011, Prana CO2,
//          Wi-Fi, status LED
//  Same pins as the main node firmware.
//  Open Serial Monitor at 115200.
// =====================================================

#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_AHTX0.h>

// ---------- Wi-Fi (same as main code) ----------
const char* WIFI_SSID     = "SCRC_R4";
const char* WIFI_PASSWORD = "Scrciiith@123";

// ---------- Pins ----------
#define SDA_PIN  21
#define SCL_PIN  22
#define SDS_RX   16
#define SDS_TX   17
#define CO2_RX   32
#define CO2_TX   33
#define SD_CS     5
#define SD_SCK   18
#define SD_MISO  19
#define SD_MOSI  23
#define LED_PIN  12

HardwareSerial SDS011Serial(1);
HardwareSerial CO2Serial(2);
Adafruit_AHTX0 aht;
RTC_DS3231 rtc;

const byte SDS_QUERY[] = {
  0xAA,0xB4,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x02,0xAB
};
const byte SDS_WAKE[] = {
  0xAA,0xB4,0x06,0x01,0x01,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x06,0xAB
};
const byte CO2_COMMAND[] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};

bool ahtLib = false;   // true = Adafruit library works, false = use raw read
bool ahtOK = false, rtcOK = false, sdOK = false, sdsOK = false, co2OK = false, wifiOK = false;

// ---------- Sensor reads ----------
bool readSDS011(float &pm25, float &pm10) {
  while (SDS011Serial.available()) SDS011Serial.read();
  SDS011Serial.write(SDS_QUERY, sizeof(SDS_QUERY));

  byte r[10];
  int i = 0;
  unsigned long start = millis();
  while (i < 10 && millis() - start < 2500) {
    if (SDS011Serial.available()) {
      byte b = SDS011Serial.read();
      if (i == 0 && b != 0xAA) continue;
      r[i++] = b;
    }
  }
  if (i != 10 || r[1] != 0xC0 || r[9] != 0xAB) return false;

  byte sum = 0;
  for (int j = 2; j <= 7; j++) sum += r[j];
  if (sum != r[8]) return false;

  pm25 = (r[2] | (r[3] << 8)) / 10.0;
  pm10 = (r[4] | (r[5] << 8)) / 10.0;
  return true;
}

bool readCO2(int &ppm) {
  while (CO2Serial.available()) CO2Serial.read();
  CO2Serial.write(CO2_COMMAND, sizeof(CO2_COMMAND));

  byte r[9];
  int i = 0;
  unsigned long start = millis();
  while (i < 9 && millis() - start < 3000) {
    if (CO2Serial.available()) {
      byte b = CO2Serial.read();
      if (i == 0 && b != 0xFF) continue;
      r[i++] = b;
    }
  }
  if (i != 9 || r[1] != 0x86) return false;

  byte sum = 0;
  for (int j = 1; j <= 7; j++) sum += r[j];
  if ((byte)(0xFF - sum + 1) != r[8]) return false;

  ppm = (r[2] << 8) | r[3];
  return true;
}

// ---------- AHT10 raw access (backup if the library init fails) ----------
bool ahtPresent() {
  Wire.beginTransmission(0x38);
  return Wire.endTransmission() == 0;
}

void ahtRawInit() {
  Wire.beginTransmission(0x38);           // soft reset
  Wire.write(0xBA);
  Wire.endTransmission();
  delay(40);
  Wire.beginTransmission(0x38);           // AHT10 calibrate command
  Wire.write(0xE1); Wire.write(0x08); Wire.write(0x00);
  Wire.endTransmission();
  delay(300);
}

bool ahtRawRead(float &t, float &h) {
  Wire.beginTransmission(0x38);           // trigger measurement
  Wire.write(0xAC); Wire.write(0x33); Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;
  delay(100);
  if (Wire.requestFrom(0x38, 6) != 6) return false;
  uint8_t d[6];
  for (int i = 0; i < 6; i++) d[i] = Wire.read();
  if (d[0] & 0x80) return false;          // still busy
  uint32_t rh = ((uint32_t)d[1] << 12) | ((uint32_t)d[2] << 4) | (d[3] >> 4);
  uint32_t tr = (((uint32_t)d[3] & 0x0F) << 16) | ((uint32_t)d[4] << 8) | d[5];
  h = rh * 100.0 / 1048576.0;
  t = tr * 200.0 / 1048576.0 - 50.0;
  return true;
}

bool readAHT(float &t, float &h) {
  if (ahtLib) {
    sensors_event_t hum, tp;
    if (!aht.getEvent(&hum, &tp)) return false;
    t = tp.temperature; h = hum.relative_humidity;
    return true;
  }
  return ahtRawRead(t, h);
}

void printResult(const char* name, bool ok) {
  Serial.printf("%-10s : %s\n", name, ok ? "OK" : "FAILED");
}

// =====================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===== AQ NODE HARDWARE TEST =====\n");

  // ---- LED: blink 3 times ----
  pinMode(LED_PIN, OUTPUT);
  Serial.println("LED: blinking 3 times (check it visually)");
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(300);
    digitalWrite(LED_PIN, LOW);  delay(300);
  }

  // ---- AHT10 (same order as main firmware: begin right after Wire.begin) ----
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);
  for (int i = 0; i < 3 && !ahtLib; i++) {
    ahtLib = aht.begin(&Wire);
    if (!ahtLib) delay(200);
  }
  if (ahtLib) {
    Serial.println("  AHT10: library init OK");
  } else {
    Serial.printf("  AHT10: library init failed. Responds at 0x38: %s\n", ahtPresent() ? "YES" : "NO");
    ahtRawInit();
  }
  {
    float t, h;
    ahtOK = readAHT(t, h) && h > 0 && h <= 100 && t > -40 && t < 85;
    if (ahtOK) Serial.printf("  AHT10 test read: %.2f C, %.2f %%%s\n", t, h, ahtLib ? "" : " (raw mode)");
  }
  printResult("AHT10", ahtOK);

  // ---- RTC ----
  rtcOK = rtc.begin(&Wire);
  printResult("DS3231", rtcOK);
  if (rtcOK && rtc.lostPower())
    Serial.println("  WARNING: RTC lost power - check the CR2032 battery");

  // ---- SD card: write + read test ----
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sdOK = SD.begin(SD_CS);
  if (sdOK) {
    File f = SD.open("/test.txt", FILE_WRITE);
    if (f) { f.println("hello"); f.close(); }
    f = SD.open("/test.txt", FILE_READ);
    String s = f ? f.readStringUntil('\n') : "";
    if (f) f.close();
    SD.remove("/test.txt");
    sdOK = s.startsWith("hello");
    if (sdOK) Serial.printf("  SD size: %llu MB\n", SD.cardSize() / (1024 * 1024));
  }
  printResult("SD card", sdOK);

  // ---- UART sensors ----
  SDS011Serial.begin(9600, SERIAL_8N1, SDS_RX, SDS_TX);
  CO2Serial.begin(9600, SERIAL_8N1, CO2_RX, CO2_TX);
  SDS011Serial.write(SDS_WAKE, sizeof(SDS_WAKE));   // start SDS011 fan
  delay(2000);

  float pm25, pm10;
  sdsOK = readSDS011(pm25, pm10);
  printResult("SDS011", sdsOK);
  int co2;
  co2OK = readCO2(co2);
  printResult("CO2", co2OK);

  // ---- Wi-Fi ----
  Serial.print("Wi-Fi: connecting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  wifiOK = (WiFi.status() == WL_CONNECTED);
  printResult("Wi-Fi", wifiOK);
  if (wifiOK) Serial.printf("  IP: %s  Signal: %d dBm\n",
                            WiFi.localIP().toString().c_str(), WiFi.RSSI());

  // ---- Summary ----
  const char* names[] = {"AHT10", "DS3231 RTC", "SD card", "SDS011", "CO2 (Prana)", "Wi-Fi"};
  bool ok[] = {ahtOK, rtcOK, sdOK, sdsOK, co2OK, wifiOK};
  Serial.println("\n========== SUMMARY ==========");
  Serial.print("WORKING     : ");
  for (int i = 0; i < 6; i++) if (ok[i]) Serial.printf("%s  ", names[i]);
  Serial.print("\nNOT WORKING : ");
  int bad = 0;
  for (int i = 0; i < 6; i++) if (!ok[i]) { Serial.printf("%s  ", names[i]); bad++; }
  if (bad == 0) Serial.print("none - all OK");
  Serial.println("\n=============================");
  Serial.println("(LED: check by eye - it blinked 3 times at start)");

  Serial.println("\n===== LIVE READINGS (every 5 s) =====");
}

// =====================================================
void loop() {
  Serial.println("--------------------------------");

  if (rtcOK) {
    DateTime t = rtc.now();
    Serial.printf("Time   : %04d-%02d-%02d %02d:%02d:%02d\n",
                  t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
  }

  float t, h;
  if (readAHT(t, h))
    Serial.printf("Temp   : %.2f C   Humidity: %.2f %%\n", t, h);
  else
    Serial.println("AHT10  : READ FAILED");

  float pm25, pm10;
  if (readSDS011(pm25, pm10)) Serial.printf("PM2.5  : %.1f ug/m3   PM10: %.1f ug/m3\n", pm25, pm10);
  else                        Serial.println("PM     : READ FAILED");

  int co2;
  if (readCO2(co2)) Serial.printf("CO2    : %d ppm\n", co2);
  else              Serial.println("CO2    : READ FAILED");

  Serial.printf("Wi-Fi  : %s\n", WiFi.status() == WL_CONNECTED ? "connected" : "not connected");

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));   // LED toggles each loop
  delay(5000);
}
