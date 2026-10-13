/*
  =====================================================================
   Embedded Air-Quality Node  -  Field Deployment Firmware (ESP32)
  =====================================================================
   Sensors : SDS011 (PM2.5/PM10), Prana Air NDIR CO2, AHT10 (T/RH)
   Time    : DS3231 RTC  (primary timestamp source)
   Storage : microSD     (primary data store, Wi-Fi independent)
   Network : Wi-Fi -> NTP (corrects RTC) + ThingSpeak bulk upload
   Node    : uncomment one "#define node_X" below (sets channel + calibration)
   Calib   : y = m*x + c per node
   SD file : /AQ_Latest.csv  (raw + calibrated values)
   Cloud   : temp, humidity, pm25_cal, pm10_cal, co2_cal (calibrated only)
   Status  : LED on GPIO 13

   DESIGN RULES
     NTP   sets / corrects the RTC
     RTC   timestamps every measurement
     SD    persistently stores every measurement
     Wi-Fi is optional: losing it NEVER stops sensing or logging

   FREERTOS TASKS
     sensorTask  : 3-min warm-up, then fixed-interval acquisition
     sdLoggerTask: writes records to SD, handles SD failure/re-init
     timeTask    : NTP synchronisation -> RTC correction, RTC health
     networkTask : Wi-Fi (re)connection, backlog upload to ThingSpeak
     statusTask  : node state machine + GPIO 13 LED

   Tasks talk through:
     logQueue   (sensor -> SD logger, holds records if SD is missing)
     sysEvents  (event group: warm-up done, SD ok, Wi-Fi up, internet ok,
                 NTP request)
     i2cMutex   (RTC + AHT10 share the I2C bus)
     sdMutex    (SD logger + uploader share the card)

   NODE STATE MACHINE (statusTask)
     BOOT -> INITIALIZATION -> WARMUP -> NORMAL OPERATION
        NORMAL OPERATION = NETWORK_CONNECTED  <->  OFFLINE_LOGGING
        any state        -> LOGGING_FAULT when the SD card fails
     Power loss at any point -> BOOT -> RTC recovery -> continue

   LED (GPIO 13)
     OFF   : not logging to SD (warm-up, or SD failure)
     BLINK : logging + Wi-Fi AND internet confirmed
     ON    : logging, but offline (no Wi-Fi or no internet)
  =====================================================================
*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_AHTX0.h>   // works for AHT10 and AHT20
#include <time.h>
#include <esp_sntp.h>
#include <stdarg.h>

// =====================================================================
// NODE SELECTION  <-- uncomment ONE line before uploading to each node
// =====================================================================
// #define node_1
// #define node_2
#define node_7 // mam node

// =====================================================================
// PER-NODE SETTINGS: ThingSpeak channel + calibration (calibrated = m*raw + c)
// =====================================================================
struct Calib {
  float m_pm25, c_pm25;
  float m_pm10, c_pm10;
  float m_co2,  c_co2;
};

#if defined(node_1)
  #define NODE_NAME            "Node 1"
  #define THINGSPEAK_CHANNEL   "3520966"
  #define THINGSPEAK_WRITE_KEY "9EQGD3S2NLVT7XL7"
  const Calib NODE_CAL = {
    1.4204f,    2.5961f,     // m_pm25, c_pm25
    1.0154f,   -1.5812f,     // m_pm10, c_pm10
    1.0818f, -111.0455f      // m_co2,  c_co2   <-- replace after refit
  };

#elif defined(node_2)
  #define NODE_NAME            "Node 2"
  #define THINGSPEAK_CHANNEL   "3520968"
  #define THINGSPEAK_WRITE_KEY "RM2W42PX5OCQ52T3"
  const Calib NODE_CAL = {
    1.3788f,    0.2565f,     // m_pm25, c_pm25
    0.9780f,    0.6027f,     // m_pm10, c_pm10
    1.0071f,  -92.1238f      // m_co2,  c_co2   <-- replace after refit
  };

#elif defined(node_7)
  #define NODE_NAME            "Node 7"
  #define THINGSPEAK_CHANNEL   "3521024"
  #define THINGSPEAK_WRITE_KEY "G3PNHMQ4CFJBXZTS"
  const Calib NODE_CAL = {
    1.1991f,   -0.0042f,     // m_pm25, c_pm25
    0.8614f,   -0.7030f,     // m_pm10, c_pm10
    1.1706f, -146.7228f      // m_co2,  c_co2   <-- replace after refit
  };

#else
  #error "Uncomment exactly one node line (node_1, node_2 or node_7) above"
#endif

// =====================================================================
// TYPES used by functions below. They must come BEFORE the first function
// in the file, because the Arduino IDE auto-generates function prototypes
// right above the first function definition.
// =====================================================================
// =====================================================================
// DATA RECORD (one measurement)
// =====================================================================
struct Record {
  char  ts[20];                 // "YYYY-MM-DD HH:MM:SS" from RTC at measurement time
  float pm25, pm10, temperature, humidity;          // raw
  int   co2;                                         // raw
  float pm25Cal, pm10Cal, co2Cal;                    // calibrated (y = m*x + c)
  bool  pmValid, climateValid, co2Valid;
  bool  timeValid;              // false if RTC time was invalid (lost power / not found)
};

// Node state machine
enum NodeState {
  ST_BOOT, ST_INIT, ST_WARMUP,
  ST_NETWORK_CONNECTED, ST_OFFLINE_LOGGING, ST_LOGGING_FAULT
};


Calib CAL = NODE_CAL;   // active calibration for this node

// y = m*x + c, never below 0 (a concentration cannot be negative)
float applyCal(float raw, float m, float c) {
  float y = m * raw + c;
  return (y < 0) ? 0.0f : y;
}

// =====================================================================
// USER SETTINGS
// =====================================================================
const char* WIFI_SSID                = "esw-m19@iiith";
const char* WIFI_PASSWORD            = "e5W-eMai@3!20hOct";
// ThingSpeak channel + key come from the node block above
const char* THINGSPEAK_WRITE_API_KEY = THINGSPEAK_WRITE_KEY;
const char* THINGSPEAK_CHANNEL_ID    = THINGSPEAK_CHANNEL;

// Local time zone (India Standard Time). The RTC stores LOCAL time.
const long  TZ_OFFSET_SEC = 19800;        // +5:30 in seconds
const char* TZ_OFFSET_STR = "+0530";      // used for ThingSpeak created_at

const char* NTP_SERVER_1 = "pool.ntp.org";
const char* NTP_SERVER_2 = "time.google.com";
const char* NTP_SERVER_3 = "time.nist.gov";

// Timing
const uint32_t SAMPLE_INTERVAL_MS  = 60000UL;            // 1 measurement per minute
const uint32_t WARMUP_SECONDS      = 180;                // 3-minute sensor warm-up
const uint32_t WIFI_RETRY_MS       = 30000UL;            // Wi-Fi reconnect attempt interval
const uint32_t NTP_TIMEOUT_MS      = 15000UL;            // max wait for one NTP sync
const uint32_t NTP_RETRY_MS        = 60000UL;            // retry interval if NTP fails
const uint32_t NTP_RESYNC_MS       = 6UL * 3600UL * 1000UL;  // periodic re-sync (6 h)
const long     RTC_CORRECT_MIN_S   = 2;                  // correct RTC if off by >= 2 s
const uint32_t UPLOAD_INTERVAL_MS  = 15000UL;            // ThingSpeak free: >= 15 s between updates
const int      UPLOAD_BATCH        = 50;                 // records per bulk upload
const uint32_t SD_RETRY_MS         = 5000UL;             // SD re-init interval after failure
const int      LOG_QUEUE_LEN       = 60;                 // RAM buffer: 60 records (1 h) if SD is out

// =====================================================================
// PINS
// =====================================================================
#define SDA_PIN   21
#define SCL_PIN   22
#define SDS_RX    16
#define SDS_TX    17
#define CO2_RX    32
#define CO2_TX    33
#define SD_CS      5
#define SD_SCK    18
#define SD_MISO   19
#define SD_MOSI   23
#define LED_PIN   12      // status LED (via ~220-330 ohm resistor to GND)

// =====================================================================
// FILES
// =====================================================================
// New log file: raw + calibrated values. The old /aq_log.csv is left untouched.
const char* LOG_FILE   = "/AQ_Latest.csv";
const char* POS_FILE   = "/AQ_Latest_pos.txt";   // byte offset of next record to upload
const char* POS_TMP    = "/AQ_Latest_pos.tmp";
const char* CSV_HEADER = "timestamp,temperature_C,humidity_percent,"
                         "pm25_raw,pm25_cal,pm10_raw,pm10_cal,co2_raw,co2_cal,time_valid";
const int   CSV_COLS   = 10;

// =====================================================================
// HARDWARE OBJECTS
// =====================================================================
HardwareSerial SDS011Serial(1);
HardwareSerial CO2Serial(2);
Adafruit_AHTX0 aht;
RTC_DS3231     rtc;

const byte SDS_QUERY[] = {
  0xAA,0xB4,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x02,0xAB
};
const byte SDS_WAKE[] = {           // make sure fan + laser are running
  0xAA,0xB4,0x06,0x01,0x01,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x06,0xAB
};
const byte CO2_COMMAND[] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};

// =====================================================================
// RTOS OBJECTS
// =====================================================================
QueueHandle_t      logQueue;
EventGroupHandle_t sysEvents;
SemaphoreHandle_t  i2cMutex, sdMutex, printMutex;

#define EVT_WARMUP_DONE  BIT0
#define EVT_SD_OK        BIT1
#define EVT_WIFI_UP      BIT2
#define EVT_INTERNET_OK  BIT3
#define EVT_NTP_REQUEST  BIT4

volatile bool rtcPresent   = false;
volatile bool rtcTimeValid = false;
volatile bool ntpSyncDone  = false;   // set by SNTP callback

volatile NodeState nodeState = ST_BOOT;

// =====================================================================
// HELPERS
// =====================================================================
// Thread-safe serial debug print
void dbg(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  if (printMutex) xSemaphoreTake(printMutex, portMAX_DELAY);
  Serial.println(buf);
  if (printMutex) xSemaphoreGive(printMutex);
}

void formatTimestamp(const DateTime& t, char* out, size_t n) {
  snprintf(out, n, "%04d-%02d-%02d %02d:%02d:%02d",
           t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
}

// Detects default / garbage RTC values (e.g. 2000-01-01, 2165-165-165)
bool isPlausibleTime(const DateTime& t) {
  return t.year() >= 2024 && t.year() <= 2099 &&
         t.month() >= 1 && t.month() <= 12 &&
         t.day() >= 1 && t.day() <= 31 &&
         t.hour() < 24 && t.minute() < 60 && t.second() < 60;
}

// Read RTC safely (I2C shared with AHT10). Returns false if time is implausible.
bool readRtc(DateTime& out) {
  if (!rtcPresent) return false;
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  out = rtc.now();
  xSemaphoreGive(i2cMutex);
  return isPlausibleTime(out);
}

// SNTP callback: fires every time a fresh NTP response is applied
void onNtpSync(struct timeval* tv) {
  ntpSyncDone = true;
}

// =====================================================================
// SENSOR DRIVERS
// =====================================================================
bool readSDS011(float &pm25, float &pm10) {
  while (SDS011Serial.available()) SDS011Serial.read();
  SDS011Serial.write(SDS_QUERY, sizeof(SDS_QUERY));
  SDS011Serial.flush();

  byte r[10];
  int i = 0;
  unsigned long start = millis();
  while (i < 10 && millis() - start < 2500) {
    if (SDS011Serial.available()) {
      byte b = SDS011Serial.read();
      if (i == 0 && b != 0xAA) continue;
      r[i++] = b;
    } else {
      vTaskDelay(pdMS_TO_TICKS(5));      // yield while waiting
    }
  }
  if (i != 10 || r[1] != 0xC0 || r[9] != 0xAB) return false;

  byte checksum = 0;
  for (int j = 2; j <= 7; j++) checksum += r[j];
  if (checksum != r[8]) return false;

  pm25 = (r[2] | (r[3] << 8)) / 10.0;
  pm10 = (r[4] | (r[5] << 8)) / 10.0;
  return true;
}

bool readCO2(int &co2ppm) {
  while (CO2Serial.available()) CO2Serial.read();
  CO2Serial.write(CO2_COMMAND, sizeof(CO2_COMMAND));
  CO2Serial.flush();

  byte r[9];
  int i = 0;
  unsigned long start = millis();
  while (i < 9 && millis() - start < 3000) {
    if (CO2Serial.available()) {
      byte b = CO2Serial.read();
      if (i == 0 && b != 0xFF) continue;
      r[i++] = b;
    } else {
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }
  if (i != 9 || r[1] != 0x86) return false;

  byte sum = 0;
  for (int j = 1; j <= 7; j++) sum += r[j];
  if ((byte)(0xFF - sum + 1) != r[8]) return false;   // datasheet checksum

  co2ppm = ((int)r[2] << 8) | r[3];
  return co2ppm <= 10000;                             // sensor range 0-10000 ppm
}

bool readAHT(float &t, float &h) {
  sensors_event_t hum, temp;
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  bool ok = aht.getEvent(&hum, &temp);
  xSemaphoreGive(i2cMutex);
  if (!ok) return false;
  t = temp.temperature;
  h = hum.relative_humidity;
  return true;
}

// =====================================================================
// SD CARD
// =====================================================================
// (Re)initialise the card. Never overwrites an existing log: it appends.
// If the last line was cut by a power failure, a newline is added so the
// next record starts on a clean line (the broken line is ignored on upload).
bool sdInit(bool atBoot) {
  bool ok;
  xSemaphoreTake(sdMutex, portMAX_DELAY);
  SD.end();
  ok = SD.begin(SD_CS);

  if (ok && !SD.exists(LOG_FILE)) {
    File f = SD.open(LOG_FILE, FILE_WRITE);
    if (f) {
      f.println(CSV_HEADER);
      f.close();
      dbg("[SD] New log file created: %s", LOG_FILE);
    } else ok = false;

  } else if (ok) {
    File f = SD.open(LOG_FILE, FILE_READ);
    if (!f) ok = false;
    else {
      size_t size = f.size();
      bool needNewline = false;
      String last = "";
      if (size > 0) {
        f.seek(size > 256 ? size - 256 : 0);
        String tail = f.readString();
        needNewline = !tail.endsWith("\n");
        tail.trim();
        int nl = tail.lastIndexOf('\n');
        last = (nl >= 0) ? tail.substring(nl + 1) : tail;
        last.trim();
      }
      f.close();

      if (needNewline) {
        File a = SD.open(LOG_FILE, FILE_APPEND);
        if (a) { a.print("\n"); a.close(); }
        dbg("[SD] Repaired incomplete last line (power lost during a write).");
      }
      if (atBoot) {
        dbg("[BOOT] Power-cycle recovery: existing log found (%u bytes), appending.", (unsigned)size);
        dbg("[BOOT] Last stored record: %s", last.length() ? last.c_str() : "(none)");
      } else {
        dbg("[SD] Card re-initialised, appending to existing log.");
      }
    }
  }
  xSemaphoreGive(sdMutex);

  if (ok) {
    xEventGroupSetBits(sysEvents, EVT_SD_OK);
    dbg("[SD] Logging initialized (%s).", LOG_FILE);
  } else {
    xEventGroupClearBits(sysEvents, EVT_SD_OK);
    dbg("[SD] Initialization FAILED - SD logging disabled.");
  }
  return ok;
}

// Column order:
// timestamp,temperature_C,humidity_percent,pm25_raw,pm25_cal,pm10_raw,pm10_cal,co2_raw,co2_cal,time_valid
void formatRecordLine(const Record& r, char* line, size_t n) {
  char cl[32], pm[64], co[32];
  if (r.climateValid) snprintf(cl, sizeof(cl), "%.2f,%.2f", r.temperature, r.humidity);
  else                strcpy(cl, "NA,NA");
  if (r.pmValid) snprintf(pm, sizeof(pm), "%.1f,%.1f,%.1f,%.1f", r.pm25, r.pm25Cal, r.pm10, r.pm10Cal);
  else           strcpy(pm, "NA,NA,NA,NA");
  if (r.co2Valid) snprintf(co, sizeof(co), "%d,%.0f", r.co2, r.co2Cal);
  else            strcpy(co, "NA,NA");
  snprintf(line, n, "%s,%s,%s,%s,%d\n", r.ts, cl, pm, co, r.timeValid ? 1 : 0);
}

// One record = one complete line, written and closed immediately.
// Closing flushes to the card, so at most one line can be lost on power failure.
bool writeRecord(const Record& r) {
  char line[200];
  formatRecordLine(r, line, sizeof(line));
  size_t len = strlen(line);

  xSemaphoreTake(sdMutex, portMAX_DELAY);
  File f = SD.open(LOG_FILE, FILE_APPEND);
  bool ok = false;
  if (f) {
    ok = (f.write((const uint8_t*)line, len) == len);
    f.close();
  }
  xSemaphoreGive(sdMutex);
  return ok;
}

// Upload pointer (byte offset into LOG_FILE). Call with sdMutex held.
uint32_t readUploadPos() {
  if (!SD.exists(POS_FILE) && SD.exists(POS_TMP)) SD.rename(POS_TMP, POS_FILE);
  File f = SD.open(POS_FILE, FILE_READ);
  if (!f) return 0;
  String s = f.readString();
  f.close();
  s.trim();
  return (uint32_t)strtoul(s.c_str(), nullptr, 10);
}

// Write-temp-then-rename, so a power cut can't leave a half-written pointer.
bool saveUploadPos(uint32_t pos) {
  File f = SD.open(POS_TMP, FILE_WRITE);
  if (!f) return false;
  f.print(pos);
  f.close();
  SD.remove(POS_FILE);
  return SD.rename(POS_TMP, POS_FILE);
}

// =====================================================================
// NTP -> RTC SYNCHRONISATION
// =====================================================================
// Only a FRESH NTP response counts (Wi-Fi association alone is not enough).
// RTC is corrected for FUTURE measurements. Stored records are never touched.
bool syncRtcFromNtp() {
  dbg("[NTP] Synchronizing time...");
  ntpSyncDone = false;
  configTime(TZ_OFFSET_SEC, 0, NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3);

  uint32_t t0 = millis();
  while (!ntpSyncDone && millis() - t0 < NTP_TIMEOUT_MS) {
    vTaskDelay(pdMS_TO_TICKS(200));        // waits here only; sensor task keeps running
  }
  if (!ntpSyncDone) {
    dbg("[NTP] No response (server unreachable or no internet).");
    return false;
  }

  time_t nowT = time(nullptr);
  struct tm tmv;
  localtime_r(&nowT, &tmv);
  DateTime ntpTime(tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
                   tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
  if (!isPlausibleTime(ntpTime)) {
    dbg("[NTP] Received implausible time - ignored.");
    return false;
  }

  char ts[20];
  formatTimestamp(ntpTime, ts, sizeof(ts));
  dbg("[NTP] Time synchronized: %s", ts);

  if (!rtcPresent) {
    dbg("[RTC] Not present - cannot store NTP time.");
    return true;                             // internet is still confirmed
  }

  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  DateTime rtcNow = rtc.now();
  long diff = (long)ntpTime.unixtime() - (long)rtcNow.unixtime();
  bool wasValid = rtcTimeValid && isPlausibleTime(rtcNow);
  if (!wasValid || labs(diff) >= RTC_CORRECT_MIN_S) {
    rtc.adjust(ntpTime);                     // also clears DS3231 "lost power" flag
    rtcTimeValid = true;
    xSemaphoreGive(i2cMutex);
    if (!wasValid) dbg("[RTC] Was invalid -> set from NTP: %s", ts);
    else {
      dbg("[RTC] Time corrected by %ld s -> %s", diff, ts);
      if (labs(diff) > 60) dbg("[RTC] Large drift detected. Past records keep their original timestamps.");
    }
  } else {
    xSemaphoreGive(i2cMutex);
    dbg("[RTC] Within %ld s of NTP - no correction needed.", diff);
  }
  return true;
}

// =====================================================================
// THINGSPEAK BACKLOG UPLOAD
// =====================================================================
// Uploads records from the SD log, starting at the saved pointer, using
// ThingSpeak bulk_update with created_at = ORIGINAL measurement timestamp.
// Records with time_valid=0 stay on SD but are not uploaded.

// Parse one CSV line and append a JSON object. Returns true if added.
// ThingSpeak fields (calibrated values only):
//   field1 = temperature, field2 = humidity,
//   field3 = pm25_cal, field4 = pm10_cal, field5 = co2_cal
bool appendRecordJson(const String& line, String& body, bool needComma) {
  String tok[CSV_COLS];
  int n = 0, start = 0;
  while (n < CSV_COLS) {
    int c = line.indexOf(',', start);
    if (c < 0) { tok[n++] = line.substring(start); break; }
    tok[n++] = line.substring(start, c);
    start = c + 1;
  }
  // tok: 0 ts | 1 temp | 2 hum | 3 pm25_raw | 4 pm25_cal | 5 pm10_raw | 6 pm10_cal
  //      7 co2_raw | 8 co2_cal | 9 time_valid
  if (n != CSV_COLS || tok[9] != "1" || tok[0].length() != 19) return false;

  static const char* names[5] = {"field1", "field2", "field3", "field4", "field5"};
  static const int   cols[5]  = {1, 2, 4, 6, 8};          // temp, hum, pm25_cal, pm10_cal, co2_cal
  String rec = "{\"created_at\":\"" + tok[0] + " " + TZ_OFFSET_STR + "\"";
  int fields = 0;
  for (int i = 0; i < 5; i++) {
    const String& v = tok[cols[i]];
    if (v.length() && v != "NA") {
      rec += ",\""; rec += names[i]; rec += "\":\""; rec += v; rec += "\"";
      fields++;
    }
  }
  rec += "}";
  if (!fields) return false;
  if (needComma) body += ",";
  body += rec;
  return true;
}

void uploadBacklog() {
  static bool warnedNoChannel = false;
  if (strcmp(THINGSPEAK_CHANNEL_ID, "YOUR_CHANNEL_ID") == 0) {
    if (!warnedNoChannel) { dbg("[CLOUD] Set THINGSPEAK_CHANNEL_ID to enable uploads."); warnedNoChannel = true; }
    return;
  }
  if (WiFi.status() != WL_CONNECTED) return;                  // post ONLY when Wi-Fi is up
  if (!(xEventGroupGetBits(sysEvents) & EVT_SD_OK)) return;   // backlog lives on SD

  // ---- read a batch from SD (short, under mutex) ----
  String body = "{\"write_api_key\":\"";
  body += THINGSPEAK_WRITE_API_KEY;
  body += "\",\"updates\":[";
  int count = 0, skipped = 0;
  uint32_t pos, newPos;
  bool more;

  xSemaphoreTake(sdMutex, portMAX_DELAY);
  pos = readUploadPos();
  File f = SD.open(LOG_FILE, FILE_READ);
  if (!f) { xSemaphoreGive(sdMutex); return; }
  if (pos > f.size()) pos = 0;               // log was replaced -> start over
  f.seek(pos);
  while (f.available() && count < UPLOAD_BATCH) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length() || !isDigit(line[0])) continue;       // header / blank
    if (appendRecordJson(line, body, count > 0)) count++;
    else skipped++;                                           // invalid time / broken line
  }
  newPos = f.position();
  more = f.available();
  f.close();
  if (count == 0 && newPos != pos) saveUploadPos(newPos);    // only skipped lines
  xSemaphoreGive(sdMutex);

  if (skipped) dbg("[CLOUD] Skipped %d record(s) (invalid time or damaged line) - kept on SD.", skipped);
  if (count == 0) return;
  body += "]}";

  // ---- send (no mutex held, sensor + SD tasks keep running) ----
  WiFiClientSecure client;
  client.setInsecure();                      // ThingSpeak cert not pinned
  HTTPClient http;
  String url = String("https://api.thingspeak.com/channels/") + THINGSPEAK_CHANNEL_ID + "/bulk_update.json";
  http.setTimeout(10000);
  if (!http.begin(client, url)) { dbg("[CLOUD] HTTP begin failed."); return; }
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);

  if (code == 200 || code == 202) {
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    saveUploadPos(newPos);
    xSemaphoreGive(sdMutex);
    xEventGroupSetBits(sysEvents, EVT_INTERNET_OK);
    dbg("[CLOUD] Uploaded %d record(s) with original timestamps%s.", count,
        more ? " (more backlog pending)" : "");
  } else if (code < 0) {
    xEventGroupClearBits(sysEvents, EVT_INTERNET_OK);
    dbg("[CLOUD] Upload failed: %s (records kept, will retry).", http.errorToString(code).c_str());
  } else {
    dbg("[CLOUD] Server rejected upload: HTTP %d %s", code, http.getString().c_str());
  }
  http.end();
}

// =====================================================================
// TASK: SENSOR  (warm-up, then fixed-interval acquisition)
// =====================================================================
void sensorTask(void* arg) {
  // ---- WARM-UP: no logging until complete ----
  nodeState = ST_WARMUP;
  dbg("[WARMUP] Starting 3-minute warm-up...");
  for (uint32_t s = WARMUP_SECONDS; s > 0; s--) {
    if (s % 30 == 0 && s != WARMUP_SECONDS) {
      int c;
      if (readCO2(c)) dbg("[WARMUP] Remaining: %u s | CO2 now: %d ppm", (unsigned)s, c);
      else            dbg("[WARMUP] Remaining: %u s", (unsigned)s);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  dbg("[WARMUP] Complete.");
  xEventGroupSetBits(sysEvents, EVT_WARMUP_DONE);

  // ---- NORMAL ACQUISITION ----
  // vTaskDelayUntil keeps a fixed cadence independent of how long reads take,
  // so timestamps advance by exactly the sample interval.
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastUnix = 0;

  for (;;) {
    Record r = {};

    // 1) Timestamp FIRST, from RTC = moment of measurement
    DateTime now;
    bool plausible = readRtc(now);
    if (rtcPresent) formatTimestamp(now, r.ts, sizeof(r.ts));
    else            strcpy(r.ts, "0000-00-00 00:00:00");
    r.timeValid = plausible && rtcTimeValid;
    if (plausible && rtcTimeValid) {
      if (now.unixtime() <= lastUnix)
        dbg("[RTC] Warning: timestamp not increasing (RTC was corrected backwards).");
      lastUnix = now.unixtime();
    } else {
      dbg("[RTC] Time INVALID - record flagged time_valid=0.");
    }

    // 2) Sensors
    r.pmValid      = readSDS011(r.pm25, r.pm10);
    r.climateValid = readAHT(r.temperature, r.humidity);
    r.co2Valid     = readCO2(r.co2);

    // 3) Calibrate: y = m*x + c
    if (r.pmValid) {
      r.pm25Cal = applyCal(r.pm25, CAL.m_pm25, CAL.c_pm25);
      r.pm10Cal = applyCal(r.pm10, CAL.m_pm10, CAL.c_pm10);
    }
    if (r.co2Valid) r.co2Cal = applyCal((float)r.co2, CAL.m_co2, CAL.c_co2);

    String sT  = r.climateValid ? String(r.temperature, 2) : "NA";
    String sH  = r.climateValid ? String(r.humidity, 2)    : "NA";
    String s25 = r.pmValid  ? String(r.pm25, 1) + " -> " + String(r.pm25Cal, 1) : "NA";
    String s10 = r.pmValid  ? String(r.pm10, 1) + " -> " + String(r.pm10Cal, 1) : "NA";
    String sC  = r.co2Valid ? String(r.co2)     + " -> " + String(r.co2Cal, 0)  : "NA";
    dbg("[DATA] %s | T %s | RH %s | PM2.5 %s | PM10 %s | CO2 %s  (raw -> cal)",
        r.ts, sT.c_str(), sH.c_str(), s25.c_str(), s10.c_str(), sC.c_str());

    // 4) Hand to SD logger (never waits on network)
    if (xQueueSend(logQueue, &r, 0) != pdTRUE)
      dbg("[SD] RAM buffer full (SD unavailable) - record dropped.");

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
  }
}

// =====================================================================
// TASK: SD LOGGER
// =====================================================================
// Peeks a record, writes it, and only removes it from the queue after a
// successful write. If the card is removed, records wait in the queue
// while the card is re-initialised every SD_RETRY_MS.
void sdLoggerTask(void* arg) {
  Record r;
  uint32_t lastRetry = 0;
  for (;;) {
    if (xQueuePeek(logQueue, &r, portMAX_DELAY) != pdTRUE) continue;

    if (!(xEventGroupGetBits(sysEvents) & EVT_SD_OK)) {
      if (millis() - lastRetry < SD_RETRY_MS) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
      lastRetry = millis();
      dbg("[SD] Retrying card init... (%u record(s) waiting in RAM)",
          (unsigned)uxQueueMessagesWaiting(logQueue));
      if (!sdInit(false)) continue;
    }

    if (writeRecord(r)) {
      xQueueReceive(logQueue, &r, 0);
      dbg("[SD] Saved: %s", r.ts);
    } else {
      dbg("[SD] WRITE FAILED - logging failure detected (card removed?).");
      xEventGroupClearBits(sysEvents, EVT_SD_OK);
      lastRetry = millis();
    }
  }
}

// =====================================================================
// TASK: TIME  (NTP -> RTC, RTC health)
// =====================================================================
void timeTask(void* arg) {
  bool syncedThisConnection = false;
  uint32_t lastSync = 0, lastTry = 0, lastHealth = 0;

  for (;;) {
    EventBits_t b = xEventGroupWaitBits(sysEvents, EVT_NTP_REQUEST, pdTRUE, pdFALSE,
                                        pdMS_TO_TICKS(5000));
    if (b & EVT_NTP_REQUEST) { syncedThisConnection = false; lastTry = 0; }

    bool wifi = xEventGroupGetBits(sysEvents) & EVT_WIFI_UP;
    if (!wifi) {
      syncedThisConnection = false;
    } else {
      bool due     = !syncedThisConnection || (millis() - lastSync >= NTP_RESYNC_MS);
      bool retryOk = (lastTry == 0) || (millis() - lastTry >= NTP_RETRY_MS);
      if (due && retryOk) {
        lastTry = millis();
        if (syncRtcFromNtp()) {
          syncedThisConnection = true;
          lastSync = millis();
          xEventGroupSetBits(sysEvents, EVT_INTERNET_OK);   // real internet confirmed
        } else {
          dbg("[NTP] Continuing with RTC time. Retry in %u s.", (unsigned)(NTP_RETRY_MS / 1000));
        }
      }
    }

    // RTC health check every 60 s
    if (rtcPresent && millis() - lastHealth >= 60000UL) {
      lastHealth = millis();
      DateTime t;
      if (!readRtc(t) && rtcTimeValid) {
        rtcTimeValid = false;
        dbg("[RTC] Health check FAILED - time implausible, records will be flagged.");
      }
    }
  }
}

// =====================================================================
// TASK: NETWORK  (Wi-Fi reconnection + upload)
// =====================================================================
// WiFi.begin() is non-blocking; this task never touches sensor timing.
void networkTask(void* arg) {
  bool wasUp = false, everConnected = false;
  uint32_t lastAttempt = millis(), lastUpload = 0;

  for (;;) {
    bool up = (WiFi.status() == WL_CONNECTED);

    if (up && !wasUp) {                                   // --- connected edge
      xEventGroupSetBits(sysEvents, EVT_WIFI_UP);
      if (everConnected) dbg("[WIFI] Connection restored. IP=%s", WiFi.localIP().toString().c_str());
      else               dbg("[WIFI] Connected. IP=%s", WiFi.localIP().toString().c_str());
      everConnected = true;
      xEventGroupSetBits(sysEvents, EVT_NTP_REQUEST);     // ask time task to sync
    }
    if (!up && wasUp) {                                   // --- disconnected edge
      xEventGroupClearBits(sysEvents, EVT_WIFI_UP | EVT_INTERNET_OK);
      dbg("[WIFI] Connection lost.");
    }
    wasUp = up;

    if (!up) {
      if (millis() - lastAttempt >= WIFI_RETRY_MS) {
        lastAttempt = millis();
        dbg("[WIFI] Reconnecting...");
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      }
    } else if (millis() - lastUpload >= UPLOAD_INTERVAL_MS) {
      lastUpload = millis();
      uploadBacklog();                                   // also confirms internet
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// =====================================================================
// TASK: STATUS  (node state machine + GPIO 13 LED)
// =====================================================================
const char* stateName(NodeState s) {
  switch (s) {
    case ST_BOOT:              return "BOOT";
    case ST_INIT:              return "INITIALIZATION";
    case ST_WARMUP:            return "WARMUP";
    case ST_NETWORK_CONNECTED: return "NETWORK_CONNECTED";
    case ST_OFFLINE_LOGGING:   return "OFFLINE_LOGGING";
    case ST_LOGGING_FAULT:     return "LOGGING_FAULT";
  }
  return "?";
}

void onStateChange(NodeState from, NodeState to, bool normalStarted) {
  bool firstNormal = !normalStarted &&
                     (to == ST_NETWORK_CONNECTED || to == ST_OFFLINE_LOGGING);
  if (firstNormal) {
    dbg("[NODE] Normal operation started.");
    dbg("[SD] Logging enabled.");
  }
  switch (to) {
    case ST_NETWORK_CONNECTED:
      if (from == ST_OFFLINE_LOGGING) {
        dbg("[SD] Offline records preserved (uploading with original timestamps).");
        dbg("[NODE] Network operation resumed.");
      }
      break;
    case ST_OFFLINE_LOGGING:
      dbg("[NODE] Entering offline logging mode.");
      dbg("[RTC] Using RTC time.");
      dbg("[SD] Local logging active.");
      break;
    case ST_LOGGING_FAULT:
      dbg("[SD] LOGGING FAILURE - check SD card. Sensing continues, records held in RAM.");
      break;
    default: break;
  }
  dbg("[NODE] State: %s -> %s", stateName(from), stateName(to));
}

void statusTask(void* arg) {
  NodeState last = ST_INIT;
  bool normalStarted = false, ledOn = false;
  uint8_t tick = 0;

  for (;;) {
    EventBits_t b = xEventGroupGetBits(sysEvents);
    bool warm = b & EVT_WARMUP_DONE;
    bool sd   = b & EVT_SD_OK;
    bool net  = (b & EVT_WIFI_UP) && (b & EVT_INTERNET_OK);   // Wi-Fi alone is not enough

    NodeState s = !warm ? ST_WARMUP
                : !sd   ? ST_LOGGING_FAULT
                : net   ? ST_NETWORK_CONNECTED
                        : ST_OFFLINE_LOGGING;

    if (s != last) {
      onStateChange(last, s, normalStarted);
      if (s == ST_NETWORK_CONNECTED || s == ST_OFFLINE_LOGGING) normalStarted = true;
      last = s;
      nodeState = s;
    }

    // LED: OFF = not logging | BLINK = logging + online | ON = logging + offline
    if (!warm || !sd) {
      ledOn = false;
    } else if (net) {
      if (++tick >= 5) { tick = 0; ledOn = !ledOn; }   // 500 ms on / 500 ms off
    } else {
      ledOn = true;
    }
    digitalWrite(LED_PIN, ledOn ? HIGH : LOW);

    vTaskDelay(pdMS_TO_TICKS(100));                     // non-blocking blink
  }
}

// =====================================================================
// SETUP  (BOOT -> INITIALIZATION -> start tasks)
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  printMutex = xSemaphoreCreateMutex();
  i2cMutex   = xSemaphoreCreateMutex();
  sdMutex    = xSemaphoreCreateMutex();
  sysEvents  = xEventGroupCreate();
  logQueue   = xQueueCreate(LOG_QUEUE_LEN, sizeof(Record));

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  dbg("========================================");
  dbg("Embedded AQ Node Starting");
  dbg("========================================");
  nodeState = ST_INIT;

  // ---- Calibration ----
  dbg("[CAL] %s, ThingSpeak channel %s", NODE_NAME, THINGSPEAK_CHANNEL_ID);
  dbg("[CAL] Calibration loaded (calibrated = m*raw + c):");
  dbg("[CAL]   PM2.5: m=%.4f c=%.4f", CAL.m_pm25, CAL.c_pm25);
  dbg("[CAL]   PM10 : m=%.4f c=%.4f", CAL.m_pm10, CAL.c_pm10);
  dbg("[CAL]   CO2  : m=%.4f c=%.4f", CAL.m_co2,  CAL.c_co2);

  // ---- Sensors ----
  dbg("[BOOT] Initializing sensors...");
  Wire.begin(SDA_PIN, SCL_PIN);
  dbg(aht.begin(&Wire) ? "[BOOT] AHT10: OK" : "[BOOT] AHT10: FAILED");
  SDS011Serial.begin(9600, SERIAL_8N1, SDS_RX, SDS_TX);
  CO2Serial.begin(9600, SERIAL_8N1, CO2_RX, CO2_TX);
  SDS011Serial.write(SDS_WAKE, sizeof(SDS_WAKE));
  SDS011Serial.flush();

  // ---- RTC (never set from compile time) ----
  dbg("[BOOT] Initializing RTC...");
  if (rtc.begin(&Wire)) {
    rtcPresent = true;
    bool lost = rtc.lostPower();
    DateTime t = rtc.now();
    char ts[20];
    formatTimestamp(t, ts, sizeof(ts));
    dbg("[RTC] Time: %s", ts);
    if (lost) {
      dbg("[RTC] INVALID: RTC lost power (check CR2032 battery). Records flagged until NTP sync.");
    } else if (!isPlausibleTime(t)) {
      dbg("[RTC] INVALID: implausible/default time. Records flagged until NTP sync.");
    } else {
      rtcTimeValid = true;
      dbg("[RTC] Time valid (battery-backed). Continuing RTC timeline.");
    }
  } else {
    dbg("[RTC] DS3231 NOT FOUND - all records will be flagged time_valid=0.");
  }

  // ---- SD card ----
  dbg("[BOOT] Initializing SD card...");
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sdInit(true);

  // ---- Wi-Fi (non-blocking; network task takes over) ----
  dbg("[BOOT] Initializing Wi-Fi...");
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  sntp_set_time_sync_notification_cb(onNtpSync);

  // ---- Tasks ----
  // Core 1: sensing, logging, status.  Core 0: networking (with the Wi-Fi stack).
  xTaskCreatePinnedToCore(sensorTask,   "sensor",  6144, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(sdLoggerTask, "sdlog",   6144, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(statusTask,   "status",  3072, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(timeTask,     "time",    6144, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(networkTask,  "network", 16384, NULL, 1, NULL, 0);
}

void loop() {
  vTaskDelete(NULL);   // all work is done in FreeRTOS tasks
}
