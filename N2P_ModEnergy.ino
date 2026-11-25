// N2P ModEnergy - ESP32
// วัด pH / อุณหภูมิ / กระแส แล้วคุมรีเลย์ + เว็บ dashboard
// มีส่วนวิเคราะห์แนวโน้มแบบง่ายๆ (z-score + เส้นตรง) ไว้เตือนล่วงหน้า

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

// ---------------- WiFi ----------------
const char* WIFI_SSID = "PUT_YOUR_WIFI_SSID";
const char* WIFI_PASS = "PUT_YOUR_WIFI_PASSWORD";
const char* AP_SSID   = "N2P-ESP32";
const char* AP_PASS   = "12345678";   // อย่างน้อย 8 ตัว

// ---------------- ขา ----------------
const int PIN_RELAY   = 22;   // รีเลย์ (active LOW)
const int PIN_PH_ADC  = 34;
const int PIN_ONEWIRE = 4;    // DS18B20
const int PIN_BUZZER  = 25;
const int PIN_SDA     = 21;   // INA219 (ย้าย SCL ไป 19 ไม่ให้ชนรีเลย์ที่ 22)
const int PIN_SCL     = 19;

const bool RELAY_ACTIVE_LOW  = true;
const bool BUZZER_ACTIVE_LOW = true;

// ---------------- เวลา ----------------
const uint32_t LOOP_DELAY_MS      = 20;
const uint32_t SAMPLE_INTERVAL_MS = 500;
const uint32_t AI_INTERVAL_MS     = 1000;

// ---------------- ค่าความปลอดภัย (แก้ผ่านเว็บได้) ----------------
float    PH_LOW_TH        = 4.0;
float    PH_HIGH_TH       = 9.0;
float    OVERCURRENT_A_TH = 0.60;
float    OVERTEMP_C_TH    = 35.0;
uint32_t TRIP_DELAY_MS    = 6000;

// ---------------- ตัวแปรหลัก ----------------
WebServer server(80);
Preferences prefs;
Adafruit_INA219 ina219;
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature ds18b20(&oneWire);

enum RunMode { MODE_AUTO = 0, MODE_MANUAL = 1 };
RunMode runMode = MODE_AUTO;

enum SafetyState { SAFE = 0, WARN = 1, TRIP = 2 };
SafetyState safety = SAFE;
String lastTripReason = "";
uint32_t outOfRangeSince = 0;
bool tripBeeped = false;

bool relayOn = false;

// calibration pH 2 จุด
float phSlope  = -5.70;
float phOffset = 21.34;
bool  haveV7 = false, haveV4 = false;
float vAtPH7 = 0, vAtPH4 = 0;

float lastPhVoltage = 0;
int   lastPhRaw = 0;

float tempC    = NAN;
float loadV    = NAN;
float currentA = NAN;
float powerW   = NAN;
float energyWh = 0;

uint32_t lastSampleMs = 0;
uint32_t lastEnergyMs = 0;
uint32_t lastAIMs     = 0;
String   lastAIJson   = "{}";

// pulse duty ของ EC (ตอนนี้แค่คำนวณสถานะไว้ ยังไม่ได้ไปคุมรีเลย์ เหมือนโค้ดเดิม)
float    EC_DUTY      = 0.70;
uint32_t EC_PERIOD_MS = 10000;
bool     ecPulseOn    = false;
uint32_t ecPulseStart = 0;

// ---------------- ค่าเฉลี่ยเคลื่อนที่ 10 ค่า ----------------
const int AVG_N = 10;

struct MovAvg {
  float v[AVG_N];
  int   idx = 0;
  bool  full = false;

  void init() {
    for (int i = 0; i < AVG_N; i++) v[i] = NAN;
    idx = 0;
    full = false;
  }

  void push(float x) {
    v[idx] = x;
    idx = (idx + 1) % AVG_N;
    if (idx == 0) full = true;
  }

  float avg() const {
    float sum = 0;
    int cnt = 0;
    if (!full) {
      for (int i = 0; i < AVG_N; i++) {
        if (!isnan(v[i])) { sum += v[i]; cnt++; }
      }
      return cnt > 0 ? sum / cnt : NAN;
    }
    for (int i = 0; i < AVG_N; i++) sum += v[i];
    return sum / AVG_N;
  }
};

MovAvg phAvgBuf, tempAvgBuf, currAvgBuf;

void beep(uint16_t ms = 80, uint8_t n = 1, uint16_t gap = 60);

// =====================================================
//  ส่วนวิเคราะห์แนวโน้ม
// =====================================================
const int   WIN_SIZE  = 60;     // เก็บย้อนหลัง 60 ค่า (30 วินาที)
const int   PRED_STEPS = 6;     // ทำนายล่วงหน้า 6 ค่า (~3 วินาที)
const float Z_WARN = 1.8;
const float Z_TRIP = 2.8;

struct Trend {
  float value, mean, stddev, zscore, slope, prediction;
  uint8_t anomaly;   // 0 ปกติ, 1 เฝ้าระวัง, 2 ผิดปกติ
};

struct Window {
  float data[WIN_SIZE];
  uint8_t head = 0;
  uint8_t count = 0;

  void clear() {
    head = 0;
    count = 0;
    memset(data, 0, sizeof(data));
  }

  void push(float x) {
    data[head] = x;
    head = (head + 1) % WIN_SIZE;
    if (count < WIN_SIZE) count++;
  }

  // i = 0 คือเก่าสุด
  float at(uint8_t i) const {
    return data[(head - count + i + WIN_SIZE) % WIN_SIZE];
  }
};

Window phWin, tempWin, currWin;

// ระดับคำแนะนำ
enum AdviceLevel { ADV_OK = 0, ADV_INFO = 1, ADV_WARN = 2, ADV_ACT = 3 };

struct Advice {
  AdviceLevel level;
  char channel[8];
  char message[120];
  float predicted;
};

Trend analyze(const Window& w) {
  Trend r = {};
  if (w.count < 4) return r;

  float sum = 0;
  for (uint8_t i = 0; i < w.count; i++) sum += w.at(i);
  r.mean = sum / w.count;

  float sq = 0;
  for (uint8_t i = 0; i < w.count; i++) {
    float d = w.at(i) - r.mean;
    sq += d * d;
  }
  r.stddev = sqrtf(sq / w.count);
  if (r.stddev < 1e-6f) r.stddev = 1e-6f;   // กันหารศูนย์

  r.value  = w.at(w.count - 1);
  r.zscore = (r.value - r.mean) / r.stddev;

  // ความชันของเส้นตรงที่ fit กับข้อมูล
  float n  = w.count;
  float mx = (n - 1) / 2.0f;
  float num = 0, den = 0;
  for (uint8_t i = 0; i < w.count; i++) {
    float dx = i - mx;
    float dy = w.at(i) - r.mean;
    num += dx * dy;
    den += dx * dx;
  }
  r.slope = (den > 1e-9f) ? num / den : 0;
  r.prediction = r.value + r.slope * PRED_STEPS;

  float az = fabsf(r.zscore);
  if (az >= Z_TRIP)      r.anomaly = 2;
  else if (az >= Z_WARN) r.anomaly = 1;
  else                   r.anomaly = 0;

  return r;
}

// ---------------- คำแนะนำ ----------------
// ความชันต่อ sample ที่ถือว่าเริ่มน่าห่วง
const float PH_SLOPE_WARN   = 0.008;
const float TEMP_SLOPE_WARN = 0.04;
const float CURR_SLOPE_WARN = 0.003;

// เลือกคำแนะนำที่หนักสุดมา 1 อัน
Advice makeAdvice(const Trend& ph, const Trend& t, const Trend& cur) {
  Advice best = {ADV_OK, "system", "Operating within normal parameters", 0};

  // กระแสสูง / กำลังขึ้น
  if (cur.anomaly == 2 || (cur.slope > CURR_SLOPE_WARN && cur.prediction > OVERCURRENT_A_TH * 0.85f)) {
    Advice a;
    a.level = (cur.anomaly == 2) ? ADV_ACT : ADV_WARN;
    strncpy(a.channel, "curr", 8);
    snprintf(a.message, 120,
             "Current rising (slope %.4fA/s). Predicted %.3fA in ~3s. "
             "Consider reducing load before %.2fA threshold.",
             cur.slope * 2.0f, cur.prediction, OVERCURRENT_A_TH);
    a.predicted = cur.prediction;
    if (a.level > best.level) best = a;
  }

  // อุณหภูมิ
  if (t.anomaly >= 1 || t.slope > TEMP_SLOPE_WARN) {
    Advice a;
    if (t.anomaly == 2)                  a.level = ADV_ACT;
    else if (t.slope > TEMP_SLOPE_WARN)  a.level = ADV_WARN;
    else                                 a.level = ADV_INFO;
    strncpy(a.channel, "temp", 8);
    snprintf(a.message, 120,
             "Temp trending up %.2f°C/s (now %.1f°C, predicted %.1f°C). "
             "Increase ventilation or reduce duty cycle.",
             t.slope * 2.0f, t.value, t.prediction);
    a.predicted = t.prediction;
    if (a.level > best.level) best = a;
  }

  // pH ไหลขึ้น
  if (ph.slope > PH_SLOPE_WARN) {
    Advice a;
    if (ph.prediction > PH_HIGH_TH * 0.92f) a.level = ADV_ACT;
    else if (ph.anomaly >= 1)               a.level = ADV_WARN;
    else                                    a.level = ADV_INFO;
    strncpy(a.channel, "pH", 8);
    snprintf(a.message, 120,
             "pH rising (%.4f/s). Predicted %.2f in ~3s. "
             "Check alkaline contamination. Target range %.1f–%.1f.",
             ph.slope * 2.0f, ph.prediction, PH_LOW_TH, PH_HIGH_TH);
    a.predicted = ph.prediction;
    if (a.level > best.level) best = a;
  }

  // pH ไหลลง
  if (ph.slope < -PH_SLOPE_WARN) {
    Advice a;
    if (ph.prediction < PH_LOW_TH * 1.08f) a.level = ADV_ACT;
    else if (ph.anomaly >= 1)              a.level = ADV_WARN;
    else                                   a.level = ADV_INFO;
    strncpy(a.channel, "pH", 8);
    snprintf(a.message, 120,
             "pH dropping (%.4f/s). Predicted %.2f in ~3s. "
             "Add buffer solution. Target range %.1f–%.1f.",
             ph.slope * 2.0f, ph.prediction, PH_LOW_TH, PH_HIGH_TH);
    a.predicted = ph.prediction;
    if (a.level > best.level) best = a;
  }

  // ค่ากระโดดทันที (slope อาจยังเล็ก)
  if (ph.anomaly == 2 && best.level < ADV_WARN) {
    Advice a = {ADV_WARN, "pH",
                "pH spike detected (Z-score anomaly). "
                "Check sensor connection and solution turbidity.",
                ph.value};
    best = a;
  }

  return best;
}

// ต่อ JSON ของแต่ละ channel
String channelJson(const char* name, const Trend& r, int dVal, int dSlope) {
  String s = "\"";
  s += name;
  s += "\":{\"val\":"  + String(r.value, dVal);
  s += ",\"mean\":"     + String(r.mean, dVal);
  s += ",\"z\":"        + String(r.zscore, 2);
  s += ",\"slope\":"    + String(r.slope, dSlope);
  s += ",\"pred\":"     + String(r.prediction, dVal);
  s += ",\"anom\":"     + String(r.anomaly) + "}";
  return s;
}

String buildAIJson(const Trend& ph, const Trend& t, const Trend& cur, const Advice& adv) {
  String j = "{";
  j += channelJson("ph",   ph,  2, 4) + ",";
  j += channelJson("temp", t,   1, 4) + ",";
  j += channelJson("curr", cur, 3, 5) + ",";
  j += "\"advice\":{\"level\":" + String((int)adv.level);
  j += ",\"ch\":\""  + String(adv.channel) + "\"";
  j += ",\"msg\":\"" + String(adv.message) + "\"";
  j += ",\"pred\":"  + String(adv.predicted, 2) + "}}";
  return j;
}

void runAnalysis() {
  if (phWin.count < 10) return;   // รอให้มีข้อมูลพอก่อน

  Trend ph   = analyze(phWin);
  Trend temp = analyze(tempWin);
  Trend curr = analyze(currWin);

  Advice adv = makeAdvice(ph, temp, curr);
  lastAIJson = buildAIJson(ph, temp, curr, adv);

  if (adv.level >= ADV_WARN) {
    Serial.print("[AI] ");
    Serial.print(adv.channel);
    Serial.print(" lvl=");
    Serial.print(adv.level);
    Serial.print(" | ");
    Serial.println(adv.message);
  }
  if (adv.level == ADV_ACT && safety == SAFE) beep(40, 1, 0);
}

// =====================================================
//  buzzer / relay
// =====================================================
void buzzerOn()  { digitalWrite(PIN_BUZZER, BUZZER_ACTIVE_LOW ? LOW : HIGH); }
void buzzerOff() { digitalWrite(PIN_BUZZER, BUZZER_ACTIVE_LOW ? HIGH : LOW); }

void beep(uint16_t ms, uint8_t n, uint16_t gap) {
  for (uint8_t i = 0; i < n; i++) {
    buzzerOn();
    delay(ms);
    buzzerOff();
    if (i + 1 < n) delay(gap);
  }
}

void setRelay(bool on) {
  relayOn = on;
  bool level = RELAY_ACTIVE_LOW ? !on : on;
  digitalWrite(PIN_RELAY, level ? HIGH : LOW);
}

// =====================================================
//  เก็บค่าลง flash
// =====================================================
void saveCalibration() {
  prefs.begin("cal", false);
  prefs.putFloat("phSlope", phSlope);
  prefs.putFloat("phOffset", phOffset);
  prefs.end();
}

void loadCalibration() {
  prefs.begin("cal", true);
  phSlope  = prefs.getFloat("phSlope", phSlope);
  phOffset = prefs.getFloat("phOffset", phOffset);
  prefs.end();
}

void saveLimits() {
  prefs.begin("lim", false);
  prefs.putFloat("phLo", PH_LOW_TH);
  prefs.putFloat("phHi", PH_HIGH_TH);
  prefs.putFloat("iMax", OVERCURRENT_A_TH);
  prefs.putFloat("tMax", OVERTEMP_C_TH);
  prefs.putUInt("tdly", TRIP_DELAY_MS);
  prefs.putUChar("mode", (uint8_t)runMode);
  prefs.end();
}

void loadLimits() {
  prefs.begin("lim", true);
  PH_LOW_TH        = prefs.getFloat("phLo", PH_LOW_TH);
  PH_HIGH_TH       = prefs.getFloat("phHi", PH_HIGH_TH);
  OVERCURRENT_A_TH = prefs.getFloat("iMax", OVERCURRENT_A_TH);
  OVERTEMP_C_TH    = prefs.getFloat("tMax", OVERTEMP_C_TH);
  TRIP_DELAY_MS    = prefs.getUInt("tdly", TRIP_DELAY_MS);
  runMode          = (RunMode)prefs.getUChar("mode", (uint8_t)runMode);
  prefs.end();
}

// =====================================================
//  อ่านเซนเซอร์
// =====================================================
float readPhVoltage(int& rawOut) {
  const int N = 21;
  int a[N];

  analogSetPinAttenuation(PIN_PH_ADC, ADC_11db);
  for (int i = 0; i < N; i++) {
    a[i] = analogRead(PIN_PH_ADC);
    delay(3);
  }

  // เรียงแล้วเอาค่ากลาง (median) กัน noise
  for (int i = 0; i < N - 1; i++) {
    for (int j = i + 1; j < N; j++) {
      if (a[j] < a[i]) {
        int tmp = a[i];
        a[i] = a[j];
        a[j] = tmp;
      }
    }
  }
  rawOut = a[N / 2];
  return rawOut / 4095.0f * 3.3f;
}

float phFromVoltage(float v) {
  return constrain(phSlope * v + phOffset, 0.0f, 14.0f);
}

void updateSafety(float phAvg, float iAvg, float tAvg) {
  bool phBad = !isnan(phAvg) && (phAvg < PH_LOW_TH || phAvg > PH_HIGH_TH);
  bool iBad  = !isnan(iAvg) && iAvg > OVERCURRENT_A_TH;
  bool tBad  = !isnan(tAvg) && tAvg > OVERTEMP_C_TH;

  uint32_t now = millis();

  if (phBad || iBad || tBad) {
    if (outOfRangeSince == 0) outOfRangeSince = now;
    safety = WARN;

    if (now - outOfRangeSince >= TRIP_DELAY_MS) {
      safety = TRIP;

      if (phBad)      lastTripReason = "pH out of range";
      else if (iBad)  lastTripReason = "Overcurrent";
      else if (tBad)  lastTripReason = "Overtemperature";
      else            lastTripReason = "Unknown";

      // บี๊ปครั้งเดียวตอนเข้า TRIP
      if (!tripBeeped) {
        if (runMode == MODE_AUTO) {
          if (relayOn) setRelay(false);
          beep(80, 3, 60);
        } else {
          beep(80, 2, 80);   // MANUAL ไม่ตัด แค่เตือน
        }
        tripBeeped = true;
      }
    }
  } else {
    outOfRangeSince = 0;
    safety = SAFE;
    lastTripReason = "";
    tripBeeped = false;
  }
}

void sampleSensors() {
  // pH
  lastPhVoltage = readPhVoltage(lastPhRaw);
  float ph = phFromVoltage(lastPhVoltage);
  phAvgBuf.push(ph);

  // อุณหภูมิ
  ds18b20.requestTemperatures();
  float t = ds18b20.getTempCByIndex(0);
  tempC = (t > -50 && t < 125) ? t : NAN;
  tempAvgBuf.push(tempC);

  // INA219
  float busV    = ina219.getBusVoltage_V();
  float shuntMv = ina219.getShuntVoltage_mV();
  loadV    = busV + shuntMv / 1000.0f;
  currentA = ina219.getCurrent_mA() / 1000.0f;
  powerW   = loadV * currentA;
  currAvgBuf.push(currentA);

  // พลังงานสะสม
  uint32_t now = millis();
  if (lastEnergyMs == 0) lastEnergyMs = now;
  float dtHours = (now - lastEnergyMs) / 3600000.0f;
  lastEnergyMs = now;
  if (dtHours >= 0 && !isnan(powerW)) energyWh += powerW * dtHours;

  updateSafety(phAvgBuf.avg(), currAvgBuf.avg(), tempAvgBuf.avg());

  // ป้อนให้ส่วนวิเคราะห์ (ถ้าเซนเซอร์เสียใช้ค่ากลางๆ แทน)
  phWin.push(ph);
  tempWin.push(isnan(tempC) ? 25.0f : tempC);
  currWin.push(isnan(currentA) ? 0.0f : currentA);
}

void updateECPulse() {
  if (!relayOn) { ecPulseOn = false; return; }
  if (EC_DUTY >= 0.99f) { ecPulseOn = true; return; }

  uint32_t phase  = (millis() - ecPulseStart) % EC_PERIOD_MS;
  uint32_t onTime = (uint32_t)(EC_PERIOD_MS * EC_DUTY);
  ecPulseOn = phase < onTime;
}

// =====================================================
//  WiFi
// =====================================================
void startAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("[AP] SSID: ");
  Serial.println(AP_SSID);
  Serial.print("[AP] Open: http://");
  Serial.println(WiFi.softAPIP());
}

void startWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] Connecting");

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] Connected. IP: http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] Failed -> AP mode");
    startAP();
  }
}

// =====================================================
//  หน้าเว็บ
// =====================================================
static const char PAGE[] = R"HTML(
<!doctype html>
<html lang="th">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>N2P ModEnergy</title>
<style>
:root{
  --bg:#0b0b14; --panel:#14141f; --line:#2a2a3a;
  --text:#f2f2f8; --muted:#a8a8bd;
  --accent:#b46cf5; --good:#22c55e; --warn:#f59e0b; --bad:#ef4444;
}
*{box-sizing:border-box}
body{
  margin:0; background:var(--bg); color:var(--text);
  font-family:system-ui,-apple-system,"Segoe UI","Noto Sans Thai",sans-serif;
}
.wrap{max-width:1100px; margin:0 auto; padding:20px 16px 40px}
h1{font-size:20px; margin:0}
.sub{color:var(--muted); font-size:13px; margin-top:4px}

.top{display:flex; justify-content:space-between; align-items:center; flex-wrap:wrap; gap:12px; margin-bottom:16px}
.pills{display:flex; flex-wrap:wrap; gap:8px}
.pill{
  display:flex; align-items:center; gap:6px;
  padding:6px 12px; border:1px solid var(--line); border-radius:999px;
  font-size:12px; color:var(--muted);
}
.dot{width:9px; height:9px; border-radius:50%; background:var(--muted)}
.dot.good{background:var(--good)}
.dot.warn{background:var(--warn)}
.dot.bad{background:var(--bad)}

.status{display:grid; grid-template-columns:repeat(4,1fr); gap:10px; margin-bottom:16px}
@media(max-width:800px){.status{grid-template-columns:1fr 1fr}}
.box{background:var(--panel); border:1px solid var(--line); border-radius:12px; padding:14px}
.box .label{font-size:12px; color:var(--muted)}
.box .value{font-size:24px; font-weight:700; margin-top:6px}
.box .note{font-size:12px; color:var(--muted); margin-top:4px}

.grid{display:grid; grid-template-columns:1.2fr .8fr; gap:14px}
@media(max-width:900px){.grid{grid-template-columns:1fr}}
.card{background:var(--panel); border:1px solid var(--line); border-radius:12px; margin-bottom:14px}
.card h2{font-size:14px; margin:0; padding:12px 16px; border-bottom:1px solid var(--line)}
.card .body{padding:16px}

.metrics{display:grid; grid-template-columns:repeat(3,1fr); gap:10px}
@media(max-width:700px){.metrics{grid-template-columns:1fr 1fr}}
@media(max-width:480px){.metrics{grid-template-columns:1fr}}

.advice{
  margin-top:12px; padding:12px 14px; border-radius:10px;
  border:1px solid var(--line); font-size:13px; line-height:1.6; color:var(--muted);
}

.btns{display:grid; grid-template-columns:1fr 1fr; gap:8px}
button{
  border:1px solid var(--line); border-radius:10px; padding:11px;
  background:#1d1d2b; color:var(--text); font-size:14px; font-weight:600; cursor:pointer;
}
button:hover{border-color:var(--accent)}
button.go{background:var(--accent); color:#160b22; border-color:var(--accent)}
button.stop{background:rgba(239,68,68,.15); border-color:rgba(239,68,68,.4); color:#ffd9d9}

.form{display:grid; grid-template-columns:1fr 1fr; gap:10px}
.field label{display:block; font-size:12px; color:var(--muted); margin-bottom:4px}
input,select{
  width:100%; padding:9px 10px; border-radius:8px;
  border:1px solid var(--line); background:#0d0d17; color:var(--text);
}
input:focus,select:focus{outline:none; border-color:var(--accent)}
hr{border:none; border-top:1px solid var(--line); margin:14px 0}
.small{font-size:12px; color:var(--muted); line-height:1.7}
code{background:#1d1d2b; padding:2px 6px; border-radius:6px; font-size:12px}
</style>
</head>
<body>
<div class="wrap">

  <div class="top">
    <div>
      <h1>N2P ModEnergy</h1>
      <div class="sub">Electrochemical nitrogen recovery - prototype</div>
    </div>
    <div class="pills">
      <div class="pill"><span class="dot" id="dotRun"></span><span id="run">-</span></div>
      <div class="pill"><span class="dot" id="dotSafe"></span><span id="safe">-</span></div>
      <div class="pill">MODE <b id="mode">-</b></div>
    </div>
  </div>

  <div class="status">
    <div class="box">
      <div class="label">System</div>
      <div class="value" id="relay">-</div>
      <div class="note" id="reason">-</div>
    </div>
    <div class="box">
      <div class="label">Power</div>
      <div class="value"><span id="pow">-</span> W</div>
      <div class="note">Energy <span id="ener">-</span> Wh</div>
    </div>
    <div class="box">
      <div class="label">Safety</div>
      <div class="value" id="safeHero">-</div>
      <div class="note">trip-delay monitoring</div>
    </div>
    <div class="box">
      <div class="label">Updates</div>
      <div class="value" id="tick">0</div>
      <div class="note">dashboard refresh count</div>
    </div>
  </div>

  <div class="grid">
    <div>
      <div class="card">
        <h2>Live metrics</h2>
        <div class="body">
          <div class="metrics">
            <div class="box">
              <div class="label">pH (avg)</div>
              <div class="value" id="ph">-</div>
              <div class="note"><span id="vph">-</span> V | ADC <span id="adc">-</span></div>
            </div>
            <div class="box">
              <div class="label">Temperature (avg)</div>
              <div class="value"><span id="temp">-</span> °C</div>
              <div class="note">DS18B20</div>
            </div>
            <div class="box">
              <div class="label">Voltage</div>
              <div class="value"><span id="volt">-</span> V</div>
              <div class="note">INA219</div>
            </div>
            <div class="box">
              <div class="label">Current (avg)</div>
              <div class="value"><span id="curr">-</span> A</div>
              <div class="note">load</div>
            </div>
            <div class="box">
              <div class="label">Calibration</div>
              <div class="value" style="font-size:16px">slope <span id="slope">-</span></div>
              <div class="note">offset <span id="offset">-</span></div>
            </div>
            <div class="box">
              <div class="label">Mode</div>
              <div class="value" id="modeMetric">-</div>
              <div class="note">AUTO / MANUAL</div>
            </div>
          </div>
        </div>
      </div>

      <div class="card">
        <h2>Trend monitor</h2>
        <div class="body">
          <div class="metrics">
            <div class="box">
              <div class="label">pH z-score</div>
              <div class="value" id="z-ph" style="font-size:20px">-</div>
              <div class="note">predicted: <span id="pred-ph">-</span></div>
            </div>
            <div class="box">
              <div class="label">Temp z-score</div>
              <div class="value" id="z-temp" style="font-size:20px">-</div>
              <div class="note">predicted: <span id="pred-temp">-</span></div>
            </div>
            <div class="box">
              <div class="label">Current z-score</div>
              <div class="value" id="z-curr" style="font-size:20px">-</div>
              <div class="note">predicted: <span id="pred-curr">-</span></div>
            </div>
          </div>
          <div class="advice" id="ai-advice">กำลังเก็บข้อมูล...</div>
        </div>
      </div>
    </div>

    <div>
      <div class="card">
        <h2>Control &amp; settings</h2>
        <div class="body">
          <div class="btns">
            <button class="go" onclick="relay(1)">START</button>
            <button class="stop" onclick="relay(0)">STOP</button>
            <button onclick="resetEnergy()">Reset Wh</button>
            <button onclick="refresh()">Refresh</button>
          </div>

          <hr>

          <div class="form">
            <div class="field">
              <label>Mode</label>
              <select id="modeSel">
                <option value="0">AUTO (cut off)</option>
                <option value="1">MANUAL (warn only)</option>
              </select>
            </div>
            <div class="field"><label>Trip delay (ms)</label><input id="tdly" type="number" min="0" step="500"></div>
            <div class="field"><label>pH low</label><input id="phlo" type="number" step="0.1"></div>
            <div class="field"><label>pH high</label><input id="phhi" type="number" step="0.1"></div>
            <div class="field"><label>I max (A)</label><input id="imax" type="number" step="0.01"></div>
            <div class="field"><label>T max (°C)</label><input id="tmax" type="number" step="0.5"></div>
          </div>
          <div style="height:10px"></div>
          <button style="width:100%" onclick="saveLimits()">Save settings</button>

          <hr>

          <div class="btns">
            <button onclick="cal7()">Cal pH 7</button>
            <button onclick="cal4()">Cal pH 4</button>
          </div>
          <div class="small" style="margin-top:10px">
            AUTO จะตัดรีเลย์เมื่อค่าหลุดเกณฑ์ต่อเนื่องตาม Trip delay<br>
            MANUAL ไม่ตัดเอง แค่แจ้ง WARN / TRIP
          </div>
        </div>
      </div>

      <div class="card">
        <h2>EC pulse</h2>
        <div class="body">
          <div class="form">
            <div class="field"><label>Duty (%)</label><input id="ec-duty" type="number" min="5" max="100" step="5" value="70"></div>
            <div class="field"><label>Period (ms)</label><input id="ec-period" type="number" min="1000" max="60000" step="1000" value="10000"></div>
          </div>
          <div style="height:10px"></div>
          <button style="width:100%" onclick="saveEC()">Apply</button>
        </div>
      </div>
    </div>
  </div>

  <div class="small">API: <code>/api/data</code> <code>/api/ai</code> <code>/api/relay</code> <code>/api/mode</code> <code>/api/limits</code></div>
</div>

<script>
let tick = 0;
const $ = id => document.getElementById(id);
const fmt = (x, n = 2) => Number.isFinite(x) ? x.toFixed(n) : "-";

async function post(url) { return (await fetch(url, {method: 'POST'})).text(); }
async function get(url)  { return (await fetch(url)).json(); }

async function relay(s)      { await post('/api/relay?state=' + s); refresh(); }
async function cal7()        { await post('/api/cal7'); refresh(); }
async function cal4()        { await post('/api/cal4'); refresh(); }
async function resetEnergy() { await post('/api/reset_energy'); refresh(); }

async function saveLimits() {
  const q = `phlo=${$('phlo').value}&phhi=${$('phhi').value}&imax=${$('imax').value}&tmax=${$('tmax').value}&tdly=${$('tdly').value}`;
  await post('/api/limits?' + q);
  await post('/api/mode?mode=' + $('modeSel').value);
  refresh();
  alert("Saved");
}

async function saveEC() {
  const duty = parseFloat($('ec-duty').value) / 100;
  const period = parseInt($('ec-period').value);
  await post(`/api/ec?duty=${duty}&period=${period}`);
  alert("EC settings applied");
}

function setSafety(text, cls) {
  $('safe').innerText = text;
  $('safeHero').innerText = text;
  $('dotSafe').className = 'dot ' + cls;
}

async function refresh() {
  const d = await get('/api/data');
  tick++;
  $('tick').innerText = tick;

  $('ph').innerText   = fmt(d.phAvg, 2);
  $('vph').innerText  = fmt(d.vph, 3);
  $('adc').innerText  = d.adc;
  $('temp').innerText = fmt(d.tempAvg, 1);
  $('volt').innerText = fmt(d.volt, 2);
  $('curr').innerText = fmt(d.currentAvg, 3);
  $('pow').innerText  = fmt(d.powerW, 2);
  $('ener').innerText = fmt(d.energyWh, 2);

  $('relay').innerText  = d.relay ? "RUNNING" : "STOPPED";
  $('reason').innerText = d.tripReason ? "TRIP: " + d.tripReason : "System ready";
  $('slope').innerText  = fmt(d.slope, 4);
  $('offset').innerText = fmt(d.offset, 4);

  const modeName = d.mode == 0 ? "AUTO" : "MANUAL";
  $('mode').innerText = modeName;
  $('modeMetric').innerText = modeName;
  $('modeSel').value = String(d.mode);

  // อย่าทับค่าที่กำลังพิมพ์อยู่
  const typing = document.activeElement && document.activeElement.tagName === 'INPUT';
  if (!typing) {
    $('tdly').value = d.tripDelayMs;
    $('phlo').value = d.phLo;
    $('phhi').value = d.phHi;
    $('imax').value = d.iMax;
    $('tmax').value = d.tMax;
  }

  $('run').innerText = d.relay ? "RUN" : "STOP";
  $('dotRun').className = 'dot ' + (d.relay ? 'good' : 'warn');

  if (d.safety == 0)      setSafety("SAFE", "good");
  else if (d.safety == 1) setSafety("WARN", "warn");
  else                    setSafety("TRIP", "bad");
}

const zColor = z => Math.abs(z) > 2.5 ? '#ef4444' : Math.abs(z) > 1.8 ? '#f59e0b' : '#22c55e';
const showZ = (id, z) => {
  $(id).style.color = zColor(z);
  $(id).innerText = (z >= 0 ? '+' : '') + z.toFixed(2);
};

async function fetchTrend() {
  try {
    const d = await get('/api/ai');
    if (!d.ph) return;

    showZ('z-ph',   parseFloat(d.ph.z));
    showZ('z-temp', parseFloat(d.temp.z));
    showZ('z-curr', parseFloat(d.curr.z));
    $('pred-ph').innerText   = fmt(d.ph.pred, 2) + ' pH';
    $('pred-temp').innerText = fmt(d.temp.pred, 1) + ' °C';
    $('pred-curr').innerText = fmt(d.curr.pred, 3) + ' A';

    if (d.advice) {
      const colors = ['#22c55e', '#60a5fa', '#f59e0b', '#ef4444'];
      const c = colors[Math.min(d.advice.level, 3)];
      const el = $('ai-advice');
      el.style.borderColor = c + '66';
      el.style.background  = c + '18';
      el.style.color = '#f2f2f8';
      el.innerHTML = '<b style="color:' + c + '">[' + d.advice.ch + ']</b> ' + d.advice.msg;
    }
  } catch (e) {}
}

setInterval(refresh, 900);
setInterval(fetchTrend, 1000);
refresh();
fetchTrend();
</script>
</body>
</html>
)HTML";

// =====================================================
//  API
// =====================================================
void handleRoot() {
  server.send(200, "text/html", PAGE);
}

void handleData() {
  float phAvg = phAvgBuf.avg();
  float tAvg  = tempAvgBuf.avg();
  float iAvg  = currAvgBuf.avg();

  String j = "{";
  j += "\"phNow\":"      + String(phFromVoltage(lastPhVoltage), 2) + ",";
  j += "\"phAvg\":"      + String(isnan(phAvg) ? 0.0 : phAvg, 2) + ",";
  j += "\"vph\":"        + String(lastPhVoltage, 3) + ",";
  j += "\"adc\":"        + String(lastPhRaw) + ",";
  j += "\"tempC\":"      + String(isnan(tempC) ? 0.0 : tempC, 2) + ",";
  j += "\"tempAvg\":"    + String(isnan(tAvg) ? 0.0 : tAvg, 2) + ",";
  j += "\"volt\":"       + String(isnan(loadV) ? 0.0 : loadV, 3) + ",";
  j += "\"currentA\":"   + String(isnan(currentA) ? 0.0 : currentA, 4) + ",";
  j += "\"currentAvg\":" + String(isnan(iAvg) ? 0.0 : iAvg, 4) + ",";
  j += "\"powerW\":"     + String(isnan(powerW) ? 0.0 : powerW, 3) + ",";
  j += "\"energyWh\":"   + String(energyWh, 4) + ",";

  j += "\"relay\":"      + String(relayOn ? 1 : 0) + ",";
  j += "\"mode\":"       + String((int)runMode) + ",";
  j += "\"safety\":"     + String((int)safety) + ",";
  j += "\"tripReason\":\"" + lastTripReason + "\",";

  j += "\"phLo\":"       + String(PH_LOW_TH, 2) + ",";
  j += "\"phHi\":"       + String(PH_HIGH_TH, 2) + ",";
  j += "\"iMax\":"       + String(OVERCURRENT_A_TH, 3) + ",";
  j += "\"tMax\":"       + String(OVERTEMP_C_TH, 2) + ",";
  j += "\"tripDelayMs\":" + String(TRIP_DELAY_MS) + ",";

  j += "\"slope\":"      + String(phSlope, 5) + ",";
  j += "\"offset\":"     + String(phOffset, 5);
  j += "}";

  server.send(200, "application/json", j);
}

void handleRelay() {
  if (!server.hasArg("state")) {
    server.send(400, "text/plain", "missing state");
    return;
  }
  int s = server.arg("state").toInt();

  // AUTO + TRIP: ห้ามสั่ง START ตรงๆ
  if (runMode == MODE_AUTO && safety == TRIP && s == 1) {
    beep(80, 2, 80);
    server.send(409, "text/plain", "TRIP active. Fix condition then STOP->START.");
    return;
  }

  if (s == 1) {
    setRelay(true);
    beep(60, 1, 0);
  } else {
    setRelay(false);
    beep(60, 2, 60);
  }
  server.send(200, "text/plain", "OK");
}

void handleMode() {
  if (!server.hasArg("mode")) {
    server.send(400, "text/plain", "missing mode");
    return;
  }
  runMode = (server.arg("mode").toInt() == 1) ? MODE_MANUAL : MODE_AUTO;
  saveLimits();
  beep(50, 1, 0);
  server.send(200, "text/plain", "OK");
}

void handleLimits() {
  if (server.hasArg("phlo")) PH_LOW_TH        = server.arg("phlo").toFloat();
  if (server.hasArg("phhi")) PH_HIGH_TH       = server.arg("phhi").toFloat();
  if (server.hasArg("imax")) OVERCURRENT_A_TH = server.arg("imax").toFloat();
  if (server.hasArg("tmax")) OVERTEMP_C_TH    = server.arg("tmax").toFloat();
  if (server.hasArg("tdly")) TRIP_DELAY_MS    = (uint32_t)server.arg("tdly").toInt();

  saveLimits();
  beep(50, 1, 0);
  server.send(200, "text/plain", "OK");
}

void handleCal7() {
  vAtPH7 = lastPhVoltage;
  haveV7 = true;
  beep(70, 1, 0);
  server.send(200, "text/plain", "Captured pH7");
}

void handleCal4() {
  vAtPH4 = lastPhVoltage;
  haveV4 = true;

  if (haveV7 && haveV4) {
    float dv = vAtPH7 - vAtPH4;
    if (fabs(dv) > 0.0001f) {
      phSlope  = (7.0f - 4.0f) / dv;
      phOffset = 7.0f - phSlope * vAtPH7;
      saveCalibration();
      beep(80, 2, 80);
    }
    haveV7 = false;
    haveV4 = false;
  } else {
    beep(70, 1, 0);
  }
  server.send(200, "text/plain", "Captured pH4 (and calibrated if pH7 exists)");
}

void handleResetEnergy() {
  energyWh = 0;
  lastEnergyMs = millis();
  beep(60, 1, 0);
  server.send(200, "text/plain", "OK");
}

void handleAI() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", lastAIJson);
}

void handleEC() {
  if (server.hasArg("duty"))   EC_DUTY = constrain(server.arg("duty").toFloat(), 0.05f, 1.0f);
  if (server.hasArg("period")) EC_PERIOD_MS = (uint32_t)constrain(server.arg("period").toInt(), 1000, 60000);
  beep(50, 1, 0);
  server.send(200, "text/plain", "OK");
}

// =====================================================
//  setup / loop
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  phAvgBuf.init();
  tempAvgBuf.init();
  currAvgBuf.init();
  phWin.clear();
  tempWin.clear();
  currWin.clear();

  pinMode(PIN_RELAY, OUTPUT);
  setRelay(false);

  pinMode(PIN_BUZZER, OUTPUT);
  buzzerOff();

  analogReadResolution(12);

  loadCalibration();
  loadLimits();

  Wire.begin(PIN_SDA, PIN_SCL);
  if (!ina219.begin()) {
    Serial.println("INA219 not found! Check wiring.");
    beep(120, 3, 60);
  }

  ds18b20.begin();

  startWiFi();

  server.on("/", handleRoot);
  server.on("/api/data", handleData);
  server.on("/api/ai", HTTP_GET, handleAI);
  server.on("/api/relay", HTTP_POST, handleRelay);
  server.on("/api/mode", HTTP_POST, handleMode);
  server.on("/api/limits", HTTP_POST, handleLimits);
  server.on("/api/cal7", HTTP_POST, handleCal7);
  server.on("/api/cal4", HTTP_POST, handleCal4);
  server.on("/api/reset_energy", HTTP_POST, handleResetEnergy);
  server.on("/api/ec", HTTP_POST, handleEC);
  server.begin();
  Serial.println("[Server] Started");

  beep(60, 1, 0);

  lastSampleMs = millis();
  lastEnergyMs = millis();
}

void loop() {
  server.handleClient();

  uint32_t now = millis();

  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    sampleSensors();
  }

  if (now - lastAIMs >= AI_INTERVAL_MS) {
    lastAIMs = now;
    runAnalysis();
  }

  updateECPulse();

  delay(LOOP_DELAY_MS);
}
