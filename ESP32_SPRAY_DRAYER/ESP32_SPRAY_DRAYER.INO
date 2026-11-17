/*
================================================================================
 FIRMWARE ESP32 - PROTOTYPE SMART SALT PROCESSOR (SPRAY DRYER GARAM)  v1.0
 Universitas Trunodjoyo Madura
 Board  : ESP32 DOIT DevKit V1 (WROOM-32)  |  Arduino core ESP32 2.0.17
 Peran  : KONTROL (heater, pompa, sensor, keselamatan) + Web (SoftAP + WebSocket)
          + master UART ke Arduino Uno (HMI TFT 3,5")
================================================================================

LIBRARY (Library Manager):
  1. Adafruit MAX31865            (Adafruit)            -> sensor RTD PT100
  2. WebSockets                   (Markus Sattler / Links2004, v2.4.x) -> WebSocketsServer
  (WiFi, WebServer, Preferences sudah bawaan core ESP32)

PIN (sesuai netlist v1.6 + revisi UART Uno):
  GPIO18 SCK, GPIO19 MISO, GPIO23 MOSI, GPIO26 CS  -> MAX31865 (VSPI)
  GPIO27  -> driver SSR heater (time-proportioning, window 2 s)
  GPIO13  -> TC4427 -> MOSFET pompa (PWM 20 kHz, 10-bit, LEDC ch0)
  GPIO35  -> pulsa flow meter (input-only, pull-up eksternal R86)
  GPIO16  <- RX2 dari Uno D11 (lewat pembagi 10k/20k)
  GPIO17  -> TX2 ke Uno A5 (lewat R32 330 Ohm)
  GPIO32, GPIO2 -> tidak dipakai (fan langsung AC, tanpa servo) -> dipaksa LOW

WIFI: Access Point "SprayDryer-UTM", password "12345678", buka http://192.168.4.1
      WebSocket ws://192.168.4.1:81  (bisa juga dipakai Python untuk logging nanti)

PROTOKOL UART (19200 8N1, ESP32 = MASTER, kirim tiap 200 ms, 2 jenis frame bergantian):
  ESP -> Uno : $T,st,md,temp10,flow100,totml,heat,pump,flt,ack*CS\n   (data proses)
               $S,hm,pm,sp,pumpA,kf,ack*CS\n                         (setelan)
      st   : 0=IDLE 1=JALAN 2=FAULT ; md : 0=MANUAL 1=AUTO
      temp10 : suhu x10 (1234 = 123,4 C), -9999 = sensor error
      flow100: debit L/menit x100 ; totml : volume total mL
      heat, pump : duty aktual (%) ; flt : kode fault (bit)
      hm, pm : duty manual heater/pompa ; sp : setpoint auto ; pumpA : pompa auto ; kf : pulsa/L
      ack    : nomor perintah terakhir yang diterima ESP32
  Uno -> ESP : $C,seq,CMD,val*CS\n   (seq 0-99, dibalas SETIAP kali menerima $T)
      CMD: NOP, MODE(0/1), START, STOP, RESET, HEAT(0-100), PUMP(0-100),
           SP(40-180), PUMPA(0-100), KF(1-20000), TOTRST
  CS = XOR semua karakter di antara '$' dan '*', 2 digit hex.
  Uno mengirim ulang perintah yang sama sampai 'ack' == seq.
  Jika Uno tidak membalas > 3 s saat JALAN -> STOP aman (heater & pompa OFF).

PERINTAH WEB (teks via WebSocket): MODE=0/1, START, STOP, RESET, HEAT=x, PUMP=x,
  SP=x, PUMPA=x, KF=x, TOTRST. Status dikirim JSON tiap 500 ms.
  Kontrol dari LCD & web: perintah terakhir yang menang; keadaan resmi ada di ESP32.

KESELAMATAN (fan TIDAK dipantau karena langsung ke AC):
  - Boot -> IDLE, semua aktuator OFF. Jalan hanya setelah START.
  - START mode AUTO butuh RTD terbaca. Mode MANUAL boleh START & mengontrol heater dan
    pompa walau RTD error -> TANPA proteksi suhu; heater dimatikan otomatis setelah
    MANUAL_NO_RTD_MAX_MIN menit tanpa RTD (set ulang duty untuk menyalakan lagi).
  - Suhu >= 200 C atau RTD fault 3x berturut -> FAULT (heater & pompa OFF), butuh RESET.
  - Heater diperintah OFF >= 60 s tapi suhu naik > 5 C dan > 60 C -> FAULT
    "SSR MACET - TEKAN E-STOP".
  - Operator: JANGAN matikan panel sebelum suhu < 60 C (fan = post-purge).
================================================================================
*/

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Preferences.h>
#include <Adafruit_MAX31865.h>

// ---------------------------- PIN -------------------------------------------
#define PIN_RTD_CS     26
#define PIN_HEATER     27
#define PIN_PUMP       13
#define PIN_FLOW       35
#define PIN_UNO_RX     16
#define PIN_UNO_TX     17
#define PIN_UNUSED_1   32
#define PIN_UNUSED_2   2

// ---------------------------- KONSTANTA -------------------------------------
#define RREF            430.0f
#define RNOMINAL        100.0f
#define T_MAX_SAFE      200.0f     // batas keselamatan
#define SP_MIN          40
#define SP_MAX          180
#define HEAT_WINDOW_MS  2000UL     // window time-proportioning SSR
#define FUZZY_TS_MS     2000UL     // periode fuzzy
#define RTD_PERIOD_MS   500UL
#define FLOW_PERIOD_MS  1000UL
#define UNO_PERIOD_MS   200UL
#define UNO_TIMEOUT_MS  3000UL
#define WEB_PERIOD_MS   500UL
#define PUMP_RAMP_PCT_S 100.0f     // soft-start: naik maks 100 %/detik
#define PUMP_LEDC_CH    0
#define PUMP_PWM_FREQ   20000
#define PUMP_PWM_BITS   10

// Kode fault (bit)
#define F_OVERTEMP   0x01
#define F_RTD        0x02
#define F_SSR_STUCK  0x04
#define F_HMI_LOST   0x08          // informasi (bukan latch)
#define F_START_REJ  0x10          // informasi 3 s: START ditolak (RTD belum terbaca di mode AUTO)
#define F_NORTD_TO   0x20          // informasi: heater manual dimatikan karena batas waktu tanpa RTD

// Mode MANUAL: heater TETAP bisa dikontrol walau RTD error (permintaan operator).
// Selama itu TIDAK ADA proteksi suhu (batas 200 C & deteksi SSR macet butuh RTD).
// Pengaman waktu: heater manual tanpa RTD dimatikan otomatis setelah N menit
// (operator bisa menyalakan lagi). 0 = tanpa batas waktu (TIDAK disarankan).
#define MANUAL_NO_RTD_MAX_MIN  15

enum State : uint8_t { ST_IDLE = 0, ST_RUN = 1, ST_FAULT = 2 };
enum Mode  : uint8_t { MD_MANUAL = 0, MD_AUTO = 1 };

// ---------------------------- OBJEK -----------------------------------------
Adafruit_MAX31865 rtd(PIN_RTD_CS);          // pakai SPI default = VSPI 18/19/23
WebServer         http(80);
WebSocketsServer  ws(81);
Preferences       prefs;
HardwareSerial&   Uno = Serial2;

// ---------------------------- STATUS ----------------------------------------
volatile uint32_t flowPulses = 0;
portMUX_TYPE flowMux = portMUX_INITIALIZER_UNLOCKED;

State  state = ST_IDLE;
Mode   mode  = MD_MANUAL;
uint8_t faults = 0;

float  tempC = NAN;          // suhu RTD
bool   rtdOk = false;
uint8_t rtdBadCount = 0;

float  flowLpm = 0;          // L/menit
double totalMl = 0;          // mL
uint16_t kFactor = 450;      // pulsa per liter (ubah setelah spesifikasi flow meter pasti)

uint8_t heatManual = 0;      // duty manual heater (%)
uint8_t pumpManual = 0;      // duty manual pompa (%)
uint8_t setpoint   = 120;    // setpoint auto (C)
uint8_t pumpAuto   = 30;     // duty pompa saat auto (%)

float  heatDuty = 0;         // duty heater aktual (%)
float  pumpTarget = 0;       // duty pompa target (%)
float  pumpDuty = 0;         // duty pompa aktual setelah ramp (%)

// fuzzy
float  fuzzyU = 0;           // keluaran terakumulasi (%)
float  ePrev  = 0;
bool   fuzzyInit = false;

// deteksi SSR macet
uint32_t heatOffSince = 0;
uint32_t startRejAt = 0;
uint32_t noRtdHeatSince = 0;
float    tempAtHeatOff = NAN;

// UART Uno
char     unoBuf[64];
uint8_t  unoLen = 0;
uint32_t unoLastRx = 0;
int      lastSeqDone = -1;
bool     unoAlive = false;

// waktu
uint32_t tRtd = 0, tFlow = 0, tFuzzy = 0, tUno = 0, tWeb = 0, tRamp = 0;
uint32_t heatWindowStart = 0;

// ---------------------------- ISR FLOW --------------------------------------
void IRAM_ATTR flowISR() {
  portENTER_CRITICAL_ISR(&flowMux);
  flowPulses++;
  portEXIT_CRITICAL_ISR(&flowMux);
}

// ---------------------------- UTIL ------------------------------------------
uint8_t xorCs(const char* s, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) c ^= (uint8_t)s[i];
  return c;
}

void saveSettings() {
  prefs.putUChar("sp", setpoint);
  prefs.putUChar("pumpA", pumpAuto);
  prefs.putUShort("kf", kFactor);
}

void allOutputsOff() {
  heatDuty = 0;
  pumpTarget = 0;
  pumpDuty = 0;
  digitalWrite(PIN_HEATER, LOW);
  ledcWrite(PUMP_LEDC_CH, 0);
}

void goIdle() {
  state = ST_IDLE;
  allOutputsOff();
  fuzzyU = 0;
  fuzzyInit = false;
}

void raiseFault(uint8_t f) {
  faults |= f;
  state = ST_FAULT;
  allOutputsOff();
  fuzzyU = 0;
  fuzzyInit = false;
  Serial.printf("[FAULT] kode 0x%02X\n", faults);
}

// ---------------------------- PERINTAH (dari Uno & web) ---------------------
// return true jika perintah dikenali
bool handleCommand(const String& cmd, long val) {
  if (cmd == "NOP") return true;
  if (cmd == "STOP") { if (state == ST_RUN) goIdle(); return true; }
  if (cmd == "START") {
    if (state != ST_IDLE) return true;
    // MANUAL boleh jalan tanpa RTD (mis. uji pompa); heater tetap TERKUNCI selama RTD
    // tidak terbaca (lihat updateHeater). AUTO wajib RTD terbaca.
    if (mode == MD_AUTO && !rtdOk) {
      faults |= F_START_REJ; startRejAt = millis();
      Serial.println("[START] ditolak: RTD belum terbaca (mode AUTO)");
      return true;
    }
    state = ST_RUN;
    fuzzyU = 0; fuzzyInit = false;
    heatWindowStart = millis();
    Serial.println("[START] OK");
    return true;
  }
  if (cmd == "RESET") {
    if (state == ST_FAULT) {
      // fault suhu lebih hanya bisa di-reset jika suhu sudah turun & terbaca;
      // fault lain (RTD, SSR macet) boleh di-reset (heater tetap terkunci tanpa RTD)
      bool safeNow = !(faults & F_OVERTEMP) || (rtdOk && tempC < (T_MAX_SAFE - 10));
      if (safeNow) { faults &= (F_HMI_LOST | F_START_REJ); goIdle(); }
    }
    return true;
  }
  if (cmd == "MODE")  { if (state == ST_IDLE) mode = (val ? MD_AUTO : MD_MANUAL); return true; }
  if (cmd == "HEAT")  { heatManual = constrain(val, 0, 100); faults &= ~F_NORTD_TO; return true; }
  if (cmd == "PUMP")  { pumpManual = constrain(val, 0, 100); return true; }
  if (cmd == "SP")    { setpoint = constrain(val, SP_MIN, SP_MAX); saveSettings(); return true; }
  if (cmd == "PUMPA") { pumpAuto = constrain(val, 0, 100); saveSettings(); return true; }
  if (cmd == "KF")    { kFactor = constrain(val, 1, 20000); saveSettings(); return true; }
  if (cmd == "TOTRST"){ totalMl = 0; return true; }
  return false;
}

// ---------------------------- SENSOR RTD ------------------------------------
void readRtd() {
  // CATATAN: temperature() memblok ~75 ms (bias + konversi one-shot). Dipanggil tiap 500 ms.
  float t = rtd.temperature(RNOMINAL, RREF);
  uint8_t f = rtd.readFault();
  bool bad = (f != 0) || isnan(t) || t < -50 || t > 400;
  if (f) rtd.clearFault();

  if (bad) {
    if (rtdBadCount < 255) rtdBadCount++;
    if (rtdBadCount >= 3) {
      rtdOk = false;
      tempC = NAN;
      // AUTO: FAULT (fuzzy butuh suhu). MANUAL: TIDAK fault, hanya tanda peringatan;
      // heater & pompa tetap bisa dikontrol manual (tanpa proteksi suhu!).
      if (state == ST_RUN && mode == MD_AUTO) raiseFault(F_RTD);
      else faults |= F_RTD;
    }
  } else {
    rtdBadCount = 0;
    rtdOk = true;
    tempC = t;
    if (state != ST_FAULT) faults &= ~F_RTD;   // peringatan hilang saat RTD terbaca lagi
  }

  // batas suhu
  if (rtdOk && tempC >= T_MAX_SAFE && state != ST_FAULT) raiseFault(F_OVERTEMP);

  // deteksi SSR macet: heater OFF >= 60 s tetapi suhu naik > 5 C di atas 60 C
  if (rtdOk) {
    if (heatDuty <= 0.5f) {
      if (heatOffSince == 0) { heatOffSince = millis(); tempAtHeatOff = tempC; }
      else if (millis() - heatOffSince >= 60000UL) {
        if (tempC > 60 && tempC - tempAtHeatOff > 5.0f) raiseFault(F_SSR_STUCK);
        heatOffSince = millis();   // geser jendela pengamatan
        tempAtHeatOff = tempC;
      }
    } else {
      heatOffSince = 0;
    }
  }
}

// ---------------------------- FLOW ------------------------------------------
void readFlow(uint32_t dtMs) {
  uint32_t p;
  portENTER_CRITICAL(&flowMux);
  p = flowPulses; flowPulses = 0;
  portEXIT_CRITICAL(&flowMux);
  float liters = (float)p / (float)kFactor;
  totalMl += liters * 1000.0;
  flowLpm = liters * (60000.0f / (float)dtMs);
}

// ---------------------------- FUZZY MAMDANI INKREMENTAL ---------------------
// Input: e = SP - T (C), de = e(k) - e(k-1) per Ts. Output: delta-u (%) diakumulasi.
// 5 MF segitiga (NB NS Z PS PB) pada semesta ternormalisasi [-1, 1].
const float KE  = 1.0f / 20.0f;   // e +-20 C  -> +-1
const float KDE = 1.0f / 2.0f;    // de +-2 C per 2 s -> +-1
const float KU  = 8.0f;           // keluaran +-1 -> +-8 % per langkah
const float C5[5] = { -1.0f, -0.5f, 0.0f, 0.5f, 1.0f };

// aturan [e][de] -> indeks MF keluaran (0=NB .. 4=PB)
const uint8_t RULE[5][5] = {
  /* e=NB */ {0, 0, 0, 1, 2},
  /* e=NS */ {0, 0, 1, 2, 3},
  /* e=Z  */ {0, 1, 2, 3, 4},
  /* e=PS */ {1, 2, 3, 4, 4},
  /* e=PB */ {2, 3, 4, 4, 4}
};

float mf(float x, uint8_t i) {
  // bahu kiri/kanan untuk MF paling luar
  if (i == 0 && x <= C5[0]) return 1.0f;
  if (i == 4 && x >= C5[4]) return 1.0f;
  float d = fabsf(x - C5[i]) / 0.5f;
  return d >= 1.0f ? 0.0f : 1.0f - d;
}

float fuzzyStep(float e, float de) {
  float en  = constrain(e * KE,  -1.0f, 1.0f);
  float den = constrain(de * KDE, -1.0f, 1.0f);
  float w[5] = {0, 0, 0, 0, 0};             // kekuatan tiap MF keluaran (max)
  for (uint8_t i = 0; i < 5; i++) {
    float me = mf(en, i);
    if (me <= 0) continue;
    for (uint8_t j = 0; j < 5; j++) {
      float md = mf(den, j);
      if (md <= 0) continue;
      float a = fminf(me, md);              // AND = min
      uint8_t o = RULE[i][j];
      if (a > w[o]) w[o] = a;               // agregasi = max
    }
  }
  // defuzzifikasi centroid (diskrit 41 titik)
  float num = 0, den2 = 0;
  for (int k = 0; k <= 40; k++) {
    float y = -1.0f + k * 0.05f;
    float mu = 0;
    for (uint8_t o = 0; o < 5; o++) {
      float clip = fminf(w[o], mf(y, o));   // implikasi = min
      if (clip > mu) mu = clip;
    }
    num += y * mu; den2 += mu;
  }
  float du = (den2 > 0) ? (num / den2) : 0;
  return du * KU;
}

// ---------------------------- AKTUATOR --------------------------------------
void updateHeater() {
  uint32_t now = millis();
  if (now - heatWindowStart >= HEAT_WINDOW_MS) heatWindowStart += HEAT_WINDOW_MS;
  bool sensorOk = rtdOk || (mode == MD_MANUAL);   // manual: boleh tanpa RTD
  bool on = (state == ST_RUN) && sensorOk &&
            ((now - heatWindowStart) < (uint32_t)(heatDuty * HEAT_WINDOW_MS / 100.0f));
  digitalWrite(PIN_HEATER, on ? HIGH : LOW);
}

void updatePump() {
  uint32_t now = millis();
  float dt = (now - tRamp) / 1000.0f;
  tRamp = now;
  float step = PUMP_RAMP_PCT_S * dt;
  if (pumpDuty < pumpTarget) pumpDuty = fminf(pumpDuty + step, pumpTarget);
  else                       pumpDuty = pumpTarget;     // turun langsung (aman)
  uint32_t maxv = (1UL << PUMP_PWM_BITS) - 1;
  ledcWrite(PUMP_LEDC_CH, (uint32_t)(pumpDuty * maxv / 100.0f));
}

void controlLogic() {
  if (state != ST_RUN) { heatDuty = 0; pumpTarget = 0; return; }
  if (mode == MD_MANUAL) {
    heatDuty   = heatManual;
    pumpTarget = pumpManual;
  } else {
    pumpTarget = pumpAuto;
    if (millis() - tFuzzy >= FUZZY_TS_MS) {
      tFuzzy = millis();
      if (rtdOk) {
        float e = (float)setpoint - tempC;
        if (!fuzzyInit) { ePrev = e; fuzzyInit = true; }
        float de = e - ePrev;
        ePrev = e;
        fuzzyU = constrain(fuzzyU + fuzzyStep(e, de), 0.0f, 100.0f);   // anti-windup
      }
    }
    heatDuty = fuzzyU;
  }
}

// ---------------------------- UART UNO --------------------------------------
// Dua jenis frame bergantian supaya tiap frame <= ~45 byte (buffer SoftwareSerial Uno 64 byte):
//   $T,st,md,temp10,flow100,totml,heat,pump,flt,ack*CS   (data proses)
//   $S,hm,pm,sp,pumpA,kf,ack*CS                           (setelan -> sinkron LCD <-> web)
void sendToUno() {
  static bool sendS = false;
  char body[64];
  if (!sendS) {
    int t10 = rtdOk ? (int)lroundf(tempC * 10) : -9999;
    snprintf(body, sizeof(body), "T,%u,%u,%d,%u,%lu,%u,%u,%u,%d",
             state, mode, t10, (unsigned)lroundf(flowLpm * 100),
             (unsigned long)totalMl, (unsigned)lroundf(heatDuty), (unsigned)lroundf(pumpDuty),
             faults, lastSeqDone);
  } else {
    snprintf(body, sizeof(body), "S,%u,%u,%u,%u,%u,%d",
             heatManual, pumpManual, setpoint, pumpAuto, kFactor, lastSeqDone);
  }
  sendS = !sendS;
  char out[96];
  snprintf(out, sizeof(out), "$%s*%02X\n", body, xorCs(body, strlen(body)));
  Uno.print(out);
}

void parseUnoFrame(char* f) {
  // f tanpa '$', berisi "C,seq,CMD,val*CS"
  char* star = strchr(f, '*');
  if (!star) return;
  *star = 0;
  uint8_t cs = (uint8_t)strtoul(star + 1, nullptr, 16);
  if (xorCs(f, strlen(f)) != cs) return;
  if (f[0] != 'C' || f[1] != ',') return;

  char* p = f + 2;
  int seq = atoi(p);
  p = strchr(p, ','); if (!p) return; p++;
  char* c2 = strchr(p, ','); if (!c2) return;
  *c2 = 0;
  String cmd = String(p);
  long val = atol(c2 + 1);

  unoLastRx = millis();
  unoAlive = true;
  faults &= ~F_HMI_LOST;
  if (cmd != "NOP" && seq != lastSeqDone) {
    if (handleCommand(cmd, val)) lastSeqDone = seq;
  }
}

void pollUno() {
  while (Uno.available()) {
    char c = Uno.read();
    if (c == '$') { unoLen = 0; continue; }
    if (c == '\n' || c == '\r') {
      if (unoLen > 0) { unoBuf[unoLen] = 0; parseUnoFrame(unoBuf); }
      unoLen = 0;
      continue;
    }
    if (unoLen < sizeof(unoBuf) - 1) unoBuf[unoLen++] = c;
    else unoLen = 0;   // frame terlalu panjang -> buang
  }
  // watchdog HMI
  if (unoAlive && millis() - unoLastRx > UNO_TIMEOUT_MS) {
    unoAlive = false;
    faults |= F_HMI_LOST;
    lastSeqDone = -1;          // Uno mungkin reboot -> nomor perintah mulai ulang
    if (state == ST_RUN) { Serial.println("[HMI] Uno hilang -> STOP aman"); goIdle(); }
  }
}

// ---------------------------- WEB -------------------------------------------
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="id"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Smart Salt Processor</title>
<style>
:root{--bg:#0e131a;--card:#1c2530;--line:#2c3846;--txt:#e9eef3;--dim:#8a99a8;
--org:#ff9a2e;--blu:#3b9bf0;--go:#23a55a;--red:#d33c3c;--gry:#5f6b7a;--nav:#2a3644}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--txt);
font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif}
.bar{position:sticky;top:0;z-index:5;display:flex;align-items:center;gap:12px;
padding:10px 14px;background:var(--gry);transition:background .3s}
.bar b{font-size:18px;letter-spacing:.5px}.bar .sub{font-size:13px;opacity:.85}
.dot{width:10px;height:10px;border-radius:50%;background:#bbb;display:inline-block;margin-right:6px}
.sp{flex:1}.stop{background:var(--red);color:#fff;border:2px solid #fff;border-radius:10px;
font-weight:800;font-size:18px;padding:8px 22px;cursor:pointer}
.wrap{max-width:1100px;margin:auto;padding:12px;display:grid;gap:12px;
grid-template-columns:repeat(auto-fit,minmax(300px,1fr))}
.card{background:var(--card);border-radius:14px;padding:14px 16px}
.lbl{font-size:12px;color:var(--dim);text-transform:uppercase;letter-spacing:1px}
.big{font-size:48px;font-weight:800;line-height:1.1;margin:6px 0;font-variant-numeric:tabular-nums}
.unit{font-size:20px;font-weight:600;margin-left:4px}.small{font-size:13px;color:var(--dim)}
.t{color:var(--org)}.f{color:var(--blu)}
.meter{height:14px;background:var(--line);border-radius:7px;overflow:hidden;margin:6px 0 12px}
.meter>div{height:100%;width:0;transition:width .4s}
.row{display:flex;justify-content:space-between;align-items:baseline}
.wide{grid-column:1/-1}canvas{width:100%;height:220px;display:block}
.tabs{display:flex;gap:6px;margin-bottom:10px}.tab{flex:1;padding:10px;border:0;border-radius:10px;
background:var(--nav);color:var(--txt);font-weight:700;cursor:pointer}.tab.on{background:#2f6fc0}
.ctl{display:flex;align-items:center;gap:8px;margin:8px 0}.ctl .name{flex:1;font-size:14px}
.st{width:42px;height:42px;border:0;border-radius:10px;background:var(--nav);color:#fff;font-size:22px;cursor:pointer}
.val{min-width:76px;text-align:center;font-size:22px;font-weight:700;font-variant-numeric:tabular-nums}
.btn{border:0;border-radius:10px;padding:12px 16px;font-weight:800;font-size:16px;cursor:pointer;color:#fff}
.go{background:var(--go)}.rs{background:var(--nav)}.dl{background:#2f6fc0}
.acts{display:flex;gap:8px;flex-wrap:wrap;margin-top:10px}
#flt{display:none;background:#3a1416;border:1px solid var(--red);color:#ffb4b4;
border-radius:12px;padding:10px 14px;font-weight:700}
.leg{font-size:12px;color:var(--dim)}.leg i{display:inline-block;width:14px;height:3px;margin:0 4px 3px 10px}
</style></head><body>
<div class="bar" id="bar"><div><b id="stx">MENGHUBUNGKAN...</b><div class="sub" id="mdx">-</div></div>
<div class="sp"></div><div class="sub"><span class="dot" id="dot"></span><span id="con">-</span></div>
<div class="sub" id="rt">00:00:00</div><button class="stop" onclick="s('STOP')">STOP</button></div>
<div class="wrap">
<div class="wide" id="flt"></div>
<div class="card"><div class="lbl">Suhu RTD</div><div class="big t"><span id="t">--</span><span class="unit">°C</span></div>
<div class="small" id="spx">-</div></div>
<div class="card"><div class="lbl">Debit larutan</div><div class="big f"><span id="f">--</span><span class="unit">L/mnt</span></div>
<div class="small">Total <span id="tot">0</span> mL</div></div>
<div class="card"><div class="row"><span class="lbl">Heater</span><b id="h">0%</b></div>
<div class="meter"><div id="hb" style="background:var(--org)"></div></div>
<div class="row"><span class="lbl">Pompa</span><b id="p">0%</b></div>
<div class="meter"><div id="pb" style="background:var(--blu)"></div></div>
<div class="small">Fan selalu ON (langsung AC)</div></div>
<div class="card wide"><div class="row"><span class="lbl">Tren 5 menit</span>
<span class="leg"><i style="background:var(--org)"></i>Suhu °C<i style="background:var(--blu)"></i>Debit L/mnt<i style="background:#8a99a8"></i>Setpoint</span></div>
<canvas id="cv"></canvas></div>
<div class="card wide">
<div class="tabs"><button class="tab" id="tm" onclick="s('MODE=0')">MANUAL</button>
<button class="tab" id="ta" onclick="s('MODE=1')">AUTO</button></div>
<div id="pm">
<div class="ctl"><span class="name">Heater manual (duty)</span><button class="st" onclick="st('hm',-5,0,100,'HEAT')">−</button>
<span class="val t" id="hm">0%</span><button class="st" onclick="st('hm',5,0,100,'HEAT')">+</button></div>
<div class="ctl"><span class="name">Pompa manual (duty)</span><button class="st" onclick="st('pm2',-5,0,100,'PUMP')">−</button>
<span class="val f" id="pm2">0%</span><button class="st" onclick="st('pm2',5,0,100,'PUMP')">+</button></div></div>
<div id="pa" style="display:none">
<div class="ctl"><span class="name">Setpoint suhu (fuzzy)</span><button class="st" onclick="st('sp',-5,40,180,'SP')">−</button>
<span class="val t" id="sp">120°</span><button class="st" onclick="st('sp',5,40,180,'SP')">+</button></div>
<div class="ctl"><span class="name">Pompa auto (duty)</span><button class="st" onclick="st('pa2',-5,0,100,'PUMPA')">−</button>
<span class="val f" id="pa2">0%</span><button class="st" onclick="st('pa2',5,0,100,'PUMPA')">+</button></div></div>
<div class="acts"><button class="btn go" onclick="s('START')">START</button>
<button class="btn rs" onclick="s('RESET')">RESET FAULT</button></div>
<div class="small" style="margin-top:8px">Mode hanya bisa diganti saat IDLE. Perintah terakhir (LCD atau web) yang berlaku.</div></div>
<div class="card wide"><div class="lbl">Setelan & data</div>
<div class="ctl"><span class="name">Faktor flow (pulsa/liter)</span><button class="st" onclick="st('kf',-10,1,20000,'KF')">−</button>
<span class="val f" id="kf">450</span><button class="st" onclick="st('kf',10,1,20000,'KF')">+</button></div>
<div class="acts"><button class="btn rs" onclick="s('TOTRST')">RESET TOTAL VOLUME</button>
<button class="btn dl" onclick="csv()">UNDUH CSV SESI INI</button></div>
<div class="small" style="margin-top:8px">CSV berisi data sejak halaman ini dibuka (1 baris/detik).</div></div>
</div>
<script>
const $=i=>document.getElementById(i),ST=['IDLE','JALAN','FAULT'],COL=['#5f6b7a','#23a55a','#d33c3c'];
let w,D=null,H=[],L=[],lastLog=0,val={hm:0,pm2:0,sp:120,pa2:30,kf:450},touched={};
function con(){w=new WebSocket('ws://'+location.hostname+':81/');
w.onopen=()=>{$('con').textContent='terhubung';$('dot').style.background='#5be37f'};
w.onclose=()=>{$('con').textContent='terputus';$('dot').style.background='#ff6b6b';
$('bar').style.background='#d33c3c';$('stx').textContent='KOMUNIKASI PUTUS';setTimeout(con,2000)};
w.onmessage=e=>{D=JSON.parse(e.data);upd()}}
function s(c){if(w&&w.readyState==1)w.send(c)}
function st(id,d,lo,hi,c){val[id]=Math.min(hi,Math.max(lo,val[id]+d));touched[id]=Date.now();show();s(c+'='+val[id])}
function show(){$('hm').textContent=val.hm+'%';$('pm2').textContent=val.pm2+'%';$('sp').textContent=val.sp+'°';
$('pa2').textContent=val.pa2+'%';$('kf').textContent=val.kf}
function sync(id,v){if(!touched[id]||Date.now()-touched[id]>3000)val[id]=v}
function hms(x){x=Math.floor(x);return[Math.floor(x/3600),Math.floor(x/60)%60,x%60].map(n=>String(n).padStart(2,'0')).join(':')}
let runT0=null;
function upd(){const d=D;$('bar').style.background=COL[d.st];$('stx').textContent=ST[d.st];
$('mdx').textContent='Mode '+(d.md?'AUTO (fuzzy)':'MANUAL');
if(d.st==1){if(runT0===null)runT0=Date.now()}else runT0=null;
$('rt').textContent=hms(runT0?(Date.now()-runT0)/1000:0);
$('t').textContent=d.t===null?'ERR':d.t.toFixed(1);$('f').textContent=d.f.toFixed(2);$('tot').textContent=d.tot;
$('spx').textContent=d.md?'Setpoint '+d.sp+' °C':'Mode manual';
$('h').textContent=d.h+'%';$('p').textContent=d.p+'%';$('hb').style.width=d.h+'%';$('pb').style.width=d.p+'%';
$('tm').classList.toggle('on',d.md==0);$('ta').classList.toggle('on',d.md==1);
$('pm').style.display=d.md?'none':'';$('pa').style.display=d.md?'':'none';
sync('hm',d.hm);sync('pm2',d.pm);sync('sp',d.sp);sync('pa2',d.pa);sync('kf',d.kf);show();
let m=[];if(d.flt&1)m.push('Suhu ≥ 200 °C');if(d.flt&2)m.push(d.st==1&&d.md==0?'Sensor RTD error — heater MANUAL TANPA PROTEKSI SUHU, awasi terus!':'Sensor RTD error');
if(d.flt&32)m.push('Heater manual dimatikan otomatis: batas waktu tanpa RTD');
if(d.flt&4)m.push('SSR MACET — TEKAN E-STOP SEKARANG');if(d.flt&8)m.push('LCD/Uno tidak merespons');
if(d.flt&16)m.push('START ditolak: sensor RTD belum terbaca (mode AUTO butuh RTD)');
$('flt').style.display=m.length?'block':'none';$('flt').textContent='⚠ '+m.join(' • ');
const now=Date.now();if(now-lastLog>=1000){lastLog=now;
L.push([new Date().toISOString(),ST[d.st],d.md?'AUTO':'MANUAL',d.t,d.f,d.tot,d.h,d.p,d.sp]);
H.push({t:d.t,f:d.f,sp:d.md?d.sp:null});if(H.length>300)H.shift();draw()}}
function draw(){const c=$('cv'),r=devicePixelRatio||1,W=c.clientWidth,Hh=c.clientHeight;
c.width=W*r;c.height=Hh*r;const g=c.getContext('2d');g.scale(r,r);g.clearRect(0,0,W,Hh);
const pl=40,pr=44,pt=10,pb=18,cw=W-pl-pr,ch=Hh-pt-pb;
let ts=H.map(x=>x.t).filter(v=>v!==null).concat(H.map(x=>x.sp).filter(v=>v!==null));
let lo=Math.min(...ts,25),hi=Math.max(...ts,30);if(!isFinite(lo)){lo=20;hi=40}lo=Math.floor(lo-3);hi=Math.ceil(hi+3);
let fh=Math.max(1,...H.map(x=>x.f))*1.2;
g.strokeStyle='#2c3846';g.fillStyle='#8a99a8';g.font='11px system-ui';g.lineWidth=1;
for(let i=0;i<=4;i++){const y=pt+ch*i/4;g.beginPath();g.moveTo(pl,y);g.lineTo(pl+cw,y);g.stroke();
g.fillText((hi-(hi-lo)*i/4).toFixed(0),4,y+4);g.fillText((fh-fh*i/4).toFixed(2),pl+cw+4,y+4)}
g.fillText('-5 mnt',pl,Hh-4);g.fillText('sekarang',pl+cw-48,Hh-4);
const X=i=>pl+cw*(i+300-H.length)/299;
function line(key,col,sc,dash){g.strokeStyle=col;g.lineWidth=2;g.setLineDash(dash||[]);g.beginPath();let m=false;
H.forEach((p,i)=>{const v=p[key];if(v===null){m=false;return}const y=pt+ch*sc(v);m?g.lineTo(X(i),y):g.moveTo(X(i),y);m=true});
g.stroke();g.setLineDash([])}
line('sp','#8a99a8',v=>(hi-v)/(hi-lo),[6,5]);line('f','#3b9bf0',v=>(fh-v)/fh);line('t','#ff9a2e',v=>(hi-v)/(hi-lo))}
function csv(){let t='waktu,status,mode,suhu_C,debit_Lpm,total_mL,heater_pct,pompa_pct,setpoint_C\n'+L.map(r=>r.join(',')).join('\n');
const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([t],{type:'text/csv'}));
a.download='spraydryer_'+new Date().toISOString().replace(/[:.]/g,'-')+'.csv';a.click()}
addEventListener('resize',draw);con();
</script></body></html>
)rawliteral";

void wsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  if (type != WStype_TEXT) return;
  String msg; msg.reserve(len);
  for (size_t i = 0; i < len; i++) msg += (char)payload[i];
  msg.trim();
  int eq = msg.indexOf('=');
  String cmd = (eq < 0) ? msg : msg.substring(0, eq);
  long val   = (eq < 0) ? 0 : msg.substring(eq + 1).toInt();
  cmd.toUpperCase();
  handleCommand(cmd, val);
}

void sendWeb() {
  char js[220];
  char tbuf[12];
  if (rtdOk) snprintf(tbuf, sizeof(tbuf), "%.1f", tempC); else strcpy(tbuf, "null");
  snprintf(js, sizeof(js),
    "{\"st\":%u,\"md\":%u,\"t\":%s,\"f\":%.2f,\"tot\":%lu,\"h\":%u,\"p\":%u,"
    "\"sp\":%u,\"pa\":%u,\"hm\":%u,\"pm\":%u,\"kf\":%u,\"flt\":%u,\"ms\":%lu}",
    state, mode, tbuf, flowLpm, (unsigned long)totalMl,
    (unsigned)lroundf(heatDuty), (unsigned)lroundf(pumpDuty),
    setpoint, pumpAuto, heatManual, pumpManual, kFactor, faults, (unsigned long)millis());
  ws.broadcastTXT(js);
}

// ---------------------------- SETUP & LOOP ----------------------------------
void setup() {
  // keluaran AMAN dulu
  pinMode(PIN_HEATER, OUTPUT);   digitalWrite(PIN_HEATER, LOW);
  pinMode(PIN_UNUSED_1, OUTPUT); digitalWrite(PIN_UNUSED_1, LOW);
  pinMode(PIN_UNUSED_2, OUTPUT); digitalWrite(PIN_UNUSED_2, LOW);
  ledcSetup(PUMP_LEDC_CH, PUMP_PWM_FREQ, PUMP_PWM_BITS);
  ledcAttachPin(PIN_PUMP, PUMP_LEDC_CH);
  ledcWrite(PUMP_LEDC_CH, 0);

  Serial.begin(115200);
  Uno.begin(19200, SERIAL_8N1, PIN_UNO_RX, PIN_UNO_TX);

  prefs.begin("dryer", false);
  setpoint = prefs.getUChar("sp", 120);
  pumpAuto = prefs.getUChar("pumpA", 30);
  kFactor  = prefs.getUShort("kf", 450);

  rtd.begin(MAX31865_3WIRE);
  rtd.enable50Hz(true);          // filter jala-jala 50 Hz

  pinMode(PIN_FLOW, INPUT);      // GPIO35 tanpa pull internal; pull-up R86 di PCB
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), flowISR, FALLING);

  WiFi.mode(WIFI_AP);
  WiFi.softAP("SprayDryer-UTM", "12345678");
  http.on("/", []() { http.send_P(200, "text/html", PAGE); });
  http.begin();
  ws.begin();
  ws.onEvent(wsEvent);

  uint32_t now = millis();
  tRtd = tFlow = tFuzzy = tUno = tWeb = tRamp = heatWindowStart = now;
  Serial.printf("Siap. AP SprayDryer-UTM, IP %s\n", WiFi.softAPIP().toString().c_str());
  goIdle();
}

void loop() {
  uint32_t now = millis();

  http.handleClient();
  ws.loop();
  pollUno();

  if ((faults & F_START_REJ) && now - startRejAt > 3000) faults &= ~F_START_REJ;
  // pengaman waktu heater manual tanpa RTD
  if (state == ST_RUN && mode == MD_MANUAL && !rtdOk && heatManual > 0) {
    if (noRtdHeatSince == 0) noRtdHeatSince = now;
#if MANUAL_NO_RTD_MAX_MIN > 0
    if (now - noRtdHeatSince >= (uint32_t)MANUAL_NO_RTD_MAX_MIN * 60000UL) {
      heatManual = 0;
      faults |= F_NORTD_TO;
      noRtdHeatSince = 0;
      Serial.println("[HEATER] dimatikan: batas waktu manual tanpa RTD");
    }
#endif
  } else {
    noRtdHeatSince = 0;
  }
  if (now - tRtd >= RTD_PERIOD_MS)   { tRtd = now; readRtd(); }
  if (now - tFlow >= FLOW_PERIOD_MS) { uint32_t dt = now - tFlow; tFlow = now; readFlow(dt); }

  controlLogic();
  updateHeater();
  updatePump();

  if (now - tUno >= UNO_PERIOD_MS)   { tUno = now; sendToUno(); }
  if (now - tWeb >= WEB_PERIOD_MS)   { tWeb = now; sendWeb(); }
}