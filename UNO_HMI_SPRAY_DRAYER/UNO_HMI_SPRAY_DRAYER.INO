/*
================================================================================
 FIRMWARE ARDUINO UNO - HMI TFT 3,5" TOUCH  (SMART SALT PROCESSOR)  v1.1
 Universitas Trunodjoyo Madura
 LCD   : shield 3,5" 480x320 driver ILI9486 + touch resistif (landscape)
 Peran : HANYA tampilan & input sentuh. Semua kontrol & keselamatan ada di ESP32.
 Tampilan: tema gelap, bar status berwarna, kartu, bar duty, grafik tren suhu.
================================================================================

LIBRARY (Library Manager):
  1. MCUFRIEND_kbv   (David Prentice)
  2. Adafruit GFX Library
  3. Adafruit TouchScreen
  (SoftwareSerial sudah bawaan)

SAMBUNGAN KE PCB KONTROL (J8, kabel 4 kawat, disolder di bawah PCB Uno):
  Uno 5V  <- J8.1 (+5V_LCD)       Uno GND <- J8.2 (GND)
  Uno D11 -> J8.3 (TX Uno -> pembagi 10k/20k -> GPIO16 ESP32)
  Uno A5  <- J8.4 (dari GPIO17 ESP32 lewat R32 330 Ohm)
  D0/D1 tetap bebas untuk upload & Serial Monitor.

LANGKAH PERTAMA (WAJIB, sekali saja) - SCAN CONTROLLER & KALIBRASI TOUCH:
  1. Pasang shield ke Uno, upload contoh MCUFRIEND_kbv > "diagnose_TFT_support".
     Buka Serial Monitor 9600 -> pastikan "ID = 0x9486" (driver shield ini: ILI9486).
     Kode ini memakai tft.readID() otomatis; jika terbaca 0xD3D3 (shield tanpa jalur
     baca) kode memaksa 0x9486.
  2. Upload contoh MCUFRIEND_kbv > "TouchScreen_Calibr_native".
     Ikuti petunjuk di layar (sentuh tanda +), lalu salin dari Serial Monitor:
       const int XP=..,XM=..,YP=..,YM=..;
       const int TS_LEFT=..,TS_RT=..,TS_TOP=..,TS_BOT=..;
     ke bagian "KALIBRASI TOUCH" di bawah. Nilai di bawah hanya CONTOH.
  3. Upload kode ini. Jika sentuhan terbalik/tertukar, ubah TOUCH_SWAP_XY,
     TOUCH_INV_X, TOUCH_INV_Y. Set TOUCH_DEBUG 1 untuk melihat koordinat di Serial.

PROTOKOL: lihat header kode ESP32 (Uno = SLAVE, hanya membalas setelah menerima $T).
================================================================================
*/

#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <TouchScreen.h>
#include <SoftwareSerial.h>

// ---------------------------- KALIBRASI TOUCH (GANTI!) -----------------------
const int XP = 8, XM = A2, YP = A3, YM = 9;                         // CONTOH
const int TS_LEFT = 907, TS_RT = 136, TS_TOP = 942, TS_BOT = 139;   // CONTOH
#define TOUCH_SWAP_XY  1
#define TOUCH_INV_X    0
#define TOUCH_INV_Y    0
#define TOUCH_DEBUG    0
// ID controller LCD: 0 = baca otomatis. Jika layar PUTIH/kosong, isi dengan ID dari
// diagnose_TFT_support / graphictest_kbv, contoh 0x9486, 0x9487, 0x9488, 0x6814.
#define FORCE_ID       0x0000
#define MINPRESSURE    200
#define MAXPRESSURE    1000

// ---------------------------- PIN UART ---------------------------------------
#define PIN_ESP_RX   A5
#define PIN_ESP_TX   11
SoftwareSerial esp(PIN_ESP_RX, PIN_ESP_TX);

MCUFRIEND_kbv tft;
TouchScreen ts(XP, YP, XM, YM, 300);

// ---------------------------- WARNA (RGB565) ---------------------------------
#define C_BG     0x0882    // latar gelap
#define C_CARD   0x2126    // kartu
#define C_TXT    0xEF7E    // teks terang
#define C_DIM    0x8C93    // teks redup
#define C_ORG    0xFCA5    // suhu / heater
#define C_BLU    0x2C7C    // debit / pompa
#define C_GO     0x1CA8    // hijau
#define C_RED    0xC945    // merah
#define C_GRY    0x5AF7    // status idle
#define C_NAV    0x29A8    // tombol
#define C_NAVA   0x2377    // tab aktif
#define C_TRK    0x3A0F    // latar bar
#define C_OK     0x570F    // indikator link

// ---------------------------- DATA DARI ESP32 --------------------------------
struct Tele {
  uint8_t st, md;
  int     t10;
  uint16_t flow100;
  uint32_t totml;
  uint8_t heat, pump, sp, pumpA, flt;
  int     ack;
  uint16_t kf;
} d, shown;

bool     haveData = false;
uint32_t lastRx = 0;
bool     linkOk = false;
int8_t   linkShown = -1;

uint8_t  cmdSeq = 0;
char     cmdName[7] = "NOP";
long     cmdVal = 0;
bool     cmdPending = false;

int edHeat = 0, edPump = 0, edSp = 120, edPumpA = 30, edKf = 450;
int shEd1 = -1, shEd2 = -1;           // nilai editor yang sedang tampil

enum Page : uint8_t { PG_HOME, PG_MANUAL, PG_AUTO, PG_SET };
Page page = PG_HOME;
bool needFull = true;

char rxBuf[72];
uint8_t rxLen = 0;

// waktu proses (dihitung di Uno sejak status menjadi JALAN)
uint32_t runStart = 0;
uint32_t shownSec = 0xFFFFFFFF;

// tren suhu: 1 sampel/detik, 105 sampel x 2 px = 210 px
#define TR_N 105
uint8_t trend[TR_N];
uint8_t trCount = 0;
uint32_t trLast = 0;
bool trDirty = true;

// ---------------------------- GEOMETRI ---------------------------------------
struct Btn { int x, y, w, h; };
const Btn B_STOP  = {382,   3,  94,  32};
const Btn B_START = {364,  46, 112, 114};
const Btn B_RESET = {364, 168, 112,  58};
const Btn B_TOT   = { 16, 150, 330,  40};
const int ROW1 = 46, ROW2 = 124;        // kartu editor
const int GX = 254, GY = 184, GW = 210, GH = 84;   // area grafik

void drawBtn(const Btn& b, const char* label, uint16_t col, uint8_t sz = 2);

// ---------------------------- UTIL -------------------------------------------
uint8_t xorCs(const char* s, uint8_t n) {
  uint8_t c = 0;
  for (uint8_t i = 0; i < n; i++) c ^= (uint8_t)s[i];
  return c;
}

void queueCmd(const char* name, long val) {
  // Saat komunikasi putus, perintah selain STOP DIBUANG (tidak ditunda), supaya
  // START/ubah nilai tidak tiba-tiba dieksekusi ketika komunikasi pulih.
  if (!linkOk && strcmp(name, "STOP") != 0) return;
  if (cmdPending && strcmp(cmdName, "STOP") == 0 && strcmp(name, "STOP") != 0) return;
  cmdSeq = (cmdSeq + 1) % 100;
  strncpy(cmdName, name, sizeof(cmdName) - 1);
  cmdName[sizeof(cmdName) - 1] = 0;
  cmdVal = val;
  cmdPending = true;
}

void sendReply() {
  char body[32];
  if (cmdPending) snprintf(body, sizeof(body), "C,%u,%s,%ld", cmdSeq, cmdName, cmdVal);
  else            snprintf(body, sizeof(body), "C,%u,NOP,0", cmdSeq);
  char out[40];
  snprintf(out, sizeof(out), "$%s*%02X\n", body, xorCs(body, strlen(body)));
  esp.print(out);
}

// frame T: st,md,t10,flow100,totml,heat,pump,flt,ack   |  frame S: hm,pm,sp,pumpA,kf,ack
uint32_t lastEdit = 0;     // editor tidak ditimpa data ESP selama 3 s setelah disentuh

bool parseTele(char* f) {
  char* star = strchr(f, '*');
  if (!star) return false;
  *star = 0;
  uint8_t cs = (uint8_t)strtoul(star + 1, NULL, 16);
  if (xorCs(f, strlen(f)) != cs) return false;
  char typ = f[0];
  if ((typ != 'T' && typ != 'S') || f[1] != ',') return false;
  long v[9];
  uint8_t n = 0;
  char* p = f + 2;
  while (p && n < 9) {
    v[n++] = atol(p);
    p = strchr(p, ',');
    if (p) p++;
  }
  if (typ == 'T') {
    if (n < 9) return false;
    uint8_t prevSt = d.st;
    d.st = v[0]; d.md = v[1]; d.t10 = v[2]; d.flow100 = v[3]; d.totml = v[4];
    d.heat = v[5]; d.pump = v[6]; d.flt = v[7]; d.ack = v[8];
    if (d.st == 1 && prevSt != 1) runStart = millis();
  } else {
    if (n < 6) return false;
    d.sp = v[2]; d.pumpA = v[3]; d.kf = v[4]; d.ack = v[5];
    // sinkron editor dengan nilai resmi ESP32 (mis. diubah dari web)
    if (millis() - lastEdit > 3000 && !cmdPending) {
      edHeat = v[0]; edPump = v[1]; edSp = v[2]; edPumpA = v[3]; edKf = v[4];
    }
  }
  return true;
}

void pollEsp() {
  while (esp.available()) {
    char c = esp.read();
    if (c == '$') { rxLen = 0; continue; }
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) {
        rxBuf[rxLen] = 0;
        if (parseTele(rxBuf)) {
          haveData = true;
          lastRx = millis();
          if (cmdPending && d.ack == cmdSeq) cmdPending = false;
          sendReply();
        }
      }
      rxLen = 0;
      continue;
    }
    if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = c;
    else rxLen = 0;
  }
  bool was = linkOk;
  linkOk = haveData && (millis() - lastRx < 2000);
  if (was && !linkOk && cmdPending && strcmp(cmdName, "STOP") != 0) cmdPending = false;
}

// ---------------------------- TOUCH ------------------------------------------
bool readTouch(int& x, int& y) {
  TSPoint p = ts.getPoint();
  pinMode(YP, OUTPUT);
  pinMode(XM, OUTPUT);
  if (p.z < MINPRESSURE || p.z > MAXPRESSURE) return false;
  int rx = p.x, ry = p.y;
#if TOUCH_SWAP_XY
  x = map(ry, TS_TOP, TS_BOT, 0, tft.width());
  y = map(rx, TS_RT, TS_LEFT, 0, tft.height());
#else
  x = map(rx, TS_LEFT, TS_RT, 0, tft.width());
  y = map(ry, TS_TOP, TS_BOT, 0, tft.height());
#endif
#if TOUCH_INV_X
  x = tft.width() - x;
#endif
#if TOUCH_INV_Y
  y = tft.height() - y;
#endif
#if TOUCH_DEBUG
  Serial.print(rx); Serial.print(','); Serial.print(ry);
  Serial.print(F(" -> ")); Serial.print(x); Serial.print(','); Serial.println(y);
#endif
  return true;
}

// ---------------------------- PRIMITIF GAMBAR --------------------------------
bool hit(const Btn& b, int x, int y) { return x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h; }

// teks rata kiri dengan latar (tanpa kedip); str dipad agar menimpa sisa lama
void txt(int x, int y, const char* s, uint8_t sz, uint16_t fg, uint16_t bg) {
  tft.setTextSize(sz);
  tft.setTextColor(fg, bg);
  tft.setCursor(x, y);
  tft.print(s);
}

void txtC(int cx, int y, const char* s, uint8_t sz, uint16_t fg) {   // rata tengah, transparan
  tft.setTextSize(sz);
  tft.setTextColor(fg);
  tft.setCursor(cx - (int)strlen(s) * 3 * sz, y);
  tft.print(s);
}

void drawBtn(const Btn& b, const char* label, uint16_t col, uint8_t sz) {
  tft.fillRoundRect(b.x, b.y, b.w, b.h, 6, col);
  txtC(b.x + b.w / 2, b.y + (b.h - 8 * sz) / 2, label, sz, C_TXT);
}

void card(int x, int y, int w, int h) { tft.fillRoundRect(x, y, w, h, 8, C_CARD); }

void bar(int x, int y, int w, int h, int pct, uint16_t col) {
  pct = constrain(pct, 0, 100);
  int f = (long)w * pct / 100;
  if (f > 0) tft.fillRect(x, y, f, h, col);
  if (f < w) tft.fillRect(x + f, y, w - f, h, C_TRK);
}

void fmtTemp(char* s, uint8_t n, int t10, uint8_t width) {
  if (t10 == -9999) { snprintf(s, n, "%*s", width, "ERR"); return; }
  int a = abs(t10);
  char tmp[10];
  snprintf(tmp, sizeof(tmp), "%s%d.%d", t10 < 0 ? "-" : "", a / 10, a % 10);
  snprintf(s, n, "%*s", width, tmp);
}

void fmtFlow(char* s, uint8_t n, uint16_t f100, uint8_t width) {
  char tmp[10];
  snprintf(tmp, sizeof(tmp), "%u.%02u", f100 / 100, f100 % 100);
  snprintf(s, n, "%*s", width, tmp);
}

// ---------------------------- BAR STATUS & NAVIGASI --------------------------
void drawStatusBar() {
  uint16_t col = !linkOk ? C_RED : (d.st == 2 ? C_RED : (d.st == 1 ? C_GO : C_GRY));
  tft.fillRect(0, 0, 380, 39, col);
  char s[24];
  if (!linkOk)            strcpy(s, "KOMUNIKASI PUTUS");
  else if (d.flt & 4)     strcpy(s, "SSR MACET! E-STOP");
  else if (d.flt & 1)     strcpy(s, "FAULT SUHU>=200C");
  else if (d.flt & 32)    strcpy(s, "HEATER OFF:BATAS WKT");
  else if (d.flt & 2)     strcpy(s, d.st == 1 ? "RTD ERR!AWASI HEATER" : "FAULT SENSOR RTD");
  else if (d.flt & 16)    strcpy(s, "START DITOLAK: RTD");
  else snprintf(s, sizeof(s), "%s | %s", d.st == 1 ? "JALAN" : (d.st == 2 ? "FAULT" : "IDLE"),
                d.md ? "AUTO" : "MANUAL");
  txt(8, 12, s, 2, C_TXT, col);
  tft.fillCircle(270, 19, 6, linkOk ? C_OK : C_TXT);
  txt(282, 16, linkOk ? "LINK OK" : "LINK --", 1, C_TXT, col);
}

void drawNav() {
  const char* tabs[4] = { "BERANDA", "MANUAL", "AUTO", "SETELAN" };
  for (uint8_t i = 0; i < 4; i++) {
    Btn b = { i * 120 + 2, 282, 116, 36 };
    drawBtn(b, tabs[i], page == i ? C_NAVA : C_NAV, 2);
  }
  tft.fillRoundRect(B_STOP.x - 2, B_STOP.y - 2, B_STOP.w + 4, B_STOP.h + 4, 7, C_TXT);
  drawBtn(B_STOP, "STOP", C_RED, 3);
}

// ---------------------------- HALAMAN: BERANDA -------------------------------
void homeStatic() {
  card(6, 44, 230, 108);
  txt(16, 52, "SUHU RTD", 2, C_DIM, C_CARD);
  txt(196, 78, "C", 3, C_ORG, C_CARD);
  tft.drawCircle(190, 78, 3, C_ORG);
  card(244, 44, 230, 108);
  txt(254, 52, "DEBIT", 2, C_DIM, C_CARD);
  txt(434, 90, "L/m", 1, C_BLU, C_CARD);
  card(6, 158, 230, 118);
  txt(16, 164, "HEATER", 2, C_DIM, C_CARD);
  txt(16, 210, "POMPA", 2, C_DIM, C_CARD);
  card(244, 158, 230, 118);
  txt(254, 164, "TREN SUHU", 2, C_DIM, C_CARD);
  trDirty = true;
}

void drawTrend() {
  tft.fillRect(GX, GY, GW, GH, C_BG);
  tft.drawRect(GX, GY, GW, GH, C_TRK);
  if (trCount < 2) { txt(GX + 40, GY + 38, "menunggu data", 1, C_DIM, C_BG); return; }
  int lo = 255, hi = 0;
  for (uint8_t i = 0; i < trCount; i++) { lo = min(lo, (int)trend[i]); hi = max(hi, (int)trend[i]); }
  if (d.md == 1) { lo = min(lo, (int)d.sp); hi = max(hi, (int)d.sp); }
  lo -= 3; hi += 3;
  if (hi - lo < 10) { int m = (hi + lo) / 2; lo = m - 5; hi = m + 5; }
  // garis setpoint (putus-putus) saat AUTO
  if (d.md == 1) {
    int ys = GY + GH - 2 - (long)(d.sp - lo) * (GH - 4) / (hi - lo);
    for (int x = GX + 2; x < GX + GW - 2; x += 8) tft.drawFastHLine(x, ys, 4, C_DIM);
  }
  int px = 0, py = 0;
  uint8_t start = TR_N - trCount;
  for (uint8_t i = 0; i < trCount; i++) {
    int x = GX + (start + i) * 2;
    int y = GY + GH - 2 - (long)(trend[i] - lo) * (GH - 4) / (hi - lo);
    if (i) { tft.drawLine(px, py, x, y, C_ORG); tft.drawLine(px, py + 1, x, y + 1, C_ORG); }
    px = x; py = y;
  }
  char s[12];
  snprintf(s, sizeof(s), "%d", hi);  txt(GX + 3, GY + 3, s, 1, C_DIM, C_BG);
  snprintf(s, sizeof(s), "%d", lo);  txt(GX + 3, GY + GH - 10, s, 1, C_DIM, C_BG);
}

void homeUpdate() {
  char s[16];
  if (d.t10 != shown.t10) { fmtTemp(s, sizeof(s), d.t10, 5); txt(16, 76, s, 5, C_ORG, C_CARD); }
  if (d.sp != shown.sp || d.md != shown.md) {
    if (d.md) snprintf(s, sizeof(s), "SP %3u C   ", d.sp); else strcpy(s, "mode manual");
    txt(16, 128, s, 2, C_DIM, C_CARD);
  }
  if (d.flow100 != shown.flow100) { fmtFlow(s, sizeof(s), d.flow100, 5); txt(270, 76, s, 5, C_BLU, C_CARD); }
  if (d.totml != shown.totml) {
    snprintf(s, sizeof(s), "Tot %6lu mL", (unsigned long)d.totml);
    txt(254, 128, s, 2, C_DIM, C_CARD);
  }
  if (d.heat != shown.heat) {
    snprintf(s, sizeof(s), "%3u%%", d.heat); txt(178, 164, s, 2, C_TXT, C_CARD);
    bar(16, 184, 210, 16, d.heat, C_ORG);
  }
  if (d.pump != shown.pump) {
    snprintf(s, sizeof(s), "%3u%%", d.pump); txt(178, 210, s, 2, C_TXT, C_CARD);
    bar(16, 230, 210, 16, d.pump, C_BLU);
  }
  uint32_t sec = (d.st == 1) ? (millis() - runStart) / 1000 : 0;
  if (sec != shownSec) {
    snprintf(s, sizeof(s), "%02lu:%02lu:%02lu", sec / 3600, (sec / 60) % 60, sec % 60);
    txt(16, 254, s, 2, d.st == 1 ? C_TXT : C_DIM, C_CARD);
    txt(118, 258, "waktu proses", 1, C_DIM, C_CARD);
    shownSec = sec;
  }
  if (trDirty) { drawTrend(); trDirty = false; }
}

// ---------------------------- HALAMAN: MANUAL / AUTO / SETELAN ---------------
void editorStatic(int y, const char* label) {
  card(6, y, 350, 72);
  txt(16, y + 6, label, 2, C_DIM, C_CARD);
  drawBtn({16, y + 26, 60, 40}, "-", C_NAV, 3);
  drawBtn({196, y + 26, 60, 40}, "+", C_NAV, 3);
}

void editorValue(int y, int val, const char* unit, uint16_t col, int pct) {
  char s[12];
  snprintf(s, sizeof(s), "%3d%s", val, unit);
  txt(88, y + 32, s, 4, col, C_CARD);
  bar(268, y + 40, 78, 14, pct, col);
}

void infoStatic(const char* l1, const char* l2) {
  card(6, 202, 350, 74);
  txt(16, 208, l1, 2, C_DIM, C_CARD);
  txt(190, 208, l2, 2, C_DIM, C_CARD);
}

void ctrlStatic(bool withStart) {
  if (withStart) drawBtn(B_START, "START", C_GO, 3);
  drawBtn(B_RESET, "RESET", C_NAV, 2);
  txt(366, 236, "Fan selalu", 1, C_DIM, C_BG);
  txt(366, 248, "ON (AC)", 1, C_DIM, C_BG);
}

void manualUpdate() {
  char s[16];
  if (edHeat != shEd1) { editorValue(ROW1, edHeat, "%", C_ORG, edHeat); shEd1 = edHeat; }
  if (edPump != shEd2) { editorValue(ROW2, edPump, "%", C_BLU, edPump); shEd2 = edPump; }
  if (d.t10 != shown.t10) { fmtTemp(s, sizeof(s), d.t10, 6); txt(16, 230, s, 3, C_ORG, C_CARD); }
  if (d.flow100 != shown.flow100) { fmtFlow(s, sizeof(s), d.flow100, 5); txt(190, 230, s, 3, C_BLU, C_CARD); }
  if (d.heat != shown.heat || d.pump != shown.pump) {
    snprintf(s, sizeof(s), "aktual H%3u P%3u", d.heat, d.pump);
    txt(16, 258, s, 1, C_DIM, C_CARD);
  }
}

void autoUpdate() {
  char s[16];
  if (edSp != shEd1)    { editorValue(ROW1, edSp, "C", C_ORG, (edSp - 40) * 100 / 140); shEd1 = edSp; }
  if (edPumpA != shEd2) { editorValue(ROW2, edPumpA, "%", C_BLU, edPumpA); shEd2 = edPumpA; }
  if (d.t10 != shown.t10) { fmtTemp(s, sizeof(s), d.t10, 6); txt(16, 230, s, 3, C_ORG, C_CARD); }
  if (d.heat != shown.heat) {
    snprintf(s, sizeof(s), "%3u%%", d.heat); txt(190, 230, s, 3, C_ORG, C_CARD);
    bar(270, 236, 76, 12, d.heat, C_ORG);
  }
}

void setUpdate() {
  char s[16];
  if (edKf != shEd1) {
    snprintf(s, sizeof(s), "%5d", edKf);
    txt(84, ROW1 + 34, s, 3, C_BLU, C_CARD);
    shEd1 = edKf;
  }
  if (d.totml != shown.totml) {
    snprintf(s, sizeof(s), "%7lu mL", (unsigned long)d.totml);
    txt(16, 230, s, 2, C_TXT, C_CARD);
  }
  if (d.flow100 != shown.flow100) { fmtFlow(s, sizeof(s), d.flow100, 5); txt(190, 230, s, 2, C_BLU, C_CARD); }
}

// ---------------------------- GAMBAR PENUH -----------------------------------
void drawFull() {
  tft.fillScreen(C_BG);
  memset(&shown, 0xFF, sizeof(shown));
  shEd1 = shEd2 = -1;
  shownSec = 0xFFFFFFFF;
  linkShown = -1;
  drawNav();
  switch (page) {
    case PG_HOME:
      homeStatic();
      break;
    case PG_MANUAL:
      editorStatic(ROW1, "HEATER (duty)");
      editorStatic(ROW2, "POMPA (duty)");
      infoStatic("SUHU C", "DEBIT L/m");
      ctrlStatic(true);
      break;
    case PG_AUTO:
      editorStatic(ROW1, "SETPOINT SUHU");
      editorStatic(ROW2, "POMPA (duty)");
      infoStatic("SUHU C", "HEATER FUZZY");
      ctrlStatic(true);
      break;
    case PG_SET:
      card(6, ROW1, 350, 72);
      txt(16, ROW1 + 6, "FAKTOR FLOW (pulsa/L)", 2, C_DIM, C_CARD);
      drawBtn({16, ROW1 + 26, 60, 40}, "-", C_NAV, 3);
      drawBtn({196, ROW1 + 26, 60, 40}, "+", C_NAV, 3);
      card(6, ROW2, 350, 72);
      drawBtn(B_TOT, "RESET TOTAL VOLUME", C_NAV, 2);
      infoStatic("TOTAL", "DEBIT L/m");
      ctrlStatic(false);
      txt(366, 46, "WiFi:", 1, C_DIM, C_BG);
      txt(366, 58, "SprayDryer-UTM", 1, C_TXT, C_BG);
      txt(366, 76, "Web:", 1, C_DIM, C_BG);
      txt(366, 88, "192.168.4.1", 1, C_TXT, C_BG);
      break;
  }
  needFull = false;
}

void drawValues() {
  if (d.st != shown.st || d.md != shown.md || d.flt != shown.flt || (int8_t)linkOk != linkShown) {
    drawStatusBar();
    linkShown = linkOk;
    if (page == PG_HOME) trDirty = true;
  }
  switch (page) {
    case PG_HOME:   homeUpdate();   break;
    case PG_MANUAL: manualUpdate(); break;
    case PG_AUTO:   autoUpdate();   break;
    case PG_SET:    setUpdate();    break;
  }
  shown = d;
}

// ---------------------------- TREN SUHU --------------------------------------
void sampleTrend() {
  if (!linkOk || d.t10 == -9999) return;
  if (millis() - trLast < 1000) return;
  trLast = millis();
  int t = constrain((d.t10 + 5) / 10, 0, 255);
  if (trCount < TR_N) trend[trCount++] = t;
  else { memmove(trend, trend + 1, TR_N - 1); trend[TR_N - 1] = t; }
  if (page == PG_HOME) trDirty = true;
}

// ---------------------------- AKSI SENTUH ------------------------------------
bool inBtn(int x, int y, int bx, int by) { return x >= bx && x < bx + 60 && y >= by && y < by + 40; }

void stepEdit(int x, int y, int rowY, int& val, int step, int lo, int hi, const char* cmd) {
  if (inBtn(x, y, 16, rowY + 26))       val = constrain(val - step, lo, hi);
  else if (inBtn(x, y, 196, rowY + 26)) val = constrain(val + step, lo, hi);
  else return;
  lastEdit = millis();
  queueCmd(cmd, val);
}

void handleTouch(int x, int y) {
  if (x >= 370 && y < 42) { queueCmd("STOP", 0); return; }          // area STOP diperlebar
  if (y >= 280) {
    Page np = (Page)constrain(x / 120, 0, 3);
    if (np != page) {
      page = np; needFull = true;
      if (np == PG_MANUAL) queueCmd("MODE", 0);
      if (np == PG_AUTO)   queueCmd("MODE", 1);
    }
    return;
  }
  if (page != PG_HOME && hit(B_RESET, x, y)) { queueCmd("RESET", 0); return; }
  if ((page == PG_MANUAL || page == PG_AUTO) && hit(B_START, x, y)) { queueCmd("START", 0); return; }
  if (page == PG_MANUAL) {
    stepEdit(x, y, ROW1, edHeat, 5, 0, 100, "HEAT");
    stepEdit(x, y, ROW2, edPump, 5, 0, 100, "PUMP");
  } else if (page == PG_AUTO) {
    stepEdit(x, y, ROW1, edSp, 5, 40, 180, "SP");
    stepEdit(x, y, ROW2, edPumpA, 5, 0, 100, "PUMPA");
  } else if (page == PG_SET) {
    stepEdit(x, y, ROW1, edKf, 10, 1, 20000, "KF");
    if (hit(B_TOT, x, y)) queueCmd("TOTRST", 0);
  }
}

// ---------------------------- SETUP & LOOP -----------------------------------
void setup() {
  Serial.begin(9600);
  esp.begin(19200);
  uint16_t id = tft.readID();
  Serial.print(F("TFT ID terbaca = 0x")); Serial.println(id, HEX);
  if (FORCE_ID) id = FORCE_ID;
  else if (id == 0xD3D3 || id == 0x0000 || id == 0xFFFF || id == 0x0404) id = 0x9486;
  Serial.print(F("TFT ID dipakai = 0x")); Serial.println(id, HEX);
  tft.begin(id);
  tft.setRotation(1);
  // layar uji: jika tulisan ini tidak muncul, masalah di LCD/ID, bukan di komunikasi
  tft.fillScreen(C_BG);
  txtC(240, 130, "SMART SALT PROCESSOR", 3, C_ORG);
  txtC(240, 170, "menunggu ESP32...", 2, C_TXT);
  delay(1500);
  memset(&d, 0, sizeof(d));
  d.t10 = -9999;
}

void loop() {
  pollEsp();
  if (needFull) drawFull();
  pollEsp();
  sampleTrend();
  drawValues();

  static uint32_t lastTouch = 0;
  int x, y;
  if (millis() - lastTouch > 250 && readTouch(x, y)) {
    lastTouch = millis();
    handleTouch(x, y);
  }
}