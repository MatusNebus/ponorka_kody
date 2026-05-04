// PREVÁDZKA STEPPERA – CH3 = absolútna poloha z EEPROM (iba LOW-PASS filter na vstup)
// PPM SUM: D3
// A4988:   STEP D12, DIR D13, EN D8 (LOW = enable)
// LED (D5, active-LOW) OFF, ESC piny (D6, D9) ticho

#include <PPMReader.h>
#include <EEPROM.h>

// --- PINY ---
#define PPM_PIN        3
#define POCET_KANALOV  8
#define STEP_PIN      12
#define DIR_PIN       13
#define EN_PIN         8
#define LED_PIN_PWM    5
#define ESC_PIN_1      6
#define ESC_PIN_2      9

// --- RÝCHLOSŤ ---
const int STEP_PULSE_US    = 3;       // šírka pulzu na STEP
const int KROK_INTERVAL_US = 1500;    // menšie = rýchlejšie

// --- PRAHY kanála (µs) ---
const int CH_DOLU_MAX   = 1300;       // <= 1300 = dole
const int CH_STRED_MIN  = 1301;       // 1301..1700 = stred
const int CH_STRED_MAX  = 1700;       // >= 1700 = hore
const unsigned long DRZ_SYNC_MS = 2000; // držanie na extréme pre zosúladenie

// --- LOW-PASS FILTER parametre ---
float TAU_S = 0.20f;                  // časová konštanta filtra (~0.2 s)
unsigned long lastFiltMs = 0;
int ch3_raw  = 1500;
int ch3_filt = 1500;

// --- EEPROM ---
const int  EE_MAGIC_ADDR = 0;
const int  EE_DATA_ADDR  = 8;
const long EE_MAGIC      = 0x31504F49L;

// --- PREMENNÉ ---
PPMReader ppm(PPM_PIN, POCET_KANALOV);

long pozicia_kroky = 0;
long min_kroky = 0, max_kroky = 0;
bool rozsahOK = false;
bool zosuladene = false;

unsigned long posledny_krok_us = 0;
unsigned long ch3_dole_od_ms = 0, ch3_hore_od_ms = 0;

long mapLong(long x, long in_min, long in_max, long out_min, long out_max) {
  if (x < in_min) x = in_min;
  if (x > in_max) x = in_max;
  return out_min + ((x - in_min) * (out_max - out_min)) / (in_max - in_min);
}

void krok(bool smer_k_max) {          // true = k MAX, false = k MIN
  digitalWrite(DIR_PIN, smer_k_max ? HIGH : LOW);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(STEP_PIN, LOW);
  pozicia_kroky += smer_k_max ? 1 : -1;
}

void bezpecne() {
  pinMode(STEP_PIN, OUTPUT); digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN,  OUTPUT); digitalWrite(DIR_PIN,  LOW);
  pinMode(EN_PIN,   OUTPUT); digitalWrite(EN_PIN,   HIGH);  // driver OFF
  pinMode(LED_PIN_PWM, OUTPUT); analogWrite(LED_PIN_PWM, 255); // LED OFF (active-LOW)
  pinMode(ESC_PIN_1, OUTPUT); digitalWrite(ESC_PIN_1, LOW);
  pinMode(ESC_PIN_2, OUTPUT); digitalWrite(ESC_PIN_2, LOW);
}

void nacitajEEPROM() {
  long magic = 0;
  EEPROM.get(EE_MAGIC_ADDR, magic);
  if (magic == EE_MAGIC) {
    EEPROM.get(EE_DATA_ADDR, min_kroky);
    EEPROM.get(EE_DATA_ADDR + (int)sizeof(long), max_kroky);
    if (max_kroky > min_kroky + 100) rozsahOK = true;
  }
}

// --- low-pass filter na CH3: EMA s α z τ a dt ---
int filterCH3(int input) {
  unsigned long nowMs = millis();
  float dt = (lastFiltMs == 0) ? 0.02f : (nowMs - lastFiltMs) / 1000.0f; // s
  lastFiltMs = nowMs;
  if (dt < 0.002f) dt = 0.002f;   // ochrana
  float alpha = dt / (TAU_S + dt);
  // y = y + α (x - y)
  ch3_filt = (int)(ch3_filt + alpha * (input - ch3_filt));
  return ch3_filt;
}

void setup() {
  bezpecne();
  nacitajEEPROM();
  if (rozsahOK) pozicia_kroky = (min_kroky + max_kroky) / 2;
  lastFiltMs = millis();
}

void loop() {
  // --- načítaj a filtruj CH3 ---
  ch3_raw = ppm.latestValidChannelValue(3, 1500);
  if (ch3_raw < 1000) ch3_raw = 1000;
  if (ch3_raw > 2000) ch3_raw = 2000;
  int ch3 = filterCH3(ch3_raw);   // použijeme filtrovanú hodnotu

  unsigned long teraz_ms = millis();
  unsigned long teraz_us = micros();

  if (!rozsahOK) {
    // JOG kým nie je kalibrácia
    if (ch3 <= CH_DOLU_MAX) {
      digitalWrite(EN_PIN, LOW);
      if (teraz_us - posledny_krok_us >= (unsigned long)KROK_INTERVAL_US) {
        posledny_krok_us = teraz_us; krok(false);
      }
    } else if (ch3 >= CH_STRED_MAX) {
      digitalWrite(EN_PIN, LOW);
      if (teraz_us - posledny_krok_us >= (unsigned long)KROK_INTERVAL_US) {
        posledny_krok_us = teraz_us; krok(true);
      }
    } else {
      digitalWrite(EN_PIN, HIGH);
    }
    return;
  }

  // rýchle zosúladenie po štarte (drž 2 s na extréme)
  if (!zosuladene) {
    if (ch3 <= CH_DOLU_MAX) {
      if (ch3_dole_od_ms == 0) ch3_dole_od_ms = teraz_ms;
      digitalWrite(EN_PIN, LOW);
      if (teraz_us - posledny_krok_us >= (unsigned long)KROK_INTERVAL_US) {
        posledny_krok_us = teraz_us; krok(false);
      }
      if (teraz_ms - ch3_dole_od_ms >= DRZ_SYNC_MS) {
        pozicia_kroky = min_kroky; zosuladene = true; ch3_dole_od_ms = 0;
      }
      return;
    } else ch3_dole_od_ms = 0;

    if (ch3 >= CH_STRED_MAX) {
      if (ch3_hore_od_ms == 0) ch3_hore_od_ms = teraz_ms;
      digitalWrite(EN_PIN, LOW);
      if (teraz_us - posledny_krok_us >= (unsigned long)KROK_INTERVAL_US) {
        posledny_krok_us = teraz_us; krok(true);
      }
      if (teraz_ms - ch3_hore_od_ms >= DRZ_SYNC_MS) {
        pozicia_kroky = max_kroky; zosuladene = true; ch3_hore_od_ms = 0;
      }
      return;
    } else ch3_hore_od_ms = 0;

    digitalWrite(EN_PIN, HIGH);
    return;
  }

  // normálna prevádzka (iba s filtrovaným CH3)
  long ciel = mapLong(ch3, 1000, 2000, min_kroky, max_kroky);
  long chyba = ciel - pozicia_kroky;

  if (chyba == 0) { digitalWrite(EN_PIN, HIGH); return; } // v cieli = vypni driver

  digitalWrite(EN_PIN, LOW);
  bool smer_k_max = (chyba > 0);

  if (teraz_us - posledny_krok_us >= (unsigned long)KROK_INTERVAL_US) {
    posledny_krok_us = teraz_us;
    krok(smer_k_max);
  }
}
