// ================================================================
// ROV – ALL-IN-ONE (LED CH6, STEPPER CH3 s filtrom, 2x ESC CH2/CH1 mix)
// - PPM SUM: D3  (PPMReader)
// - LED PWM: D5  (active-LOW driver: 255=OFF, 0=FULL)
// - STEPPER: STEP D12, DIR D13, EN D8 (LOW=enable), absolútna poloha podľa EEPROM
// - ESC L/R: D9 / D6 (Servo 1000..2000 µs, 1500 = neutrál, bidirectional)
// - EEPROM: MIN/MAX z kalibračného sketcha
// - Anti-chvenie: low-pass filter na CH3 (EMA s časovou konštantou TAU_S)
// ================================================================

#include <PPMReader.h>
#include <Servo.h>
#include <EEPROM.h>

// ---------- PINY ----------
#define PPM_PIN          3
#define POCET_KANALOV    8

#define LED_PIN_PWM      5    // active-LOW (255=OFF)
#define STEPPER_STEP     7 //12 //10
#define STEPPER_DIR      10 //13 //7
#define STEPPER_EN       8    // LOW = zapnutý driver

#define ESC_LEFT_PIN     9
#define ESC_RIGHT_PIN    6

// ---------- ESC (mix) ----------
Servo escL, escR;
const int ESC_MIN_US      = 1000;
const int ESC_MID_US      = 1500;
const int ESC_MAX_US      = 2000;
const int MRTVA_ZONA_US   = 30;      // ±30 µs okolo stredu
const float ESC_SCALE     = 0.30f;   // 30 % maximálneho výkonu BLDC motorov

// ---------- LED (CH6) ----------
const int CH6_OFF_MAX     = 1300;    // ≤ OFF
const int CH6_MID_MAX     = 1700;    // 1301..1700 = ~50 %, ≥1701 = ON

// ---------- STEPPER ----------
const int STEP_PULSE_US    = 3;      // A4988 minimum je 1 µs, 3 µs je OK :contentReference[oaicite:0]{index=0}
const int KROK_INTERVAL_US = 3000;   // menšie = rýchlejšie (ak bude treba, skús 2500-4000)

// Low-pass filter (EMA) na CH3
float TAU_S = 0.20f;                 // časová konštanta filtra
unsigned long lastFiltMs = 0;        // LEN pre filter
unsigned long lastStepUs = 0;        // LEN pre stepper
int ch3_raw  = 1500;
int ch3_filt = 1500;

// Zosúladenie po štarte (držanie páčky na extréme)
const unsigned long DRZ_SYNC_MS = 3000;
unsigned long ch3_dole_od_ms = 0;
unsigned long ch3_hore_od_ms = 0;
bool zosuladene = false;

// EEPROM (rovnaké adresy/magic ako v kalibračnom sketchi)
const int  EE_MAGIC_ADDR = 0;
const int  EE_DATA_ADDR  = 8;
const long EE_MAGIC      = 0x31504F49L;

// Rozsah a poloha v krokoch
long min_kroky = 0;
long max_kroky = 0;
long pozicia_kroky = 0;
bool rozsahOK = false;

// ---------- PPM ----------
PPMReader ppm(PPM_PIN, POCET_KANALOV);

// ---------- POMOCNÉ ----------
long mapLong(long x, long in_min, long in_max, long out_min, long out_max) {
  if (x < in_min) x = in_min;
  if (x > in_max) x = in_max;
  return out_min + ((x - in_min) * (out_max - out_min)) / (in_max - in_min);
}

int deadband1500(int us, int db) {
  if (us > (ESC_MID_US - db) && us < (ESC_MID_US + db)) return ESC_MID_US;
  if (us < ESC_MIN_US) return ESC_MIN_US;
  if (us > ESC_MAX_US) return ESC_MAX_US;
  return us;
}

void stepperEnable(bool enableOn) {
  // LOW = enable, HIGH = disable
  digitalWrite(STEPPER_EN, enableOn ? LOW : HIGH);
}

void clampPozicia() {
  if (rozsahOK) {
    if (pozicia_kroky < min_kroky) pozicia_kroky = min_kroky;
    if (pozicia_kroky > max_kroky) pozicia_kroky = max_kroky;
  }
}

void stepperKrok(bool smer_k_max) {   // true = k MAX, false = k MIN
  digitalWrite(STEPPER_DIR, smer_k_max ? HIGH : LOW);

  digitalWrite(STEPPER_STEP, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(STEPPER_STEP, LOW);

  pozicia_kroky += smer_k_max ? 1 : -1;
  clampPozicia();
}

bool jeCasNaKrok(unsigned long teraz_us) {
  if ((unsigned long)(teraz_us - lastStepUs) >= (unsigned long)KROK_INTERVAL_US) {
    lastStepUs = teraz_us;
    return true;
  }
  return false;
}

void bezpecneStavy() {
  // LED OFF (active-LOW)
  pinMode(LED_PIN_PWM, OUTPUT);
  analogWrite(LED_PIN_PWM, 255);

  // Stepper OFF
  pinMode(STEPPER_STEP, OUTPUT);
  digitalWrite(STEPPER_STEP, LOW);

  pinMode(STEPPER_DIR, OUTPUT);
  digitalWrite(STEPPER_DIR, LOW);

  pinMode(STEPPER_EN, OUTPUT);
  digitalWrite(STEPPER_EN, HIGH);

  // ESC neutrál budeme posielať až po attach
}

void nacitajEEPROM() {
  long magic = 0;
  EEPROM.get(EE_MAGIC_ADDR, magic);

  if (magic == EE_MAGIC) {
    EEPROM.get(EE_DATA_ADDR, min_kroky);
    EEPROM.get(EE_DATA_ADDR + (int)sizeof(long), max_kroky);

    if (max_kroky > min_kroky + 100) {
      rozsahOK = true;
      pozicia_kroky = (min_kroky + max_kroky) / 2; // štart "niekde uprostred"
    }
  }
}

// ---------- FILTER CH3 ----------
int filterCH3(int input) {
  unsigned long nowMs = millis();
  float dt = (lastFiltMs == 0) ? 0.02f : (nowMs - lastFiltMs) / 1000.0f; // sekundy
  lastFiltMs = nowMs;

  if (dt < 0.002f) dt = 0.002f;

  float alpha = dt / (TAU_S + dt);
  ch3_filt = (int)(ch3_filt + alpha * (input - ch3_filt));
  return ch3_filt;
}

// ---------- LED z CH6 ----------
void riadLED() {
  int ch6 = ppm.latestValidChannelValue(6, 1500);

  if (ch6 <= CH6_OFF_MAX) {
    analogWrite(LED_PIN_PWM, 255);   // OFF (active-LOW)
  } else if (ch6 <= CH6_MID_MAX) {
    analogWrite(LED_PIN_PWM, 128);   // ~50 %
  } else {
    analogWrite(LED_PIN_PWM, 0);     // 100 %
  }
}

// ---------- MIX pre 2 ESC (CH2 = dopredu/dozadu, CH1 = doľava/doprava) ----------
void riadESCmix() {
  int ch1 = deadband1500(ppm.latestValidChannelValue(1, ESC_MID_US), MRTVA_ZONA_US);
  int ch2 = deadband1500(ppm.latestValidChannelValue(2, ESC_MID_US), MRTVA_ZONA_US);

  int turn = ch1 - ESC_MID_US;   // -500..+500
  int thr  = ch2 - ESC_MID_US;   // -500..+500

  int left  = thr + turn;
  int right = thr - turn;

  // limit na 30 %
  left  = (int)(left  * ESC_SCALE);
  right = (int)(right * ESC_SCALE);

  int outL = ESC_MID_US + left;
  int outR = ESC_MID_US + right;

  if (outL < ESC_MIN_US) outL = ESC_MIN_US;
  if (outL > ESC_MAX_US) outL = ESC_MAX_US;
  if (outR < ESC_MIN_US) outR = ESC_MIN_US;
  if (outR > ESC_MAX_US) outR = ESC_MAX_US;

  escL.writeMicroseconds(outL);
  escR.writeMicroseconds(outR);
}

// ---------- STEPPER z CH3 (absolútna poloha s filtrom) ----------
void riadStepper() {
  // čítanie + filtrovanie CH3
  ch3_raw = ppm.latestValidChannelValue(3, 1500);
  if (ch3_raw < 1000) ch3_raw = 1000;
  if (ch3_raw > 2000) ch3_raw = 2000;

  int ch3 = filterCH3(ch3_raw);

  unsigned long teraz_ms = millis();
  unsigned long teraz_us = micros();

  // ------------------------------------------------
  // Režim bez EEPROM kalibrácie: jednoduchý JOG
  // ------------------------------------------------
  if (!rozsahOK) {
    if (ch3 <= 1300) {
      stepperEnable(true);
      if (jeCasNaKrok(teraz_us)) {
        stepperKrok(false);
      }
      return;
    }

    if (ch3 >= 1700) {
      stepperEnable(true);
      if (jeCasNaKrok(teraz_us)) {
        stepperKrok(true);
      }
      return;
    }

    stepperEnable(false);
    return;
  }

  // ------------------------------------------------
  // Po štarte treba držať páčku 2 s na extréme,
  // aby sa softvérovo zosúladila poloha páčky a mechanický rozsah
  // ------------------------------------------------
  if (!zosuladene) {
    if (ch3 <= 1300) {
      if (ch3_dole_od_ms == 0) ch3_dole_od_ms = teraz_ms;
      ch3_hore_od_ms = 0;

      stepperEnable(true);
      if (jeCasNaKrok(teraz_us)) {
        stepperKrok(false);
      }

      if (teraz_ms - ch3_dole_od_ms >= DRZ_SYNC_MS) {
        pozicia_kroky = min_kroky;
        zosuladene = true;
        ch3_dole_od_ms = 0;
      }
      return;
    } else {
      ch3_dole_od_ms = 0;
    }

    if (ch3 >= 1700) {
      if (ch3_hore_od_ms == 0) ch3_hore_od_ms = teraz_ms;
      ch3_dole_od_ms = 0;

      stepperEnable(true);
      if (jeCasNaKrok(teraz_us)) {
        stepperKrok(true);
      }

      if (teraz_ms - ch3_hore_od_ms >= DRZ_SYNC_MS) {
        pozicia_kroky = max_kroky;
        zosuladene = true;
        ch3_hore_od_ms = 0;
      }
      return;
    } else {
      ch3_hore_od_ms = 0;
    }

    // kým nie je páčka podržaná na extréme, driver vypni
    stepperEnable(false);
    return;
  }

  // ------------------------------------------------
  // Normálna prevádzka: CH3 -> cieľová poloha v rozsahu min..max
  // ------------------------------------------------
  long ciel = mapLong(ch3, 1000, 2000, max_kroky, min_kroky);

  if (ciel < min_kroky) ciel = min_kroky;
  if (ciel > max_kroky) ciel = max_kroky;

  long chyba = ciel - pozicia_kroky;

  if (chyba == 0) {
    stepperEnable(false);   // v cieli vypni driver
    return;
  }

  bool smer_k_max = (chyba > 0);
  stepperEnable(true);

  if (jeCasNaKrok(teraz_us)) {
    stepperKrok(smer_k_max);
  }
}

// ================== SETUP / LOOP ==================
void setup() {
  bezpecneStavy();
  nacitajEEPROM();

  // ESC neutrál + arming pauza
  escL.attach(ESC_LEFT_PIN, ESC_MIN_US, ESC_MAX_US);
  escR.attach(ESC_RIGHT_PIN, ESC_MIN_US, ESC_MAX_US);
  escL.writeMicroseconds(ESC_MID_US);
  escR.writeMicroseconds(ESC_MID_US);
  delay(2000);

  // inicializácia filtrov a timerov až po štarte
  ch3_raw = ppm.latestValidChannelValue(3, 1500);
  if (ch3_raw < 1000) ch3_raw = 1000;
  if (ch3_raw > 2000) ch3_raw = 2000;
  ch3_filt = ch3_raw;

  lastFiltMs = millis();
  lastStepUs = micros();
}

void loop() {
  riadLED();       // CH6
  riadESCmix();    // CH2 + CH1
  riadStepper();   // CH3
}