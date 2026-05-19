#include <PPMReader.h>
#include <Servo.h>
#include <EEPROM.h>
#include <Wire.h>
#include "iBUSTelemetry.h"
#include "MS5837.h"

MS5837 sensor;
float depth_real = 0.0f;
float depth_offset = 0.0f;
float temp_real = 0.0f;
bool hladinaVynulovana = false;
//temp znac
// ================================================================
// ROV – ALL-IN-ONE
// CH5 LOW  = manual stepper cez CH3
// CH5 HIGH = AUTO PI regulácia hĺbky na 1.0 m (so simulovanou hĺbkou)
// ================================================================

// ---------- PINY ----------
#define PPM_PIN          3
#define POCET_KANALOV    8

#define LED_PIN_PWM      5
#define STEPPER_STEP     7
#define STEPPER_DIR      10
#define STEPPER_EN       8

#define ESC_LEFT_PIN     9
#define ESC_RIGHT_PIN    6

// ---------- LOGOVANIE ----------
const int LOG_MAGIC_ADDR = 88;
const int LOG_COUNT_ADDR = 92;
const int LOG_WRITE_INDEX_ADDR = 96;
const int LOG_START_ADDR = 100;   // kde začneme ukladať
const int LOG_COUNT = 100;        // 200 s / 2 s = 100 vzoriek
const long LOG_MAGIC = 0x31474F4CL;

int log_index = 0;
int log_saved_count = 0;
unsigned long log_start_ms = 0;
unsigned long log_last_ms = 0;
bool log_active = false;
bool log_started = false;


// ---------- ESC (mix) ----------
Servo escL, escR;
const int ESC_MIN_US      = 1000;
const int ESC_MID_US      = 1500;
const int ESC_MAX_US      = 2000;
const int MRTVA_ZONA_US   = 50;
const float ESC_SCALE     = 0.30f;

// ---------- LED (CH6) ----------
const int CH6_OFF_MAX     = 1300;
const int CH6_MID_MAX     = 1700;

// ---------- AUTO/MANUAL SWITCH (CH5) ----------
const int CH5_AUTO_MIN    = 1700;   // ak je CH5 >= 1700, zapni AUTO

// ---------- STEPPER ----------
const int STEP_PULSE_US    = 3;
const int KROK_INTERVAL_US = 3000;

// Low-pass filter (EMA) na CH3
float TAU_S = 0.20f;
unsigned long lastFiltMs = 0;
unsigned long lastStepUs = 0;
int ch3_raw  = 1500;
int ch3_filt = 1500;

// Zosúladenie po štarte (držanie páčky na extréme)
const unsigned long DRZ_SYNC_MS = 3000;
unsigned long ch3_dole_od_ms = 0;
unsigned long ch3_hore_od_ms = 0;
bool zosuladene = false;

// EEPROM
const int  EE_MAGIC_ADDR = 0;
const int  EE_DATA_ADDR  = 8;
const long EE_MAGIC      = 0x31504F49L;

// Rozsah a poloha v krokoch
long min_kroky = 0;
long max_kroky = 0;
long pozicia_kroky = 0;
bool rozsahOK = false;

// Softverove rozsirenie rozsahu piestu smerom k MAX.
// 0.5 cm = 5 mm z celkoveho zdvihu skrutky 80 mm.
const float STROKE_MM = 80.0f;
const float EXTRA_MAX_STROKE_MM = 5.0f;

// ---------- PPM ----------
PPMReader ppm(PPM_PIN, POCET_KANALOV);

struct StabilnyPpmKanal {
  int hodnota;
  int kandidat;
  uint8_t pocetKandidatov;
  bool inicializovany;
};

struct StabilnyEscKanal {
  int hodnota;
  int kandidat;
  unsigned long kandidatOdMs;
  bool inicializovany;
};

StabilnyPpmKanal ppmCh1 = {1500, 1500, 0, false};
StabilnyPpmKanal ppmCh2 = {1500, 1500, 0, false};
StabilnyPpmKanal ppmCh3 = {1500, 1500, 0, false};
StabilnyPpmKanal ppmCh5 = {1000, 1000, 0, false};
StabilnyPpmKanal ppmCh6 = {1500, 1500, 0, false};
StabilnyEscKanal escCh1 = {1500, 1500, 0, false};
StabilnyEscKanal escCh2 = {1500, 1500, 0, false};

const int PPM_MIN_PLATNE_US = 900;
const int PPM_MAX_PLATNE_US = 2100;
const int PPM_SKOK_ESC_US = 300;
const int PPM_SKOK_CH3_US = 180;
const int PPM_SKOK_CH5_US = 250;
const int PPM_SKOK_CH6_US = 180;
const int PPM_KANDIDAT_TOLERANCIA_US = 80;
const uint8_t PPM_POTVRDENIA_ESC = 2;
const uint8_t PPM_POTVRDENIA_CH3 = 4;
const uint8_t PPM_POTVRDENIA_CH5 = 4;
const uint8_t PPM_POTVRDENIA_CH6 = 5;

const unsigned long PPM_ARM_STABLE_MS = 1500;
const unsigned long PPM_LOST_MS = 300;
const int PPM_ARM_NEUTRAL_ESC_US = 130;
const int PPM_ARM_NEUTRAL_CH3_US = 220;
const long STEPPER_TARGET_DEADBAND_STEPS = 15;
const unsigned long STEPPER_TARGET_STABLE_MS = 500;
const unsigned long ESC_TARGET_STABLE_MS = 100;
const int ESC_TARGET_TOLERANCE_US = 8;
const unsigned long LED_ON_STABLE_MS = 1000;

bool ppmRiadenieAktivne = false;
unsigned long ppmSafeOdMs = 0;
unsigned long ppmLastGoodMs = 0;

int ledPwmAktualny = 255;
int ledPwmKandidat = 255;
unsigned long ledKandidatOdMs = 0;

long manualCielKroky = 0;
long manualKandidatCielKroky = 0;
unsigned long manualKandidatOdMs = 0;
bool manualCielInicializovany = false;

// ================================================================
// PI REGULÁTOR
// ================================================================
const float Kp_steps = 1.20f; // cast rozsahu piestu na 1 m chyby
const float Ki_steps = 0.025f; // cast rozsahu piestu na 1 m*s integralnej chyby

const float depth_ref = 0.35f;        //ZELANA HLBKA oooooooooooooooooooooooooooooooooooooooooooo
const float REG_TS    = 0.10f;     // 100 ms
const float DEADBAND_DEPTH = 0.01f; // 1 cm
const float DEADBAND_EXIT_DEPTH = 0.02f; // znovu reguluj az za 2 cm

float integral_e = 0.0f;
unsigned long lastRegMs = 0;
long ciel_kroky_reg = 0;
long auto_base_kroky = 0;
bool depthHoldPaused = false;

// prepínanie režimov
bool autoMode = false;
bool lastAutoMode = false;
const unsigned long CH5_DEBOUNCE_MS = 500;
unsigned long ch5_auto_on_od_ms = 0;
unsigned long ch5_auto_off_od_ms = 0;

unsigned long lastPrintMs = 0;

// ---------- iBUS TELEMETRIA ----------
// 1 = iBUS telemetria cez softverovy half-duplex pin IBUS_TELEMETRY_PIN.
// 0 = Serial Monitor/debug/log vypisy cez USB.
#define ENABLE_IBUS_TELEMETRY 1
#define IBUS_USE_DUMMY_VALUES 1
#define IBUS_TELEMETRY_PIN 11

#if ENABLE_IBUS_TELEMETRY
iBUSTelemetry IBusTelemetry(IBUS_TELEMETRY_PIN);
bool ibusWindowActive = false;
#endif

const unsigned long TELEMETRY_SENSOR_READ_INTERVAL_MS = 1000;
unsigned long lastTelemetrySensorReadMs = 0;
const unsigned long IBUS_WINDOW_PERIOD_MS = 1000;
const unsigned long IBUS_WINDOW_ACTIVE_MS = 120;

const uint8_t IBUS_CMD_DISCOVER = 0x80;
const uint8_t IBUS_CMD_TYPE     = 0x90;
const uint8_t IBUS_CMD_VALUE    = 0xA0;

const uint8_t IBUS_SENSOR_TEMP  = 0x01;
const uint8_t IBUS_SENSOR_EXTV  = 0x03;

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

bool ppmHodnotaPlatna(int v) {
  return v >= PPM_MIN_PLATNE_US && v <= PPM_MAX_PLATNE_US;
}

int citajPpmRaw(uint8_t kanal) {
  return ppm.latestValidChannelValue(kanal, 0);
}

void nastavBezpecneVystupy() {
  analogWrite(LED_PIN_PWM, 255);
  ledPwmAktualny = 255;
  ledPwmKandidat = 255;
  ledKandidatOdMs = 0;
  escL.writeMicroseconds(ESC_MID_US);
  escR.writeMicroseconds(ESC_MID_US);
  stepperEnable(false);
}

bool ppmSignalPlatnyTeraz() {
  int ch1 = citajPpmRaw(1);
  int ch2 = citajPpmRaw(2);
  int ch3 = citajPpmRaw(3);
  int ch5 = citajPpmRaw(5);
  int ch6 = citajPpmRaw(6);

  return ppmHodnotaPlatna(ch1) &&
         ppmHodnotaPlatna(ch2) &&
         ppmHodnotaPlatna(ch3) &&
         ppmHodnotaPlatna(ch5) &&
         ppmHodnotaPlatna(ch6);
}

bool ppmBezpecneNaZapnutieRiadenia() {
  int ch1 = citajPpmRaw(1);
  int ch2 = citajPpmRaw(2);
  int ch3 = citajPpmRaw(3);
  int ch5 = citajPpmRaw(5);

  if (!ppmSignalPlatnyTeraz()) return false;
  if (abs(ch1 - ESC_MID_US) > PPM_ARM_NEUTRAL_ESC_US) return false;
  if (abs(ch2 - ESC_MID_US) > PPM_ARM_NEUTRAL_ESC_US) return false;
  if (abs(ch3 - ESC_MID_US) > PPM_ARM_NEUTRAL_CH3_US) return false;
  if (ch5 >= CH5_AUTO_MIN) return false;

  return true;
}

void aktualizujPpmBezpecnost() {
  unsigned long nowMs = millis();

  if (ppmSignalPlatnyTeraz()) {
    ppmLastGoodMs = nowMs;
  }

  if (!ppmRiadenieAktivne) {
    if (ppmBezpecneNaZapnutieRiadenia()) {
      if (ppmSafeOdMs == 0) ppmSafeOdMs = nowMs;
      if ((unsigned long)(nowMs - ppmSafeOdMs) >= PPM_ARM_STABLE_MS) {
        ppmRiadenieAktivne = true;
      }
    } else {
      ppmSafeOdMs = 0;
    }
    return;
  }

  if ((unsigned long)(nowMs - ppmLastGoodMs) > PPM_LOST_MS) {
    ppmRiadenieAktivne = false;
    ppmSafeOdMs = 0;
    autoMode = false;
    lastAutoMode = false;
    nastavBezpecneVystupy();
  }
}

int citajPpmStabilne(uint8_t kanal, int fallback, StabilnyPpmKanal &stav, int maxSkokUs, uint8_t potrebnePotvrdenia) {
  int v = ppm.latestValidChannelValue(kanal, fallback);

  if (v < PPM_MIN_PLATNE_US || v > PPM_MAX_PLATNE_US) {
    return stav.inicializovany ? stav.hodnota : fallback;
  }

  if (!stav.inicializovany) {
    stav.hodnota = v;
    stav.kandidat = v;
    stav.pocetKandidatov = 0;
    stav.inicializovany = true;
    return stav.hodnota;
  }

  if (abs(v - stav.hodnota) <= maxSkokUs) {
    stav.hodnota = v;
    stav.kandidat = v;
    stav.pocetKandidatov = 0;
    return stav.hodnota;
  }

  if (abs(v - stav.kandidat) <= PPM_KANDIDAT_TOLERANCIA_US) {
    if (stav.pocetKandidatov < 255) stav.pocetKandidatov++;
  } else {
    stav.kandidat = v;
    stav.pocetKandidatov = 1;
  }

  if (stav.pocetKandidatov >= potrebnePotvrdenia) {
    stav.hodnota = stav.kandidat;
    stav.pocetKandidatov = 0;
  }

  return stav.hodnota;
}

void nastavLedSOneskorenim(int cielPwm) {
  unsigned long nowMs = millis();

  if (cielPwm == 255) {
    if (ledPwmAktualny != 255) {
      analogWrite(LED_PIN_PWM, 255);
    }
    ledPwmAktualny = 255;
    ledPwmKandidat = 255;
    ledKandidatOdMs = 0;
    return;
  }

  if (cielPwm == ledPwmAktualny) {
    ledPwmKandidat = cielPwm;
    ledKandidatOdMs = 0;
    return;
  }

  if (cielPwm != ledPwmKandidat) {
    ledPwmKandidat = cielPwm;
    ledKandidatOdMs = nowMs;
    return;
  }

  if (ledKandidatOdMs != 0 && (unsigned long)(nowMs - ledKandidatOdMs) >= LED_ON_STABLE_MS) {
    analogWrite(LED_PIN_PWM, cielPwm);
    ledPwmAktualny = cielPwm;
    ledKandidatOdMs = 0;
  }
}

long stabilizujManualnyCiel(long ciel) {
  unsigned long nowMs = millis();

  if (!manualCielInicializovany) {
    manualCielKroky = ciel;
    manualKandidatCielKroky = ciel;
    manualKandidatOdMs = 0;
    manualCielInicializovany = true;
    return manualCielKroky;
  }

  if (abs(ciel - manualCielKroky) <= STEPPER_TARGET_DEADBAND_STEPS) {
    manualKandidatCielKroky = manualCielKroky;
    manualKandidatOdMs = 0;
    return manualCielKroky;
  }

  if (abs(ciel - manualKandidatCielKroky) > STEPPER_TARGET_DEADBAND_STEPS) {
    manualKandidatCielKroky = ciel;
    manualKandidatOdMs = nowMs;
    return manualCielKroky;
  }

  if (manualKandidatOdMs != 0 && (unsigned long)(nowMs - manualKandidatOdMs) >= STEPPER_TARGET_STABLE_MS) {
    manualCielKroky = manualKandidatCielKroky;
    manualKandidatOdMs = 0;
  }

  return manualCielKroky;
}

int stabilizujEscKanal(int ciel, StabilnyEscKanal &stav) {
  unsigned long nowMs = millis();

  if (!stav.inicializovany) {
    stav.hodnota = ciel;
    stav.kandidat = ciel;
    stav.kandidatOdMs = 0;
    stav.inicializovany = true;
    return stav.hodnota;
  }

  if (abs(ciel - stav.hodnota) <= ESC_TARGET_TOLERANCE_US) {
    stav.kandidat = stav.hodnota;
    stav.kandidatOdMs = 0;
    return stav.hodnota;
  }

  if (abs(ciel - stav.kandidat) > ESC_TARGET_TOLERANCE_US) {
    stav.kandidat = ciel;
    stav.kandidatOdMs = nowMs;
    return stav.hodnota;
  }

  if (stav.kandidatOdMs != 0 && (unsigned long)(nowMs - stav.kandidatOdMs) >= ESC_TARGET_STABLE_MS) {
    stav.hodnota = stav.kandidat;
    stav.kandidatOdMs = 0;
  }

  return stav.hodnota;
}

void stepperEnable(bool enableOn) {
  digitalWrite(STEPPER_EN, enableOn ? LOW : HIGH); // LOW = enable
}

void clampPozicia() {
  if (rozsahOK) {
    if (pozicia_kroky < min_kroky) pozicia_kroky = min_kroky;
    if (pozicia_kroky > max_kroky) pozicia_kroky = max_kroky;
  }
}

void stepperKrok(bool smer_k_max) {
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
  pinMode(LED_PIN_PWM, OUTPUT);
  analogWrite(LED_PIN_PWM, 255);

  pinMode(STEPPER_STEP, OUTPUT);
  digitalWrite(STEPPER_STEP, LOW);

  pinMode(STEPPER_DIR, OUTPUT);
  digitalWrite(STEPPER_DIR, LOW);

  pinMode(STEPPER_EN, OUTPUT);
  digitalWrite(STEPPER_EN, HIGH);
}

void citajTlakomer() {
  sensor.read();
  depth_real = sensor.depth() - depth_offset;
  temp_real = sensor.temperature();
}

float ibusTeplotaC() {
#if IBUS_USE_DUMMY_VALUES
  return 22.5f;
#else
  return temp_real;
#endif
}

float ibusHlbkaM() {
#if IBUS_USE_DUMMY_VALUES
  return 0.42f;
#else
  float h = depth_real;
  if (h < 0.0f) h = 0.0f;
  return h;
#endif
}

void aktualizujIbusCache() {
#if ENABLE_IBUS_TELEMETRY
  IBusTelemetry.setSensorValueFP(1, ibusTeplotaC());
  IBusTelemetry.setSensorValueFP(2, ibusHlbkaM());
#endif
}

void obsluzIbusTelemetriu() {
#if ENABLE_IBUS_TELEMETRY
  unsigned long phase = millis() % IBUS_WINDOW_PERIOD_MS;
  bool shouldBeActive = phase < IBUS_WINDOW_ACTIVE_MS;

  if (shouldBeActive && !ibusWindowActive) {
    IBusTelemetry.flush();
    IBusTelemetry.listen();
    ibusWindowActive = true;
  } else if (!shouldBeActive && ibusWindowActive) {
    IBusTelemetry.stopListening();
    ibusWindowActive = false;
  }

  if (ibusWindowActive) {
    IBusTelemetry.run();
  }
#endif
}

void aktualizujSenzorPreTelemetriu() {
#if ENABLE_IBUS_TELEMETRY
  if (autoMode) return;

  unsigned long nowMs = millis();
  if ((unsigned long)(nowMs - lastTelemetrySensorReadMs) >= TELEMETRY_SENSOR_READ_INTERVAL_MS) {
    lastTelemetrySensorReadMs = nowMs;
#if IBUS_USE_DUMMY_VALUES
    aktualizujIbusCache();
#else
    citajTlakomer();
    aktualizujIbusCache();
#endif
  }
#endif
}

void nacitajEEPROM() {
  long magic = 0;
  EEPROM.get(EE_MAGIC_ADDR, magic);

  if (magic == EE_MAGIC) {
    EEPROM.get(EE_DATA_ADDR, min_kroky);
    EEPROM.get(EE_DATA_ADDR + (int)sizeof(long), max_kroky);

    if (max_kroky > min_kroky + 100) {
      long span = max_kroky - min_kroky;
      long extra_max_kroky = (long)((float)span * EXTRA_MAX_STROKE_MM / STROKE_MM);
      max_kroky += extra_max_kroky;

      rozsahOK = true;
      pozicia_kroky = (min_kroky + max_kroky) / 2;
    }
  }
}

// ---------- FILTER CH3 ----------
int filterCH3(int input) {
  unsigned long nowMs = millis();
  float dt = (lastFiltMs == 0) ? 0.02f : (nowMs - lastFiltMs) / 1000.0f;
  lastFiltMs = nowMs;

  if (dt < 0.002f) dt = 0.002f;

  float alpha = dt / (TAU_S + dt);
  ch3_filt = (int)(ch3_filt + alpha * (input - ch3_filt));
  return ch3_filt;
}

// ---------- LED z CH6 ----------
void riadLED() {
  int ch6 = citajPpmStabilne(6, 1500, ppmCh6, PPM_SKOK_CH6_US, PPM_POTVRDENIA_CH6);
  int cielPwm = 255;

  if (ch6 <= CH6_OFF_MAX) {
    cielPwm = 255;
  } else if (ch6 <= CH6_MID_MAX) {
    cielPwm = 128;
  } else {
    cielPwm = 0;
  }

  nastavLedSOneskorenim(cielPwm);
}

// ---------- MIX pre 2 ESC ----------
void riadESCmix() {
  int ch1 = citajPpmStabilne(1, ESC_MID_US, ppmCh1, PPM_SKOK_ESC_US, PPM_POTVRDENIA_ESC);
  int ch2 = citajPpmStabilne(2, ESC_MID_US, ppmCh2, PPM_SKOK_ESC_US, PPM_POTVRDENIA_ESC);
  ch1 = deadband1500(ch1, MRTVA_ZONA_US);
  ch2 = deadband1500(ch2, MRTVA_ZONA_US);
  ch1 = stabilizujEscKanal(ch1, escCh1);
  ch2 = stabilizujEscKanal(ch2, escCh2);

  int turn = ch1 - ESC_MID_US;
  int thr  = ch2 - ESC_MID_US;

  int left  = thr + turn;
  int right = thr - turn;

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

// ================================================================
// AUTO PI REGULÁCIA
// ================================================================
void riadStepperAutoPI() {
  if (!rozsahOK) return;
  if (!zosuladene) return;

  unsigned long nowMs = millis();

  // 🔴 REGULÁTOR beží len každých 100 ms
  if ((nowMs - lastRegMs) >= (unsigned long)(REG_TS * 1000.0f)) {
    lastRegMs = nowMs;

    citajTlakomer();
    aktualizujIbusCache();
    float e = depth_ref - depth_real;

    float abs_e = e;
    if (abs_e < 0.0f) abs_e = -abs_e;

    if (depthHoldPaused) {
      if (abs_e >= DEADBAND_EXIT_DEPTH) {
        depthHoldPaused = false;
      }
    } else if (abs_e <= DEADBAND_DEPTH) {
      depthHoldPaused = true;
    }

    if (depthHoldPaused) {
      ciel_kroky_reg = pozicia_kroky;
    } else {
      float e_control = e;

      long span = max_kroky - min_kroky;
      float integral_candidate = integral_e + e_control * REG_TS;
      float u = Kp_steps * e_control + Ki_steps * integral_candidate;
      float ciel_float = (float)auto_base_kroky + u * (float)span;

      if (ciel_float > (float)max_kroky) {
        ciel_float = (float)max_kroky;
        if (e_control < 0.0f) {
          integral_e = integral_candidate;
        }
      } else if (ciel_float < (float)min_kroky) {
        ciel_float = (float)min_kroky;
        if (e_control > 0.0f) {
          integral_e = integral_candidate;
        }
      } else {
        integral_e = integral_candidate;
      }

      ciel_kroky_reg = (long)ciel_float;
    }

    // debug
#if !ENABLE_IBUS_TELEMETRY
    if (millis() - lastPrintMs >= 2000) {
      lastPrintMs = millis();

      Serial.print("AUTO, h=");
      Serial.print(depth_real, 4);
      Serial.print(", href=");
      Serial.print(depth_ref, 4);
      Serial.print(", e=");
      Serial.print(e, 4);
      Serial.print(", pos=");
      Serial.print(pozicia_kroky);
      Serial.print(", tgt=");
      Serial.print(ciel_kroky_reg);
      Serial.print(", span=");
      Serial.print(max_kroky - min_kroky);
      Serial.print(", hold=");
      Serial.println(depthHoldPaused ? 1 : 0);
    }
#endif
  }

  long chyba = ciel_kroky_reg - pozicia_kroky;

  // limit ochrana
  if ((pozicia_kroky <= min_kroky && chyba < 0) ||
      (pozicia_kroky >= max_kroky && chyba > 0)) {
    stepperEnable(false);
    return;
  }

  if (chyba == 0) {
    stepperEnable(false);
    return;
  }

  bool smer_k_max = (chyba > 0);
  stepperEnable(true);

  if (jeCasNaKrok(micros())) {
    stepperKrok(smer_k_max);
  }
}

void vypisLog() {
#if !ENABLE_IBUS_TELEMETRY
  long log_magic = 0;
  int count = LOG_COUNT;
  int write_index = 0;

  EEPROM.get(LOG_MAGIC_ADDR, log_magic);
  if (log_magic == LOG_MAGIC) {
    EEPROM.get(LOG_COUNT_ADDR, count);
    EEPROM.get(LOG_WRITE_INDEX_ADDR, write_index);
    if (count < 0) count = 0;
    if (count > LOG_COUNT) count = LOG_COUNT;
    if (write_index < 0) write_index = 0;
    if (write_index >= LOG_COUNT) write_index = 0;
  }

  Serial.println("---- LOG ----");
  Serial.print("POCET=");
  Serial.println(count);

  for (int i = 0; i < count; i++) {
    int eeprom_index = i;
    if (count == LOG_COUNT) {
      eeprom_index = (write_index + i) % LOG_COUNT;
    }

    float h;
    EEPROM.get(LOG_START_ADDR + eeprom_index * sizeof(float), h);
    if (i > 0) {
      Serial.print(",");
    }
    Serial.print(h, 4);
  }
  Serial.println();
#endif
}

// ---------- STEPPER MANUAL z CH3 ----------
void riadStepperManual() {
  ch3_raw = citajPpmStabilne(3, 1500, ppmCh3, PPM_SKOK_CH3_US, PPM_POTVRDENIA_CH3);
  if (ch3_raw < 1000) ch3_raw = 1000;
  if (ch3_raw > 2000) ch3_raw = 2000;

  int ch3 = filterCH3(ch3_raw);

  unsigned long teraz_ms = millis();
  unsigned long teraz_us = micros();

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

  if (!zosuladene) {
    manualCielInicializovany = false;
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

    stepperEnable(false);
    return;
  }

  long ciel = mapLong(ch3, 1000, 2000, max_kroky, min_kroky);

  if (ciel < min_kroky) ciel = min_kroky;
  if (ciel > max_kroky) ciel = max_kroky;
  ciel = stabilizujManualnyCiel(ciel);

  long chyba = ciel - pozicia_kroky;

  if (abs(chyba) <= STEPPER_TARGET_DEADBAND_STEPS) {
    stepperEnable(false);
    return;
  }

  bool smer_k_max = (chyba > 0);
  stepperEnable(true);

  if (jeCasNaKrok(teraz_us)) {
    stepperKrok(smer_k_max);
  }
}

// ================================================================
// REŽIM AUTO / MANUAL
// ================================================================
void aktualizujRezim() {
  int ch5 = citajPpmStabilne(5, 1000, ppmCh5, PPM_SKOK_CH5_US, PPM_POTVRDENIA_CH5);
  unsigned long nowMs = millis();

  // 🔴 hysterézia (dve hranice)
  const int AUTO_ON = 1800;   // musí byť fakt hore
  const int AUTO_OFF = 1600;  // musí byť fakt dole

  if (ch5 >= AUTO_ON) {
    if (ch5_auto_on_od_ms == 0) ch5_auto_on_od_ms = nowMs;
  } else {
    ch5_auto_on_od_ms = 0;
  }

  if (ch5 <= AUTO_OFF) {
    if (ch5_auto_off_od_ms == 0) ch5_auto_off_od_ms = nowMs;
  } else {
    ch5_auto_off_od_ms = 0;
  }

  bool autoOnStable = ch5_auto_on_od_ms != 0 && (unsigned long)(nowMs - ch5_auto_on_od_ms) >= CH5_DEBOUNCE_MS;
  bool autoOffStable = ch5_auto_off_od_ms != 0 && (unsigned long)(nowMs - ch5_auto_off_od_ms) >= CH5_DEBOUNCE_MS;

  if (!autoMode && autoOnStable && rozsahOK && zosuladene) {
    autoMode = true;
    ch5_auto_on_od_ms = 0;
    if (!log_started) {
      log_index = 0;
      log_saved_count = 0;
      log_start_ms = millis();
      log_last_ms = millis();
      log_active = true;
      log_started = true;
      EEPROM.put(LOG_MAGIC_ADDR, LOG_MAGIC);
      EEPROM.put(LOG_COUNT_ADDR, log_saved_count);
      EEPROM.put(LOG_WRITE_INDEX_ADDR, log_index);
    } else {
      log_last_ms = millis();
      log_active = true;
    }
  } 
  else if (autoMode && autoOffStable) {
    autoMode = false;
    ch5_auto_off_od_ms = 0;
  }

  if (autoMode != lastAutoMode) {
    if (autoMode) {
      integral_e = 0.0f;
      depthHoldPaused = false;

      if (!hladinaVynulovana) {
        sensor.read();
        depth_offset = sensor.depth();
        depth_real = 0.0f;
        temp_real = sensor.temperature();
        lastTelemetrySensorReadMs = millis();
        hladinaVynulovana = true;
        aktualizujIbusCache();
#if !ENABLE_IBUS_TELEMETRY
        Serial.print("HLADINA VYNULOVANA, offset=");
        Serial.println(depth_offset, 4);
#endif
      }

      if (rozsahOK) {
        auto_base_kroky = pozicia_kroky;
        ciel_kroky_reg = pozicia_kroky;
      }

#if !ENABLE_IBUS_TELEMETRY
      Serial.println("PREPINAM NA AUTO PI");
#endif
    } else {
#if !ENABLE_IBUS_TELEMETRY
      Serial.println("PREPINAM NA MANUAL");
#endif
    }

    lastRegMs = millis();
    lastAutoMode = autoMode;
  }
}

// ================== SETUP / LOOP ==================
void setup() {
#if !ENABLE_IBUS_TELEMETRY
  Serial.begin(115200);
#endif

  bezpecneStavy();
  nacitajEEPROM();

  escL.attach(ESC_LEFT_PIN, ESC_MIN_US, ESC_MAX_US);
  escR.attach(ESC_RIGHT_PIN, ESC_MIN_US, ESC_MAX_US);
  escL.writeMicroseconds(ESC_MID_US);
  escR.writeMicroseconds(ESC_MID_US);
  delay(2000);

  ch3_raw = ppm.latestValidChannelValue(3, 1500);
  if (ch3_raw < 1000) ch3_raw = 1000;
  if (ch3_raw > 2000) ch3_raw = 2000;
  ch3_filt = ch3_raw;

  lastFiltMs = millis();
  lastStepUs = micros();
  lastRegMs  = millis();

  Wire.begin();

#if !IBUS_USE_DUMMY_VALUES
  while (!sensor.init()) {
#if !ENABLE_IBUS_TELEMETRY
    Serial.println("Init failed!");
#endif
    delay(2000);
  }

  sensor.setModel(MS5837::MS5837_02BA);
  sensor.setFluidDensity(997); // sladká voda
  citajTlakomer();
#else
  depth_real = 0.0f;
  temp_real = 22.5f;
#endif
  lastTelemetrySensorReadMs = millis();

#if ENABLE_IBUS_TELEMETRY
  IBusTelemetry.begin();
  IBusTelemetry.addSensor(IBUS_MEAS_TYPE_TEM);
  IBusTelemetry.addSensor(IBUS_MEAS_TYPE_EXTV);
  aktualizujIbusCache();
  IBusTelemetry.stopListening();
  ibusWindowActive = false;
#endif

  //vypisLog();
  //while(1);
}

void loop() {
  aktualizujPpmBezpecnost();

  if (!ppmRiadenieAktivne) {
    nastavBezpecneVystupy();
    aktualizujSenzorPreTelemetriu();
    obsluzIbusTelemetriu();
    return;
  }

  riadLED();
  riadESCmix();
  aktualizujRezim();

  if (autoMode) {
    riadStepperAutoPI();
  } else {
    riadStepperManual();
  }

  if (autoMode && log_active) {
    if (millis() - log_last_ms >= 2000) {
      log_last_ms = millis();

      EEPROM.put(LOG_START_ADDR + log_index * sizeof(float), depth_real);

      log_index++;
      if (log_index >= LOG_COUNT) {
        log_index = 0;
      }

      if (log_saved_count < LOG_COUNT) {
        log_saved_count++;
      }

      EEPROM.put(LOG_COUNT_ADDR, log_saved_count);
      EEPROM.put(LOG_WRITE_INDEX_ADDR, log_index);
    }
  }

  aktualizujSenzorPreTelemetriu();
  obsluzIbusTelemetriu();
}
