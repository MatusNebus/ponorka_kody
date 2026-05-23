#include <PPMReader.h>
#include <Servo.h>
#include <EEPROM.h>
#include <Wire.h>
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

#define IBUS_TELEMETRY_PIN 11  // main nepouziva iBUS, ale SENS je fyzicky zapojeny na D11

// ---------- LOGOVANIE ----------
const int LOG_MAGIC_ADDR = 88;
const int LOG_COUNT_ADDR = 92;
const int LOG_WRITE_INDEX_ADDR = 96;
const int LOG_START_ADDR = 100;   // kde začneme ukladať
const int LOG_COUNT = 100;        // 200 s / 2 s = 100 vzoriek
const int LOG_TEMP_START_ADDR = LOG_START_ADDR + LOG_COUNT * (int)sizeof(float);
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
const int MRTVA_ZONA_US   = 30;
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

// ---------- PPM ----------
PPMReader ppm(PPM_PIN, POCET_KANALOV);

// ================================================================
// PI REGULÁTOR
// ================================================================
const float Kp_steps = 1.20f; // cast rozsahu piestu na 1 m chyby
const float Ki_steps = 0.025f; // cast rozsahu piestu na 1 m*s integralnej chyby

const float depth_ref = 0.3f;        //ZELANA HLBKA oooooooooooooooooooooooooooooooooooooooooooo
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
  pinMode(IBUS_TELEMETRY_PIN, INPUT_PULLUP);

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

void nacitajEEPROM() {
  long magic = 0;
  EEPROM.get(EE_MAGIC_ADDR, magic);

  if (magic == EE_MAGIC) {
    EEPROM.get(EE_DATA_ADDR, min_kroky);
    EEPROM.get(EE_DATA_ADDR + (int)sizeof(long), max_kroky);

    if (max_kroky > min_kroky + 100) {
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
  int ch6 = ppm.latestValidChannelValue(6, 1500);

  if (ch6 <= CH6_OFF_MAX) {
    analogWrite(LED_PIN_PWM, 255);
  } else if (ch6 <= CH6_MID_MAX) {
    analogWrite(LED_PIN_PWM, 128);
  } else {
    analogWrite(LED_PIN_PWM, 0);
  }
}

// ---------- MIX pre 2 ESC ----------
void riadESCmix() {
  int ch1 = deadband1500(ppm.latestValidChannelValue(1, ESC_MID_US), MRTVA_ZONA_US);
  int ch2 = deadband1500(ppm.latestValidChannelValue(2, ESC_MID_US), MRTVA_ZONA_US);

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

  Serial.println("---- TEPLOTA ----");
  Serial.print("POCET=");
  Serial.println(count);

  for (int i = 0; i < count; i++) {
    int eeprom_index = i;
    if (count == LOG_COUNT) {
      eeprom_index = (write_index + i) % LOG_COUNT;
    }

    float t;
    EEPROM.get(LOG_TEMP_START_ADDR + eeprom_index * sizeof(float), t);
    if (i > 0) {
      Serial.print(",");
    }
    Serial.print(t, 4);
  }
  Serial.println();
}

// ---------- STEPPER MANUAL z CH3 ----------
void riadStepperManual() {
  ch3_raw = ppm.latestValidChannelValue(3, 1500);
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

  long chyba = ciel - pozicia_kroky;

  if (chyba == 0) {
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
  int ch5 = ppm.latestValidChannelValue(5, 1000);
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
        hladinaVynulovana = true;
        Serial.print("HLADINA VYNULOVANA, offset=");
        Serial.println(depth_offset, 4);
      }

      if (rozsahOK) {
        auto_base_kroky = pozicia_kroky;
        ciel_kroky_reg = pozicia_kroky;
      }

      Serial.println("PREPINAM NA AUTO PI");
    } else {
      Serial.println("PREPINAM NA MANUAL");
    }

    lastRegMs = millis();
    lastAutoMode = autoMode;
  }
}

// ================== SETUP / LOOP ==================
void setup() {
  Serial.begin(115200);

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

  while (!sensor.init()) {
    Serial.println("Init failed!");
    delay(2000);
  } 

  sensor.setModel(MS5837::MS5837_02BA);
  sensor.setFluidDensity(997); // sladká voda
  citajTlakomer();

  vypisLog();
  while(1);
}

void loop() {
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
      EEPROM.put(LOG_TEMP_START_ADDR + log_index * sizeof(float), temp_real);

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
}
