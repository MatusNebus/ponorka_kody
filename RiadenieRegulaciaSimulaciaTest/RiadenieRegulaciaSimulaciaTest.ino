#include <PPMReader.h>
#include <Servo.h>
#include <EEPROM.h>

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
// PARAMETRE MODELU PONORKY
// ================================================================
const float rho  = 1000.0f;      // kg/m^3
const float g    = 9.81f;        // m/s^2
const float b    = 3.0f;         // N*s/m

// ---- Piest ----
const float Vpiestu_max_ml = 40.0f;
const float Vpiestu_max    = Vpiestu_max_ml * 1e-6f;   // m^3
const float Vpiestu_eq     = 20.0e-6f;                 // 20 ml = neutrál

// ---- Ponorka ----
const float Vpon = 0.003f;       // m^3 (3 litre)

// hmotnosť dopočítaná tak, aby bola neutrál pri 20 ml
const float m0 = rho * (Vpon - Vpiestu_eq);

// ================================================================
// PI REGULÁTOR
// ================================================================
const float Kp = 0.00018f; //0.00005f;
const float Ki = 0.0000001f; //0.00000002f;

const float depth_ref = 1.0f;      // chceme 1 meter
const float REG_TS    = 0.10f;     // 100 ms
const float DEADBAND_DEPTH = 0.05f; // 5 cm

float depth_sim = 0.0f;
float velocity_sim = 0.0f;
float integral_e = 0.0f;

unsigned long lastRegMs = 0;
long ciel_kroky_reg = 0;

// prepínanie režimov
bool autoMode = false;
bool lastAutoMode = false;

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
  pinMode(LED_PIN_PWM, OUTPUT);
  analogWrite(LED_PIN_PWM, 255);

  pinMode(STEPPER_STEP, OUTPUT);
  digitalWrite(STEPPER_STEP, LOW);

  pinMode(STEPPER_DIR, OUTPUT);
  digitalWrite(STEPPER_DIR, LOW);

  pinMode(STEPPER_EN, OUTPUT);
  digitalWrite(STEPPER_EN, HIGH);
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
// PREVOD KROKY <-> OBJEM
// ================================================================
float volumeFromSteps(long steps) {
  if (!rozsahOK) return Vpiestu_eq;

  long span = max_kroky - min_kroky;
  if (span <= 0) return Vpiestu_eq;

  float x = (float)(steps - min_kroky) / (float)span;
  if (x < 0.0f) x = 0.0f;
  if (x > 1.0f) x = 1.0f;

  return x * Vpiestu_max;
}

long stepsFromVolume(float V) {
  if (!rozsahOK) return pozicia_kroky;

  if (V < 0.0f) V = 0.0f;
  if (V > Vpiestu_max) V = Vpiestu_max;

  float x = V / Vpiestu_max;
  long steps = min_kroky + (long)(x * (float)(max_kroky - min_kroky));

  if (steps < min_kroky) steps = min_kroky;
  if (steps > max_kroky) steps = max_kroky;

  return steps;
}

// ================================================================
// SIMULÁCIA HĹBKY
// ================================================================
void updateDepthSimulation(float dt) {
  float Vvalca = volumeFromSteps(pozicia_kroky);

  float m  = m0 + rho * Vvalca;
  float Fg = m * g;
  float Fv = rho * g * Vpon;
  float Fd = b * velocity_sim;

  float a = (Fg - Fv - Fd) / m;

  velocity_sim += a * dt;
  depth_sim    += velocity_sim * dt;

  if (depth_sim < 0.0f) {
    depth_sim = 0.0f;
    if (velocity_sim < 0.0f) velocity_sim = 0.0f;
  }
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

    updateDepthSimulation(REG_TS);

    float e = depth_ref - depth_sim;
    if (e > -DEADBAND_DEPTH && e < DEADBAND_DEPTH) {
      integral_e = 0.0f; //anti windup
    }

    integral_e += e * REG_TS;

    if (integral_e > 2000.0f) integral_e = 2000.0f;
    if (integral_e < -2000.0f) integral_e = -2000.0f;

    float u = Kp * e + Ki * integral_e;
    // 🔴 LIMIT riadiaceho zásahu (kľúčové!)
    if (u > 10e-6f) u = 10e-6f;
    if (u < -10e-6f) u = -10e-6f;


    float Vtarget = Vpiestu_eq + u;
    if (Vtarget < 0.0f) Vtarget = 0.0f;
    if (Vtarget > Vpiestu_max) Vtarget = Vpiestu_max;

    ciel_kroky_reg = stepsFromVolume(Vtarget);

    // debug
    if (millis() - lastPrintMs >= 2000) {
      lastPrintMs = millis();

      Serial.print("AUTO, h=");
      Serial.print(depth_sim, 4);
      Serial.print(", href=");
      Serial.print(depth_ref, 4);
      Serial.print(", e=");
      Serial.print(e, 4);
      Serial.print(", V=");
      Serial.print(volumeFromSteps(pozicia_kroky), 7);
      Serial.print(", pos=");
      Serial.print(pozicia_kroky);
      Serial.print(", tgt=");
      Serial.println(ciel_kroky_reg);
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

  // 🔴 hysterézia (dve hranice)
  const int AUTO_ON  = 1800;  // musí byť fakt hore
  const int AUTO_OFF = 1600;  // musí byť fakt dole

  if (!autoMode && ch5 >= AUTO_ON) {
    autoMode = true;
  } 
  else if (autoMode && ch5 <= AUTO_OFF) {
    autoMode = false;
  }

  if (autoMode != lastAutoMode) {
    if (autoMode) {
      integral_e = 0.0f;
      depth_sim = 0.0f;
      velocity_sim = 0.0f;

      if (rozsahOK) {
        pozicia_kroky = stepsFromVolume(Vpiestu_eq);
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
}