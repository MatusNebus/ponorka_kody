// ===============================================
// KALIBRÁCIA ROZSAHU STEPPERA: PPM (CH3) + Serial príkazy
// - CH3 (ľavá páčka) JOG: dole = k MIN, stred = STOP, hore = k MAX (konštantná rýchlosť)
// - Do Serial Monitora (115200) píš:
//     "min"  -> uloží aktuálnu polohu ako MIN (do RAM)
//     "max"  -> uloží aktuálnu polohu ako MAX (do RAM)
//     "save" -> zapíše MIN/MAX do EEPROM (ak sú v rozumnom odstupe).
//               Ak je MAX < MIN, automaticky ich prehodí.
//     "show" -> zobrazí hodnoty z RAM aj EEPROM
//     "clear"-> zmaže "magic" (ako reset kalibrácie)
// - Zapojenie: PPM SUM → D3, STEP D12, DIR D13, EN D8 (LOW = enable)
// - LED (D5, active-LOW) držíme zhasnutú, ESC piny (D6, D9) ticho
// ===============================================

#include <PPMReader.h>
#include <EEPROM.h>

// --- PINY ---
#define PPM_PIN        3
#define POCET_KANALOV  8
#define STEP_PIN      12
#define DIR_PIN       13
#define EN_PIN         8
#define LED_PIN_PWM    5   // active-LOW -> 255 = OFF
#define ESC_PIN_1      6
#define ESC_PIN_2      9

// --- JOG prahy a rýchlosť ---
const int CH_DOLU_MAX      = 1300;   // ≤ 1300 µs = smer k MIN
const int CH_STRED_MIN     = 1301;   // 1301..1700 µs = STOP
const int CH_STRED_MAX     = 1700;   // ≥ 1700 µs = smer k MAX
const int STEP_PULSE_US    = 3;      // šírka pulzu na STEP
const int KROK_INTERVAL_US = 1500;   // menšie = rýchlejšie (konštantná rýchlosť)

// --- EEPROM (adresy + "magic") ---
const int  EE_MAGIC_ADDR = 0;
const int  EE_DATA_ADDR  = 8;
const long EE_MAGIC      = 0x31504F49L; // značka našich dát

// --- PREMENNÉ ---
PPMReader ppm(PPM_PIN, POCET_KANALOV);

int  ch3_us = 1500;            // 1000..2000 µs z CH3
long pozicia_kroky = 0;        // náš čítač polohy (relatívny)
long min_kroky = 0;            // MIN v krokoch (RAM)
long max_kroky = 0;            // MAX v krokoch (RAM)
bool maMin = false, maMax = false;

unsigned long posledny_krok_us = 0;

// --- Pomocné ---
long absLong(long v) { return (v < 0) ? -v : v; }

void krok(bool smer_k_max) {
  digitalWrite(DIR_PIN, smer_k_max ? HIGH : LOW);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(STEP_PIN, LOW);
  pozicia_kroky += smer_k_max ? 1 : -1;
}

// --- Bezpečné stavy ---
void bezpecne() {
  pinMode(STEP_PIN, OUTPUT); digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN,  OUTPUT); digitalWrite(DIR_PIN,  LOW);
  pinMode(EN_PIN,   OUTPUT); digitalWrite(EN_PIN,   HIGH); // HIGH = vypnutý driver

  pinMode(LED_PIN_PWM, OUTPUT); analogWrite(LED_PIN_PWM, 255); // LED OFF (active-LOW)
  pinMode(ESC_PIN_1, OUTPUT); digitalWrite(ESC_PIN_1, LOW);
  pinMode(ESC_PIN_2, OUTPUT); digitalWrite(ESC_PIN_2, LOW);
}

// --- EEPROM I/O ---
void eepromSave() {
  EEPROM.put(EE_MAGIC_ADDR, EE_MAGIC);
  EEPROM.put(EE_DATA_ADDR, min_kroky);
  EEPROM.put(EE_DATA_ADDR + (int)sizeof(long), max_kroky);
}

void eepromShow() {
  long magic=0, emin=0, emax=0;
  EEPROM.get(EE_MAGIC_ADDR, magic);
  EEPROM.get(EE_DATA_ADDR, emin);
  EEPROM.get(EE_DATA_ADDR + (int)sizeof(long), emax);
  Serial.print(F("EEPROM magic=0x")); Serial.print(magic, HEX);
  Serial.print(F("  MIN=")); Serial.print(emin);
  Serial.print(F("  MAX=")); Serial.println(emax);
}

// --- Serial príkazy ---
String vstup = "";

void spracujPrikaz(String s) {
  s.trim();
  s.toLowerCase();

  if (s == "min") {
    min_kroky = pozicia_kroky; maMin = true;
    Serial.print(F("[OK] MIN ulozeny (RAM): ")); Serial.println(min_kroky);
  }
  else if (s == "max") {
    max_kroky = pozicia_kroky; maMax = true;
    Serial.print(F("[OK] MAX ulozeny (RAM): ")); Serial.println(max_kroky);
  }
  else if (s == "save") {
    long diff = absLong(max_kroky - min_kroky);
    if (maMin && maMax && diff > 100) {
      // ak sú obrátene, prehoď
      if (max_kroky < min_kroky) {
        long tmp = min_kroky; min_kroky = max_kroky; max_kroky = tmp;
        Serial.println(F("Pozn.: MAX < MIN -> prehodil som ich pred ulozenim."));
      }
      eepromSave();
      Serial.print(F("[OK] Ulozene do EEPROM. MIN=")); Serial.print(min_kroky);
      Serial.print(F("  MAX=")); Serial.println(max_kroky);
      eepromShow();
    } else {
      Serial.println(F("[ERR] Najprv nastav MIN a MAX (a nech maju rozumny rozdiel)."));
    }
  }
  else if (s == "show") {
    Serial.print(F("RAM: MIN=")); Serial.print(min_kroky);
    Serial.print(F("  MAX=")); Serial.println(max_kroky);
    eepromShow();
  }
  else if (s == "clear") {
    long nula = 0; EEPROM.put(EE_MAGIC_ADDR, nula);
    Serial.println(F("[OK] EEPROM magic zmazany."));
    eepromShow();
  }
  else if (s.length() > 0) {
    Serial.println(F("Prikazy: min | max | save | show | clear"));
  }
}

void setup() {
  bezpecne();
  Serial.begin(115200);
  Serial.println(F("\nKalibracia: CH3 jog (dole/STOP/hore). Napis 'min', potom 'max', nakoniec 'save'."));
  Serial.println(F("Prikazy: min | max | save | show | clear"));
}

void loop() {
  // --- JOG cez CH3: konštantná rýchlosť ---
  ch3_us = ppm.latestValidChannelValue(3, 1500);
  if (ch3_us < 1000) ch3_us = 1000;
  if (ch3_us > 2000) ch3_us = 2000;

  unsigned long teraz = micros();

  if (ch3_us <= CH_DOLU_MAX) {
    digitalWrite(EN_PIN, LOW); // zapni driver
    if (teraz - posledny_krok_us >= (unsigned long)KROK_INTERVAL_US) {
      posledny_krok_us = teraz;
      krok(false); // smer k MIN (k motoru)
    }
  } else if (ch3_us >= CH_STRED_MAX) {
    digitalWrite(EN_PIN, LOW);
    if (teraz - posledny_krok_us >= (unsigned long)KROK_INTERVAL_US) {
      posledny_krok_us = teraz;
      krok(true); // smer k MAX (od motora)
    }
  } else {
    digitalWrite(EN_PIN, HIGH); // STOP: vypni driver (nebude sa hriat)
  }

  // --- Serial príkazy ---
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      spracujPrikaz(vstup);
      vstup = "";
    } else {
      if (vstup.length() < 32) vstup += c; // jednoduchá ochrana
    }
  }
}
