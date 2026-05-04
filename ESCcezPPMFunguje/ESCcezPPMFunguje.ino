// ===============================================
// ESC na pine 6 cez CH2 (pravá páčka hore/dole)
// - PPM vstup: D3  (PPM SUM z prijímača, spoločná GND!)
// - ESC výstup: D6  (Servo signál 1000..2000 µs, stred 1500 µs = neutrál)
// - Bezpečné stavy: LED vypnutá (D5), stepper EN HIGH, druhý ESC pin LOW
// Knižnice: PPMReader, Servo
// ===============================================

#include <PPMReader.h>
#include <Servo.h>

// --------- PINY ---------
#define PPM_PIN           3     // PPM SUM z prijímača
#define POCET_KANALOV     8     // čítame do 8 kanálov
#define ESC_PIN           6     // náš ESC (motor) na D6

// umlčíme zvyšok (nepovinné, ale bezpečné)
#define LED_PIN_PWM       5     // ak máš LED driver na D5
#define ESC_DRUHY_PIN     9     // druhý ESC signál (ak je zapojený), držíme LOW
#define STEPPER_EN_PIN    8     // A4988 EN pin – HIGH = vypnutý driver

// --------- OBJEKTY ---------
PPMReader ppm(PPM_PIN, POCET_KANALOV);
Servo esc;

// --------- PREMENNÉ ---------
int plyn_ch2_us = 1500;     // hodnota z CH2 (1000..2000 µs)
int vystup_esc_us = 1500;   // čo posielame do ESC
const int mrtva_zona = 30;  // ±30 µs okolo stredu = stoj

void nastavBezpecneStavy() {
  // stepper vypnúť
  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, HIGH);

  // ak máš LED driver na D5, vypni ho
  pinMode(LED_PIN_PWM, OUTPUT);
  analogWrite(LED_PIN_PWM, 255);

  // druhý ESC signál (ak je), drž LOW
  pinMode(ESC_DRUHY_PIN, OUTPUT);
  digitalWrite(ESC_DRUHY_PIN, LOW);
}

void setup() {
  nastavBezpecneStavy();

  // pripoj ESC a pošli neutrál (arm/kalibrácia)
  esc.attach(ESC_PIN, 1000, 2000);
  esc.writeMicroseconds(1500);
  delay(2000); // nech si ESC pípne a prejde do neutrál režimu
}

void loop() {
  // 1) načítaj CH2 (pravá páčka hore/dole)
  plyn_ch2_us = ppm.latestValidChannelValue(2, 1500);

  // 2) orez na 1000..2000 a aplikuj mŕtvu zónu okolo 1500
  if (plyn_ch2_us < 1000) plyn_ch2_us = 1000;
  if (plyn_ch2_us > 2000) plyn_ch2_us = 2000;

  if (abs(plyn_ch2_us - 1500) <= mrtva_zona) {
    vystup_esc_us = 1500;              // stoj
  } else {
    vystup_esc_us = plyn_ch2_us;       // priamo prepusti hodnotu
  }

  // 3) pošli pulz do ESC (bidirectional ESC: <1500 = dozadu, >1500 = dopredu)
  esc.writeMicroseconds(vystup_esc_us);

  delay(5); // jemné vyhladenie
}
