// ===============================================
// LED cez trojpolohový prepínač na CH6 (OFF/50%/ON)
// - PPM vstup: D3 (PPM SUM z prijímača)  [PPMReader indexuje kanály 1..N]
// - LED PWM:   D5 -> EN/DIM na LED driveri
// - Ostatné piny umlčané (ESC LOW, stepper EN HIGH)
// Knižnica: PPMReader (Library Manager)
// ===============================================

#include <PPMReader.h>

// ---------- PINY ----------
#define PPM_PIN           3      // PPM SUM z prijímača (D3)
#define POCET_KANALOV     8      // čítame do 8 kanálov
#define LED_PIN           5      // PWM výstup na EN/DIM LED drivera

// Bezpečné piny (nech nič inde "nekope")
#define ESC_LAVY_PIN      9
#define ESC_PRAVY_PIN     6
#define STEPPER_EN_PIN    8

// ---------- NASTAVENIA ----------
#define LED_AKTIVNA_LOGIKA_HIGH  true
// true  = väčší PWM -> viac svetla (bežné drivery)
// false = invertovaná logika (ak máš active-LOW driver)

// Prahy pre CH6 (µs). FS-i6 dáva zvyčajne cca 1000 / 1500 / 2000.
// Zvolíme rozumné pásma, aby to „necvakalo“ okolo stredu.
#define PRAH_OFF_MAX      1300   // <= 1300 µs = OFF
#define PRAH_MID_MIN      1301   // 1301..1700 µs = 50%
#define PRAH_MID_MAX      1700
// > 1700 µs = ON

// ---------- OBJEKTY ----------
PPMReader ppm(PPM_PIN, POCET_KANALOV);

// ---------- PREMENNÉ ----------
int ch6_mikrosekundy = 1500; // hodnota z CH6 (1000..2000 µs)
int jas_pwm           = 0;   // 0..255 pre PWM

void nastavBezpecneStavy() {
  // Stepper EN HIGH = vypnutý driver
  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, HIGH);

  // ESC signály drž LOW (ESC nedostanú validný pulz)
  pinMode(ESC_LAVY_PIN, OUTPUT);  digitalWrite(ESC_LAVY_PIN, LOW);
  pinMode(ESC_PRAVY_PIN, OUTPUT); digitalWrite(ESC_PRAVY_PIN, LOW);

  // LED zhasnuté na štarte
  pinMode(LED_PIN, OUTPUT);
  analogWrite(LED_PIN, 0);
}

void setup() {
  Serial.begin(115200);     // voliteľné: na kontrolu
  nastavBezpecneStavy();
}

void loop() {
  // 1) Prečítaj CH6 (prepínač). PPMReader indexuje kanály od 1.
  ch6_mikrosekundy = ppm.latestValidChannelValue(6, 1500);

  // 2) Orez na 1000..2000 µs (pre istotu)
  if (ch6_mikrosekundy < 1000) ch6_mikrosekundy = 1000;
  if (ch6_mikrosekundy > 2000) ch6_mikrosekundy = 2000;

  // 3) Rozhodni podľa pásma: OFF / 50 % / ON
  if (ch6_mikrosekundy <= PRAH_OFF_MAX) {
    jas_pwm = 0;                   // OFF
  } else if (ch6_mikrosekundy <= PRAH_MID_MAX) {
    jas_pwm = 200;                 // 50 %
  } else {
    jas_pwm = 255;                 // ON (100 %)
  }

  // Invertuj, ak máš active-LOW driver
  if (!LED_AKTIVNA_LOGIKA_HIGH) {
    jas_pwm = 255 - jas_pwm;
  }

  // 4) Pošli PWM na LED driver
  analogWrite(LED_PIN, jas_pwm);

  // 5) (Voliteľné) Výpis pre kontrolu každých 500 ms
  static unsigned long poslednyVypis = 0;
  if (millis() - poslednyVypis > 500) {
    const char* stav =
      (jas_pwm == 0)   ? "OFF" :
      (jas_pwm == 128) ? "50%" : "ON";
    Serial.print("CH6 = "); Serial.print(ch6_mikrosekundy);
    Serial.print(" us  | LED = "); Serial.print(stav);
    Serial.print(" (PWM "); Serial.print(jas_pwm); Serial.println(")");
    poslednyVypis = millis();
  }

  delay(5);
}
