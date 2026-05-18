// Diagnostika: FlySky PPM na D3 + iBUS telemetria na D0/D1.
// Ciel: zistit, ci Arduino UNO vie naraz citat PPM a posielat iBUS senzory.
//
// Test bez motorov, bez steppera, bez tlakomeru.
// LED na D5 reaguje na CH6.
// Vstavana LED D13 ukazuje, ci prichadza PPM signal.
// FlySky sensors list ma ukazat dummy TEMP=22.5 a EXTV=0.42.

#include <PPMReader.h>
#include <IBusBM.h>

// ---------- NASTAVENIE TESTU ----------
// 1 = pouzi IBusBM default timer, ako vo funkcnom testujemTelemetriu.ino
// 0 = vypni IBusBM timer a volaj IBusSensor.loop() rucne v hlavnom loop()
#define IBUS_USE_LIBRARY_TIMER 0

// ---------- PINY ----------
#define PPM_PIN          3
#define POCET_KANALOV    8

#define LED_PIN_PWM      5   // aktivne LOW: 255 vypnute, 0 naplno
#define STATUS_LED_PIN   13

// Bezpecne nastavime vystupy z hlavneho ROV kodu tak, aby nic nebezalo.
#define STEPPER_STEP     7
#define STEPPER_DIR      10
#define STEPPER_EN       8
#define ESC_LEFT_PIN     9
#define ESC_RIGHT_PIN    6

PPMReader ppm(PPM_PIN, POCET_KANALOV);
IBusBM IBusSensor;

unsigned long lastTelemetryUpdateMs = 0;
unsigned long lastStatusBlinkMs = 0;
bool statusBlinkState = false;

void nastavBezpecneStavy() {
  pinMode(LED_PIN_PWM, OUTPUT);
  analogWrite(LED_PIN_PWM, 255);

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  pinMode(STEPPER_STEP, OUTPUT);
  digitalWrite(STEPPER_STEP, LOW);

  pinMode(STEPPER_DIR, OUTPUT);
  digitalWrite(STEPPER_DIR, LOW);

  pinMode(STEPPER_EN, OUTPUT);
  digitalWrite(STEPPER_EN, HIGH); // HIGH = A4988 vypnuty

  pinMode(ESC_LEFT_PIN, OUTPUT);
  pinMode(ESC_RIGHT_PIN, OUTPUT);
  digitalWrite(ESC_LEFT_PIN, LOW);
  digitalWrite(ESC_RIGHT_PIN, LOW);
}

void riadLedZCh6() {
  int ch6 = ppm.latestValidChannelValue(6, 1500);

  if (ch6 <= 1300) {
    analogWrite(LED_PIN_PWM, 255);
  } else if (ch6 <= 1700) {
    analogWrite(LED_PIN_PWM, 128);
  } else {
    analogWrite(LED_PIN_PWM, 0);
  }
}

bool ppmVyzeraZive() {
  unsigned ch1 = ppm.rawChannelValue(1);
  unsigned ch2 = ppm.rawChannelValue(2);
  unsigned ch3 = ppm.rawChannelValue(3);
  return ch1 >= 900 && ch1 <= 2100 &&
         ch2 >= 900 && ch2 <= 2100 &&
         ch3 >= 900 && ch3 <= 2100;
}

void indikujPpmStav() {
  if (ppmVyzeraZive()) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    return;
  }

  if (millis() - lastStatusBlinkMs >= 300) {
    lastStatusBlinkMs = millis();
    statusBlinkState = !statusBlinkState;
    digitalWrite(STATUS_LED_PIN, statusBlinkState ? HIGH : LOW);
  }
}

void aktualizujTelemetriu() {
  if (millis() - lastTelemetryUpdateMs < 500) {
    return;
  }
  lastTelemetryUpdateMs = millis();

  float testTempC = 22.5f;
  float testDepthM = 0.42f;

  int16_t ibusTemp = (int16_t)((testTempC + 40.0f) * 10.0f);
  int16_t ibusDepthAsVoltage = (int16_t)(testDepthM * 100.0f);

  IBusSensor.setSensorMeasurement(1, ibusTemp);
  IBusSensor.setSensorMeasurement(2, ibusDepthAsVoltage);
}

void setup() {
  nastavBezpecneStavy();

#if IBUS_USE_LIBRARY_TIMER
  IBusSensor.begin(Serial);
#else
  IBusSensor.begin(Serial, IBUSBM_NOTIMER);
#endif

  IBusSensor.addSensor(IBUSS_TEMP);
  IBusSensor.addSensor(IBUSS_EXTV);
}

void loop() {
#if !IBUS_USE_LIBRARY_TIMER
  IBusSensor.loop();
#endif

  riadLedZCh6();
  indikujPpmStav();
  aktualizujTelemetriu();

#if !IBUS_USE_LIBRARY_TIMER
  IBusSensor.loop();
#endif
}
