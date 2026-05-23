#include "iBUSTelemetry.h"
#include <Servo.h>

// Dummy iBUS telemetria cez softverovy half-duplex pin D11.
// Zapojenie: FS-iA6B SENS signal -> Arduino D11, GND -> GND.
// Nepouziva D0/D1 ani Serial Monitor.

#define IBUS_TELEMETRY_PIN 11

#define LED_PIN_PWM      5
#define STEPPER_STEP     7
#define STEPPER_DIR      10
#define STEPPER_EN       8
#define ESC_LEFT_PIN     9
#define ESC_RIGHT_PIN    6

const int ESC_MIN_US = 1000;
const int ESC_MID_US = 1500;
const int ESC_MAX_US = 2000;

const unsigned long SENSOR_UPDATE_INTERVAL_MS = 500;
const unsigned long IBUS_WINDOW_PERIOD_MS = 1000;
const unsigned long IBUS_WINDOW_ACTIVE_MS = 120;

iBUSTelemetry IBusTelemetry(IBUS_TELEMETRY_PIN);
Servo escL;
Servo escR;

bool ibusWindowActive = false;
unsigned long lastSensorUpdateMs = 0;

float dummyTempC = 22.5f;
float dummyDepthM = 0.42f;

void nastavBezpecneStavy() {
  pinMode(LED_PIN_PWM, OUTPUT);
  analogWrite(LED_PIN_PWM, 255); // LED aktivne LOW: 255 = vypnute

  pinMode(STEPPER_STEP, OUTPUT);
  digitalWrite(STEPPER_STEP, LOW);

  pinMode(STEPPER_DIR, OUTPUT);
  digitalWrite(STEPPER_DIR, LOW);

  pinMode(STEPPER_EN, OUTPUT);
  digitalWrite(STEPPER_EN, HIGH); // HIGH = A4988 vypnuty

  escL.attach(ESC_LEFT_PIN, ESC_MIN_US, ESC_MAX_US);
  escR.attach(ESC_RIGHT_PIN, ESC_MIN_US, ESC_MAX_US);
  escL.writeMicroseconds(ESC_MID_US);
  escR.writeMicroseconds(ESC_MID_US);

  delay(2000);
}

void drzBezpecneStavy() {
  analogWrite(LED_PIN_PWM, 255);
  digitalWrite(STEPPER_STEP, LOW);
  digitalWrite(STEPPER_DIR, LOW);
  digitalWrite(STEPPER_EN, HIGH);
  escL.writeMicroseconds(ESC_MID_US);
  escR.writeMicroseconds(ESC_MID_US);
}

void aktualizujDummyHodnoty() {
  unsigned long nowMs = millis();
  if ((unsigned long)(nowMs - lastSensorUpdateMs) < SENSOR_UPDATE_INTERVAL_MS) {
    return;
  }
  lastSensorUpdateMs = nowMs;

  // Male zmeny hodnot, aby bolo na ovladaci vidiet, ze telemetria zije.
  float phase = (nowMs % 10000UL) / 10000.0f;
  phase = 0;
  dummyTempC = 22.0f + phase;          // 22.0 az 23.0 C
  dummyDepthM = 0.35f + 0.20f * phase; // 0.30 az 0.50 m

  IBusTelemetry.setSensorValueFP(1, dummyTempC);
  IBusTelemetry.setSensorValueFP(2, dummyDepthM);
}

void obsluzIbusTelemetriu() {
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
}

void setup() {
  nastavBezpecneStavy();

  IBusTelemetry.begin();
  IBusTelemetry.addSensor(IBUS_MEAS_TYPE_TEM);  // teplota
  IBusTelemetry.addSensor(IBUS_MEAS_TYPE_EXTV); // hlbka zobrazena ako external voltage

  IBusTelemetry.setSensorValueFP(1, dummyTempC);
  IBusTelemetry.setSensorValueFP(2, dummyDepthM);

  IBusTelemetry.stopListening();
  ibusWindowActive = false;
  lastSensorUpdateMs = millis();
}

void loop() {
  drzBezpecneStavy();
  aktualizujDummyHodnoty();
  obsluzIbusTelemetriu();
}
