#include <IBusBM.h>
#include <Servo.h>

IBusBM IBusSensor;

// ---------- PINY Z TVOJHO ROV ----------
#define LED_PIN_PWM      5

#define STEPPER_STEP     7
#define STEPPER_DIR      10
#define STEPPER_EN       8

#define ESC_LEFT_PIN     9
#define ESC_RIGHT_PIN    6

Servo escL;
Servo escR;

const int ESC_MIN_US = 1000;
const int ESC_MID_US = 1500;
const int ESC_MAX_US = 2000;

unsigned long lastUpdateMs = 0;

// ------------------------------------------------
// Bezpecne stavy pre vsetky vystupy
// ------------------------------------------------
void nastavBezpecneStavy() {
  // LED mas aktivne LOW:
  // 255 = vypnute, 0 = naplno
  pinMode(LED_PIN_PWM, OUTPUT);
  analogWrite(LED_PIN_PWM, 255);

  // Stepper
  pinMode(STEPPER_STEP, OUTPUT);
  digitalWrite(STEPPER_STEP, LOW);

  pinMode(STEPPER_DIR, OUTPUT);
  digitalWrite(STEPPER_DIR, LOW);

  pinMode(STEPPER_EN, OUTPUT);

  // Dolezite:
  // HIGH = driver vypnuty = stepper sa nema hriat
  // LOW  = driver zapnuty
  digitalWrite(STEPPER_EN, HIGH);

  // ESC motory do neutralu
  escL.attach(ESC_LEFT_PIN, ESC_MIN_US, ESC_MAX_US);
  escR.attach(ESC_RIGHT_PIN, ESC_MIN_US, ESC_MAX_US);

  escL.writeMicroseconds(ESC_MID_US);
  escR.writeMicroseconds(ESC_MID_US);

  delay(2000);
}

void setup() {
  nastavBezpecneStavy();

  // i-BUS telemetria ide cez Arduino Serial:
  // D0/RX a D1/TX
  IBusSensor.begin(Serial);

  // Sensor 1: teplota
  IBusSensor.addSensor(IBUSS_TEMP);

  // Sensor 2: externé napätie
  // Pouzijeme ako hlbku:
  // 0.42 V = 0.42 m
  IBusSensor.addSensor(IBUSS_EXTV);
}

void loop() {
  // Pre istotu stale drzime bezpecne vystupy
  analogWrite(LED_PIN_PWM, 255);
  digitalWrite(STEPPER_STEP, LOW);
  digitalWrite(STEPPER_EN, HIGH);

  escL.writeMicroseconds(ESC_MID_US);
  escR.writeMicroseconds(ESC_MID_US);

  if (millis() - lastUpdateMs >= 500) {
    lastUpdateMs = millis();

    // Testovacie hodnoty
    float testTempC = 22.5;
    float testDepthM = 0.42;

    // FlySky TEMP:
    // hodnota = (teplota + 40) * 10
    int16_t ibusTemp = (int16_t)((testTempC + 40.0) * 10.0);

    // FlySky EXTV:
    // hodnota = volty * 100
    // 0.42 V = 42
    // My si to vysvetlime ako 0.42 m
    int16_t ibusDepthAsVoltage = (int16_t)(testDepthM * 100.0);

    IBusSensor.setSensorMeasurement(1, ibusTemp);
    IBusSensor.setSensorMeasurement(2, ibusDepthAsVoltage);
  }
}