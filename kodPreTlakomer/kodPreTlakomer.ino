#include <Wire.h>
#include "MS5837.h"   // BlueRobotics library

// --- Bezpečné stavy ostatných pinov (nič iné netestujeme) ---
const uint8_t PIN_ESC_LEFT      = 9;
const uint8_t PIN_ESC_RIGHT     = 6;
const uint8_t PIN_STEPPER_EN    = 8;   // LOW=enable, chceme OFF → HIGH
const uint8_t PIN_STEPPER_STEP  = 12;
const uint8_t PIN_STEPPER_DIR   = 13;
const uint8_t PIN_LED_PWM1      = 5;
const uint8_t PIN_LED_PWM2      = 10;  // ak náhodou máš LED ešte na D10
const uint8_t PIN_PPM           = 3;   // len vstup, nič s ním nerobíme

// --- MS5837 ---
MS5837 ms;
bool ms_ok = false;
const uint8_t MS5837_MODEL = MS5837::MS5837_02BA;

// Kalibrácia „povrchového“ tlaku
float P0_mbar = 1013.25f;
const int P0_SAMPLES = 20;
const float MBAR_PER_M = 98.0665f; // ~mbar na 1 meter hĺbky (sladká voda)

void safePins() {
  pinMode(PIN_ESC_LEFT,  OUTPUT); digitalWrite(PIN_ESC_LEFT,  LOW);
  pinMode(PIN_ESC_RIGHT, OUTPUT); digitalWrite(PIN_ESC_RIGHT, LOW);
  pinMode(PIN_STEPPER_EN,OUTPUT); digitalWrite(PIN_STEPPER_EN,HIGH);
  pinMode(PIN_STEPPER_STEP,OUTPUT); digitalWrite(PIN_STEPPER_STEP,LOW);
  pinMode(PIN_STEPPER_DIR, OUTPUT); digitalWrite(PIN_STEPPER_DIR, LOW);
  pinMode(PIN_LED_PWM1, OUTPUT);   analogWrite(PIN_LED_PWM1, 0);
  pinMode(PIN_LED_PWM2, OUTPUT);   digitalWrite(PIN_LED_PWM2, LOW);
  pinMode(PIN_PPM, INPUT);
}

void calibrateP0() {
  double sum = 0; int cnt = 0;
  for (int i = 0; i < P0_SAMPLES; i++) {
    ms.read();
    sum += ms.pressure(); // mbar
    cnt++;
    delay(50);
  }
  if (cnt > 0) P0_mbar = sum / cnt;
}

void setup() {
  safePins();

  Serial.begin(115200);
  Serial.println(F("MS5837 test (UNO + BlueRobotics lib)"));
  Serial.println(F("SDA=A4, SCL=A5, GND spolocna. Stlac 'r' pre re-kalibraciu P0."));

  Wire.begin();

  if (ms.init()) {
    ms_ok = true;
    ms.setModel(MS5837_MODEL);
    ms.setFluidDensity(997); // sladka voda
    delay(20);
    calibrateP0();
    Serial.print(F("Init OK, P0 = ")); Serial.print(P0_mbar, 2); Serial.println(F(" mbar"));
  } else {
    Serial.println(F("!! MS5837 init FAILED – skontroluj SDA(A4), SCL(A5), napajanie a GND"));
  }
}

void loop() {
  if (!ms_ok) { delay(500); return; }

  // umožni rýchlo prekalibrovať P0 stlačením 'r' v Serial Monitore
  if (Serial.available()) {
    int c = Serial.read();
    if (c == 'r' || c == 'R') { calibrateP0(); Serial.print(F("Re-cal P0 = ")); Serial.println(P0_mbar, 2); }
  }

  ms.read();
  float P_mbar = ms.pressure();     // absolútny tlak (mbar)
  float T_c    = ms.temperature();  // °C zo senzora
  float depth_calc = (P_mbar - P0_mbar) / MBAR_PER_M;
  if (depth_calc < 0) depth_calc = 0; // v meste/na vzduchu daj 0

  Serial.print(F("P=")); Serial.print(P_mbar, 2); Serial.print(F(" mbar | "));
  Serial.print(F("T=")); Serial.print(T_c, 2);    Serial.print(F(" C | "));
  Serial.print(F("D=")); Serial.print(depth_calc, 2); Serial.println(F(" m"));

  delay(2000); // ~5 Hz
}
