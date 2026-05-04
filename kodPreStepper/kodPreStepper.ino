// Nastavenie pinov
const int stepPin = 12;   // STEP pin na A4988
const int dirPin  = 13;   // DIR pin na A4988
const int enPin   = 8;    // ENABLE pin na A4988 (LOW = zapnutý, HIGH = vypnutý)

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);

  // Začneme s vypnutým motorom (oddýchuje)
  digitalWrite(enPin, HIGH);
}

void loop() {
  // ---- Otáčanie dopredu (napr. vysúvanie piestu) ----
  digitalWrite(enPin, LOW);      // zapni driver
  digitalWrite(dirPin, HIGH);    // nastav smer
  for (int i = 0; i < 1000; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);     // rýchlosť krokov
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
  digitalWrite(enPin, HIGH);     // vypni driver = motor oddychuje

  delay(5000); // pauza medzi pohybmi

  // ---- Otáčanie dozadu (napr. zasúvanie piestu) ----
  digitalWrite(enPin, LOW);      // zapni driver
  digitalWrite(dirPin, LOW);     // opačný smer
  for (int i = 0; i < 1000; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
  digitalWrite(enPin, HIGH);     // vypni driver = motor oddychuje

  delay(5000); // pauza medzi cyklami
}
