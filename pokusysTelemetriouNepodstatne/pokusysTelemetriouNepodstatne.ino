#define STEPPER_STEP 12
#define STEPPER_DIR  13
#define STEPPER_EN   8

void setup() {
  // Stepper výstupy do bezpečného stavu
  pinMode(STEPPER_STEP, OUTPUT);
  digitalWrite(STEPPER_STEP, LOW);

  pinMode(STEPPER_DIR, OUTPUT);
  digitalWrite(STEPPER_DIR, LOW);

  pinMode(STEPPER_EN, OUTPUT);
  digitalWrite(STEPPER_EN, HIGH);   // HIGH = driver vypnutý, motor sa nehreje

  // Test TX pinu
  pinMode(1, OUTPUT); // D1 / TX
}

void loop() {
  // Pre istotu stále držíme stepper vypnutý
  digitalWrite(STEPPER_STEP, LOW);
  digitalWrite(STEPPER_EN, HIGH);

  // Test D1/TX pinu
  digitalWrite(1, HIGH);
  delay(1000);

  digitalWrite(1, LOW);
  delay(1000);
}