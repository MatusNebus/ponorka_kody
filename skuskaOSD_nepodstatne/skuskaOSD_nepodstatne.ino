//tetno kod je len na skusku ci osd funguje a ci ho viem programovat

void setup() {
  pinMode(13, OUTPUT);

  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("MINIMOSD SERIAL TEST OK");
  Serial.println("Send text and I will echo it back.");
}

void loop() {
  static unsigned long lastPrint = 0;
  static bool ledState = false;

  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    ledState = !ledState;
    digitalWrite(13, ledState);

    Serial.println("alive");
  }

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    Serial.print("RX: ");
    Serial.println(input);
  }
}