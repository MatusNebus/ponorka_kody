void setup() {
  // Spustenie sériovej komunikácie s OSD (musí byť rovnaký baudrate ako OSD!)
  Serial.begin(115200); // alebo 57600 ak si to nastavil inak
}

void loop() {
  float temp = 23.4;   // testovacia teplota
  float depth = 1.6;   // testovacia hĺbka

  // Pošli text do OSD
  Serial.print("TEMP: ");
  Serial.print(temp);
  Serial.println("°C");

  Serial.print("DEPTH: ");
  Serial.print(depth);
  Serial.println("m");

  delay(1000); // počkaj 1 sekundu
}
