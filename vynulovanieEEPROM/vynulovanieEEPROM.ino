// FULL WIPE EEPROM – vymaže celé EEPROM do 0xFF
#include <EEPROM.h>

void setup() {
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.update(i, 0xFF); // alebo 0x00, je to jedno – hlavne nech to nie je tvoja „magic“ hodnota
  }
}

void loop() {}
