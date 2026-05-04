#include <SPI.h>
#include <stdlib.h>

// ================================================================
// SIMPLE ROV OSD FIRMWARE - MINIMAL STABLE VERSION
//
// Input cez Serial:
// D:0.42;T:22.1
//
// Zobrazenie vlavo hore:
// H:0.42M
// T:22.1C
//
// Poznatok z font-grid testu:
// - cisla, pismena a znaky ako . : - su ASCII
// - medzera nie je 0x20
// - prazdny znak / blank riesime hlavne cez DMM_CLEAR
// ================================================================

const byte MAX_CS_PIN = 6;
const unsigned long SERIAL_BAUD = 115200;

const bool USE_PAL = true;
const bool DEBUG_SERIAL = false;

// MAX7456 registre
const byte VM0_REG  = 0x00;
const byte DMM_REG  = 0x04;
const byte DMAH_REG = 0x05;
const byte DMAL_REG = 0x06;
const byte DMDI_REG = 0x07;

// MAX7456 bity
const byte VM0_SOFT_RESET = 0x02;
const byte VM0_OSD_ENABLE = 0x08;
const byte VM0_PAL        = 0x40;
const byte DMM_CLEAR      = 0x04;
const byte DMM_8BIT_MODE  = 0x40;

float depthM = 0.00;
float tempC  = 0.00;
bool haveData = false;

char rxBuffer[50];
byte rxIndex = 0;

// ------------------------------------------------
// SPI zapis do MAX7456 registra
// ------------------------------------------------
void maxWrite(byte reg, byte value) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(MAX_CS_PIN, LOW);
  SPI.transfer(reg);
  SPI.transfer(value);
  digitalWrite(MAX_CS_PIN, HIGH);
  SPI.endTransaction();
}

// ------------------------------------------------
// Inicializacia MAX7456
// ------------------------------------------------
void osdInit() {
  pinMode(MAX_CS_PIN, OUTPUT);
  digitalWrite(MAX_CS_PIN, HIGH);

  SPI.begin();
  delay(200);

  maxWrite(VM0_REG, VM0_SOFT_RESET);
  delay(500);

  maxWrite(DMM_REG, DMM_CLEAR);
  delay(100);

  if (USE_PAL) {
    maxWrite(VM0_REG, VM0_OSD_ENABLE | VM0_PAL);
  } else {
    maxWrite(VM0_REG, VM0_OSD_ENABLE);
  }

  delay(100);
}

// ------------------------------------------------
// Prevod znaku na kod vo fonte
// ------------------------------------------------
byte osdCode(char c) {
  if (c >= 'a' && c <= 'z') {
    c = c - 32;
  }

  // Pre istotu: ak by sme niekde pouzili medzeru, posli blank 0x00
  if (c == ' ') return 0x00;

  if (c >= 33 && c <= 126) {
    return (byte)c;
  }

  return 0x00;
}

// ------------------------------------------------
// Vymazanie celej OSD obrazovky hardverovym prikazom
// ------------------------------------------------
void osdClear() {
  maxWrite(DMM_REG, DMM_CLEAR);
  delay(30);
}

// ------------------------------------------------
// Zapis jedneho znaku presne na poziciu
// col: 0..29
// row: PAL 0..15
// ------------------------------------------------
void osdWriteChar(byte col, byte row, char c) {
  if (col > 29) return;
  if (row > 15) return;

  unsigned int address = row * 30 + col;

  maxWrite(DMM_REG, DMM_8BIT_MODE);
  maxWrite(DMAH_REG, (address >> 8) & 0x01);
  maxWrite(DMAL_REG, address & 0xFF);
  maxWrite(DMDI_REG, osdCode(c));
}

// ------------------------------------------------
// Zapis textu
// ------------------------------------------------
void osdPrint(byte col, byte row, const char *text) {
  byte x = col;

  while (*text && x < 30) {
    osdWriteChar(x, row, *text);
    x++;
    text++;
  }
}

// ------------------------------------------------
// Vykreslenie obrazovky
// ------------------------------------------------
void drawOsdScreen() {
  char depthStr[12];
  char tempStr[12];
  char line[20];

  // Toto odstrani stare zbytky ako AD4
  osdClear();

  if (!haveData) {
    osdPrint(1, 1, "H:--.--M");
    osdPrint(1, 2, "T:--.-C");
    return;
  }

  dtostrf(depthM, 5, 2, depthStr);  // napr. " 0.42" alebo "-0.15"
  dtostrf(tempC,  4, 1, tempStr);   // napr. "25.0"

  // Ziadne medzery v texte, aby sme sa vyhli problemom s fontom
  snprintf(line, sizeof(line), "H:%sM", depthStr);
  osdPrint(1, 1, line);

  snprintf(line, sizeof(line), "T:%sC", tempStr);
  osdPrint(1, 2, line);
}

// ------------------------------------------------
// Najde hodnotu za D: alebo T:
// ------------------------------------------------
float readValueAfterLetter(char *p) {
  while (*p && *p != ':' && *p != '=') {
    p++;
  }

  if (*p == ':' || *p == '=') {
    p++;
  }

  return atof(p);
}

// ------------------------------------------------
// Spracovanie riadku
// Ocakavany format: D:0.42;T:22.1
// ------------------------------------------------
void handleLine(char *line) {
  char *dPtr = strchr(line, 'D');
  if (!dPtr) dPtr = strchr(line, 'd');

  char *tPtr = strchr(line, 'T');
  if (!tPtr) tPtr = strchr(line, 't');

  if (dPtr && tPtr) {
    depthM = readValueAfterLetter(dPtr);
    tempC  = readValueAfterLetter(tPtr);
    haveData = true;

    drawOsdScreen();

    if (DEBUG_SERIAL) {
      Serial.print("OK H=");
      Serial.print(depthM, 2);
      Serial.print(" T=");
      Serial.println(tempC, 1);
    }
  } else {
    if (DEBUG_SERIAL) {
      Serial.println("BAD FORMAT");
    }
  }
}

// ------------------------------------------------
// Setup
// ------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  osdInit();
  drawOsdScreen();
}

// ------------------------------------------------
// Loop
// ------------------------------------------------
void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (rxIndex > 0) {
        rxBuffer[rxIndex] = '\0';
        handleLine(rxBuffer);
        rxIndex = 0;
      }
    } else {
      if (rxIndex < sizeof(rxBuffer) - 1) {
        rxBuffer[rxIndex++] = c;
      } else {
        rxIndex = 0;
      }
    }
  }
}