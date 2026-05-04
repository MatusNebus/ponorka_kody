#include <SPI.h>
#include <stdlib.h>

// ================================================================
// SIMPLE ROV OSD FIRMWARE FOR MINIMOSD
// Input cez Serial:
// D:0.42;T:22.1
//
// Zobrazenie:
// ROV SENSOR OSD
// DEPTH: 0.42 M
// TEMP : 22.1 C
// ================================================================

// Na vacsine MinimOSD dosiek je CS pin MAX7456 na D6.
// Ak neskor pri videu neuvidime text, prva vec bude skusit 10.
const byte MAX_CS_PIN = 6;

// Serial rychlost medzi PC/Arduinom a MinimOSD
const unsigned long SERIAL_BAUD = 115200;

// PAL je bezne v Europe. Ak budes mat NTSC kameru a text bude blbnut,
// zmen toto na false.
const bool USE_PAL = true;

// Debug vypisy do Serial Monitoru.
// Ked bude OSD napevno v ponorke a Arduino bude len posielat data,
// mozes to dat na false.
const bool DEBUG_SERIAL = true;

// MAX7456 registre
const byte VM0_REG  = 0x00;
const byte DMM_REG  = 0x04;
const byte DMAH_REG = 0x05;
const byte DMAL_REG = 0x06;
const byte DMDI_REG = 0x07;

// MAX7456 prikazy / bity
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

  // soft reset MAX7456
  maxWrite(VM0_REG, VM0_SOFT_RESET);
  delay(500);

  // clear screen
  maxWrite(DMM_REG, DMM_CLEAR);
  delay(50);

  // zapnut OSD
  if (USE_PAL) {
    maxWrite(VM0_REG, VM0_OSD_ENABLE | VM0_PAL);
  } else {
    maxWrite(VM0_REG, VM0_OSD_ENABLE);
  }

  delay(100);
}

// ------------------------------------------------
// Prevod ASCII znakov na default MAX7456 font adresy
// Funguje pre cisla, velke pismena a zakladnu interpunkciu.
// ------------------------------------------------
byte osdCode(char c) {
  if (c >= 'a' && c <= 'z') {
    c = c - 32; // lowercase -> uppercase
  }

  if (c == ' ') return 0x00;

  // MAX7456 default: 1..9 su 0x01..0x09, 0 je 0x0A
  if (c >= '1' && c <= '9') return c - '0';
  if (c == '0') return 0x0A;

  // MAX7456 default: A zacina na 0x0B
  if (c >= 'A' && c <= 'Z') return 0x0B + (c - 'A');

  if (c == '.') return 0x41;
  if (c == '?') return 0x42;
  if (c == ';') return 0x43;
  if (c == ':') return 0x44;
  if (c == ',') return 0x45;
  if (c == '\'') return 0x46;
  if (c == '/') return 0x47;
  if (c == '#') return 0x48;
  if (c == '-') return 0x49;
  if (c == '<') return 0x4A;
  if (c == '>') return 0x4B;
  if (c == '@') return 0x4C;

  return 0x00; // neznamy znak = medzera
}

// ------------------------------------------------
// Zapis jedneho znaku na poziciu
// col: 0..29
// row: PAL 0..15, NTSC 0..12
// ------------------------------------------------
void osdWriteChar(byte col, byte row, char c) {
  if (col > 29) return;

  unsigned int address = row * 30 + col;

  maxWrite(DMM_REG, DMM_8BIT_MODE);
  maxWrite(DMAH_REG, (address >> 8) & 0x01);
  maxWrite(DMAL_REG, address & 0xFF);
  maxWrite(DMDI_REG, osdCode(c));
}

// ------------------------------------------------
// Zapis textu na obrazovku
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
// Vymazanie celej obrazovky
// ------------------------------------------------
void osdClear() {
  maxWrite(DMM_REG, DMM_CLEAR);
  delay(30);
}

// ------------------------------------------------
// Vykreslenie obrazovky
// ------------------------------------------------
void drawOsdScreen() {
  char depthStr[12];
  char tempStr[12];
  char line[31];

  osdClear();

  osdPrint(2, 1, "ROV SENSOR OSD");

  if (!haveData) {
    osdPrint(2, 3, "WAITING DATA");
    osdPrint(2, 5, "SEND D:0.42;T:22.1");
    return;
  }

  dtostrf(depthM, 5, 2, depthStr); // napr. " 0.42"
  dtostrf(tempC,  5, 1, tempStr);  // napr. " 22.1"

  snprintf(line, sizeof(line), "DEPTH:%s M", depthStr);
  osdPrint(2, 3, line);

  snprintf(line, sizeof(line), "TEMP :%s C", tempStr);
  osdPrint(2, 5, line);
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
// Spracovanie jedneho riadku zo Serialu
// Očakavany format: D:0.42;T:22.1
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
      Serial.print("OK  depth=");
      Serial.print(depthM, 2);
      Serial.print(" m, temp=");
      Serial.print(tempC, 1);
      Serial.println(" C");
    }
  } else {
    if (DEBUG_SERIAL) {
      Serial.print("BAD FORMAT: ");
      Serial.println(line);
      Serial.println("Use format: D:0.42;T:22.1");
    }
  }
}

// ------------------------------------------------
// Setup
// ------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  if (DEBUG_SERIAL) {
    Serial.println();
    Serial.println("MINIMOSD ROV OSD START");
    Serial.println("Send format: D:0.42;T:22.1");
  }

  osdInit();
  drawOsdScreen();

  if (DEBUG_SERIAL) {
    Serial.println("OSD firmware running.");
  }
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
        if (DEBUG_SERIAL) {
          Serial.println("RX BUFFER OVERFLOW");
        }
      }
    }
  }
}