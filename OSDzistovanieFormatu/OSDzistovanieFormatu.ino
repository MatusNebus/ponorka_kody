#include <SPI.h>

// ================================================================
// MINIMOSD / MAX7456 FONT GRID DIAGNOSTIC
//
// Tento kod nezobrazuje hlbku ani teplotu.
// Zobrazi surove znaky z MAX7456 font pamate.
//
// Serial Monitor commands:
// 0 = page 0, znaky 0x00 - 0x3F
// 1 = page 1, znaky 0x40 - 0x7F
// 2 = page 2, znaky 0x80 - 0xBF
// 3 = page 3, znaky 0xC0 - 0xFF
// n = dalsia strana
// p = PAL rezim
// t = NTSC rezim
// ================================================================

#include <SPI.h>

const byte MAX_CS_PIN = 6;
const unsigned long SERIAL_BAUD = 115200;

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

byte currentPage = 0;
bool usePAL = true;

// ------------------------------------------------
// Zapis do registra MAX7456
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
// Zapnutie PAL/NTSC + OSD
// ------------------------------------------------
void setVideoMode() {
  if (usePAL) {
    maxWrite(VM0_REG, VM0_OSD_ENABLE | VM0_PAL);
  } else {
    maxWrite(VM0_REG, VM0_OSD_ENABLE);
  }
}

// ------------------------------------------------
// Inicializacia OSD cipu
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

  setVideoMode();
  delay(100);
}

// ------------------------------------------------
// Vymazanie obrazovky
// ------------------------------------------------
void osdClear() {
  maxWrite(DMM_REG, DMM_CLEAR);
  delay(80);
}

// ------------------------------------------------
// Zapis suroveho znaku na poziciu
// col: 0..29
// row: PAL 0..15
// rawCode: adresa znaku v MAX7456 font pamati
// ------------------------------------------------
void osdWriteRaw(byte col, byte row, byte rawCode) {
  if (col > 29) return;

  unsigned int address = row * 30 + col;

  maxWrite(DMM_REG, DMM_8BIT_MODE);
  maxWrite(DMAH_REG, (address >> 8) & 0x01);
  maxWrite(DMAL_REG, address & 0xFF);
  maxWrite(DMDI_REG, rawCode);
}

// ------------------------------------------------
// Vyplni obrazovku medzerami = raw code 0x00
// ------------------------------------------------
void clearByWritingSpaces() {
  for (byte row = 0; row < 16; row++) {
    for (byte col = 0; col < 30; col++) {
      osdWriteRaw(col, row, 0x00);
    }
  }
}

// ------------------------------------------------
// Zobrazi 8x8 grid znakov
//
// Strana 0:
// 0x00 0x01 0x02 ... 0x07
// 0x08 0x09 ...
//
// Strana 1:
// 0x40 0x41 ...
//
// atd.
// ------------------------------------------------
void drawPage(byte page) {
  if (page > 3) page = 0;
  currentPage = page;

  osdClear();
  clearByWritingSpaces();

  byte baseCode = page * 64;

  // Grid je v strede obrazu.
  // 8 stlpcov, 8 riadkov.
  // Kazdy znak je od seba vzdialeny 3 OSD bunky.
  const byte startCol = 4;
  const byte startRow = 4;
  const byte colStep = 3;
  const byte rowStep = 1;

  byte code = baseCode;

  for (byte r = 0; r < 8; r++) {
    for (byte c = 0; c < 8; c++) {
      byte col = startCol + c * colStep;
      byte row = startRow + r * rowStep;

      osdWriteRaw(col, row, code);
      code++;
    }
  }

  Serial.print("SHOWING PAGE ");
  Serial.print(page);
  Serial.print("  RANGE 0x");

  if (baseCode < 16) Serial.print("0");
  Serial.print(baseCode, HEX);

  Serial.print(" - 0x");

  byte endCode = baseCode + 63;
  if (endCode < 16) Serial.print("0");
  Serial.println(endCode, HEX);

  Serial.println("Send 0,1,2,3 or n.");
}

// ------------------------------------------------
// Setup
// ------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println();
  Serial.println("MINIMOSD FONT GRID DIAGNOSTIC");
  Serial.println("Commands:");
  Serial.println("0 = 0x00-0x3F");
  Serial.println("1 = 0x40-0x7F");
  Serial.println("2 = 0x80-0xBF");
  Serial.println("3 = 0xC0-0xFF");
  Serial.println("n = next page");
  Serial.println("p = PAL");
  Serial.println("t = NTSC");

  osdInit();
  drawPage(0);
}

// ------------------------------------------------
// Loop
// ------------------------------------------------
void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if (c == '0') {
      drawPage(0);
    } 
    else if (c == '1') {
      drawPage(1);
    } 
    else if (c == '2') {
      drawPage(2);
    } 
    else if (c == '3') {
      drawPage(3);
    } 
    else if (c == 'n' || c == 'N') {
      currentPage++;
      if (currentPage > 3) currentPage = 0;
      drawPage(currentPage);
    } 
    else if (c == 'p' || c == 'P') {
      usePAL = true;
      setVideoMode();
      drawPage(currentPage);
      Serial.println("PAL mode set.");
    } 
    else if (c == 't' || c == 'T') {
      usePAL = false;
      setVideoMode();
      drawPage(currentPage);
      Serial.println("NTSC mode set.");
    }
  }
}