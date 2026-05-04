#include <SPI.h>
#include <max7456.h>

// CS/SS pin čipu MAX7456 (na MinimOSD býva D10)
Max7456 osd(10);

// Buffer na prichádzajúci riadok zo sériového portu
String line = "";

// Premenné, ktoré budeme zobrazovať
float depth_m = 0.0;
float temp_c  = 0.0;

// Pomocné: „vymaže“ text na riadku tým, že prepíše medzerami (šírka ~28 znakov)
void osdClearRow(uint8_t row) {
  static const char blanks[] = "                            "; // 28 medzier
  osd.print(blanks, 0, row, 0, 0);
}

// Vytlačí text na pozíciu (stĺpec, riadok)
void osdPrintXY(uint8_t col, uint8_t row, const String &s) {
  osd.print(s.c_str(), col, row, 0, 0); // blink=0, invert=0
}

// Prekreslenie „obrazovky“ (prepíšeme riadky medzerami a znova vypíšeme)
void redraw() {
  osdClearRow(1);
  osdClearRow(3);
  osdClearRow(4);

  osdPrintXY(2, 1,  "ROV STATUS");
  osdPrintXY(2, 3,  String("Hlbka:   ") + String(depth_m, 2) + " m");
  osdPrintXY(2, 4,  String("Teplota: ") + String(temp_c, 1) + " C");
}

// Očakáva riadok vo formáte:  D:12.34,T:18.7
void parseLine(const String &s) {
  int dpos = s.indexOf("D:");
  int tpos = s.indexOf("T:");
  if (dpos >= 0) {
    int comma = s.indexOf(',', dpos);
    String ds = (comma > 0) ? s.substring(dpos + 2, comma) : s.substring(dpos + 2);
    depth_m = ds.toFloat();
  }
  if (tpos >= 0) {
    int comma = s.indexOf(',', tpos);
    String ts = (comma > 0) ? s.substring(tpos + 2, comma) : s.substring(tpos + 2);
    temp_c = ts.toFloat();
  }
  redraw();
}

void setup() {
  Serial.begin(115200);  // MUSÍ sedieť s Arduinom odosielajúcim dáta
  // Niektoré verzie tejto knižnice nevyžadujú žiadne osd.begin().
  // Ak by si v príkladoch knižnice videl metódu init()/start(), môžeš ju sem doplniť.
  redraw(); // zobraz úvodné hodnoty
}

unsigned long lastRefresh = 0;

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length() > 0) { parseLine(line); line = ""; }
    } else {
      line += c;
      if (line.length() > 64) line = ""; // jednoduchá ochrana
    }
  }

  // Občas obnov riadky (ak by nové dáta dlho neprišli)
  if (millis() - lastRefresh > 2000) {
    redraw();
    lastRefresh = millis();
  }
}
