// neoOLED Library Test

#include "neoOLED.h"

neoOLED oled = neoOLED();

const char* fontName[] = {
  "Arial14",
  "Arial_bold_14",
  "Callibri11",
  "Callibri11_bold",
  "Callibri11_italic",
  "Callibri15",
  "Corsiva_12",
  "fixed_bold10x15",
  "font5x7",
  "font8x8",
  "Iain5x7",
  "lcd5x7",
  "Stang5x7",
  "System5x7",
  "TimesNewRoman16",
  "TimesNewRoman16_bold",
  "TimesNewRoman16_italic",
  "utf8font10x16",
  "Verdana12",
  "Verdana12_bold",
  "Verdana12_italic",
  "X11fixed7x14",
  "X11fixed7x14B",
  "ZevvPeep8x16" 
};
const uint8_t* fontList[] = {
  Arial14,
  Arial_bold_14,
  Callibri11,
  Callibri11_bold,
  Callibri11_italic,
  Callibri15,
  Corsiva_12,
  fixed_bold10x15,
  font5x7,
  font8x8,
  Iain5x7,
  lcd5x7,
  Stang5x7,
  System5x7,
  TimesNewRoman16,
  TimesNewRoman16_bold,
  TimesNewRoman16_italic,
  utf8font10x16,
  Verdana12,
  Verdana12_bold,
  Verdana12_italic,
  X11fixed7x14,
  X11fixed7x14B,
  ZevvPeep8x16
};
uint8_t nFont = sizeof(fontList)/sizeof(uint8_t*);

//------------------------------------------------------------------------------
void setup() {    
  oled.begin();  
}
void loop() {
  for (uint8_t i = 0; i < nFont; i++) {
    oled.setFont(System5x7);
    oled.clear();
    oled.println(fontName[i]);
    oled.println();  
    oled.setFont(fontList[i]);
    oled.println("*+,-./0123456789:");
    oled.println("abcdefghijklmno");
    oled.println("ABCDEFGHIJKLMNO");
    delay(2000);    
  }
  oled.clear();
  oled.print("Done!");
  delay(5000);    
}