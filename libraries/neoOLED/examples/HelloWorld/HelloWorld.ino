// neoOLED Library Test

#include "neoOLED.h"

neoOLED oled = neoOLED();

void setup() {            
  oled.begin();
  oled.setFont(System5x7);
  oled.setContrast(255);
  
  oled.clear();
  oled.println("neoPLC");
  oled.println("app + hw");  
  oled.println("ecosystem");  
}

void loop() {
   oled.setRow(4);
   oled.clearToEOL();
   oled.print("  ");
   oled.println((float)millis()/1000.0);
   delay(100);
}
