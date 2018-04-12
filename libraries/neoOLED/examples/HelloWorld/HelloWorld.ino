// neoOLED Library Demo

#include <neoOLED.h>
#include <neoBLE.h>

neoOLED oled = neoOLED();

void setup() {    
  BLE.begin();
  oled.begin();  
  oled.setFont(System5x7);

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
  copyToApp();
  delay(1000);    
  
}



void copyToApp(){
  // because there are 384 bytes in the OLED display, it requires 24 updates over BLE to transfer the image via bluetooth
  for (int i=0; i<24; i++){
    // faster but less reliable if we only update the pixels that have changed since the last update:
    //if (oled.updateDX(i)) {
    oled.updateDX(i);
    BLE.post(oled.DX.raw);
    delay(10);
    //}
  }
}