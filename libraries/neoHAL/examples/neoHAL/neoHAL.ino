// neoHAL Test

#include "neoHAL.h"

neoHAL hal = neoHAL();

void setup() {
  Wire.begin();
  delay(2000);
  hal.begin();
  hal.setOrigin();
}

void loop() {
  int x = hal.status();
  if (x != MAGNET_OK){
    Serial.printf("magnet error #%d\n",x);
    delay(500);
  } else {
    uint16_t angle = hal.readAngle(); // this command also sets the value of hal.degrees and hall.rad
    Serial.printf("%2.3f\n",hal.degrees);
    delay(10);
  }
}





