// neoHAL Demo

#include <neoHAL.h>
#include <neoBLE.h>

neoHAL hal = neoHAL();

void setup() {
  delay(2000); // wait for serial to 'warm up'
  BLE.begin();
  hal.begin();
  hal.setOrigin(); // Set the origin to the currently measured angle relative to the IC
}

void loop() {
  
  int x = hal.status();
  if (x != MAGNET_OK){
    Serial.printf("magnet error #%d\n",x);
    BLE.post(hal.DX.raw); // standard DX message for streaming this sensor over BLE to neoPLC apps
    delay(500);
  } else {
    uint16_t angle = hal.readAngle(); // this command also sets the value of hal.degrees and hall.rad
    Serial.printf("%2.3f\n",hal.degrees);
    BLE.post(hal.DX.raw); // standard DX message for streaming this sensor over BLE to neoPLC apps
    delay(10);
  }
  
}
