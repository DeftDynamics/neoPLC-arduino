// neoTCA - thermocouple amplifier demo

#include <neoTCA.h>
#include <neoBLE.h>

neoTCA tca = neoTCA();

void setup() {
  BLE.begin();
  tca.begin();
  delay(1000);
}


void loop() {

  float temperature = tca.read();
  Serial.printf("Temp = %2.3f, refTemp = %2.3f\n",tca.temp,tca.refTemp);
  Serial.printf("FAULT = %d, SCV = %d, SCG = %d, OC = %d\n\n",tca.fault,tca.SCV,tca.SCG,tca.OC);

  BLE.post(tca.DX.raw);
  
  delay(1000);
  
}

