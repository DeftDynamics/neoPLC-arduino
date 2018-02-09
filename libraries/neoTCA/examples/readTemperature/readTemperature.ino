// neoTCA - thermocouple amplifier test

#include <neoTCA.h>
neoTCA tca = neoTCA();

void setup() {
  tca.begin();
  delay(1000);
}


void loop() {

  float temperature = tca.read();
  Serial.printf("Temp = %2.3f, refTemp = %2.3f\n",tca.temp,tca.refTemp);
  Serial.printf("FAULT = %d, SCV = %d, SCG = %d, OC = %d\n\n",tca.fault,tca.SCV,tca.SCG,tca.OC);
  delay(1000);
  
}


