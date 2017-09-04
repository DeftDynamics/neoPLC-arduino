// neoPLC-ADC librray

#include "Arduino.h"
#include "neoADC.h"
#include "Wire.h"

neoADC::neoADC(uint8_t addr){
   // constructor
   ads_address = addr;
}

void neoADC::begin(){
  Wire.begin();   // Initialize i2c
}

float neoADC::read(unsigned char channel, bool mode)
{
  unsigned char command = 0;
  unsigned int reading = 0;
  
  command = channels[channel];
  if (mode){
    command = command ^ 0x80;
  }
  if (ads_vref_int_enabled){ 
    command = command ^ 0x08;
  }
  command = command ^ 0x04; // keep the power on between reads

  Wire.beginTransmission(ads_address);
  Wire.write(command);
  Wire.endTransmission();
  Wire.requestFrom(ads_address, 2);   // Request 2 bytes from the ADC

  if(2 <= Wire.available())       // if two bytes were received
  {
    reading = Wire.read();
    reading = (reading << 8) | Wire.read();
  }
  return reading*(3.3/4096.0);
}
