// neoPLC-ADC librray

#include "Arduino.h"
#include "neoADC.h"
#include "Wire.h"

int ads_address = 0x48;
bool ads_vref_int_enabled = 0;
unsigned char channels[8] = {0x00, 0x40, 0x10, 0x50, 0x20, 0x60, 0x30, 0x70}; 
neoADC::neoADC(){
   // constructor
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
