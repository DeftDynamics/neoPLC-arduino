// neoPLC-ADC librray

#ifndef NEOADC_H
#define NEOADC_H

#include "Arduino.h"
#include <Wire.h>

#define EXT 0 // External VREF
#define INT 1 // Internal VREF

#define SD 1 // Single ended mode
#define DF 0 // Differential mode


class neoADC
{  

public:
  neoADC(uint8_t addr=0x48);
  void begin();
  float read(unsigned char channel, bool mode=1);
private:
  int ads_address = 0x48;
  bool ads_vref_int_enabled = 0;
  unsigned char channels[8] = {0x00, 0x40, 0x10, 0x50, 0x20, 0x60, 0x30, 0x70}; 
};


#endif
