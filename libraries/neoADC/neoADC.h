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
  neoADC();
  void begin();
  float read(unsigned char channel, bool mode=1);
};


#endif
