/***************************************************************************
nePLC-ADC Library

A library for ADS7828 by Texas instruments

Copyright (c) 2017, Deft Dynamics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by Deft Dynamics.
4. Neither the name of Deft Dynamics nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY DEFT DYNAMICS ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL DEFT DYNAMICS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************/

#include "Arduino.h"
#include "neoADC.h"
#include "Wire.h"

neoADC::neoADC(uint8_t addr){
   // constructor
   address = addr;
}

void neoADC::begin(){
  Wire.begin();
}

void neoADC::updateDX(){
	DX.pcs.header = 0xD0;
	DX.pcs.ID = 6;
	// other components are updated in 'read'
}

float neoADC::read(unsigned char channel, bool singleEnded)
{
  char command = 0;
  uint16_t reading = 0;
  
  command = channels[channel];
  if (singleEnded){
    command = command ^ 0x80;
  }
  if (internalRef){ 
    command = command ^ 0x08;
  }
  command = command ^ 0x04;           // keep power on between reads

  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.endTransmission();
  Wire.requestFrom(address, 2);   // Request 2 bytes from the ADC

  if (Wire.available() >= 2)
  {
    reading = Wire.read();
    reading = (reading << 8) | Wire.read();
  }
  
  float voltage = reading*(vref/4096.0);
  DX.pcs.V[channel] = reading;
  DX.pcs.active = DX.pcs.active | (0b1 << channel);
  if (singleEnded) {
	DX.pcs.mode = (DX.pcs.mode) | (0b1 << channel);
  } else {
	DX.pcs.mode = (DX.pcs.mode) & ~(0b1 << channel);
  }
  updateDX();
  
  return voltage;
}
