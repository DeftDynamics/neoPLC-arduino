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
	float vref = 3.3;
  
	union {
	  char raw[20];
	  struct {
		 uint8_t header;      // alignment header
		 uint8_t ID;          // message ID
		 uint8_t active;
		 uint8_t mode;
		 int16_t V[8];
	  } pcs;
	} DX;
	
	void updateDX();

private:
	int address = 0x48;
	bool internalRef = false;
	unsigned char channels[8] = {0x00, 0x40, 0x10, 0x50, 0x20, 0x60, 0x30, 0x70}; 
};


#endif
