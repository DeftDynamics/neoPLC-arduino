/***************************************************************************
nePLC-BAT Library

A library for MAX17043 by Maxim

Copyright (c) 2018, Deft Dynamics
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
 

#ifndef NEOBAT_H
#define NEOBAT_H

#include "Arduino.h"
#include <Wire.h>


class neoBAT
{  

public:
  neoBAT(uint8_t addr=0x36);
  void begin();
 float poll();
  void pollAll();
  void reset();

  // parse and print
 uint16_t VCELL;
    float vcell;
 uint16_t SOC;
    float soc;
 uint16_t version;
 uint16_t CONFIG;
  uint8_t RCOMP;
     bool sleepStatus;
     bool alertStatus;
  uint8_t ATHD;
  uint8_t athd;
  
	union {
	  char raw[20];
	  struct {
		 uint8_t header;      // alignment header
		 uint8_t ID;          // message ID
		   float voltage;
		   float charge;
		uint16_t reserve1;
		uint16_t reserve2;
		uint16_t reserve3;
		uint16_t reserve4;
		uint16_t reserve5;
	  } pcs;
	} DX;
	
	void updateDX();
  
private:
  int _addr = 0x36;
  float two8 = pow(2,8);
float two12 = pow(2,12);
float two16 = pow(2,16);
};


#endif

