/***************************************************************************
nePLC-TCA Thermocouple Amplifier Library

A library for MAX31855K + SC18IS602B by Maxim + NXP

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
 

#ifndef NEOTCA_H
#define NEOTCA_H

#include "Arduino.h"
#include <Wire.h>


class neoTCA
{  

public:
  neoTCA(uint8_t addr=0x28);
  void begin();
  float read();
  float temp = 0;
  float refTemp = 0;
  bool fault = 1;
  bool SCV = 0;
  bool SCG = 0;
  bool  OC = 0;
  
  union {
    char raw[20];
    struct {
	   uint8_t header;      // alignment header
	   uint8_t ID;          // message ID
	   uint8_t reserve1;
	   uint8_t reserve2;
	   float temperature;
	   float refTemp;
	   bool fault;
	   bool vccShort;
	   bool gndShort;
	   bool openCircuit;
	   uint16_t reserve3;
	   uint16_t reserve4;
    } pcs;
  } DX;

  void updateDX();

private:
  uint8_t funID = 0;
  void readBufferBytes(uint8_t* dest, uint8_t len);
  void bridgeInit(bool order=0, uint8_t mode=0, uint8_t freq=0, uint8_t SSpin=0);
  int _addr = 0x28;

};


#endif

