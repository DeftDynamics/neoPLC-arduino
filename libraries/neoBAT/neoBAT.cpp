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

#include "Arduino.h"
#include "neoBAT.h"
#include "Wire.h"

neoBAT::neoBAT(uint8_t addr){
   // constructor
   _addr = addr;
}

void neoBAT::begin(){
  Wire.begin();
}

float neoBAT::poll(){ 
  // read only voltage and SoC
  Wire.beginTransmission(_addr);
  Wire.write(0x02);
  Wire.endTransmission(false);
  Wire.requestFrom(_addr,12);
  uint8_t buff[4];
  for (int i=0; i<4; i++){
    buff[i] = Wire.read();
  }
  VCELL = (buff[0]<<4) | (buff[1]>>4);
  vcell = ((float)VCELL/two12)*5.0;
    SOC = (buff[2]<<8) | buff[3];
    soc = ((float)SOC/two8)*1.0;
    
    return soc;
}


void neoBAT::pollAll(){   
  
  // read everything
  Wire.beginTransmission(_addr);
  Wire.write(0x02);
  Wire.endTransmission(false);
  Wire.requestFrom(_addr,12);
  uint8_t buff[12];
  for (int i=0; i<12; i++){
    buff[i] = Wire.read();
    //Serial.printf("%2X ",buff[i]);
  }
  //Serial.println();

  // parse
     VCELL = (buff[0]<<4) | (buff[1]>>4);
     vcell = ((float)VCELL/two12)*5.0;
       SOC = (buff[2]<<8) | buff[3];
       soc = ((float)SOC/two8)*1.0;
   version = (buff[6]<<8) | buff[7];
  // seems like CONFIG and COMMAND location might be switched?
    CONFIG = (buff[10]<<8) | buff[11];
  
  RCOMP = buff[10];
  sleepStatus = buff[11]>>7;
  alertStatus = buff[11]>>5 & 0b1;
   ATHD = buff[11] & 0b11111;
   athd = 32-ATHD;
}

void neoBAT::reset(){
  Wire.beginTransmission(_addr);
  uint8_t flag1 = 0x54;
  uint8_t flag2 = 0x00;
  Wire.write(0xFE);
  Wire.write(flag1);
  Wire.write(flag2);
  Wire.endTransmission(true);
  delay(250);
  pollAll();
}
