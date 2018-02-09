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

#include "Arduino.h"
#include "neoTCA.h"
#include "Wire.h"

neoTCA::neoTCA(uint8_t addr){
   // constructor
   _addr = addr;
}

void neoTCA::begin(){
  Wire.begin();
  bridgeInit();
  read();
}

float neoTCA::read()
{
  
  uint8_t Register = 0x00; // register to read
  uint8_t numBytes = 4;    // number of bytes to request
  uint8_t Data[numBytes];  // we'll store the response here
  
  // write 0s to the bridge
  uint8_t len = numBytes;
  
  Wire.beginTransmission(_addr);
  Wire.write(funID);
  // this will pass through to the SPI device:
  Wire.write(Register);
  while (len--) {
    Wire.write((uint8_t)0x00);
  }
  Wire.endTransmission();

  // wait for buffer to fill then read buffer
  delayMicroseconds(100+10*numBytes);
  readBufferBytes(Data,numBytes);

  // convert raw bytes from MAX31855K to data:
  
  int16_t TEMP = (Data[0]<<8) | (Data[1] & 0b11111100);
  temp = TEMP/16.0;
  int16_t iTEMP = (Data[2]<<8) | (Data[3] & 0b11110000);
  refTemp = iTEMP/255.0;
  fault = Data[1] & 0b01;
  SCV = (Data[3]>>2) & 0b01;
  SCG = (Data[3]>>1) & 0b01;
   OC = (Data[3]>>0) & 0b01;

  return temp;
}


void neoTCA::readBufferBytes(uint8_t* dest, uint8_t len)
{
  Wire.requestFrom(_addr, len+1);
  while (len--) {
    *dest++ = Wire.read();
  }
  Wire.endTransmission();
  delayMicroseconds(100);
}

void neoTCA::bridgeInit(bool order, uint8_t mode, uint8_t freq, uint8_t SSpin){
  
  Wire.begin();
  Wire.setClock(400000);
  funID = pow(2,SSpin); // create a mask for SS
  
  // SPI Settings
  
  // order
  // > 0b00: MSB first
  // > 0b01: LSB first
  // mode
  // > 0b00: CPOL=0, CPHA=0
  // > 0b01: CPOL=0, CPHA=1
  // > 0b10: CPOL=1, CPHA=0
  // > 0b11: CPOL=1, CPHA=1
  // frequency
  // > 0b00: 1843 kHz 
  // > 0b01:  461 kHz
  // > 0b10:  115 kHz
  // > 0b11:   58 kHz
  
  uint8_t package = 0x00;
  package |= order<<5;
  package |=  mode<<2;
  package |=  freq<<0;
  
  Wire.beginTransmission(_addr);
  Wire.write(0xF0);
  Wire.write(package);
  Wire.endTransmission();

}
