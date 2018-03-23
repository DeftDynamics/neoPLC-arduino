/***************************************************************************
nePLC-ADC Library

A library for ADS1115 by Texas instruments

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
   must display the following acknowledgment:
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
#include "neoADC_P.h"
#include "Wire.h"

neoADC_P::neoADC_P(uint8_t addr){
   // constructor
   _addr = addr;
}

void neoADC_P::begin(uint8_t fsr, uint8_t dr, uint8_t mode){

  Wire.begin();
  
  if (fsr <= 0.256){
    PGA = FSR_0256;
    FSR = 0.256;
  } else if (fsr <= 0.512){
    PGA = FSR_0512;
    FSR = 0.512;
  } else if (fsr <= 1.024){
    PGA = FSR_1024;
    FSR = 1.024;
  } else if (fsr <= 2.048){
    PGA = FSR_2048;
    FSR = 2.048;
  } else if (fsr <= 4.096){
    PGA = FSR_4096;
    FSR = 4.096;
  } else {
    PGA = FSR_6144;
    FSR = 6.144;
  }
  
  if (dr <= 8) {
    DR = SPS_008;
    RATE = 8;
  } else if (dr <= 16){
    DR = SPS_016;
    RATE = 16;
  } else if (dr <= 32){
    DR = SPS_032;
    RATE = 32;
  } else if (dr <= 64){
    DR = SPS_064;
    RATE = 64;
  } else if (dr <= 128){
    DR = SPS_128;
    RATE = 128;
  } else if (dr <= 250){
    DR = SPS_250;
    RATE = 250;
  } else if (dr <= 475){
    DR = SPS_475;
    RATE = 475;
  } else {
    DR = SPS_860;
    RATE = 860;
  }

  MUX = 0;

  C_MODE = 0;
  C_POL = 0;
  C_LAT = 0;
  C_QUE = 0b11; // disable comparator

  MODE = mode;
  
  configure(OS,MUX,PGA,MODE,DR);

  //Serial.printf("ADS1115 setup:\n");
  //Serial.printf("> OS=%d, MUX=%d, PGA=%d, MODE=%d, DR=%d, fsr=%2.4f, rate=%d\n",OS,MUX,PGA,MODE,DR,FSR,RATE);
  //Serial.printf("> C_MODE=%d, C_POL=%d, C_LAT=%d, C_QUE=%d\n",C_MODE,C_POL,C_LAT,C_QUE);

  //readConfiguration();
  
}

void neoADC_P::configure(uint8_t OS, uint8_t MUX, uint8_t PGA, uint8_t MODE, uint8_t DR){
  
  uint8_t packageMSB = (OS<<7)|(MUX<<4)|(PGA<<1)|(MODE);
  uint8_t packageLSB = (DR<<5)|(C_MODE<<4)|(C_POL<<3)|(C_LAT<<2)|(C_QUE);

  // configure ADS1115
  Wire.beginTransmission(_addr);
  Wire.write(ADS_1115_CONF);
  Wire.write(packageMSB);
  Wire.write(packageLSB);
  Wire.endTransmission();
  
}

void neoADC_P::readConfiguration(){
  
  Wire.beginTransmission(_addr);
  Wire.write(ADS_1115_CONF);
  Wire.endTransmission();
  Wire.requestFrom(_addr,2);
  byte a = Wire.read();
  byte b = Wire.read();

  uint8_t   _OS = (a & 0b10000000)>>7;
  uint8_t  _MUX = (a & 0b01110000)>>4;
  uint8_t  _PGA = (a & 0b00001110)>>1;
  uint8_t _MODE = (a & 0b00000001)>>0;
  
  uint8_t     _DR = (b & 0b11100000)>>5;
  uint8_t _C_MODE = (b & 0b00010000)>>4;
  uint8_t  _C_POL = (b & 0b00001000)>>3;
  uint8_t  _C_LAT = (b & 0b00000100)>>2;
  uint8_t  _C_QUE = (b & 0b00000011)>>0;
  
  Serial.printf("read ADS1115 configuration:\n");
  Serial.printf("> OS=%d, MUX=%d, PGA=%d, MODE=%d, DR=%d\n\n",_OS,_MUX,_PGA,_MODE,_DR);
  Serial.printf("> C_MODE=%d, C_POL=%d, C_LAT=%d, C_QUE=%d\n",C_MODE,C_POL,C_LAT,C_QUE);
  
}

float neoADC_P::read(int AIN_P, int AIN_N){

    uint8_t old_mux = MUX;
    
    if (AIN_N < 0) {
      // single ended read of AIN_P pin
      MUX = AIN_P + 4;
    } else if ((AIN_P == 0)&&(AIN_N == 1)){
      MUX = AIN0_AIN1;
    } else if ((AIN_P == 0)&&(AIN_N == 3)){
      MUX = AIN0_AIN3;
    } else if ((AIN_P == 1)&&(AIN_N == 3)){
      MUX = AIN1_AIN3;
    } else if ((AIN_P == 2)&&(AIN_N == 3)){
      MUX = AIN2_AIN3;
    } else {
      // unknown configuration
      Serial.print("*** error - unknown pin configuration - will not be read ***\n");
      return 0.0;
    }

    if (MODE == CONTINUOUS){
      if (old_mux != MUX){
        configure(OS,MUX,PGA,MODE,DR);
        timeout = 0;
      }
    } else {
      OS = 1;
      configure(OS,MUX,PGA,MODE,DR);
      timeout = 0;
    }

    // go ahead and set the pointer in case we get delayed in a second
    Wire.beginTransmission(_addr);
    Wire.write((uint8_t)ADS_1115_CONV);
    Wire.endTransmission();

    while (timeout<=int(1+1000.0*2.0/((float)RATE))){
      // wait for 1/(sample rate) to ensure conversion occurs
    }
  
    // read conversion register
    Wire.requestFrom(_addr,2);
    byte a = Wire.read();
    byte b = Wire.read();
    int16_t val = ((a<<8)|b);
    float val2 = val*FSR/pow(2,15);

    return val2;
  
}
