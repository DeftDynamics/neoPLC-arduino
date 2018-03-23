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
 

#ifndef NEOADC_P_H
#define NEOADC_P_H

#include "Arduino.h"
#include <Wire.h>

// Register pointers
#define ADS_1115_CONV (0b00) // conversion
#define ADS_1115_CONF (0b01) // configuration
#define ADS_1115_LOTH (0b10) // low comparator threshold
#define ADS_1115_HITH (0b11) // high comparator threshold

// Mode
#define CONTINUOUS (0b0)
#define SINGLESHOT (0b1)

// Multiplex Configuration
#define AIN0_AIN1 (0b000) // default
#define AIN0_AIN3 (0b001)
#define AIN1_AIN3 (0b010)
#define AIN2_AIN3 (0b011)
#define  AIN0_GND (0b100)
#define  AIN1_GND (0b101)
#define  AIN2_GND (0b110)
#define  AIN3_GND (0b111)

#define FSR_6144 (0b000) // +/- 6.144 V
#define FSR_4096 (0b001) // +/- 4.096 V
#define FSR_2048 (0b010) // +/- 2.048 V
#define FSR_1024 (0b011) // +/- 1.024 V
#define FSR_0512 (0b100) // +/- 0.512 V
#define FSR_0256 (0b101) // +/- 0.256 V

#define SPS_008 (0b000) // 8 samples per second
#define SPS_016 (0b001)
#define SPS_032 (0b010)
#define SPS_064 (0b011)
#define SPS_128 (0b100)
#define SPS_250 (0b101)
#define SPS_475 (0b110)
#define SPS_860 (0b111)

class neoADC_P
{  

public:
  neoADC_P(uint8_t addr=0x48);
  void begin(uint8_t fsr=3.3, uint8_t dr=860, uint8_t mode=0);
  float read(int AIN_P, int AIN_N=-1);
  void readConfiguration();
private:
  void configure(uint8_t OS, uint8_t MUX, uint8_t PGA, uint8_t MODE, uint8_t DR);

  int _addr = 0x48;

  // configuration parameters
  uint8_t     OS = 0; // 0-device is converting, 1-device is not converting
  uint8_t    MUX = 0; // multiplex configuration
  uint8_t    PGA = 0; // Full Scale Range (FSR) of Programmable Gain Amp (PGA)
  uint8_t   MODE = 0; // 0 - continuous, 1-single shot
  uint8_t     DR = 0; // Data Rate (samples per second)
  uint8_t C_MODE = 0; // comparator mode: 0-traditional, 1-window
  uint8_t  C_POL = 0; // comparator polarity: 0 active high, 1-active low
  uint8_t  C_LAT = 0; // comparator latches: 0 non-latching, 1 latching
  uint8_t  C_QUE = 3; // comparator queue and disable: 00 assert after 1, 01 assert after 2, 10 assert after 4, 11 disable

  float FSR = 0.0;
  int RATE = 0;
  elapsedMillis timeout = 0;
  
};


#endif
