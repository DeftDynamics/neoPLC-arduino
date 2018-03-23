/***************************************************************************
nePLC-RAM Library

A library for M24M02 by ST

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
 

#ifndef NEORAM_H
#define NEORAM_H

#include "Arduino.h"
#include <Wire.h>

#define ACCESS_ADDRESS (0x50) // first page. total of four pages as 0x50, 0x51, 0x52, 0x53
#define ACCESS_IDENTIFIER (0x59) // will respond equivalently to 0x58, 0x59, 0x5A, 0x5B

class neoRAM
{  

public:
  neoRAM(uint8_t addr=ACCESS_ADDRESS);
  void begin(uint32_t dataSize);
  bool readLog(uint32_t sampleToRead, char* destination);
  bool addLog(char* buffer);
  void restartLog();
  uint32_t logSize();

  
private:
  void writeBytes(int deviceAddr, uint16_t registerAddr, char* buffer, uint16_t numBytes);
  void readRandomAddress(int deviceAddr, uint16_t registerAddr, int numBytes,char* destination);

  int _addr = ACCESS_ADDRESS;
  uint32_t _locationToWrite = 0x0000;
  uint32_t _logSize = 0;
  uint32_t _dataStructSize = 0;
  elapsedMillis timeSinceLastWrite = 0; 
  
};


#endif

