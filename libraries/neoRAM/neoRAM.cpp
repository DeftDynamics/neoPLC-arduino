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

#include "Arduino.h"
#include "neoRAM.h"
#include "Wire.h"

neoRAM::neoRAM(uint8_t addr){
   // constructor
   _addr = addr;
}

void neoRAM::begin(uint32_t dataSize){
  _dataStructSize = dataSize;
  Wire.begin();
}

void neoRAM::restartLog(){
  _locationToWrite = 0x00000000;
  _logSize = 0;
}

uint32_t neoRAM::logSize(){
  return _logSize;
}

bool neoRAM::addLog(char* buffer){
  
  uint32_t endLocation = _locationToWrite+_dataStructSize;
  uint8_t currentAddressOffset = ((_locationToWrite & 0x00FF0000)>>16);
  uint8_t   finalAddressOffset = ((endLocation & 0x00FF0000)>>16);
  if (currentAddressOffset != finalAddressOffset){
    _locationToWrite = (_locationToWrite & 0x00FF0000) + 0x00010000;
    currentAddressOffset = ((_locationToWrite & 0x00FF0000)>>16);
    if (_locationToWrite > 0x0003FFFF){
       return false;
    }
  }

  writeBytes(_addr+currentAddressOffset,_locationToWrite,buffer,_dataStructSize);
  _logSize++;
  timeSinceLastWrite = 0;
  _locationToWrite = _locationToWrite+_dataStructSize;

  return true;
}


bool neoRAM::readLog(uint32_t sampleToRead, char* destination){
  uint32_t locationToRead = sampleToRead*_dataStructSize;
  uint32_t endLocation = locationToRead+_dataStructSize;
  uint8_t currentAddressOffset = ((locationToRead & 0x00FF0000)>>16);
  uint8_t   finalAddressOffset = ((endLocation & 0x00FF0000)>>16);
  if (currentAddressOffset != finalAddressOffset){
    locationToRead = (locationToRead & 0x00FF0000) + 0x00010000;
    currentAddressOffset = ((locationToRead & 0x00FF0000)>>16);
    if (locationToRead > 0x0003FFFF){
       return false;
    }
  }
  readRandomAddress(_addr+currentAddressOffset, locationToRead, _dataStructSize, destination);
  return true;
}

void neoRAM::readRandomAddress(int deviceAddr, uint16_t registerAddr, int numBytes, char* destination){
  
  //Serial.printf("read %d bytes from device %2X, address %2X:\n",numBytes,deviceAddr,registerAddr);
  
  bool Ack = 1;
  while ((timeSinceLastWrite <= 7)&&(Ack!=0)){
    // wait for write delay if we have recently written
    Wire.beginTransmission(deviceAddr);
    Wire.write((uint8_t)0x00);
    Ack = Wire.endTransmission();
    //delay(1);
  }
  
  Wire.beginTransmission(deviceAddr);
  Wire.write((uint8_t)(registerAddr>>8));
  Wire.write((uint8_t)(registerAddr & 0xFF));
  Wire.endTransmission(false);
  Wire.requestFrom(deviceAddr,numBytes);
  for (int i=0; i<numBytes; i++){
    char c = Wire.read();
    destination[i] = c;
    //Serial.printf(" %2X",c);
  }
  //Serial.println();
}

void neoRAM::writeBytes(int deviceAddr, uint16_t registerAddr, char* buffer, uint16_t numBytes){
  
  //Serial.printf("write %d bytes to device %2X, address %2X:\n",numBytes,deviceAddr,registerAddr);
  
  bool Ack = 1;
  while ((timeSinceLastWrite <= 7)&&(Ack!=0)){
    // wait for write delay if we have recently written
    Wire.beginTransmission(deviceAddr);
    Wire.write((uint8_t)0x00);
    Ack = Wire.endTransmission();
    //delay(1);
  }
  
  Wire.beginTransmission(deviceAddr);
  Wire.write((uint8_t)(registerAddr>>8));
  Wire.write((uint8_t)(registerAddr & 0xFF));
  for (int i; i<numBytes; i++){
    Wire.write((uint8_t)(*buffer));
    //Serial.printf(" %2X",(*buffer));
    *buffer++;
  }
  Wire.endTransmission(true);
  //Serial.println();
  
}
