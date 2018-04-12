/***************************************************************************
neoPLC-DIO Library

A library for MCP23017 by MicroChip Technologies

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

// elements of this library are based on code by Adafruit Industries,
// released under the following license:
/*************************************************** 
 This is a library for the MCP23017 i2c port expander
 These displays use I2C to communicate, 2 pins are required to
 interface
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!
 Written by Limor Fried/Ladyada for Adafruit Industries.
 BSD license, all text above must be included in any redistribution
 ****************************************************/
 

#include "Arduino.h"
#include "neoDIO.h"
#include <Wire.h>

neoDIO::neoDIO(uint8_t addr){
   // constructor
   i2caddr = addr;
}

void neoDIO::begin() {
	Wire.begin();
	writeRegister(MCP23017_IODIRA,0xff);
	writeRegister(MCP23017_IODIRB,0xff);
}

void neoDIO::updateDX(){
	DX.pcs.header = 0xD0;
	DX.pcs.ID = 7;
	DX.pcs.mode = mode;
	DX.pcs.state = state;
}

void neoDIO::pinMode(uint8_t p, uint8_t d) {
	bitWrite(mode,p,d==INPUT);
	updateRegisterBit(p,(d==INPUT),MCP23017_IODIRB);
	updateDX();
}

void neoDIO::digitalWrite(uint8_t pin, uint8_t d) {
	uint8_t gpio;
	gpio = readRegister(MCP23017_OLATB);  // read the current GPIO output latches
	bitWrite(gpio,pin,d);                 // set the pin and direction
	writeRegister(MCP23017_GPIOB,gpio);   // write the new GPIO
	bitWrite(state,pin,d);
	updateDX();
}

void neoDIO::pullUp(uint8_t p, uint8_t d) {
	updateRegisterBit(p,d,MCP23017_GPPUB);
}

uint8_t neoDIO::digitalRead(uint8_t pin) {
	uint8_t val = (readRegister(MCP23017_GPIOB) >> pin) & 0x1;
	bitWrite(state,pin,val);
	updateDX();
	return val;
}

// -----------------------------------------------------------------

uint8_t neoDIO::readRegister(uint8_t addr){
  Wire.beginTransmission(i2caddr);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(i2caddr, 1);
  return Wire.read();
}

void neoDIO::writeRegister(uint8_t regAddr, uint8_t regValue){
  Wire.beginTransmission(i2caddr);
  Wire.write(regAddr);
  Wire.write(regValue);
  Wire.endTransmission();
}

void neoDIO::updateRegisterBit(uint8_t pin, uint8_t pValue, uint8_t Addr) {
  uint8_t regValue;
  regValue = readRegister(Addr);
  bitWrite(regValue,pin,pValue);
  writeRegister(Addr,regValue);
}
