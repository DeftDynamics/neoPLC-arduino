// library for neoPLC-DIO
// adapted from code by adafruit

#include "Arduino.h"
#include "neoDIO.h"
#include <Wire.h>

neoDIO::neoDIO(){
   // constructor
}

void neoDIO::begin() {
	i2caddr = MCP23017_ADDRESS;
	Wire.begin();
	writeRegister(MCP23017_IODIRA,0xff);
	writeRegister(MCP23017_IODIRB,0xff);
}

void neoDIO::pinMode(uint8_t p, uint8_t d) {
	updateRegisterBit(p,(d==INPUT),MCP23017_IODIRB);
}

void neoDIO::digitalWrite(uint8_t pin, uint8_t d) {
	uint8_t gpio;
	gpio = readRegister(MCP23017_OLATB);  // read the current GPIO output latches
	bitWrite(gpio,pin,d);                 // set the pin and direction
	writeRegister(MCP23017_GPIOB,gpio);   // write the new GPIO
}

void neoDIO::pullUp(uint8_t p, uint8_t d) {
	updateRegisterBit(p,d,MCP23017_GPPUB);
}

uint8_t neoDIO::digitalRead(uint8_t pin) {
	return (readRegister(MCP23017_GPIOB) >> pin) & 0x1;
}

// -----------------------------------------------------------------

uint8_t neoDIO::readRegister(uint8_t addr){
  Wire.beginTransmission(MCP23017_ADDRESS);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(MCP23017_ADDRESS, 1);
  return Wire.read();
}

void neoDIO::writeRegister(uint8_t regAddr, uint8_t regValue){
  Wire.beginTransmission(MCP23017_ADDRESS);
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
