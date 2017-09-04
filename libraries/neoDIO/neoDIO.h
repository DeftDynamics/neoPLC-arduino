// library for neoPLC-DIO

#ifndef NEODIO_H_
#define NEODIO_H_

#include "Arduino.h"
#include <Wire.h>

class neoDIO {
  
  public:
    neoDIO(uint8_t addr=0x27);
    void begin(void);
    void pinMode(uint8_t p, uint8_t d);
    void digitalWrite(uint8_t p, uint8_t d);
    void pullUp(uint8_t p, uint8_t d);
    uint8_t digitalRead(uint8_t p);

 private:
    uint8_t i2caddr = 0x27;
    uint8_t regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr);
    uint8_t readRegister(uint8_t addr);
    void writeRegister(uint8_t addr, uint8_t value);
    void updateRegisterBit(uint8_t p, uint8_t pValue, uint8_t Addr);
};

//// registers
#define MCP23017_IODIRA 0x00
#define MCP23017_IODIRB 0x01
#define MCP23017_IPOLB 0x03
#define MCP23017_GPINTENB 0x05
#define MCP23017_DEFVALB 0x07
#define MCP23017_INTCONB 0x09
#define MCP23017_IOCONB 0x0B
#define MCP23017_GPPUB 0x0D
#define MCP23017_INTFB 0x0F
#define MCP23017_INTCAPB 0x11
#define MCP23017_GPIOB 0x13
#define MCP23017_OLATB 0x15

#define MCP23017_INT_ERR 255

#endif
