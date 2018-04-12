/***************************************************************************
neoPLC-PWM Library

A library for PCA9685 by NXP

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
/***************************************************************************
  This is a library for our Adafruit 16-channel PWM & Servo driver
  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!
  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
***************************************************************************/

#include "neoPWM.h"
#include <Wire.h>

neoPWM::neoPWM(uint8_t addr) {
  _i2caddr = addr;
}

void neoPWM::reset(void) {
 write8(PCA9685_MODE1, 0x0);
}

void neoPWM::begin(float freq) {
  Wire.begin();
  reset();
  setPWMFreq(freq);
}

void neoPWM::updateDX(){
	DX.pcs.header = 0xD0;
	DX.pcs.ID = 8;
	// other components are updated elsewhere
}


void neoPWM::setPWMFreq(float freq) {
  //Serial.print("Attempting to set freq ");
  //Serial.println(freq);
  freq = freq*0.9;  // Correct for overshoot in the frequency setting (see issue #11).
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;
  uint8_t prescale = floor(prescaleval + 0.5);
  
  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
  write8(PCA9685_MODE1, newmode); // go to sleep
  write8(PCA9685_PRESCALE, prescale); // set the prescaler
  write8(PCA9685_MODE1, oldmode);
  delay(5); 
  write8(PCA9685_MODE1, oldmode | 0xa1);  //  This sets the MODE1 register to turn on auto increment.
                                          // This is why the beginTransmission below was not working.
}

void neoPWM::setPWM(uint8_t num, uint16_t on, uint16_t off) {
  Wire.beginTransmission(_i2caddr);
  Wire.write(LED0_ON_L+4*num);
  Wire.write(on);
  Wire.write(on>>8);
  Wire.write(off);
  Wire.write(off>>8);
  Wire.endTransmission();
}

// Sets pin without having to deal with on/off tick placement and properly handles
// a zero value as completely off.  Optional invert parameter supports inverting
// the pulse for sinking to ground.  Val should be a value from 0 to 4095 inclusive.
void neoPWM::analogWrite(uint8_t num, uint16_t val, bool invert)
{
  DX.pcs.val[num] = val;
  updateDX();
  // Clamp value between 0 and 4095 inclusive.
  val = min(val, 4095);
  val = val + (512*num); //delete me
  val = val % 4096;		 //delete me
  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 4095) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else {
      setPWM(num, 0, 4095-val);
    }
  }
  else {
    if (val == 4095) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 0) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else {
      setPWM(num, (512*num), val); //fix me
    }
  }
}

void neoPWM::zeroAll()
{
    for(int i = 0; i<16; i++){
		setPWM(i, 0, 4096);
	}
	for(int i = 0; i<8; i++){
        DX.pcs.val[i] = 0;
	}
    updateDX();
}

void neoPWM::softPWM(uint16_t PWM_vals[8])
{
  // smooth the current flow by shifting periods of HIGH signal between channels
  int PWM_sum = 0;
  for (uint8_t i=0; i<8; i++){
    uint16_t off = PWM_sum;                 // off until end of earlier commands 
    PWM_sum = (PWM_sum+PWM_vals[i]) % 4095; // on until end of earlier commands + new command mod 1.0
    DX.pcs.val[i] = PWM_vals[i];
    setPWM(i,off,PWM_sum);
  }
  updateDX();
}

uint8_t neoPWM::read8(uint8_t addr) {
  Wire.beginTransmission(_i2caddr);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)_i2caddr, (uint8_t)1);
  return Wire.read();
}

void neoPWM::write8(uint8_t addr, uint8_t d) {
  Wire.beginTransmission(_i2caddr);
  Wire.write(addr);
  Wire.write(d);
  Wire.endTransmission();
}
