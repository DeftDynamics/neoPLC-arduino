// neoPLC-PWM Library

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
}

void neoPWM::softPWM(uint16_t PWM_vals[8])
{
  // smooth the current flow by shifting periods of HIGH signal between channels
  int PWM_sum = 0;
  for (uint8_t i=0; i<8; i++){
    uint16_t off = PWM_sum;                 // off until end of earlier commands 
    PWM_sum = (PWM_sum+PWM_vals[i]) % 4095; // on until end of earlier commands + new command mod 1.0
    setPWM(i,off,PWM_sum);
  }
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
