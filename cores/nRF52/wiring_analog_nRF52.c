/*
  Copyright (c) 2014 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifdef NRF52

#include "nrf.h"

#include "Arduino.h"
#include "wiring_private.h"

#ifdef __cplusplus
extern "C" {
#endif

static uint32_t saadcReference = SAADC_CH_CONFIG_REFSEL_Internal;
static uint32_t saadcGain      = SAADC_CH_CONFIG_GAIN_Gain1_5;

NRF_PWM_Type* pwms[PWM_MODULE_COUNT] = {
  NRF_PWM0,
  NRF_PWM1,
  NRF_PWM2
};

struct PWMContext pwmContext[PWM_COUNT] = {
  { PIN_FREE, 0, 0 },
  { PIN_FREE, 1, 0 },
  { PIN_FREE, 2, 0 },
  { PIN_FREE, 3, 0 },
  { PIN_FREE, 0, 1 },
  { PIN_FREE, 1, 1 },
  { PIN_FREE, 2, 1 },
  { PIN_FREE, 3, 1 },
  { PIN_FREE, 0, 2 },
  { PIN_FREE, 1, 2 },
  { PIN_FREE, 2, 2 },
  { PIN_FREE, 3, 2 }
};

uint16_t pwmValue[PWM_MODULE_COUNT][PWM_CHANNEL_COUNT] = {0};

struct PWMStatus pwmStatus[PWM_MODULE_COUNT] = {0};

static int readResolution = 10;
static int writeResolution = 8;
static uint32_t halfAnalogWriteMax = 128; // default for 8b

void analogReadResolution( int res )
{
  readResolution = res;
}

void analogWriteResolution( int res )
{
  if ((res > 1) && (res < 17)) // up to 16b PWM resolution
  {
    writeResolution = res;
    halfAnalogWriteMax = (2^res) >> 1;
  }
  else //default to 8b for any invalid res values
  {
    writeResolution = 8;
    halfAnalogWriteMax = 128;
  }
}

static inline uint32_t mapResolution( uint32_t value, uint32_t from, uint32_t to )
{
  if ( from == to )
  {
    return value ;
  }

  if ( from > to )
  {
    return value >> (from-to) ;
  }
  else
  {
    return value << (to-from) ;
  }
}

/*
 * Internal Reference is at 1.0v
 * External Reference should be between 1v and VDDANA-0.6v=2.7v
 *
 * Warning : On Arduino Zero board the input/output voltage for SAMD21G18 is 3.3 volts maximum
 */
void analogReference( eAnalogReference ulMode )
{
  switch ( ulMode ) {
    case AR_DEFAULT:
    case AR_INTERNAL:
    default:
      saadcReference = SAADC_CH_CONFIG_REFSEL_Internal;
	  saadcGain      = SAADC_CH_CONFIG_GAIN_Gain1_5;
      break;

    case AR_VDD4:
      saadcReference = SAADC_CH_CONFIG_REFSEL_VDD1_4;
	  saadcGain      = SAADC_CH_CONFIG_GAIN_Gain1_4;
      break;
  }
}

uint32_t analogRead( uint32_t ulPin )
{
  uint32_t pin = SAADC_CH_PSELP_PSELP_NC;
  uint32_t saadcResolution;
  uint32_t resolution;
  int16_t value;

  if (ulPin >= PINS_COUNT) {
    return 0;
  }

  ulPin = g_ADigitalPinMap[ulPin];

  switch ( ulPin ) {
    case 2:
      pin = SAADC_CH_PSELP_PSELP_AnalogInput0;
      break;

    case 3:
      pin = SAADC_CH_PSELP_PSELP_AnalogInput1;
      break;

    case 4:
      pin = SAADC_CH_PSELP_PSELP_AnalogInput2;
      break;

    case 5:
      pin = SAADC_CH_PSELP_PSELP_AnalogInput3;
      break;

    case 28:
      pin = SAADC_CH_PSELP_PSELP_AnalogInput4;
      break;

    case 29:
      pin = SAADC_CH_PSELP_PSELP_AnalogInput5;
      break;

    case 30:
      pin = SAADC_CH_PSELP_PSELP_AnalogInput6;
      break;

    case 31:
      pin = SAADC_CH_PSELP_PSELP_AnalogInput7;
      break;

    default:
      return 0;
  }

  if (readResolution <= 8) {
    resolution = 8;
    saadcResolution = SAADC_RESOLUTION_VAL_8bit;
  } else if (readResolution <= 10) {
    resolution = 10;
    saadcResolution = SAADC_RESOLUTION_VAL_10bit;
  } else if (readResolution <= 12) {
    resolution = 12;
    saadcResolution = SAADC_RESOLUTION_VAL_12bit;
  } else {
    resolution = 14;
    saadcResolution = SAADC_RESOLUTION_VAL_14bit;
  }

  NRF_SAADC->RESOLUTION = saadcResolution;

  NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);
  for (int i = 0; i < 8; i++) {
    NRF_SAADC->CH[i].PSELN = SAADC_CH_PSELP_PSELP_NC;
    NRF_SAADC->CH[i].PSELP = SAADC_CH_PSELP_PSELP_NC;
  }
  NRF_SAADC->CH[0].CONFIG = ((SAADC_CH_CONFIG_RESP_Bypass   << SAADC_CH_CONFIG_RESP_Pos)   & SAADC_CH_CONFIG_RESP_Msk)
                            | ((SAADC_CH_CONFIG_RESP_Bypass   << SAADC_CH_CONFIG_RESN_Pos)   & SAADC_CH_CONFIG_RESN_Msk)
                            | ((SAADC_CH_CONFIG_GAIN_Gain1_4    << SAADC_CH_CONFIG_GAIN_Pos)   & SAADC_CH_CONFIG_GAIN_Msk)
                            | ((saadcReference                << SAADC_CH_CONFIG_REFSEL_Pos) & SAADC_CH_CONFIG_REFSEL_Msk)
                            | ((SAADC_CH_CONFIG_TACQ_3us      << SAADC_CH_CONFIG_TACQ_Pos)   & SAADC_CH_CONFIG_TACQ_Msk)
                            | ((SAADC_CH_CONFIG_MODE_SE       << SAADC_CH_CONFIG_MODE_Pos)   & SAADC_CH_CONFIG_MODE_Msk);
  NRF_SAADC->CH[0].PSELN = pin;
  NRF_SAADC->CH[0].PSELP = pin;


  NRF_SAADC->RESULT.PTR = (uint32_t)&value;
  NRF_SAADC->RESULT.MAXCNT = 1; // One sample

  NRF_SAADC->TASKS_START = 0x01UL;

  while (!NRF_SAADC->EVENTS_STARTED);
  NRF_SAADC->EVENTS_STARTED = 0x00UL;

  NRF_SAADC->TASKS_SAMPLE = 0x01UL;

  while (!NRF_SAADC->EVENTS_END);
  NRF_SAADC->EVENTS_END = 0x00UL;

  NRF_SAADC->TASKS_STOP = 0x01UL;

  while (!NRF_SAADC->EVENTS_STOPPED);
  NRF_SAADC->EVENTS_STOPPED = 0x00UL;

  if (value < 0) {
    value = 0;
  }

  NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos);

  return mapResolution(value, resolution, readResolution);
}

// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.
void analogWrite( uint32_t ulPin, uint32_t ulValue )
{
  if (ulPin >= PINS_COUNT) {
    return;
  }

  uint32_t ulPin_ = g_ADigitalPinMap[ulPin];

  // Turn off PWM if duty cycle == 0
  if (ulValue == 0){
    for (uint8_t i = 0; i < PWM_COUNT; i++){
      if (pwmContext[i].pin == ulPin_){
        pwmContext[i].pin = PIN_FREE;

        // Disable the PWM
        NRF_PWM_Type* pwm = pwms[pwmContext[i].module];
        // disconnect from the PWM channel
        pwm->PSEL.OUT[pwmContext[i].channel] = PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos;

        pwmStatus[pwmContext[i].module].numActive--;

        // Turn off the PWM module if no pwm channels are allocated
        if (pwmStatus[pwmContext[i].module].numActive == 0){
          pwm->ENABLE = (PWM_ENABLE_ENABLE_Disabled << PWM_ENABLE_ENABLE_Pos);
        }
        digitalWrite(ulPin, 0);
      }
    }
    return;
  }
  for (uint8_t i = 0; i < PWM_COUNT; i++){
    if (pwmContext[i].pin == PIN_FREE || pwmContext[i].pin == ulPin_){
      pwmContext[i].pin = ulPin_;
      pwmValue[pwmContext[i].module][pwmContext[i].channel] = ulValue | bit(15);
      // allocate the pwm channel
      NRF_PWM_Type* pwm = pwms[pwmContext[i].module];
      // if this is the first channel allocated to the module, turn on pwm
      if (pwmStatus[pwmContext[i].module].numActive == 0){
        pwm->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
        pwm->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_1;
        pwm->MODE = PWM_MODE_UPDOWN_Up;
        pwm->COUNTERTOP = (1 << writeResolution) - 1;
        pwm->LOOP = 0;
        pwm->DECODER = ((uint32_t)PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) | ((uint32_t)PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
      }
      
      pwm->PSEL.OUT[pwmContext[i].channel] = pwmContext[i].pin;
      
      pwm->SEQ[0].PTR = (uint32_t)&pwmValue[pwmContext[i].module];
      pwm->SEQ[0].CNT = 4;
      pwm->SEQ[0].REFRESH  = 1;
      pwm->SEQ[0].ENDDELAY = 0;
      pwm->TASKS_SEQSTART[0] = 1;
      
      pwmStatus[pwmContext[i].module].numActive++;
      return;
    }
  }
  // fallback to digitalWrite if no available PWM channel
  if (ulValue < halfAnalogWriteMax){
    digitalWrite(ulPin, LOW);
  }else{
    digitalWrite(ulPin, HIGH);
  }
}

#ifdef __cplusplus
}
#endif

#endif
