/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

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

#include "nrf.h"

#include "delay.h"
#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

static volatile uint32_t overflows = 0;
    static uint32_t previousTicks = -1;
    
uint32_t millis( void )
{
  uint64_t ticks = (uint64_t)((uint64_t)overflows << (uint64_t)24) | (uint64_t)(NRF_RTC1->COUNTER);
  // since ticks will be too low if read during the Overflow Event, make sure each time we read the value
  // is increasing, or otherwise check it again.
  if (ticks<previousTicks){
      ticks = (uint64_t)((uint64_t)overflows << (uint64_t)24) | (uint64_t)(NRF_RTC1->COUNTER);
  }
  previousTicks = ticks;
  return (ticks * 1000) / 32768;
}

uint32_t micros( void )
{
  uint64_t ticks = (uint64_t)((uint64_t)overflows << (uint64_t)24) | (uint64_t)(NRF_RTC1->COUNTER);
  if (ticks<previousTicks){
    ticks = (uint64_t)((uint64_t)overflows << (uint64_t)24) | (uint64_t)(NRF_RTC1->COUNTER);
  }
  previousTicks = ticks;
  return (ticks * 1000000) / 32768;
}

void delay( uint32_t ms )
{
  if ( ms == 0 )
  {
    return ;
  }
    
    
    // use micros() to count not millis() unless the desired delay is greater than 10 minutes.
    if (ms>600000){
        uint32_t start = millis();
        do
        {
            yield() ;
        } while ( millis()-start < (ms));
    } else {
        uint32_t start = micros();
        do
        {
            yield() ;
        } while ( micros()-start < (ms*1000));
    }
    
    /*
    // original:
    uint32_t start = millis();
    do
    {
        yield() ;
    } while ( millis()-start < ms);
    */
    
}
    
    void delayMicroseconds( uint32_t us )
    {
        if ( us == 0 )
        {
          // if zero, immediately return
          return ;
        } else if (us < 150) {
			//9600 Baud is 104us delay, need to use high res timing, so this value must be above 104.
          // if small, use the high resolution but innaccurate built-in delay using NOPs
            nrf_delay_us(us);
        } else {
          // if large, count microseconds using RTC1
          uint32_t start = micros();
          do
          {
              yield() ;
          } while ( micros()-start < us);
        }
        
    }

void RTC1_IRQHandler(void)
{
  //NRF_RTC1->TASKS_CLEAR = 1;
  //NRF_RTC1->EVENTS_OVRFLW = 0;


  overflows = (overflows + 1) & 0xff;
    
  NRF_RTC1->EVENTS_OVRFLW = 0;
    
#if __CORTEX_M == 0x04
    volatile uint32_t dummy = NRF_RTC1->EVENTS_OVRFLW;
    (void)dummy;
#endif
    
}

#ifdef __cplusplus
}
#endif
