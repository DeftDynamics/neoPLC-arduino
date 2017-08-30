/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
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

#include <Arduino.h>
#include "variant.h"
#include "nrf52.h"

const uint32_t g_ADigitalPinMap[] = {
28,
29,
4,
5,
2,
3,
20,
18,
13,
11,
12,
100,
100,
20

/*0,
1,
2,
3,
4,
5,
6,
7,
8,
9,
10,
11,
12,
13,
14,
15,
16,
17,
18,
19,
20,
21,
22,
23,
24,
25,
26,
27,
28,
29,
30,
31
*/
};

void bootload_serial(void){
	NRF_POWER->GPREGRET = BOOTLOADER_DFU_START_SERIAL;
    NVIC_SystemReset();
}
void bootload_ble(void){
	NRF_POWER->GPREGRET = BOOTLOADER_DFU_START_BLE;
    NVIC_SystemReset();
}

void initVariant(void){
	Serial.core_begin(115200);
	delay(100);
	
}

void shutdownISR() __attribute__((weak));
void shutdownISR() { }

void variant_UART_IRQ_Handler(){
	int q = Serial.available();
	if(q >= 3){
		if(Serial.peek_pos(q-3) == 36 && Serial.peek_pos(q-2) == 38 && Serial.peek_pos(q-1) == 36){
			shutdownISR();
			Serial.core_end();
			delay(100);
			bootload_serial();
		}
	}
}
