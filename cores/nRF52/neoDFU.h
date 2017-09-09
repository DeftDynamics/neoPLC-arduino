#ifndef _NEODFU_H_
#define _NEODFU_H_

#include <string.h>
#include <ble.h>
#include <ble_hci.h>
#include <nrf_nvic.h>
#include <nrf_sdm.h>
#include "variant.h"

#define boot_irq_handler 		SWI0_EGU0_IRQHandler
#define boot_irq_n 				SWI0_EGU0_IRQn
#ifdef __cplusplus
extern "C"{
#endif
	void boot_irq_handler(void);
#ifdef __cplusplus
}
#endif

void neoDFU_init(void);
void sd_forward_interrupts_to_bootloader(void);
void sd_forward_interrupts_to_application(void);


#endif