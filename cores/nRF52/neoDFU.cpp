#include <string.h>
#include <ble.h>
#include <ble_hci.h>
#include <nrf_nvic.h>
#include <nrf_sdm.h>
#include "variant.h"
#include <dfu_ble_svc.h>
#include "neoDFU.h"

void neoDFU_init(void){
	sd_nvic_ClearPendingIRQ(boot_irq_n);
	sd_nvic_SetPriority(boot_irq_n,5);
	sd_nvic_EnableIRQ(boot_irq_n);
}
void boot_irq_handler(void)
{
	sd_nvic_ClearPendingIRQ(boot_irq_n);
	sd_forward_interrupts_to_bootloader();
	dfu_ble_svc_boot_serial();
}

void sd_forward_interrupts_to_bootloader(void){
	sd_softdevice_vector_table_base_set(NRF_UICR->NRFFW[0]);
}
void sd_forward_interrupts_to_application(void){
	sd_softdevice_vector_table_base_set(APP_CODE_START);
}	

