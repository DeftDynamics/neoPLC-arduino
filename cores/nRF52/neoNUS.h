#ifndef _NEONUS_H_
#define _NEONUS_H_

#include <string.h>
#include <ble.h>
#include <ble_hci.h>
#include <nrf_sdm.h>

uint16_t uart_service_add(uint8_t uuid_type);
void uart_desc_add(uint8_t uuid_type, uint16_t service_handle, const char* attr_value);
uint16_t uart_rx_char_add(uint8_t uuid_type, uint16_t service_handle);
uint16_t uart_tx_char_add(uint8_t uuid_type, uint16_t service_handle);
#endif