#include <string.h>
#include <ble.h>
#include <ble_hci.h>
#include <nrf_nvic.h>
#include <nrf_sdm.h>
#include "variant.h"

#include "neoNUS.h"

#define BLE_UUID_NUS_SERVICE            0x0001                       /**< The UUID of the Nordic UART Service. */
#define BLE_UUID_NUS_TX_CHARACTERISTIC  0x0003                       /**< The UUID of the TX Characteristic. */
#define BLE_UUID_NUS_RX_CHARACTERISTIC  0x0002                       /**< The UUID of the RX Characteristic. */
#define BLE_ATTRIBUTE_MAX_VALUE_LENGTH 20

uint16_t uart_service_add(uint8_t uuid_type){
    ble_uuid_t service_uuid;
    uint32_t   err_code;
    
    service_uuid.uuid = BLE_UUID_NUS_SERVICE;
    service_uuid.type = uuid_type;
    uint16_t service_handle;
    sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &service_handle);
    return service_handle;
}
void uart_desc_add(uint8_t uuid_type, uint16_t service_handle, const char* attr_value){
	ble_uuid_t      desc_uuid;
	desc_uuid.type=uuid_type;
	desc_uuid.uuid=0x2901; //Characteristic User Description
	
	ble_gatts_attr_md_t desc_md;
	memset(&desc_md, 0, sizeof(desc_md));
	desc_md.vloc = BLE_GATTS_VLOC_STACK;
	desc_md.vlen = 0;
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&desc_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&desc_md.write_perm);
	
	ble_gatts_attr_t desc_attr;
	memset(&desc_attr, 0, sizeof(desc_attr));
	desc_attr.init_len = strlen(attr_value);
	desc_attr.max_len = strlen(attr_value);
	desc_attr.init_offs = 0;
	desc_attr.p_value = (uint8_t*)attr_value;
	desc_attr.p_uuid = &desc_uuid;
	desc_attr.p_attr_md = &desc_md;
	
	uint16_t		desc_handle;
	sd_ble_gatts_descriptor_add(service_handle, &desc_attr, &desc_handle);
}
uint16_t uart_rx_char_add(uint8_t uuid_type, uint16_t service_handle){
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;
    ble_uuid.type = uuid_type;
    ble_uuid.uuid = BLE_UUID_NUS_RX_CHARACTERISTIC;
    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_ATTRIBUTE_MAX_VALUE_LENGTH;
    ble_gatts_char_handles_t char_handles;
    return sd_ble_gatts_characteristic_add(service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &char_handles);
	return char_handles.value_handle;
}
uint16_t uart_tx_char_add(uint8_t uuid_type, uint16_t service_handle){
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;
    ble_uuid.type = uuid_type;
    ble_uuid.uuid = BLE_UUID_NUS_TX_CHARACTERISTIC;
    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_ATTRIBUTE_MAX_VALUE_LENGTH;
    ble_gatts_char_handles_t char_handles;
    return sd_ble_gatts_characteristic_add(service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &char_handles);
    return char_handles.value_handle;
}