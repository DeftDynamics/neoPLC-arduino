#ifndef _NEOBLE_H_
#define _NEOBLE_H_

#include <Arduino.h>
#include <string.h>
#include <ble.h>
#include <ble_gap.h>
#include <ble_hci.h>
#include <nrf_sdm.h>

#include <neoBLE_service.h>
#include <neoBLE_characteristic.h>

#define BLE_ATTRIBUTE_MAX_VALUE_LENGTH 20
#define RX_BUFFER_SIZE 			255
#define MAX_DFU_PKT_LEN         20
#define BLE_CONN_HANDLE_INVALID 0xFFFF
#define BLE_UUID_OUR_BASE_UUID  {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00} 


		enum neoBLEProperty {
		  neoBLEBroadcast            = 0x01,
		  neoBLERead                 = 0x02,
		  neoBLEWriteWithoutResponse = 0x04,
		  neoBLEWrite                = 0x08,
		  neoBLENotify               = 0x10,
		  neoBLEIndicate             = 0x20
		};


class neoBLE : public Stream{
	public:
		neoBLE();
		void eventHandler(ble_evt_t * p_ble_evt);
		void begin();
		void init();
		void setDeviceName(const char* device_name);
		void startAdvertizing();
		
		bool connected();
		int available();
		int peek();
		int read();
		void flush();
		void post(char* DX);
		size_t write(uint8_t byte);
		using Print::write;

		neoBLE_service* addService(ble_uuid128_t base_uuid, uint16_t uuid);
		neoBLE_service* addService(uint16_t uuid);
		
	private:
		uint16_t _conn_handle=BLE_CONN_HANDLE_INVALID;
		ble_gap_addr_t _peer_addr; 
		uint16_t _service_handle;
		uint16_t _rx_handle=5;
		uint16_t _tx_handle=5;
		neoBLE_service* _nus_service;
		neoBLE_characteristic* _rx_char;
		neoBLE_characteristic* _tx_char;
		
		void _received(const uint8_t* data, uint16_t size);

		uint16_t _rxHead;
    	uint16_t _rxTail;
    	uint16_t _rxCount() const;
    	uint8_t _rxBuffer[RX_BUFFER_SIZE];
    	uint16_t _txCount;
    	uint8_t _txBuffer[BLE_ATTRIBUTE_MAX_VALUE_LENGTH];
		
};



void ble_dfu_init();
static uint32_t dfu_pkt_char_add(uint8_t uuid_type, uint16_t service_handle);
static uint32_t dfu_rev_char_add(uint8_t uuid_type, uint16_t service_handle, uint16_t const revision);
static uint32_t dfu_ctrl_pt_add(uint8_t uuid_type, uint16_t service_handle);


#ifdef __cplusplus

extern neoBLE BLE;

#endif

#endif