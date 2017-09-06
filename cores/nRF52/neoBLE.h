#ifndef _NEOBLE_H_
#define _NEOBLE_H_

#include <string.h>
#include <ble.h>
#include <ble_hci.h>
#include <nrf_sdm.h>

#include <neoBLE_service.h>
#include <neoBLE_characteristic.h>

#define BLE_ATTRIBUTE_MAX_VALUE_LENGTH 20
#define MAX_DFU_PKT_LEN         20

enum BLEProperty {
  BLEBroadcast            = 0x01,
  BLERead                 = 0x02,
  BLEWriteWithoutResponse = 0x04,
  BLEWrite                = 0x08,
  BLENotify               = 0x10,
  BLEIndicate             = 0x20
};

class neoBLE{
	public:
		neoBLE();
		void event_handler(ble_evt_t * p_ble_evt);
		void begin();
		void init();
		bool connected();
		int available();
		int peek();
		int read();
		void flush();
		uint16_t write(uint8_t byte);

		neoBLE_service add_service(ble_uuid128_t base_uuid, uint16_t uuid);
		neoBLE_service add_service(uint16_t uuid);
		/*uint16_t add_service(uint16_t uuid);
		uint16_t add_characteristic(uint16_t service_handle, uint16_t uuid, uint8_t properties, const char* str_description, uint8_t format);
		uint16_t add_characteristic(uint16_t service_handle, uint16_t uuid);
		void set_value(uint16_t value_handle, const uint8_t value[], uint8_t length);
		void set_value(uint16_t value_handle, const char* value);
		void set_value(uint16_t value_handle, int value);
		uint16_t add_descriptor(uint16_t char_handle, uint16_t uuid, const char* attr_value);
		*/
		void start_advertizing();
		
	private:
		uint16_t _conn_handle=0;
		uint16_t _service_handle;
		uint16_t _rx_handle;
		uint16_t _tx_handle;
		
		void _received(const uint8_t* data, uint16_t size);

		size_t _rxHead;
    	size_t _rxTail;
    	size_t _rxCount() const;
    	uint8_t _rxBuffer[BLE_ATTRIBUTE_MAX_VALUE_LENGTH];
    	size_t _txCount;
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