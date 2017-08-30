
#include <string.h>
#include <ble.h>
#include <ble_hci.h>
#include <nrf_sdm.h>

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
		uint16_t add_service(uint16_t uuid);
		uint16_t add_characteristic(uint16_t service_handle, uint16_t uuid, uint8_t properties, const char* str_description, uint8_t format);
		uint16_t add_characteristic(uint16_t service_handle, uint16_t uuid);
		void set_value(uint16_t value_handle, const uint8_t value[], uint8_t length);
		void set_value(uint16_t value_handle, const char* value);
		void set_value(uint16_t value_handle, int value);
		uint16_t add_descriptor(uint16_t char_handle, uint16_t uuid, const char* attr_value);
		void start_advertizing();
		
	private:
		uint16_t conn_handle;
		
};

#ifdef __cplusplus

extern neoBLE BLE;

#endif

void ble_dfu_init();
static uint32_t dfu_pkt_char_add(uint8_t uuid_type, uint16_t service_handle);
static uint32_t dfu_rev_char_add(uint8_t uuid_type, uint16_t service_handle, uint16_t const revision);
static uint32_t dfu_ctrl_pt_add(uint8_t uuid_type, uint16_t service_handle);