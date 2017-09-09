#ifndef _NEOBLE_CHARACTERISTIC_H_
#define _NEOBLE_CHARACTERISTIC_H_

#include <ble.h>
#include <ble_hci.h>
#include <nrf_nvic.h>
#include <nrf_sdm.h>

class neoBLE_characteristic{
	public:
		neoBLE_characteristic(uint16_t value_handle, uint8_t uuid_type);
		void set_value(const uint8_t value[], uint8_t length);
		void set_value(const char* value);
		void set_value(int value);
		void add_descriptor(uint16_t uuid, const char* attr_value);
		uint16_t get_value_handle(void);
	private:
		uint16_t _value_handle;
		uint8_t	 _uuid_type;
};
#endif