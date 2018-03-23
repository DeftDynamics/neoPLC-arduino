#ifndef _NEOBLE_CHARACTERISTIC_H_
#define _NEOBLE_CHARACTERISTIC_H_

#include <ble.h>
#include <ble_hci.h>
#include <nrf_nvic.h>
#include <nrf_sdm.h>

class neoBLE_characteristic{
	public:
		neoBLE_characteristic(uint16_t value_handle, uint8_t uuid_type);
		void setValue(const uint8_t value[], uint8_t length);
		void setValue(const char* value);
		void setValue(int value);
		void addDescriptor(uint16_t uuid, const char* attr_value);
		uint16_t get_value_handle(void);
	private:
		uint16_t _value_handle;
		uint8_t	 _uuid_type;
};
#endif