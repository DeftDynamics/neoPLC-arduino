#ifndef _NEOBLE_SERVICE_H_
#define _NEOBLE_SERVICE_H_

#include <neoBLE_characteristic.h>
#include <ble.h>
#include <ble_hci.h>
#include <nrf_nvic.h>
#include <nrf_sdm.h>

class neoBLE_service{
	public:
		neoBLE_service(uint16_t service_handle, uint8_t uuid_type);
		neoBLE_characteristic* addCharacteristic(uint16_t uuid, uint8_t properties, const char* str_description, uint8_t format);
		neoBLE_characteristic* addCharacteristic(uint16_t uuid);
		void addDescriptor(uint16_t uuid, const char* attr_value);
	private:
		uint16_t _service_handle;
		uint8_t	 _uuid_type;
};
#endif