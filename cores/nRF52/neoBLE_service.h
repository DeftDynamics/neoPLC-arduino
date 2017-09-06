#ifndef _NEOBLE_SERVICE_H_
#define _NEOBLE_SERVICE_H_

#include <neoBLE_characteristic.h>

class neoBLE_service{
	public:
		neoBLE_service(uint16_t service_handle, uint8_t uuid_type);
		neoBLE_characteristic add_characteristic(uint16_t uuid, uint8_t properties, const char* str_description, uint8_t format);
		neoBLE_characteristic add_characteristic(uint16_t uuid);
		void add_descriptor(uint16_t uuid, const char* attr_value);
	private:
		uint16_t _service_handle;
		uint8_t	 _uuid_type;
};
#endif