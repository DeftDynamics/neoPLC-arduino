#include <neoBLE.h>
#include <neoBLE_characteristic.h>

neoBLE_characteristic::neoBLE_characteristic(uint16_t value_handle, uint8_t uuid_type){
	_value_handle=value_handle;
	_uuid_type=uuid_type;
}

void neoBLE_characteristic::setValue(const uint8_t value[], uint8_t length){
	ble_gatts_value_t new_value;
	new_value.len = length;
	new_value.offset = 0;
	new_value.p_value = (uint8_t*)value;
	sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID, _value_handle, &new_value);
}
void neoBLE_characteristic::setValue(const char* value){
	setValue((const uint8_t*) value, strlen(value));
}
void neoBLE_characteristic::setValue(int value){
	setValue((const uint8_t*) &value, sizeof(int));
}

void neoBLE_characteristic::addDescriptor(uint16_t uuid, const char* attr_value){
	//UUID
	ble_uuid_t      desc_uuid;
	ble_uuid128_t   base_uuid = BLE_UUID_OUR_BASE_UUID;
	desc_uuid.uuid = uuid;
	sd_ble_uuid_vs_add(&base_uuid, &desc_uuid.type);
	
	
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
	sd_ble_gatts_descriptor_add(_value_handle, &desc_attr, &desc_handle);
	//return desc_handle;
}

uint16_t neoBLE_characteristic::get_value_handle(void){
	return _value_handle;
}