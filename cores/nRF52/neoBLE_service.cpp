#include <neoBLE_characteristic.h>
#include <neoBLE.h>
#include <neoBLE_service.h>

neoBLE_service::neoBLE_service(uint16_t service_handle, uint8_t uuid_type){
	_service_handle = service_handle;
	_uuid_type = uuid_type;
}

neoBLE_characteristic* neoBLE_service::add_characteristic(uint16_t uuid, uint8_t properties, const char* str_description, uint8_t format){
	//UUID
	ble_uuid_t      	char_uuid;
	char_uuid.uuid      = uuid;
	char_uuid.type		= _uuid_type;
	
	//attribute metadata
	ble_gatts_attr_md_t 	attr_md;
	ble_gatts_attr_t    		attr_char_value;
	ble_gatts_char_md_t 				char_md;
	ble_gatts_attr_md_t 		cccd_md;
	
	//if cccd
	if(properties & (BLENotify | BLEIndicate)){
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
	}else{
	//characteristic metadata
		memset(&char_md, 0, sizeof(char_md));
		char_md.p_char_user_desc  = NULL;
		char_md.p_char_pf         = NULL;
		char_md.p_user_desc_md    = NULL;
		char_md.p_cccd_md         = NULL;
		char_md.p_sccd_md         = NULL;
	}
	
	if(properties & BLEBroadcast){
		char_md.char_props.broadcast 		= 1;
	}
	if(properties & BLERead){
		char_md.char_props.read 			= 1;
	}
	if(properties & BLEWriteWithoutResponse){
		char_md.char_props.write_wo_resp 	= 1;
	}
	if(properties & BLEWrite){
		char_md.char_props.write 			= 1;
	}
	if(properties & BLENotify){
		char_md.char_props.notify 			= 1;
	}
	if(properties & BLEIndicate){
		char_md.char_props.indicate 		= 1;
	}
	//char_md.char_props.auth_signed_wr 		= 0;

	//user_desc
	uint16_t desc_len = strlen(str_description);
	if(desc_len>0){
		ble_gatts_attr_md_t				user_desc_md;
		memset(&user_desc_md, 0, sizeof(user_desc_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&user_desc_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&user_desc_md.write_perm);
		user_desc_md.vloc           	= BLE_GATTS_VLOC_STACK;  
		char_md.p_user_desc_md 			= &user_desc_md;
		char_md.p_char_user_desc		= (unsigned char*)str_description;
		char_md.char_user_desc_max_size = desc_len;
		char_md.char_user_desc_size		= desc_len;
	}
	//presentation format
	if(format < 0x1C){
		ble_gatts_char_pf_t 			presentation_format;
		presentation_format.format 		= format;
		char_md.p_char_pf 				= &presentation_format;
	}
	
	memset(&attr_md, 0, sizeof(attr_md));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	attr_md.vloc        	= BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;
	//value attribute
	
	memset(&attr_char_value, 0, sizeof(attr_char_value));    
	attr_char_value.p_uuid      = &char_uuid;
	attr_char_value.p_attr_md   = &attr_md;
    attr_char_value.init_len  	= 1;
    attr_char_value.init_offs 	= 0;
    attr_char_value.max_len   	= BLE_ATTRIBUTE_MAX_VALUE_LENGTH;
	
	//add to gatts
	ble_gatts_char_handles_t 	char_handles;
	sd_ble_gatts_characteristic_add(_service_handle,&char_md,&attr_char_value,&char_handles);
	//return char_handles.value_handle;
	return new neoBLE_characteristic(char_handles.value_handle, _uuid_type);
}

neoBLE_characteristic* neoBLE_service::add_characteristic(uint16_t uuid){
	return add_characteristic(uuid, BLERead | BLEWrite, "", 0xFF);
}

void neoBLE_service::add_descriptor(uint16_t uuid, const char* attr_value){
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
	sd_ble_gatts_descriptor_add(_service_handle, &desc_attr, &desc_handle);
	//return desc_handle;
}