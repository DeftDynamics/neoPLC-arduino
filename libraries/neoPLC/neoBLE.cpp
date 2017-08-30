
#include <string.h>
#include <ble.h>
#include <ble_hci.h>
#include <nrf_nvic.h>
#include <nrf_sdm.h>
#include "variant.h"

#include "neoBLE.h"

#define DFU_REV_MAJOR                    0x00                                       /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                    0x01                                       /** DFU Minor revision number to be exposed. */
#define DFU_REVISION                     ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */
#define BLE_UUID_OUR_BASE_UUID  {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00} 
#define MAX_DFU_PKT_LEN         20                                              /**< Maximum length (in bytes) of the DFU Packet characteristic. */
#define BLE_DFU_SERVICE_UUID                 0x1530                       /**< The UUID of the DFU Service. */
#define BLE_DFU_PKT_CHAR_UUID                0x1532                       /**< The UUID of the DFU Packet Characteristic. */
#define BLE_DFU_CTRL_PT_UUID                 0x1531                       /**< The UUID of the DFU Control Point. */
#define BLE_DFU_STATUS_REP_UUID              0x1533                       /**< The UUID of the DFU Status Report Characteristic. */
#define BLE_DFU_REV_CHAR_UUID                0x1534                       /**< The UUID of the DFU Revision Characteristic. */

#define BLE_STACK_EVT_MSG_BUF_SIZE  (sizeof(ble_evt_t) + (GATT_MTU_SIZE_DEFAULT))
#define SOFTDEVICE_EVT_IRQ        	SD_EVT_IRQn       /**< SoftDevice Event IRQ number. Used for both protocol events and SoC events. */
#define SOFTDEVICE_EVT_IRQHandler 	SD_EVT_IRQHandler
static uint32_t 					BLE_EVT_BUFFER[BLE_STACK_EVT_MSG_BUF_SIZE/ sizeof(uint32_t)];
static uint8_t                      * mp_ble_evt_buffer = (uint8_t *)BLE_EVT_BUFFER;                /**< Buffer for receiving BLE events from the SoftDevice. */
static uint16_t                     m_ble_evt_buffer_size = sizeof(BLE_EVT_BUFFER);            /**< Size of BLE event buffer. */
//static ble_evt_handler_t            m_ble_evt_handler;

nrf_nvic_state_t nrf_nvic_state;
#ifdef __cplusplus
extern "C"{
#endif
	void SOFTDEVICE_EVT_IRQHandler(void);
#ifdef __cplusplus
}
#endif
enum
{
    OP_CODE_START_DFU          = 1,                                             /**< Value of the Op code field for 'Start DFU' command.*/
    OP_CODE_RECEIVE_INIT       = 2,                                             /**< Value of the Op code field for 'Initialize DFU parameters' command.*/
    OP_CODE_RECEIVE_FW         = 3,                                             /**< Value of the Op code field for 'Receive firmware image' command.*/
    OP_CODE_VALIDATE           = 4,                                             /**< Value of the Op code field for 'Validate firmware' command.*/
    OP_CODE_ACTIVATE_N_RESET   = 5,                                             /**< Value of the Op code field for 'Activate & Reset' command.*/
    OP_CODE_SYS_RESET          = 6,                                             /**< Value of the Op code field for 'Reset System' command.*/
    OP_CODE_IMAGE_SIZE_REQ     = 7,                                             /**< Value of the Op code field for 'Report received image size' command.*/
    OP_CODE_PKT_RCPT_NOTIF_REQ = 8,                                             /**< Value of the Op code field for 'Request packet receipt notification.*/
    OP_CODE_RESPONSE           = 16,                                            /**< Value of the Op code field for 'Response.*/
    OP_CODE_PKT_RCPT_NOTIF     = 17                                             /**< Value of the Op code field for 'Packets Receipt Notification'.*/
};

neoBLE::neoBLE(){
	
}

void neoBLE::event_handler(ble_evt_t * p_ble_evt){
	switch (p_ble_evt->header.evt_id){
		case BLE_GAP_EVT_CONNECTED:
			conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			break;
		case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
		ble_gatts_evt_rw_authorize_request_t * p_authorize_request;

		p_authorize_request = &(p_ble_evt->evt.gatts_evt.params.authorize_request);
		if ((p_authorize_request->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)&&
			(p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op != BLE_GATTS_OP_PREP_WRITE_REQ)&&
			(p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op != BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)&&
			(p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op != BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL)){
			
			ble_gatts_evt_write_t * p_ble_write_evt = &(p_authorize_request->request.write);
			ble_gatts_rw_authorize_reply_params_t auth_reply;

			auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
			auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
			auth_reply.params.write.update = 1;
			auth_reply.params.write.offset = p_ble_write_evt->offset;
			auth_reply.params.write.len = p_ble_write_evt->len;
			auth_reply.params.write.p_data = p_ble_write_evt->data;

			auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;

			sd_ble_gatts_rw_authorize_reply(conn_handle, &auth_reply);
			switch (p_ble_write_evt->data[0]){
				case OP_CODE_START_DFU:
						//BOOTLOADER START
						//bootload_ble(); //<- this goes back to variant.cpp
						//NRF_POWER->GPREGRET = BOOTLOADER_DFU_START_BLE;
						sd_power_gpregret_set(BOOTLOADER_DFU_START_BLE);
						sd_softdevice_disable();
						/*sd_softdevice_vector_table_base_set(NRF_UICR->NRFFW[0]);
						NVIC_ClearPendingIRQ(SWI2_IRQn);
						// Fetch the current interrupt settings.
						interrupt_setting_mask = NVIC->ISER[0];
						for (uint32_t irq = 0; irq < MAX_NUMBER_INTERRUPTS; irq++)
						{
							if (interrupt_setting_mask & (IRQ_ENABLED << irq))
							{
								// The interrupt was enabled, and hence disable it.
								NVIC_DisableIRQ((IRQn_Type)irq);
							}
						}*/
						NVIC_SystemReset();
				break;
			}
		break;
		}
	}
}

void neoBLE::begin(){
	//set clock source
	#if defined(USE_LFRC)
    nrf_clock_lf_cfg_t cfg = {
      .source        = NRF_CLOCK_LF_SRC_RC,
      .rc_ctiv       = 8, //16
      .rc_temp_ctiv  = 2,
      .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM
    };
	#elif defined(USE_LFSYNT)
    nrf_clock_lf_cfg_t cfg = {
      .source        = NRF_CLOCK_LF_SRC_SYNTH,
      .rc_ctiv       = 0,
      .rc_temp_ctiv  = 0,
      .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM
    };
	#else
    //default USE_LFXO
    nrf_clock_lf_cfg_t cfg = {
      .source        = NRF_CLOCK_LF_SRC_XTAL,
      .rc_ctiv       = 0,
      .rc_temp_ctiv  = 0,
      .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM
    };
	#endif
	//start soft device
	sd_softdevice_enable(&cfg, NULL);
	sd_nvic_EnableIRQ((IRQn_Type)SOFTDEVICE_EVT_IRQ);
	
	//tell softdevice to enable ble
	extern uint32_t __data_start__;
	volatile uint32_t ram_start = (uint32_t) &__data_start__;
	uint32_t app_ram_base = ram_start;
	ble_enable_params_t enableParams;
	memset(&enableParams, 0, sizeof(ble_enable_params_t));
	enableParams.common_enable_params.vs_uuid_count   = 10;
	enableParams.gatts_enable_params.attr_tab_size    = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
	enableParams.gatts_enable_params.service_changed  = 1;
	enableParams.gap_enable_params.periph_conn_count  = 1;
	enableParams.gap_enable_params.central_conn_count = 0;
	enableParams.gap_enable_params.central_sec_count  = 0;
	sd_ble_enable(&enableParams, &app_ram_base);
	
	//set the GAP parameters
	ble_gap_conn_params_t gap_conn_params;
	gap_conn_params.min_conn_interval = 40;  // in 1.25ms units
	gap_conn_params.max_conn_interval = 80;  // in 1.25ms unit
	gap_conn_params.slave_latency     = 0;
	gap_conn_params.conn_sup_timeout  = 4000 / 10; // in 10ms unit
	sd_ble_gap_ppcp_set(&gap_conn_params);
	sd_ble_gap_tx_power_set(0);
	
	ble_dfu_init();
}

uint16_t neoBLE::add_service(uint16_t uuid){
	ble_uuid_t      	service_uuid;
	ble_uuid128_t   	base_uuid = BLE_UUID_OUR_BASE_UUID;
	service_uuid.uuid 	= uuid;
	sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);  
	
	uint16_t		service_handle;
	sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &service_handle);
	return service_handle;
}

uint16_t neoBLE::add_characteristic(uint16_t service_handle, uint16_t uuid, uint8_t properties, const char* str_description, uint8_t format){
	//UUID
	ble_uuid_t      	char_uuid;
	ble_uuid128_t   	base_uuid = BLE_UUID_OUR_BASE_UUID;
	char_uuid.uuid      = uuid;
	sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
	
	//attribute metadata
	ble_gatts_attr_md_t 	attr_md;
	memset(&attr_md, 0, sizeof(attr_md));
	attr_md.vloc        	= BLE_GATTS_VLOC_STACK;
	
	//value attribute
	ble_gatts_attr_t    		attr_char_value;
	memset(&attr_char_value, 0, sizeof(attr_char_value));    
	attr_char_value.p_uuid      = &char_uuid;
	attr_char_value.p_attr_md   = &attr_md;
	attr_char_value.max_len     = 20;
	attr_char_value.init_len    = 20;
	uint8_t value[20]            = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	attr_char_value.p_value     = value;
	
	//characteristic metadata
	ble_gatts_char_md_t 				char_md;
	memset(&char_md, 0, sizeof(char_md));
	if(properties & BLEBroadcast){
		char_md.char_props.broadcast 		= 1;
	}
	if(properties & BLERead){
		char_md.char_props.read 			= 1;
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	}
	if(properties & BLEWriteWithoutResponse){
		char_md.char_props.write_wo_resp 	= 1;
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	}
	if(properties & BLEWrite){
		char_md.char_props.write 			= 1;
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	}
	if(properties & BLENotify){
		char_md.char_props.notify 			= 1;
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	}
	if(properties & BLEIndicate){
		char_md.char_props.indicate 		= 1;
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	}
	char_md.char_props.auth_signed_wr 		= 0;
	
	//if cccd
	if(properties & (BLENotify | BLEIndicate)){
		ble_gatts_attr_md_t 		cccd_md;
		memset(&cccd_md, 0, sizeof(cccd_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vloc            	= BLE_GATTS_VLOC_STACK; 
		char_md.p_cccd_md           = &cccd_md;
		char_md.char_props.notify   = 1;
	}
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
	
	//add to gatts
	ble_gatts_char_handles_t 	char_handles;
	sd_ble_gatts_characteristic_add(service_handle,&char_md,&attr_char_value,&char_handles);
	return char_handles.value_handle;
}

uint16_t neoBLE::add_characteristic(uint16_t service_handle, uint16_t uuid){
	return add_characteristic(service_handle, uuid, BLERead | BLEWrite, "", 0xFF);
}

void neoBLE::set_value(uint16_t value_handle, const uint8_t value[], uint8_t length){
	ble_gatts_value_t new_value;
	new_value.len = length;
	new_value.offset = 0;
	new_value.p_value = (uint8_t*)value;
	sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID, value_handle, &new_value);
}
void neoBLE::set_value(uint16_t value_handle, const char* value){
	set_value(value_handle,(const uint8_t*) value, strlen(value));
}
void neoBLE::set_value(uint16_t value_handle, int value){
	set_value(value_handle,(const uint8_t*) &value, sizeof(int));
}

uint16_t neoBLE::add_descriptor(uint16_t char_handle, uint16_t uuid, const char* attr_value){
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
	sd_ble_gatts_descriptor_add(char_handle, &desc_attr, &desc_handle);
	return desc_handle;
}

void neoBLE::start_advertizing(){
	ble_gap_adv_params_t advertisingParameters;

	memset(&advertisingParameters, 0x00, sizeof(advertisingParameters));

	advertisingParameters.type        = BLE_GAP_ADV_TYPE_ADV_IND;
	advertisingParameters.p_peer_addr = NULL;
	advertisingParameters.fp          = BLE_GAP_ADV_FP_ANY;
	advertisingParameters.p_whitelist = NULL;
	advertisingParameters.interval    = 300;//(10 * 16) / 10; // advertising interval (in units of 0.625 ms)
	advertisingParameters.timeout     = 0;

	sd_ble_gap_adv_start(&advertisingParameters);
}

void ble_dfu_init(){

    ble_uuid_t service_uuid;
	uint16_t service_handle;
    uint32_t   err_code;

    const ble_uuid128_t base_uuid128 =
	{
        {
            0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
            0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00
        }
    };
    service_uuid.uuid = BLE_DFU_SERVICE_UUID;
	sd_ble_uuid_vs_add(&base_uuid128, &(service_uuid.type));

    sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &service_handle);

    dfu_pkt_char_add(service_uuid.type, service_handle);
    dfu_ctrl_pt_add(service_uuid.type, service_handle);
    dfu_rev_char_add(service_uuid.type, service_handle, DFU_REVISION);
}
static uint32_t dfu_pkt_char_add(uint8_t uuid_type, uint16_t service_handle)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          char_uuid;
    ble_gatts_attr_md_t attr_md;
	ble_gatts_char_handles_t dfu_pkt_handles;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    char_uuid.type = uuid_type;
    char_uuid.uuid = BLE_DFU_PKT_CHAR_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 0;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MAX_DFU_PKT_LEN;
    attr_char_value.p_value   = NULL;

    return sd_ble_gatts_characteristic_add(service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &dfu_pkt_handles);
}
static uint32_t dfu_rev_char_add(uint8_t uuid_type, uint16_t service_handle, uint16_t const revision)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          char_uuid;
    ble_gatts_attr_md_t attr_md;
	ble_gatts_char_handles_t dfu_rev_handles;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read          = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    char_uuid.type = uuid_type;
    char_uuid.uuid = BLE_DFU_REV_CHAR_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint16_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint16_t);
    attr_char_value.p_value   = (uint8_t *)&revision;

    return sd_ble_gatts_characteristic_add(service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &dfu_rev_handles);
}
static uint32_t dfu_ctrl_pt_add(uint8_t uuid_type, uint16_t service_handle)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          char_uuid;
    ble_gatts_attr_md_t attr_md;
	ble_gatts_char_handles_t dfu_ctrl_pt_handles;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write  = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    char_uuid.type = uuid_type;
    char_uuid.uuid = BLE_DFU_CTRL_PT_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 1;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 0;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_L2CAP_MTU_DEF;
    attr_char_value.p_value   = NULL;

    return sd_ble_gatts_characteristic_add(service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &dfu_ctrl_pt_handles);
}

neoBLE BLE;

void SOFTDEVICE_EVT_IRQHandler(){
    bool no_more_ble_evts = false;
    for (;;){
        uint32_t err_code;
        // Fetch BLE Events.
        if (!no_more_ble_evts){
            // Pull event from stack
            uint16_t evt_len = m_ble_evt_buffer_size;
            err_code = sd_ble_evt_get(mp_ble_evt_buffer, &evt_len);
            if (err_code == NRF_ERROR_NOT_FOUND){
                no_more_ble_evts = true;
            }else if (err_code != NRF_SUCCESS){
                //APP_ERROR_HANDLER(err_code);
            }else{
                // Call application's BLE stack event handler.
                BLE.event_handler((ble_evt_t *)mp_ble_evt_buffer);
            }
        }else{
			break;
		}
    }
}