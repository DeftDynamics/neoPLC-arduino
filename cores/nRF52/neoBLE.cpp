#include <Arduino.h>

#include <string.h>
#include <ble.h>
#include <ble_hci.h>
#include <nrf_nvic.h>
#include <nrf_sdm.h>
#include <dfu_ble_svc.h>
#include "variant.h"

#include "neoBLE.h"


#define DFU_REV_MAJOR                    0x00                                       /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                    0x01                                       /** DFU Minor revision number to be exposed. */
#define DFU_REVISION                     ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */
                                              /**< Maximum length (in bytes) of the DFU Packet characteristic. */
#define BLE_DFU_SERVICE_UUID                 0x1530                       /**< The UUID of the DFU Service. */
#define BLE_DFU_PKT_CHAR_UUID                0x1532                       /**< The UUID of the DFU Packet Characteristic. */
#define BLE_DFU_CTRL_PT_UUID                 0x1531                       /**< The UUID of the DFU Control Point. */
#define BLE_DFU_STATUS_REP_UUID              0x1533                       /**< The UUID of the DFU Status Report Characteristic. */
#define BLE_DFU_REV_CHAR_UUID                0x1534                       /**< The UUID of the DFU Revision Characteristic. */

#define BLE_UUID_NUS_SERVICE            0x0001                       /**< The UUID of the Nordic UART Service. */
#define BLE_UUID_NUS_TX_CHARACTERISTIC  0x0003                       /**< The UUID of the TX Characteristic. */
#define BLE_UUID_NUS_RX_CHARACTERISTIC  0x0002                       /**< The UUID of the RX Characteristic. */

#define BLE_STACK_EVT_MSG_BUF_SIZE  (sizeof(ble_evt_t) + (GATT_MTU_SIZE_DEFAULT))
#define SOFTDEVICE_EVT_IRQ        	SD_EVT_IRQn       /**< SoftDevice Event IRQ number. Used for both protocol events and SoC events. */
#define SOFTDEVICE_EVT_IRQHandler 	SD_EVT_IRQHandler
static uint32_t 					BLE_EVT_BUFFER[BLE_STACK_EVT_MSG_BUF_SIZE/ sizeof(uint32_t)];
static uint8_t                      * mp_ble_evt_buffer = (uint8_t *)BLE_EVT_BUFFER;                /**< Buffer for receiving BLE events from the SoftDevice. */
static uint16_t                     m_ble_evt_buffer_size = sizeof(BLE_EVT_BUFFER);            /**< Size of BLE event buffer. */
//static ble_evt_handler_t            m_ble_evt_handler;


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

void neoBLE::begin(){
	_txCount = 0;
  	_rxHead = _rxTail = 0;

	init();
	
	const ble_uuid128_t NUS_UUID ={ { 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E } };
	_nus_service = add_service(NUS_UUID,BLE_UUID_NUS_SERVICE);
	_nus_service->add_descriptor(0x2901,"UART");
	_rx_char = _nus_service->add_characteristic(BLE_UUID_NUS_RX_CHARACTERISTIC, BLENotify, "", 0xFF);
	_tx_char = _nus_service->add_characteristic(BLE_UUID_NUS_TX_CHARACTERISTIC, BLEWrite|BLEWriteWithoutResponse, "", 0xFF);

	ble_dfu_init();
	
	start_advertizing();
}

void neoBLE::setDeviceName(const char* device_name){
	ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    sd_ble_gap_device_name_set(&sec_mode,(const uint8_t *)device_name,strlen(device_name));
}

bool neoBLE::connected(){
	if(_conn_handle==BLE_CONN_HANDLE_INVALID) return false;
	return true;
}

int neoBLE::available(void) {
  int retval = (_rxHead - _rxTail + sizeof(_rxBuffer)) % sizeof(_rxBuffer);
  return retval;
}

int neoBLE::peek(void) {
  if (_rxTail == _rxHead) return -1;
  uint8_t byte = _rxBuffer[_rxTail];
  return byte;
}

int neoBLE::read(void) {
  if (_rxTail == _rxHead) return -1;
  _rxTail = (_rxTail + 1) % sizeof(_rxBuffer);
  uint8_t byte = _rxBuffer[_rxTail];
  return byte;
}

void neoBLE::flush(void) {
  if (_txCount == 0) return;
	ble_gatts_hvx_params_t hvx_params;
	memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = _rx_char->get_value_handle();
    hvx_params.p_data = _txBuffer;
    hvx_params.p_len  = &_txCount;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    sd_ble_gatts_hvx(_conn_handle, &hvx_params);
  _txCount = 0;
}

size_t neoBLE::write(uint8_t byte) {
	//Serial.println("write");
  //if (_txCharacteristic.subscribed() == false) return 0;
  _txBuffer[_txCount++] = byte;
  //Serial.println(_txCount);
  if (_txCount == sizeof(_txBuffer)) flush();
  return 1;
}

void neoBLE::_received(const uint8_t* data, uint16_t size) {
  //Serial.println("_recieved");
  for (int i = 0; i < size; i++) {
    _rxHead = (_rxHead + 1) % sizeof(_rxBuffer);
    _rxBuffer[_rxHead] = data[i];
  }
  
}

void neoBLE::event_handler(ble_evt_t * p_ble_evt){
	switch (p_ble_evt->header.evt_id){
		case BLE_GAP_EVT_CONNECTED:
			_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		break;
		case BLE_GAP_EVT_DISCONNECTED:
			_conn_handle = BLE_CONN_HANDLE_INVALID;
			start_advertizing();
		break;
		case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            sd_ble_gap_sec_params_reply(_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
        break;
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            sd_ble_gatts_sys_attr_set(_conn_handle, NULL, 0, 0);
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

				sd_ble_gatts_rw_authorize_reply(_conn_handle, &auth_reply);
				switch (p_ble_write_evt->data[0]){
					case OP_CODE_START_DFU:
						sd_softdevice_disable();
						sd_softdevice_vector_table_base_set(NRF_UICR->NRFFW[0]);
						dfu_ble_svc_boot_ble();
					break;
				}
			}
		break;
		case BLE_GATTS_EVT_WRITE: 
        	uint16_t handle = p_ble_evt->evt.gatts_evt.params.write.handle;
			if (_tx_char->get_value_handle() == handle) {
				
          		_received(p_ble_evt->evt.gatts_evt.params.write.data, p_ble_evt->evt.gatts_evt.params.write.len);
        	}
        break;
		
	}
}

void neoBLE::init(){
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

neoBLE_service* neoBLE::add_service(ble_uuid128_t base_uuid, uint16_t uuid){
	ble_uuid_t      	service_uuid;
	service_uuid.uuid 	= uuid;
	sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);  

	uint16_t		service_handle;
	sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &service_handle);
	return new neoBLE_service(service_handle,service_uuid.type);
}

neoBLE_service* neoBLE::add_service(uint16_t uuid){
	ble_uuid128_t   	base_uuid = BLE_UUID_OUR_BASE_UUID;
	return add_service(base_uuid,uuid);
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