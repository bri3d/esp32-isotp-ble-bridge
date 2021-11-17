// Copied from "BLE SPP" example in ESP-IDF, which is Public Domain
// This is kind of complicated, but isn't really.
// Here's what happens:
// BLE revolves around the transmission and reception of "GATTs" - Generic Attributes.
// The Attributes allow certain Operations - Read/Write, and Notify/Indicate being the most important.
// Read/Write allows a simple Read or Write operation against an attribute. The MTU size can be defined by the connection.
// Notifiy/Indicate allow arbitrary data to be sent from the Server (peripheral) to the Client (host/phone) as a notification.

// We advertise a few BLE profiles - an app ID and a set of GATTs.
// The client (host/phone) Writes to one attribute to send data to us.
// And we (server) send a Notification against another attribute to send data back.
// Because the MTUs are so small, we need basic packet reconstruction. We do this by sending '## total_idx current_idx' as the header for multipart messages.
// And then on our end, we reconstruct a write buffer as well.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "string.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "ble_server.h"

#define BLE_TAG  							"BLE"


#define SPP_PROFILE_NUM             		1
#define SPP_PROFILE_APP_IDX         		0
#define ESP_SPP_APP_ID              		0x56
#define SPP_SVC_INST_ID	            		0
#define DEFAULT_MTU_SIZE					23
#define DEFAULT_DELAY_SEND					0
#define DEFAULT_DELAY_MULTI					0

/// SPP Service
static const uint16_t spp_service_uuid = 0xABF0;
/// Characteristic UUID
#define ESP_GATT_UUID_SPP_DATA_RECEIVE      0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY       0xABF2
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE   0xABF3
#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY    0xABF4

static uint8_t spp_adv_data[23] = {
    /* Flags */
    0x02,0x01,0x06,
    /* Complete List of 16-bit Service Class UUIDs */
    0x03,0x03,0xF0,0xAB,
    /* Complete Local Name in advertising */
    0x0F,0x09, 'B', 'L', 'E', '_', 'T', 'O', '_', 'I', 'S', 'O', 'T','P', '2', '0'
};

static char ble_gap_name[MAX_GAP_LENGTH+1]		= DEFAULT_GAP_NAME;
static uint16_t ble_delay_send 					= DEFAULT_DELAY_SEND;
static uint16_t ble_delay_multi					= DEFAULT_DELAY_MULTI;
static bool kill_ble_tasks 						= false;
static ble_server_callbacks server_callbacks;

static uint16_t spp_mtu_size 					= DEFAULT_MTU_SIZE;
static uint16_t spp_conn_id 					= 0xffff;
static esp_gatt_if_t spp_gatts_if 				= 0xff;
static QueueHandle_t spp_send_queue 			= NULL;
static QueueHandle_t cmd_cmd_queue 				= NULL;
static SemaphoreHandle_t ble_congested			= NULL;

static bool enable_data_ntf 					= false;
static bool is_connected 						= false;
static esp_bd_addr_t spp_remote_bda 			= {0x0,};

static uint16_t spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

typedef struct spp_receive_data_node{
    int32_t len;
    uint8_t * node_buff;
    struct spp_receive_data_node * next_node;
}spp_receive_data_node_t;

static spp_receive_data_node_t * temp_spp_recv_data_node_p1 = NULL;
static spp_receive_data_node_t * temp_spp_recv_data_node_p2 = NULL;

typedef struct spp_receive_data_buff{
    int32_t node_num;
    int32_t buff_size;
    spp_receive_data_node_t * first_node;
}spp_receive_data_buff_t;

static spp_receive_data_buff_t SppRecvDataBuff = {
    .node_num   = 0,
    .buff_size  = 0,
    .first_node = NULL
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ;

///SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t  spp_data_receive_val[20] = {0x00};

///SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t  spp_data_notify_val[20] = {0x00};
static const uint8_t  spp_data_notify_ccc[2] = {0x00, 0x00};

///SPP Service - command characteristic, read&write without response
static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t  spp_command_val[10] = {0x00};

///SPP Service - status characteristic, notify&read
static const uint16_t spp_status_uuid = ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
static const uint8_t  spp_status_val[10] = {0x00};
static const uint8_t  spp_status_ccc[2] = {0x00, 0x00};

///Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
{
    //SPP -  Service Declaration
    [SPP_IDX_SVC]                      	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

    //SPP -  data receive characteristic Declaration
    [SPP_IDX_SPP_DATA_RECV_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  data receive characteristic Value
    [SPP_IDX_SPP_DATA_RECV_VAL]             	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_DATA_MAX_LEN,sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

    //SPP -  data notify characteristic Declaration
    [SPP_IDX_SPP_DATA_NOTIFY_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  data notify characteristic Value
    [SPP_IDX_SPP_DATA_NTY_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ,
    SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

    //SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_DATA_NTF_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

    //SPP -  command characteristic Declaration
    [SPP_IDX_SPP_COMMAND_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  command characteristic Value
    [SPP_IDX_SPP_COMMAND_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_CMD_MAX_LEN,sizeof(spp_command_val), (uint8_t *)spp_command_val}},

    //SPP -  status characteristic Declaration
    [SPP_IDX_SPP_STATUS_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  status characteristic Value
    [SPP_IDX_SPP_STATUS_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_status_uuid, ESP_GATT_PERM_READ,
    SPP_STATUS_MAX_LEN,sizeof(spp_status_val), (uint8_t *)spp_status_val}},

    //SPP -  status characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_STATUS_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_status_ccc), (uint8_t *)spp_status_ccc}},
};

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for(int i = 0; i < SPP_IDX_NB ; i++){
        if(handle == spp_handle_table[i]){
            return i;
        }
    }

    return error;
}

static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data)
{
    temp_spp_recv_data_node_p1 = (spp_receive_data_node_t *)malloc(sizeof(spp_receive_data_node_t));

    if(temp_spp_recv_data_node_p1 == NULL){
		ESP_LOGI(BLE_TAG, "malloc error %s %d", __func__, __LINE__);
        return false;
    }
    if(temp_spp_recv_data_node_p2 != NULL){
        temp_spp_recv_data_node_p2->next_node = temp_spp_recv_data_node_p1;
    }
    temp_spp_recv_data_node_p1->len = p_data->write.len;
    SppRecvDataBuff.buff_size += p_data->write.len;
    temp_spp_recv_data_node_p1->next_node = NULL;
    temp_spp_recv_data_node_p1->node_buff = (uint8_t *)malloc(p_data->write.len);
    temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1;
    memcpy(temp_spp_recv_data_node_p1->node_buff,p_data->write.value,p_data->write.len);
    if(SppRecvDataBuff.node_num == 0){
        SppRecvDataBuff.first_node = temp_spp_recv_data_node_p1;
        SppRecvDataBuff.node_num++;
    }else{
        SppRecvDataBuff.node_num++;
    }

    return true;
}

static void disable_notification() {
    enable_data_ntf = false;
    server_callbacks.notifications_unsubscribed();
}

static void enable_notification() {
    enable_data_ntf = true;
    server_callbacks.notifications_subscribed();
}

static void free_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while(temp_spp_recv_data_node_p1 != NULL){
        temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1->next_node;
        free(temp_spp_recv_data_node_p1->node_buff);
        free(temp_spp_recv_data_node_p1);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p2;
    }

    SppRecvDataBuff.node_num = 0;
    SppRecvDataBuff.buff_size = 0;
    SppRecvDataBuff.first_node = NULL;
}

static void send_buffered_message(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;
    uint8_t buf[4096];
    uint32_t recv_len = 0;

    while(temp_spp_recv_data_node_p1 != NULL){
        if(recv_len > sizeof(buf)) {
            continue;
        }
        memcpy(buf + recv_len, (char *)(temp_spp_recv_data_node_p1->node_buff), temp_spp_recv_data_node_p1->len);
        recv_len += temp_spp_recv_data_node_p1->len;
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p1->next_node;
    }

    server_callbacks.data_received(buf, recv_len);
}

void send_task(void *pvParameters)
{
	send_message_t event;

	while(1) {
		if(xQueueReceive(spp_send_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
			//Are we shutting down?
			if(kill_ble_tasks) {
				free(event.buffer);
				while(xQueueReceive(spp_send_queue, (void * )&event, 0))
					free(event.buffer);
				break;
			}

			//If not continue
			ESP_LOGI(BLE_TAG, "Sending message [%08X]", event.msg_length);
			if(event.msg_length) {
				//Is GATT setup and ready to notify?
				if(!enable_data_ntf){
					ESP_LOGI(BLE_TAG, "%s notifications not enabled, message deleted", __func__);
					free(event.buffer);
				} else
				{  	//Connected and notifications are enabled check for congestion
					xSemaphoreTake(ble_congested, pdMS_TO_TICKS(BLE_CONGESTION_MAX));
					xSemaphoreGive(ble_congested);

                    uint32_t dataLength = event.msg_length + sizeof(ble_header_t);
					uint8_t* data = (uint8_t *)malloc(sizeof(uint8_t)*dataLength);
					if(data == NULL){
						ESP_LOGE(BLE_TAG, "%s malloc.1 failed", __func__);
						free(event.buffer);
						break;
					}
					memset(data, 0x0, dataLength);
					memcpy(data + sizeof(ble_header_t), event.buffer, event.msg_length);
					free(event.buffer);

					//Build header
					ble_header_t* header = (ble_header_t*)data;
					header->hdID = BLE_HEADER_ID;
					header->cmdSize = event.msg_length;
					header->rxID = event.rxID;
					header->txID = event.txID;

					//Can we add more?
					while(dataLength < (spp_mtu_size - 3 - sizeof(ble_header_t)))
					{
						//Do we have any other responses ready to send?
						send_message_t nextEvent;
						if(xQueuePeek(spp_send_queue, (void * )&nextEvent, ble_delay_multi)) {
							//If we add this to the packet are we oversize?
							if(nextEvent.msg_length + sizeof(ble_header_t) + dataLength <= (spp_mtu_size - 3)) {
								//We are good, add it but first remove it from the Queue
								if(xQueueReceive(spp_send_queue, (void * )&nextEvent, 0)) {
									uint32_t nextDataLength = dataLength + nextEvent.msg_length + sizeof(ble_header_t);
									uint8_t* nextData = (uint8_t *)malloc(sizeof(uint8_t)*nextDataLength);
									if(nextData == NULL){
										ESP_LOGE(BLE_TAG, "%s malloc.1 failed", __func__);
										free(nextEvent.buffer);
										break;
									}
									//Copy old data and newEvent buffer into newData
									memset(nextData, 0x0, nextDataLength);
									memcpy(nextData, data, dataLength);
									memcpy(nextData + dataLength + sizeof(ble_header_t), nextEvent.buffer, nextEvent.msg_length);
									free(nextEvent.buffer);

									//Build new header
									ble_header_t* header = (ble_header_t*)(nextData + dataLength);
									header->hdID = BLE_HEADER_ID;
									header->cmdSize = nextEvent.msg_length;
									header->rxID = nextEvent.rxID;
									header->txID = nextEvent.txID;

									//free old data and replace with new
									free(data);
									data = nextData;
									dataLength = nextDataLength;

									ESP_LOGI(BLE_TAG, "-Multisend Packet [%08X]-", nextEvent.msg_length);
								} else {
									//This shouldn't happen?
									break;
								}
							} else {
								//Oversized dont add
								break;
							}
						} else {
							//Nothing waiting in Queue
							break;
						}
					}

					//If payload is larger than MTU cut it up, this should only happen if its a single payload (multisend should not exceed mtu size)
					if(dataLength <= (spp_mtu_size - 3)) {
						esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], dataLength, data, false);
					} else {
						//determine packet size for split packets
						uint16_t pack_size = spp_mtu_size - 3;

						//allocate space for payload chunk
						uint8_t* data_chunk = (uint8_t *)malloc(pack_size*sizeof(uint8_t));
						if(data_chunk == NULL){
							ESP_LOGE(BLE_TAG, "%s malloc.2 failed", __func__);
							free(data);
							break;
						}

						//First chunk
						uint16_t data_pos = pack_size;
						memcpy(data_chunk, data, pack_size);
						data_chunk[1] |= BLE_COMMAND_FLAG_SPLIT_PK;
						esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], pack_size, data_chunk, false);
						vTaskDelay(ble_delay_send);

						//send the chunks
						uint8_t chunk_num = 1;
						while(data_pos < dataLength)
						{
							//constrain data len
							uint16_t data_len = dataLength - data_pos;
							if(data_len > pack_size - 2)
								data_len = pack_size - 2;

							data_chunk[0] = BLE_PARTIAL_ID;
							data_chunk[1] = chunk_num++;
							memcpy(data_chunk + 2, data + data_pos, data_len);
							data_pos += data_len;
							esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], data_len + 2, data_chunk, false);
						}
						free(data_chunk);
					}
					free(data);
					vTaskDelay(ble_delay_send);
				}
			}
		}
		taskYIELD();
    }
	vTaskDelete(NULL);
}

void spp_cmd_task(void * arg)
{
    uint8_t * cmd_id;

	while(1){
		if(xQueueReceive(cmd_cmd_queue, &cmd_id, portMAX_DELAY)) {
			//Are we shutting down?
			if(kill_ble_tasks) {
				free(cmd_id);
				while(xQueueReceive(cmd_cmd_queue, &cmd_id, 0))
					free(cmd_id);

				break;
			}

			//Nope continue
			esp_log_buffer_char(BLE_TAG,(char *)(cmd_id),strlen((char *)cmd_id));
			free(cmd_id);
		}
		taskYIELD();
    }
    vTaskDelete(NULL);
}

static void spp_task_init(void)
{
	spp_send_queue = xQueueCreate(SEND_QUEUE_SIZE, sizeof(send_message_t));
	xTaskCreate(send_task, "BLE_sendTask", 2048, NULL, 1, NULL);
	cmd_cmd_queue = xQueueCreate(SEND_QUEUE_SIZE, sizeof(uint32_t));
	xTaskCreate(spp_cmd_task, "spp_cmd_task", 2048, NULL, 1, NULL);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
	ESP_LOGE(BLE_TAG, "GAP_EVT, event %d", event);

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGE(BLE_TAG, "Advertising start failed: %s", esp_err_to_name(err));
        }
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
    uint8_t res = 0xff;

	ESP_LOGI(BLE_TAG, "event = %x",event);
    switch (event) {
    	case ESP_GATTS_REG_EVT:
			ESP_LOGI(BLE_TAG, "%s %d", __func__, __LINE__);
			esp_ble_gap_set_device_name(ble_gap_name);

			ESP_LOGI(BLE_TAG, "%s %d", __func__, __LINE__);
			esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, spp_adv_data[7] + 8);

			ESP_LOGI(BLE_TAG, "%s %d", __func__, __LINE__);
        	esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
       	break;
    	case ESP_GATTS_READ_EVT:
			res = find_char_and_desr_index(p_data->read.handle);
            if(res == SPP_IDX_SPP_STATUS_VAL){
                //TODO:client read the status characteristic
            }
       	 break;
    	case ESP_GATTS_WRITE_EVT: {
    	    res = find_char_and_desr_index(p_data->write.handle);
            if(p_data->write.is_prep == false){
				ESP_LOGI(BLE_TAG, "ESP_GATTS_WRITE_EVT : handle = %d", res);
                if(res == SPP_IDX_SPP_COMMAND_VAL){
                    uint8_t * spp_cmd_buff = NULL;
                    spp_cmd_buff = (uint8_t *)malloc((spp_mtu_size - 3) * sizeof(uint8_t));
                    if(spp_cmd_buff == NULL){
						ESP_LOGE(BLE_TAG, "%s malloc failed", __func__);
                        break;
                    }
                    memset(spp_cmd_buff,0x0,(spp_mtu_size - 3));
                    memcpy(spp_cmd_buff,p_data->write.value,p_data->write.len);
					xQueueSend(cmd_cmd_queue,&spp_cmd_buff,10/portTICK_PERIOD_MS);
                }else if(res == SPP_IDX_SPP_DATA_NTF_CFG){
                    if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
                        enable_notification();
                    }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
                        disable_notification();
                    }
                }
                else if(res == SPP_IDX_SPP_DATA_RECV_VAL){
                    server_callbacks.data_received((char *)(p_data->write.value), p_data->write.len);
                }else{
                    //TODO:
                }
            }else if((p_data->write.is_prep == true)&&(res == SPP_IDX_SPP_DATA_RECV_VAL)){
				ESP_LOGI(BLE_TAG, "ESP_GATTS_PREP_WRITE_EVT : handle = %d", res);
                store_wr_buffer(p_data);
            }
      	 	break;
    	}
    	case ESP_GATTS_EXEC_WRITE_EVT:{
			ESP_LOGI(BLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
    	    if(p_data->exec_write.exec_write_flag){
    	        send_buffered_message();
    	        free_write_buffer();
    	    }
    	    break;
    	}
    	case ESP_GATTS_MTU_EVT:
			spp_mtu_size = p_data->mtu.mtu;
    	    break;
    	case ESP_GATTS_CONF_EVT:
    	    break;
    	case ESP_GATTS_UNREG_EVT:
        	break;
    	case ESP_GATTS_DELETE_EVT:
        	break;
    	case ESP_GATTS_START_EVT:
        	break;
    	case ESP_GATTS_STOP_EVT:
        	break;
		case ESP_GATTS_CONNECT_EVT:
			spp_conn_id = p_data->connect.conn_id;
    	    spp_gatts_if = gatts_if;
			is_connected = true;
			memcpy(&spp_remote_bda,&p_data->connect.remote_bda,sizeof(esp_bd_addr_t));
			xSemaphoreGive(ble_congested);
        	break;
		case ESP_GATTS_DISCONNECT_EVT:
			is_connected = false;
			disable_notification();
			spp_mtu_size = DEFAULT_MTU_SIZE;
			esp_ble_gap_start_advertising(&spp_adv_params);
			break;
    	case ESP_GATTS_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CANCEL_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CLOSE_EVT:
    	    break;
    	case ESP_GATTS_LISTEN_EVT:
    	    break;
		case ESP_GATTS_CONGEST_EVT:
			if(p_data->connect.conn_id == spp_conn_id) {
				if(p_data->congest.congested) xSemaphoreTake(ble_congested, 1);
					else xSemaphoreGive(ble_congested);
			} else {
				ESP_LOGI(BLE_TAG, "Congestion: connection id does not match? %d", p_data->connect.conn_id);
			}
			break;
    	case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
			ESP_LOGI(BLE_TAG, "The number handle =%x",param->add_attr_tab.num_handle);
    	    if (param->add_attr_tab.status != ESP_GATT_OK){
				ESP_LOGE(BLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
    	    }
    	    else if (param->add_attr_tab.num_handle != SPP_IDX_NB){
    	        ESP_LOGE(BLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
    	    }
    	    else {
    	        memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
    	        esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
    	    }
    	    break;
    	}
    	default:
    	    break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	ESP_LOGI(BLE_TAG, "EVT %d, gatts if %d", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
			ESP_LOGI(BLE_TAG, "Reg app failed, app_id %04x, status %d",param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == spp_profile_tab[idx].gatts_if) {
                if (spp_profile_tab[idx].gatts_cb) {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void ble_server_setup(ble_server_callbacks callbacks)
{
	kill_ble_tasks = false;
	ble_congested = xSemaphoreCreateBinary();

    server_callbacks = callbacks;
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
		ESP_LOGE(BLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (ret) {
		ESP_LOGE(BLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

	ESP_LOGI(BLE_TAG, "%s init bluetooth", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
		ESP_LOGE(BLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
	ret = esp_bluedroid_enable();
    if (ret) {
		ESP_LOGE(BLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

	spp_task_init();

	return;
}

void ble_server_shutdown()
{
	//set kill flag
	kill_ble_tasks = true;

	//with kill flag set we need to send a message to activate the task
	send_message_t msg;
	msg.buffer = malloc(1);
	msg.msg_length = 10;
	xQueueSend(spp_send_queue, &msg, 50 / portTICK_PERIOD_MS);

	//with kill flag set we need to send a message to activate the task
	uint8_t* spp_cmd_buff = malloc(1);
	xQueueSend(cmd_cmd_queue,&spp_cmd_buff,10/portTICK_PERIOD_MS);

	//shutdown BLE
	esp_bluedroid_disable();
	esp_bluedroid_deinit();

	//delete our congestion semaphore
	vSemaphoreDelete(ble_congested);
}

void ble_send(uint32_t txID, uint32_t rxID, uint8_t flags, const void* src, size_t size)
{
	send_message_t msg;
	msg.buffer = malloc(size);
	msg.msg_length = size;
	msg.rxID = rxID;
	msg.txID = txID;
	msg.flags = flags;
	memcpy(msg.buffer, src, size);
	xQueueSend(spp_send_queue, &msg, 50 / portTICK_PERIOD_MS);
}

bool ble_connected()
{
	return is_connected;
}

uint16_t ble_queue_spaces()
{
	return uxQueueSpacesAvailable(spp_send_queue);
}

uint16_t ble_queue_waiting()
{
	return uxQueueMessagesWaiting(spp_send_queue);
}

void ble_set_delay_send(uint16_t delay)
{
	ble_delay_send = delay;
}

void ble_set_delay_multi(uint16_t delay)
{
	ble_delay_multi = delay;
}

uint16_t ble_get_delay_send()
{
	return ble_delay_send;
}

uint16_t ble_get_delay_multi()
{
	return ble_delay_multi;
}

bool ble_set_gap_name(char* name, bool set)
{
	if(name)
	{
		uint8_t len = strlen(name);
		if(len <= MAX_GAP_LENGTH)
		{
			memset((char*)&spp_adv_data[9], 0, MAX_GAP_LENGTH);
			memcpy((char*)&spp_adv_data[9], name, len);
			spp_adv_data[7] = len+1;

			strcpy(ble_gap_name, name);
			if(set)
				esp_ble_gap_set_device_name(ble_gap_name);
			ESP_LOGI(BLE_TAG, "Set GAP name [%s]", ble_gap_name);
		}
	}

	ESP_LOGI(BLE_TAG, "Unable to set GAP name");
	return false;
}

bool ble_get_gap_name(char* name)
{
	if(name)
	{
		strcpy(name, ble_gap_name);
		return true;
	}

	return false;
}

void ble_stop_advertising()
{
	esp_ble_gap_stop_advertising();
}

void ble_start_advertising()
{
	esp_ble_gap_start_advertising(&spp_adv_params);
}
