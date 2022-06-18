#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/twai.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "soc/dport_reg.h"
#include "isotp.h"
#include "ble_server.h"
#include "isotp_link_containers.h"
#include "twai.h"
#include "mutexes.h"
#include "queues.h"
#include "persist.h"
#include "constants.h"
#include "led.h"
#include "eeprom.h"
#include "uart.h"
#include "connection_handler.h"

#define BRIDGE_TAG 					"Bridge"

/* ------------------------ global vars ------------------------------------ */
static SemaphoreHandle_t	isotp_send_task_mutex	= NULL;
static bool16				isotp_run_tasks			= false;
static ble_header_t			split_header;
static uint16_t				split_enabled 			= false;
static uint8_t 				split_count 			= 0;
static uint16_t				split_length 			= 0;
static uint8_t*				split_data 				= NULL;
#if PASSWORD_CHECK
static bool16				passwordChecked			= false;
#else
static bool16				passwordChecked			= true;
#endif


/* ---------------------------- ISOTP Callbacks ---------------------------- */

int isotp_user_send_can(const uint32_t arbitration_id, const uint8_t* data, const uint16_t size) {
    twai_message_t frame = {.identifier = arbitration_id, .data_length_code = size};
    memcpy(frame.data, data, sizeof(frame.data));
	xQueueSend(can_send_queue, &frame, portMAX_DELAY);

    return ISOTP_RET_OK;                           
}

uint64_t isotp_user_get_us(void) {
	return esp_timer_get_time();
}

void isotp_user_debug(const char* message, ...) {
	ESP_LOGD(BRIDGE_TAG, "ISOTP: %s", message);
}

/* --------------------------- Functions --------------------------------- */

bool16 setCpuFrequencyMhz(uint16_t max, uint16_t min)
{
	//set config
	esp_pm_config_esp32_t cpu_config;
	cpu_config.max_freq_mhz = max;
	cpu_config.min_freq_mhz = min;
	cpu_config.light_sleep_enable = false;

	//set cpu speed and return result
	return esp_pm_configure(&cpu_config);
}

bool16 check_password(char* data)
{
	if(data) {
		char* pass = eeprom_read_str(PASSWORD_KEY);
		if(pass) {
			if(!strcmp(data, pass))
				passwordChecked = true;

			free(pass);
		} else {
			eeprom_write_str(PASSWORD_KEY, PASSWORD_DEFAULT);
			eeprom_commit();

			if(!strcmp(data, PASSWORD_DEFAULT))
				passwordChecked = true;
		}
	}

	return passwordChecked;
}

void write_password(char* data)
{
	eeprom_write_str(PASSWORD_KEY, data);
	eeprom_commit();
	ESP_LOGI(BRIDGE_TAG, "Set password [%s]", data);
}

void send_packet(uint32_t txID, uint32_t rxID, uint8_t flags, const void* src, size_t size)
{
	if(ble_connected()) {
		ble_send(txID, rxID, flags, src, size);
	} else {
		uart_send(txID, rxID, flags, src, size);
	}
}

/* --------------------------- ISOTP Tasks -------------------------------------- */

static void isotp_processing_task(void *arg)
{
    IsoTpLinkContainer *isotp_link_container = (IsoTpLinkContainer*)arg;
    IsoTpLink *link_ptr = &isotp_link_container->link;
	uint8_t *payload_buf = isotp_link_container->payload_buf;
	uint16_t number = isotp_link_container->number;

	xSemaphoreTake(isotp_link_container->task_mutex, portMAX_DELAY);
	ESP_LOGI(BRIDGE_TAG, "Receive task started: %d", number);
	xSemaphoreGive(sync_task_sem);
    while (isotp_run_tasks)
    {
        if (link_ptr->send_status != ISOTP_SEND_STATUS_INPROGRESS &&
            link_ptr->receive_status != ISOTP_RECEIVE_STATUS_INPROGRESS)
        {
            // Link is idle, wait for new data before pumping loop.
			xSemaphoreTake(isotp_link_container->wait_for_isotp_data_sem, portMAX_DELAY);
		}
        // poll
		xSemaphoreTake(isotp_mutex, pdMS_TO_TICKS(TIMEOUT_NORMAL));
        isotp_poll(link_ptr);
        xSemaphoreGive(isotp_mutex);
        // receive
		xSemaphoreTake(isotp_mutex, pdMS_TO_TICKS(TIMEOUT_NORMAL));
        uint16_t out_size;
        int ret = isotp_receive(link_ptr, payload_buf, isotp_link_container->buffer_size, &out_size);
        xSemaphoreGive(isotp_mutex);
        // if it is time to send fully received + parsed ISO-TP data over BLE and/or websocket
        if (ISOTP_RET_OK == ret) {
			ESP_LOGI(BRIDGE_TAG, "Received ISO-TP message with length: %04X", out_size);
            for (int i = 0; i < out_size; i++) {
				ESP_LOGD(BRIDGE_TAG, "payload_buf[%d] = %02x", i, payload_buf[i]);
			}

			//Are we in persist mode?
			if(number < PERSIST_COUNT && persist_enabled())
			{
				//send time stamp instead of rx/tx
				uint32_t time = (esp_timer_get_time() / 1000UL) & 0xFFFFFFFF;
				uint16_t rxID = (time >> 16) & 0xFFFF;
				uint16_t txID = time & 0xFFFF;
				send_packet(txID, rxID, 0, payload_buf, out_size);
				xSemaphoreGive(persist_message_send[number]);
			} else {
				send_packet(link_ptr->receive_arbitration_id, link_ptr->send_arbitration_id, 0, payload_buf, out_size);
			}
        }
		taskYIELD(); // Allow higher priority tasks to run, for example Rx/Tx
    }
	ESP_LOGI(BRIDGE_TAG, "Receive task stopped: %d", number);
	xSemaphoreGive(isotp_link_container->task_mutex);
    vTaskDelete(NULL);
}

static void isotp_send_queue_task(void *arg)
{
	xSemaphoreTake(isotp_send_task_mutex, portMAX_DELAY);
	ESP_LOGI(BRIDGE_TAG, "Send task started");
	xSemaphoreGive(sync_task_sem);
    while (isotp_run_tasks)
    {
        send_message_t msg;
		if (xQueueReceive(isotp_send_message_queue, &msg, portMAX_DELAY) == pdTRUE) {
			if (isotp_run_tasks) {
				xSemaphoreTake(isotp_mutex, pdMS_TO_TICKS(TIMEOUT_NORMAL));
				ESP_LOGI(BRIDGE_TAG, "isotp_send_queue_task: sending message with %d size (rx id: %04x / tx id: %04x)", msg.msg_length, msg.rxID, msg.txID);
				for (uint16_t i = 0; i < NUM_ISOTP_LINK_CONTAINERS; i++) {
					IsoTpLinkContainer* isotp_link_container = &isotp_link_containers[i];
					if (msg.txID == isotp_link_container->link.receive_arbitration_id &&
						msg.rxID == isotp_link_container->link.send_arbitration_id) {
						ESP_LOGI(BRIDGE_TAG, "container match [%d]", i);
						isotp_link_container_id = i;
						isotp_send(&isotp_link_container->link, msg.buffer, msg.msg_length);
						xSemaphoreGive(isotp_mutex);
						xSemaphoreGive(isotp_link_container->wait_for_isotp_data_sem);
						break;
					}
				}
				free(msg.buffer);
			}
		}
		taskYIELD();
	}
	ESP_LOGI(BRIDGE_TAG, "Send task stopped");
	xSemaphoreGive(isotp_send_task_mutex);
    vTaskDelete(NULL);
}

/* --------------------------- ISOTP Functions -------------------------------------- */
void isotp_stop_task()
{
	if (isotp_run_tasks) {
		isotp_run_tasks = false;
		for (uint16_t i = 0; i < NUM_ISOTP_LINK_CONTAINERS; i++) {
			IsoTpLinkContainer* isotp_link_container = &isotp_link_containers[i];
			xSemaphoreGive(isotp_link_container->wait_for_isotp_data_sem);
			xSemaphoreTake(isotp_link_container->task_mutex, portMAX_DELAY);
			xSemaphoreGive(isotp_link_container->task_mutex);
		}

		send_message_t msg;
		xQueueSend(isotp_send_message_queue, &msg, portMAX_DELAY);
		xSemaphoreTake(isotp_send_task_mutex, portMAX_DELAY);
		xSemaphoreGive(isotp_send_task_mutex);
		ESP_LOGI(BRIDGE_TAG, "Tasks stopped");
	}
}

void isotp_start_task()
{
	isotp_stop_task();

	isotp_run_tasks = true;

	ESP_LOGI(BRIDGE_TAG, "Tasks starting");
	xSemaphoreTake(sync_task_sem, 0);
	// create tasks for each isotp_link
	for (uint16_t i = 0; i < NUM_ISOTP_LINK_CONTAINERS; i++)
	{
		IsoTpLinkContainer* isotp_link_container = &isotp_link_containers[i];
		xTaskCreate(isotp_processing_task, isotp_link_container->name, TASK_STACK_SIZE, isotp_link_container, ISOTP_TSK_PRIO, NULL);
		xSemaphoreTake(sync_task_sem, portMAX_DELAY);
	}

	// "ISOTP_process" pumps the ISOTP library's "poll" method, which will call the send queue callback if a message needs to be sent.
	// ISOTP_process also polls the ISOTP library's non-blocking receive method, which will produce a message if one is ready.
	xTaskCreate(isotp_send_queue_task, "ISOTP_process_send_queue", TASK_STACK_SIZE, NULL, MAIN_TSK_PRIO, NULL);
	xSemaphoreTake(sync_task_sem, portMAX_DELAY);
	ESP_LOGI(BRIDGE_TAG, "Tasks started");
}

void isotp_deinit()
{
	bool16 didDeinit = false;

	if (isotp_mutex) {
		vSemaphoreDelete(isotp_mutex);
		isotp_mutex = NULL;
		didDeinit = true;
	}

	if (isotp_send_task_mutex) {
		vSemaphoreDelete(isotp_send_task_mutex);
		isotp_send_task_mutex = NULL;
		didDeinit = true;
	}

	if (isotp_send_message_queue) {
		//clear and delete queue
		send_message_t msg;
		while (xQueueReceive(isotp_send_message_queue, &msg, 0) == pdTRUE)
			if (msg.buffer)
				free(msg.buffer);
		vQueueDelete(isotp_send_message_queue);
		isotp_send_message_queue = NULL;
		didDeinit = true;
	}

	disable_isotp_links();

	if (didDeinit)
		ESP_LOGI(BRIDGE_TAG, "Deinit");
}

void isotp_init()
{
	isotp_mutex					= xSemaphoreCreateMutex();
	isotp_send_task_mutex		= xSemaphoreCreateMutex();
	isotp_send_message_queue	= xQueueCreate(ISOTP_QUEUE_SIZE, sizeof(send_message_t));

	configure_isotp_links();
}

/* ----------- Receive packet functions ---------------- */

void split_clear()
{
	memset(&split_header, 0, sizeof(ble_header_t));
	split_enabled = false;
	split_count = 0;
	split_length = 0;
	if(split_data) {
		free(split_data);
		split_data = NULL;
	}
}

bool16 parse_packet(ble_header_t* header, uint8_t* data)
{
	if(passwordChecked) {
		//Is client trying to set a setting?
		if(header->cmdFlags & BLE_COMMAND_FLAG_SETTINGS)
		{
			//Are we setting or getting?
			if(header->cmdFlags & BLE_COMMAND_FLAG_SETTINGS_GET)
			{   //Make sure payload is empty
				if(header->cmdSize == 0)
				{
					//send requested information
					switch(header->cmdFlags ^ (BLE_COMMAND_FLAG_SETTINGS | BLE_COMMAND_FLAG_SETTINGS_GET))
					{
						case BRG_SETTING_ISOTP_STMIN:
							for(uint16_t i = 0; i < NUM_ISOTP_LINK_CONTAINERS; i++)
							{
								IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[i];
								if(header->rxID == isotp_link_container->link.receive_arbitration_id &&
									header->txID == isotp_link_container->link.send_arbitration_id)
								{
									uint16_t stmin = isotp_link_container->link.stmin_override;
									ESP_LOGI(BRIDGE_TAG, "Sending stmin [%04X] from container [%02X]", stmin, i);
									send_packet(isotp_link_container->link.receive_arbitration_id, isotp_link_container->link.send_arbitration_id, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_ISOTP_STMIN, &stmin, sizeof(uint16_t));
								}
							}
							break;
						case BRG_SETTING_LED_COLOR:
							{
								uint32_t color = led_getcolor();
								ESP_LOGI(BRIDGE_TAG, "Sending color [%06X]", color);
								send_packet(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_LED_COLOR, &color, sizeof(uint32_t));
							}
							break;
						case BRG_SETTING_PERSIST_DELAY:
							{
								uint16_t delay = persist_get_delay();
								ESP_LOGI(BRIDGE_TAG, "Sending persist delay [%04X]", delay);
								send_packet(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_PERSIST_DELAY, &delay, sizeof(uint16_t));
							}
							break;
						case BRG_SETTING_PERSIST_Q_DELAY:
							{
								uint16_t delay = persist_get_q_delay();
								ESP_LOGI(BRIDGE_TAG, "Sending persist queue delay [%04X]", delay);
								send_packet(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_PERSIST_Q_DELAY, &delay, sizeof(uint16_t));
							}
							break;
						case BRG_SETTING_BLE_SEND_DELAY:
							{
								uint16_t delay = ble_get_delay_send();
								ESP_LOGI(BRIDGE_TAG, "Sending BLE send delay [%04X]", delay);
								send_packet(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_BLE_SEND_DELAY, &delay, sizeof(uint16_t));
							}
							break;
						case BRG_SETTING_BLE_MULTI_DELAY:
							{
								uint16_t delay = ble_get_delay_multi();
								ESP_LOGI(BRIDGE_TAG, "Sending BLE send delay [%04X]", delay);
								send_packet(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_BLE_MULTI_DELAY, &delay, sizeof(uint16_t));
							}
							break;
						case BRG_SETTING_GAP:
							{
								char str[MAX_GAP_LENGTH+1];
								ble_get_gap_name(str);
								uint8_t len = strlen(str);
								send_packet(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_GAP, &str, len);
								ESP_LOGI(BRIDGE_TAG, "Set GAP [%s]", str);
							}
							break;
						case BRG_SETTING_DQ3XX:
							{
								IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[1];
								uint16_t dq3xx = isotp_link_container->link.dq3xx_hack;
								ESP_LOGI(BRIDGE_TAG, "Sending DQ3XX [%04X] from DSG container", dq3xx);
								send_packet(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_DQ3XX, &dq3xx, sizeof(uint16_t));
							}
							break;
					}
					return true;
				}
			} else { //Setting
				switch(header->cmdFlags ^ BLE_COMMAND_FLAG_SETTINGS)
				{
					case BRG_SETTING_ISOTP_STMIN:
						//check size
						if(header->cmdSize == sizeof(uint16_t))
						{   //match rx/tx
							for(uint16_t i = 0; i < NUM_ISOTP_LINK_CONTAINERS; i++)
							{
								IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[i];
								if(header->rxID == isotp_link_container->link.receive_arbitration_id &&
									header->txID == isotp_link_container->link.send_arbitration_id)
								{
									uint16_t* stmin = (uint16_t*)data;
									isotp_link_container->link.stmin_override = *stmin;
									ESP_LOGI(BRIDGE_TAG, "Set stmin [%04X] on container [%02X]", *stmin, i);
									return true;
								}
							}
						}
						break;
					case BRG_SETTING_LED_COLOR:
						//check size
						if(header->cmdSize == sizeof(uint32_t))
						{
							uint32_t* color = (uint32_t*)data;
							led_setcolor(*color);
							ESP_LOGI(BRIDGE_TAG, "Set led color [%08X]", *color);
							return true;
						}
						break;
					case BRG_SETTING_PERSIST_DELAY:
						//check size
						if(header->cmdSize == sizeof(uint16_t))
						{   //confirm correct command size
							uint16_t* delay = (uint16_t*)data;
							persist_take_all_mutex();
							persist_set_delay(*delay);
							persist_give_all_mutex();
							ESP_LOGI(BRIDGE_TAG, "Set persist delay [%08X]", *delay);
							return true;
						}
						break;
					case BRG_SETTING_PERSIST_Q_DELAY:
						//check size
						if(header->cmdSize == sizeof(uint16_t))
						{   //confirm correct command size
							uint16_t* delay = (uint16_t*)data;
							persist_take_all_mutex();
							persist_set_q_delay(*delay);
							persist_give_all_mutex();
							ESP_LOGI(BRIDGE_TAG, "Set persist queue delay [%08X]", *delay);
							return true;
						}
						break;
					case BRG_SETTING_BLE_SEND_DELAY:
						//check size
						if(header->cmdSize == sizeof(uint16_t))
						{   //confirm correct command size
							uint16_t* delay = (uint16_t*)data;
							ble_set_delay_send(*delay);
							ESP_LOGI(BRIDGE_TAG, "Set BLE send delay [%08X]", *delay);
							return true;
						}
						break;
					case BRG_SETTING_BLE_MULTI_DELAY:
						//check size
						if(header->cmdSize == sizeof(uint16_t))
						{   //confirm correct command size
							uint16_t* delay = (uint16_t*)data;
							ble_set_delay_multi(*delay);
							ESP_LOGI(BRIDGE_TAG, "Set BLE wait for queue item [%08X]", *delay);
							return true;
						}
						break;
					case BRG_SETTING_PASSWORD:
						//check size
						if(header->cmdSize <= MAX_PASSWORD_LENGTH)
						{   //confirm correct command size
							char* str = malloc(header->cmdSize+1);
							if(str) {
								str[header->cmdSize] = 0;
								memcpy(str, data, header->cmdSize);
								write_password((char*)str);
								free(str);
								return true;
							}
						}
						break;
					case BRG_SETTING_GAP:
						if(header->cmdSize <= MAX_GAP_LENGTH)
						{   //confirm correct command size
							char str[MAX_GAP_LENGTH+1];
                            memcpy(str, (char*)data, header->cmdSize);
							str[header->cmdSize] = 0;
							ble_set_gap_name(str, true);
							eeprom_write_str(BLE_GAP_KEY, str);
							eeprom_commit();
							xSemaphoreGive(ch_sleep_sem);
							return true;
						}
						break;
					case BRG_SETTING_DQ3XX:
						if(header->cmdSize == sizeof(uint16_t))
						{
							uint16_t dq3xx = *(uint16_t*)data;
							IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[1];
							isotp_link_container->link.dq3xx_hack = dq3xx;
							ESP_LOGI(BRIDGE_TAG, "Set DQ3XX [%04X] for DSG container", dq3xx);
							return true;
						}
						break;
				}
			}
		} else {
			persist_take_all_mutex();
			if (persist_enabled()) {
				//We are in persistent mode
				//Should we clear the persist messages in memory?
				if (header->cmdFlags & BLE_COMMAND_FLAG_PER_CLEAR)
				{
					persist_clear();
				}

				//Should we disable persist mode?
				if ((header->cmdFlags & BLE_COMMAND_FLAG_PER_ENABLE) == 0)
				{
					persist_set(false);
				}
				else {
					//If we are still in persist mode only accept setting changes
					persist_give_all_mutex();
					return false;
				}
			} else {
				//Not in persistent mode
				if (header->cmdFlags & BLE_COMMAND_FLAG_PER_CLEAR)
				{
					persist_clear();
				}

				if (header->cmdFlags & BLE_COMMAND_FLAG_PER_ADD)
				{
					persist_add(header->txID, header->rxID, data, header->cmdSize);
				}

				if (header->cmdFlags & BLE_COMMAND_FLAG_PER_ENABLE)
				{
					persist_set(true);
					persist_give_all_mutex();
					return false;
				}
			}
			persist_give_all_mutex();

			if (header->cmdSize)
			{
				if (!persist_enabled())
				{
					ESP_LOGI(BRIDGE_TAG, "Received message [%04X]", header->cmdSize);

					send_message_t msg;
					msg.msg_length = header->cmdSize;
					msg.rxID = header->txID;
					msg.txID = header->rxID;
					msg.buffer = malloc(header->cmdSize);
					if (msg.buffer) {
						memcpy(msg.buffer, data, header->cmdSize);

						if (xQueueSend(isotp_send_message_queue, &msg, pdMS_TO_TICKS(TIMEOUT_NORMAL)) != pdTRUE) {
							free(msg.buffer);
						}
					}
					else {
						ESP_LOGD(BRIDGE_TAG, "parse_packet: malloc fail size(%d)", header->cmdSize);
						return false;
					}
				}

				return true;
			}
		}
	} else {
		//password has not be accepted yet, check for password
		if(header->cmdFlags & BRG_SETTING_PASSWORD & BLE_COMMAND_FLAG_SETTINGS & BLE_COMMAND_FLAG_SETTINGS_GET) {
			//check size
			if(header->cmdSize <= MAX_PASSWORD_LENGTH)
			{
				char* str = malloc(header->cmdSize+1);
				if(str) {
					str[header->cmdSize] = 0;
					memcpy(str, data, header->cmdSize);

					char c = 0;
					if(check_password((char*)data)) {
						ESP_LOGI(BRIDGE_TAG, "Password accepted [%s]", data);
						c = 0xFF;
					}

					send_packet(0xFF, 0xFF, 0xFF, &c, 1);
					free(str);
				}
				return false;
			}
		} else {
			ESP_LOGI(BRIDGE_TAG, "No access");
		}
	}

	return false;
}

void packet_received(const void* src, size_t size)
{
	//store current data pointer
	uint8_t* data = (uint8_t*)src;

	//Are we in Split packet mode?
	if(split_enabled && data[0] == BLE_PARTIAL_ID) {
		//check packet count
		if(data[1] == split_count) {
			if (split_data && split_length) {
				ESP_LOGI(BRIDGE_TAG, "Split packet [%02X] adding [%02X]", split_count, size - 2);

				uint8_t* new_data = malloc(split_length + size - 2);
				if (new_data == NULL) {
					ESP_LOGI(BRIDGE_TAG, "malloc error %s %d", __func__, __LINE__);
					split_clear();
					return;
				}
				memcpy(new_data, split_data, split_length);
				memcpy(new_data + split_length, data + 2, size - 2);
				free(split_data);
				split_data = new_data;
				split_length += size - 2;
				split_count++;

				//got complete message
				if (split_length == split_header.cmdSize)
				{   //Messsage size matches
					ESP_LOGI(BRIDGE_TAG, "Split packet size matches [%02X]", split_length);
					parse_packet(&split_header, split_data);
					split_clear();
				}
				else if (split_length > split_header.cmdSize)
				{   //Message size does not match
					ESP_LOGI(BRIDGE_TAG, "Command size is larger than packet size [%02X, %02X]", split_header.cmdSize, split_length);
					split_clear();
				}
			} else {
				//error delete and forget
				ESP_LOGI(BRIDGE_TAG, "Splitpacket data is invalid");
				split_clear();
			}
		} else {
			//error delete and forget
			ESP_LOGI(BRIDGE_TAG, "Splitpacket out of order [%02X, %02X]", data[1], split_count);
			split_clear();
		}
	} else {
		//disable splitpacket
		split_enabled = false;

		//If the packet does not contain header abort
		while(size >= sizeof(ble_header_t))
		{
			//Confirm the header is valid
			ble_header_t* header = (ble_header_t*)data;
			if(header->hdID != BLE_HEADER_ID)
			{
				ESP_LOGI(BRIDGE_TAG, "Packet header does not match [%02X, %02X]", header->hdID, BLE_HEADER_ID);
				return;
			}

			//header has pointer, skip past header in data
			data += sizeof(ble_header_t);
			size -= sizeof(ble_header_t);

			//Confirm data size is legit
			if(header->cmdSize > size)
			{
				//Are they requesting splitpacket?
				if(header->cmdFlags & BLE_COMMAND_FLAG_SPLIT_PK)
				{
					ESP_LOGI(BRIDGE_TAG, "Starting split packet [%02X]", header->cmdSize);
					split_clear();
					split_enabled = true;
					split_data = malloc(size);
					if(split_data == NULL){
						ESP_LOGI(BRIDGE_TAG, "malloc error %s %d", __func__, __LINE__);
						split_clear();
						return;
					}
					memcpy(split_data, data, size);
					memcpy(&split_header, header, sizeof(ble_header_t));
					split_count = 1;
					split_length = size;
					return;
				} else {
					ESP_LOGI(BRIDGE_TAG, "Command size is larger than packet size [%02X, %02X]", header->cmdSize, size);
					return;
				}
			}

			//looks good, parse the packet
			if(parse_packet(header, data))
			{
				data += header->cmdSize;
				size -= header->cmdSize;
			} else {
				return;
			}
		}
	}
}

void received_from_ble(const void* src, size_t size)
{
	packet_received(src, size);
}

void uart_data_received(const void* src, size_t size)
{
	if(!ble_connected()) {
		ch_reset_uart_timer();
		packet_received(src, size);
#if UART_ECHO
		send_packet(0xFF, 0xFF, 0, src+sizeof(ble_header_t), size-sizeof(ble_header_t));
#endif
	}
}

/* ----------- BLE/UART callbacks ---------------- */

void bridge_connect() 
{
	//clear persist
	persist_take_all_mutex();
	persist_clear();
	persist_give_all_mutex();

	//set to green
	led_setcolor(LED_GREEN_QRT);

	//set full speed
	setCpuFrequencyMhz(240, 40);

#if PASSWORD_CHECK
	//disable password support
	passwordChecked = false;
#endif
}

void bridge_disconnect()
{
	//clear persist
	persist_take_all_mutex();
	persist_clear();
	persist_give_all_mutex();

	//set led to low red
	led_setcolor(LED_RED_EHT);

	//set slow speed
	setCpuFrequencyMhz(80, 10);

#if PASSWORD_CHECK
	//disable password support
	passwordChecked = false;
#endif
}

void ch_on_uart_connect()
{
	bridge_connect();
}

void ch_on_uart_disconnect()
{
	bridge_disconnect();
}

void ble_notifications_enabled()
{
	bridge_connect();
}

void ble_notifications_disabled()
{
	bridge_disconnect();
}

/* ------------ Primary startup ---------------- */

void app_main(void)
{
	//set slow speed
	setCpuFrequencyMhz(80, 10);

	sync_task_sem = xSemaphoreCreateBinary();

	//start eeprom and read values
	eeprom_init();
	char* gapName = eeprom_read_str(BLE_GAP_KEY);
	if(gapName) {
		ble_set_gap_name(gapName, false);
		free(gapName);
	}

	//setup BLE server
    ble_server_callbacks callbacks = {
		.data_received = received_from_ble,
		.notifications_subscribed = ble_notifications_enabled,
		.notifications_unsubscribed = ble_notifications_disabled
    };
	ble_server_setup(callbacks);

	//init
	ch_init();
	led_init();
	uart_init();
	twai_init();
	isotp_init();
	persist_init();

	//start tasks
	twai_start_task();
	isotp_start_task();
	persist_start_task();
	uart_start_task();
	ch_start_task();

	//wait for sleep command
	xSemaphoreTake(ch_sleep_sem, portMAX_DELAY);

	//stop tasks
	ch_stop_task();
	uart_stop_task();
	persist_stop_task();
	isotp_stop_task();
	twai_stop_task();

	//deinit
	persist_deinit();
	isotp_deinit();
	twai_deinit();
	uart_deinit();
	led_deinit();
	ch_deinit();
	ble_server_shutdown();

	vSemaphoreDelete(sync_task_sem);

	//setup sleep timer
	esp_sleep_enable_timer_wakeup(SLEEP_TIME * US_TO_S);
	ESP_LOGI(BRIDGE_TAG, "CAN Timeout - Sleeping [%ds]", SLEEP_TIME);

	//Go to sleep now
	esp_deep_sleep_start();
}
