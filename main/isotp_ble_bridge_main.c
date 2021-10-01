#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/twai.h"
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

#define BRIDGE_TAG 					"Bridge"

/* ---------------------------- ISOTP Callbacks ---------------------------- */

int isotp_user_send_can(const uint32_t arbitration_id, const uint8_t* data, const uint16_t size) {
    twai_message_t frame = {.identifier = arbitration_id, .data_length_code = size};
    memcpy(frame.data, data, sizeof(frame.data));
	xQueueSend(tx_task_queue, &frame, portMAX_DELAY);
    return ISOTP_RET_OK;                           
}

uint64_t isotp_user_get_us(void) {
	return esp_timer_get_time();
}

void isotp_user_debug(const char* message, ...) {
}

/* --------------------------- Tasks and Functions -------------------------- */

static void isotp_processing_task(void *arg)
{
    IsoTpLinkContainer *isotp_link_container = (IsoTpLinkContainer*)arg;
    IsoTpLink *link_ptr = &isotp_link_container->link;;
	uint8_t *payload_buf = isotp_link_container->payload_buf;
    while (1)
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
        int ret = isotp_receive(link_ptr, payload_buf, ISOTP_BUFSIZE, &out_size);
        xSemaphoreGive(isotp_mutex);
        // if it is time to send fully received + parsed ISO-TP data over BLE and/or websocket
        if (ISOTP_RET_OK == ret) {
			ESP_LOGI(BRIDGE_TAG, "Received ISO-TP message with length: %04X", out_size);
            for (int i = 0; i < out_size; i++) {
                ESP_LOGD(BRIDGE_TAG, "payload_buf[%d] = %02x", i, payload_buf[i]);
			}

			//Are we in persist mode?
			if(persist_enabled())
			{
				//send time stamp instead of rx/tx
				uint32_t time = (esp_timer_get_time() / 1000UL) & 0xFFFFFFFF;
				uint16_t rxID = (time >> 16) & 0xFFFF;
				uint16_t txID = time & 0xFFFF;
				ble_send(txID, rxID, 0, payload_buf, out_size);
				xSemaphoreGive(persist_message_send);
			} else {
				ble_send(link_ptr->receive_arbitration_id, link_ptr->send_arbitration_id, 0, payload_buf, out_size);
			}
        }
        vTaskDelay(0); // Allow higher priority tasks to run, for example Rx/Tx
    }
    vTaskDelete(NULL);
}

static void isotp_send_queue_task(void *arg)
{
    xSemaphoreTake(isotp_send_queue_sem, portMAX_DELAY);
    while (1)
    {
        twai_status_info_t status_info;
        twai_get_status_info(&status_info);
        if (status_info.state == TWAI_STATE_BUS_OFF)
        {
            twai_initiate_recovery();
        }
        else if (status_info.state == TWAI_STATE_STOPPED)
        {
            twai_start();
        }
        send_message_t msg;
		xQueueReceive(isotp_send_message_queue, &msg, portMAX_DELAY);
		xSemaphoreTake(isotp_mutex, pdMS_TO_TICKS(TIMEOUT_NORMAL));
		ESP_LOGI(BRIDGE_TAG, "isotp_send_queue_task: sending message with %d size (rx id: %04x / tx id: %04x)", msg.msg_length, msg.rxID, msg.txID);
		for(uint16_t i = 0; i < NUM_ISOTP_LINK_CONTAINERS; i++) {
			IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[i];
			if(msg.txID == isotp_link_container->link.receive_arbitration_id &&
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
    vTaskDelete(NULL);
}

/* ----------- BLE callbacks ---------------- */

ble_header_t 	split_header;
uint16_t		split_enabled = false;
uint8_t 		split_count = 0;
uint16_t 		split_length = 0;
uint8_t* 		split_data = NULL;

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

bool parse_packet(ble_header_t* header, uint8_t* data)
{
	//Is client trying to set a setting?
	if(header->cmdFlags & BLE_COMMAND_FLAG_SETTINGS)
	{
		//Are we settings or sending a setting?
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
								uint16_t* stmin = malloc(sizeof(uint16_t));
								*stmin = isotp_link_container->link.stmin_override;
								ESP_LOGI(BRIDGE_TAG, "Sending stmin [%04X] from container [%02X]", *stmin, i);
								ble_send(isotp_link_container->link.receive_arbitration_id, isotp_link_container->link.send_arbitration_id, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_ISOTP_STMIN, stmin, sizeof(uint16_t));
							}
						}
						break;
					case BRG_SETTING_LED_COLOR:
						{
							uint32_t* color = malloc(sizeof(uint32_t));
							*color = led_getcolor();
							ESP_LOGI(BRIDGE_TAG, "Sending color [%06X]", *color);
							ble_send(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_LED_COLOR, color, sizeof(uint32_t));
						}
						break;
					case BRG_SETTING_PERSIST_DELAY:
						{
							uint16_t* delay = malloc(sizeof(uint16_t));
							*delay = persist_get_delay();
							ESP_LOGI(BRIDGE_TAG, "Sending persist delay [%04X]", *delay);
							ble_send(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_PERSIST_DELAY, delay, sizeof(uint16_t));
						}
						break;
					case BRG_SETTING_PERSIST_Q_DELAY:
						{
							uint16_t* delay = malloc(sizeof(uint16_t));
							*delay = persist_get_q_delay();
							ESP_LOGI(BRIDGE_TAG, "Sending persist queue delay [%04X]", *delay);
							ble_send(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_PERSIST_Q_DELAY, delay, sizeof(uint16_t));
						}
						break;
					case BRG_SETTING_BLE_SEND_DELAY:
						{
							uint16_t* delay = malloc(sizeof(uint16_t));
							*delay = ble_get_delay_send();
							ESP_LOGI(BRIDGE_TAG, "Sending BLE send delay [%04X]", *delay);
							ble_send(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_BLE_SEND_DELAY, delay, sizeof(uint16_t));
						}
						break;
					case BRG_SETTING_BLE_MULTI_DELAY:
						{
							uint16_t* delay = malloc(sizeof(uint16_t));
							*delay = ble_get_delay_multi();
							ESP_LOGI(BRIDGE_TAG, "Sending BLE send delay [%04X]", *delay);
							ble_send(0, 0, BLE_COMMAND_FLAG_SETTINGS | BRG_SETTING_BLE_MULTI_DELAY, delay, sizeof(uint16_t));
						}
						break;
				}
				return true;
			}
		} else {
			switch(header->cmdFlags ^ BLE_COMMAND_FLAG_SETTINGS)
			{
				case BRG_SETTING_ISOTP_STMIN:
					//check size
					if(header->cmdSize == 2)
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
					if(header->cmdSize == 4)
					{
						uint32_t* color = (uint32_t*)data;
						led_setcolor(*color);
						ESP_LOGI(BRIDGE_TAG, "Set led color [%08X]", *color);
						return true;
					}
					break;
				case BRG_SETTING_PERSIST_DELAY:
					//check size
					if(header->cmdSize == 2)
					{   //confirm correct command size
						uint16_t* delay = (uint16_t*)data;
						persist_set_delay(*delay);
						ESP_LOGI(BRIDGE_TAG, "Set persist delay [%08X]", *delay);
						return true;
					}
					break;
				case BRG_SETTING_PERSIST_Q_DELAY:
					//check size
					if(header->cmdSize == 2)
					{   //confirm correct command size
						uint16_t* delay = (uint16_t*)data;
						persist_set_delay(*delay);
						ESP_LOGI(BRIDGE_TAG, "Set persist queue delay [%08X]", *delay);
						return true;
					}
					break;
				case BRG_SETTING_BLE_SEND_DELAY:
					//check size
					if(header->cmdSize == 2)
					{   //confirm correct command size
						uint16_t* delay = (uint16_t*)data;
						ble_set_delay_send(*delay);
						ESP_LOGI(BRIDGE_TAG, "Set BLE send delay [%08X]", *delay);
						return true;
					}
					break;
				case BRG_SETTING_BLE_MULTI_DELAY:
					//check size
					if(header->cmdSize == 2)
					{   //confirm correct command size
						uint16_t* delay = (uint16_t*)data;
						ble_set_delay_multi(*delay);
						ESP_LOGI(BRIDGE_TAG, "Set BLE wait for queue item [%08X]", *delay);
						return true;
					}
					break;
			}
		}
	} else if(persist_enabled())
	{   //Are we in persistent mode?
		//Should we clear the persist messages in memory?
		if(header->cmdFlags & BLE_COMMAND_FLAG_PER_CLEAR)
		{
			persist_clear();
		}

		//Should we disable persist mode?
		if((header->cmdFlags & BLE_COMMAND_FLAG_PER_ENABLE) == 0)
		{
			persist_set(false);
		} else {
			//If we are still in persist mode only accept LED color change
			return false;
		}
	} else
	{ 	//Not in persistent mode
		if(header->cmdFlags & BLE_COMMAND_FLAG_PER_CLEAR)
		{
			persist_clear();
		}

		if(header->cmdFlags & BLE_COMMAND_FLAG_PER_ADD)
		{
			persist_add(data, header->cmdSize);
		}

		if(header->cmdFlags & BLE_COMMAND_FLAG_PER_ENABLE)
		{
			persist_set(true);
			return false;
		}
	}

	if(header->cmdSize)
	{
		if(!persist_enabled())
		{
			ESP_LOGI(BRIDGE_TAG, "Received message [%04X]", header->cmdSize);

			send_message_t msg;
			msg.msg_length = header->cmdSize;
			msg.rxID = header->txID;
			msg.txID = header->rxID;
			msg.buffer = malloc(header->cmdSize);
			memcpy(msg.buffer, data, header->cmdSize);

			xQueueSend(isotp_send_message_queue, &msg, pdMS_TO_TICKS(TIMEOUT_NORMAL));
		}

		return true;
	}

	return false;
}

void received_from_ble(const void* src, size_t size)
{
	//store current data pointer
	uint8_t* data = (uint8_t*)src;

	//Are we in Split packet mode?
	if(split_enabled && data[0] == BLE_PARTIAL_ID) {
		//check packet count
		if(data[1] == split_count) {
			ESP_LOGI(BRIDGE_TAG, "Split packet [%02X] adding [%02X]", split_count, size-2);

			uint8_t* new_data = malloc(split_length + size - 2);
			if(new_data == NULL){
				ESP_LOGI(BRIDGE_TAG, "malloc error %s %d", __func__, __LINE__);
				split_clear();
				return;
			}
			memcpy(new_data, split_data, split_length);
			memcpy(new_data + split_length, data + 2, size - 2);
			free(split_data);
			split_data = new_data;
			split_length += size-2;
			split_count++;

			//got complete message
			if(split_length == split_header.cmdSize)
			{   //Messsage size matches
				ESP_LOGI(BRIDGE_TAG, "Split packet size matches [%02X]", split_length);
				parse_packet(&split_header, split_data);
				split_clear();
			} else if(split_length > split_header.cmdSize)
			{   //Message size does not match
				ESP_LOGI(BRIDGE_TAG, "Command size is larger than packet size [%02X, %02X]", split_header.cmdSize, split_length);
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

void notifications_disabled() {
	led_setcolor(LED_RED_HALF);
}

void notifications_enabled() {
	led_setcolor(LED_GREEN_HALF);
}

/* ------------ Primary startup ---------------- */
void app_main(void)
{
	//start LED handling service
	led_start();

	// Setup BLE server
    ble_server_callbacks callbacks = {
        .data_received = received_from_ble,
        .notifications_subscribed = notifications_enabled,
        .notifications_unsubscribed = notifications_disabled
    };
    ble_server_setup(callbacks);

    // Need to pull down GPIO 21 to unset the "S" (Silent Mode) pin on CAN Xceiver.
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL(SILENT_GPIO_NUM);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(SILENT_GPIO_NUM, 0);

	//init twai can controller
	twai_install();

    //Create semaphores and tasks
	tx_task_queue = xQueueCreate(TASK_QUEUE_SIZE, sizeof(twai_message_t));
	isotp_send_message_queue = xQueueCreate(MESSAGE_QUEUE_SIZE, sizeof(send_message_t));
	done_sem = xSemaphoreCreateBinary();
	isotp_send_queue_sem = xSemaphoreCreateBinary();
	isotp_mutex = xSemaphoreCreateMutex();
	configure_isotp_links();
	persist_start();
	xSemaphoreGive(isotp_send_queue_sem);

    // Tasks :
    // "TWAI_rx" polls the receive queue (blocking) and once a message exists, forwards it into the ISO-TP library.
    // "TWAI_tx" blocks on a send queue which is populated by the callback from the ISO-TP library
    // "ISOTP_process" pumps the ISOTP library's "poll" method, which will call the send queue callback if a message needs to be sent.
    // ISOTP_process also polls the ISOTP library's non-blocking receive method, which will produce a message if one is ready.
    // "MAIN_process_send_queue" processes queued messages from the BLE stack. These messages are dynamically allocated when they are queued and freed in this task. 

	xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(isotp_processing_task, "ecu_ISOTP_process", 4096, &isotp_link_containers[0], ISOTP_TSK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(isotp_processing_task, "tcu_ISOTP_process", 4096, &isotp_link_containers[1], ISOTP_TSK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(isotp_processing_task, "ptcu_ISOTP_process", 4096, &isotp_link_containers[2], ISOTP_TSK_PRIO, NULL, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(isotp_processing_task, "gateway_ISOTP_process", 4096, &isotp_link_containers[3], ISOTP_TSK_PRIO, NULL, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(isotp_processing_task, "dtc_ISOTP_process", 4096, &isotp_link_containers[4], ISOTP_TSK_PRIO, NULL, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(isotp_send_queue_task, "ISOTP_process_send_queue", 4096, NULL, MAIN_TSK_PRIO, NULL, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(persist_task, "PERSIST_process", 4096, NULL, PERSIST_TSK_PRIO, NULL, tskNO_AFFINITY);

	xSemaphoreTake(done_sem, portMAX_DELAY);

	persist_stop();
	ble_server_shutdown();
	twai_uninstall();
	disable_isotp_links();
	led_stop();

	vSemaphoreDelete(isotp_send_queue_sem);
    vSemaphoreDelete(done_sem);
	vSemaphoreDelete(isotp_mutex);
    vQueueDelete(tx_task_queue);
    vQueueDelete(isotp_send_message_queue);
}
