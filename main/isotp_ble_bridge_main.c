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
#include "ws2812_control.h"

/* --------------------- Definitions and static variables ------------------ */
#define RX_TASK_PRIO 			 	3 // Ensure we drain the RX queue as quickly as we reasonably can to prevent overflow and ensure the message pump has fresh data.
#define TX_TASK_PRIO 			 	3 // Ensure we TX messages as quickly as we reasonably can to meet ISO15765-2 timing constraints
#define ISOTP_TSK_PRIO 				2 // Run the message pump at a higher priority than the main queue/dequeue task when messages are available
#define MAIN_TSK_PRIO 				1 // Run the main task at the same priority as the BLE queue/dequeue tasks to help in delivery ordering.
#define PERSIST_TSK_PRIO 			0
#define TX_GPIO_NUM 				5 // For A0
#define RX_GPIO_NUM 				4 // For A0
#define SILENT_GPIO_NUM 			21 // For A0
#define LED_ENABLE_GPIO_NUM 		13 // For A0
#define LED_GPIO_NUM 				2 // For A0
#define GPIO_OUTPUT_PIN_SEL(X)  	((1ULL<<X))
#define ISOTP_BUFSIZE 				4096
#define EXAMPLE_TAG 				"ISOTPtoBLE"
#define MAX_MESSAGE_PERSIST			64
#define PERSIST_MESSAGE_DELAY		15

#define ISOTP_MAX_RECEIVE_PAYLOAD 	512
#define SEND_IDENTIFIER 			0x7E0
#define RECEIVE_IDENTIFIER 			0x7E8

//Queue sizes
#define TASK_QUEUE_SIZE     		16
#define MESSAGE_QUEUE_SIZE			16

static send_message_t msgPersist[MAX_MESSAGE_PERSIST];
static uint16_t msgPersistCount = 0;
static uint16_t msgPersistPosition = 0;
static uint16_t msgPersistEnabled = false;

// TWAI/CAN configuration
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

// Mutable globals for semaphores, queues, ISO-TP link

static QueueHandle_t tx_task_queue;
static QueueHandle_t send_message_queue;
static SemaphoreHandle_t isotp_task_sem;
static SemaphoreHandle_t send_queue_start;
static SemaphoreHandle_t done_sem;
static SemaphoreHandle_t isotp_mutex;
static SemaphoreHandle_t isotp_wait_for_data;
static SemaphoreHandle_t persist_message_mutex;
static SemaphoreHandle_t persist_message_send;

static IsoTpLink isotp_link;

/* Alloc ISO-TP send and receive buffer statically in RAM, required by library */
static uint8_t isotp_recv_buf[ISOTP_BUFSIZE];
static uint8_t isotp_send_buf[ISOTP_BUFSIZE];

// LED colors
static struct led_state red_led_state = {
    .leds[0] = 0x008000
};

static struct led_state green_led_state = {
    .leds[0] = 0x800000
};

/* ---------------------------- ISOTP Callbacks ---------------------------- */

int isotp_user_send_can(const uint32_t arbitration_id, const uint8_t* data, const uint8_t size) {
    twai_message_t frame = {.identifier = arbitration_id, .data_length_code = size};
    memcpy(frame.data, data, sizeof(frame.data));
    xQueueSend(tx_task_queue, &frame, portMAX_DELAY);
    return ISOTP_RET_OK;                           
}

uint32_t isotp_user_get_ms(void) {
    return (esp_timer_get_time() / 1000ULL) & 0xFFFFFFFF;
}

void isotp_user_debug(const char* message, ...) {
}

/* --------------------------- Persist Functions -------------------------- */
int16_t message_send()
{
	xSemaphoreTake(persist_message_mutex, pdMS_TO_TICKS(100));

	//If be have been disconnected from BLE clear persist message
	if(!ble_connected())
	{
		//clear messages
		msgPersistEnabled = false;
		for(int i=0; i<msgPersistCount; i++)
		{
			if(msgPersist[i].buffer)
			{
				free(msgPersist[i].buffer);
				msgPersist[i].buffer = NULL;
			}
			msgPersist[i].msg_length = 0;
		}
		msgPersistPosition = 0;
		msgPersistCount = 0;

		xSemaphoreGive(persist_message_mutex);
		return false;
	}

	//If persist mode is disabled abort
	if(!msgPersistEnabled || !msgPersistCount)
	{
		xSemaphoreGive(persist_message_mutex);
		return false;
	}

	//don't flood the queues
	if(uxQueueMessagesWaiting(send_message_queue) || !ble_queue_spaces())
	{
		xSemaphoreGive(persist_message_mutex);
		return false;
	}

	//Cycle through messages
	if(msgPersistPosition >= msgPersistCount)
		msgPersistPosition = 0;

	//Make sure the message has length
	uint16_t mPos = msgPersistPosition++;
	send_message_t* pMsg = &msgPersist[mPos];
	if(pMsg->msg_length == 0)
	{
		xSemaphoreGive(persist_message_mutex);
		return false;
	}

	//We are good, lets send the message!
	send_message_t msg;
	msg.buffer = malloc(pMsg->msg_length);
	memcpy(msg.buffer, pMsg->buffer, pMsg->msg_length);
	msg.msg_length = pMsg->msg_length;

	xQueueSend(send_message_queue, &msg, pdMS_TO_TICKS(50));

	ESP_LOGI(EXAMPLE_TAG, "Persistent message send with length %04X", msg.msg_length);
	xSemaphoreGive(persist_message_mutex);

	return true;
}

uint16_t message_enabled()
{
	xSemaphoreTake(persist_message_mutex, pdMS_TO_TICKS(100));
	uint16_t isEnabled = msgPersistEnabled;
	xSemaphoreGive(persist_message_mutex);

	return isEnabled;
}

void message_set(uint16_t enable)
{
	xSemaphoreTake(persist_message_mutex, pdMS_TO_TICKS(100));
	msgPersistEnabled = enable;
	xSemaphoreGive(persist_message_mutex);
	ESP_LOGI(EXAMPLE_TAG, "Persistent messages: %d", enable);
}

int16_t message_add(const void* src, size_t size)
{
	if(!src || size == 0)
		return false;

	xSemaphoreTake(persist_message_mutex, pdMS_TO_TICKS(100));

	if(msgPersistCount >= MAX_MESSAGE_PERSIST)
	{
		xSemaphoreGive(persist_message_mutex);
		return false;
	}

	send_message_t* pMsg = &msgPersist[msgPersistCount++];
	pMsg->buffer = malloc(size);
	memcpy(pMsg->buffer, src, size);
	pMsg->msg_length = size;
	xSemaphoreGive(persist_message_mutex);

	ESP_LOGI(EXAMPLE_TAG, "Persistent message added with length %04X", size);

	return true;
}

void message_clear()
{
	xSemaphoreTake(persist_message_mutex, pdMS_TO_TICKS(100));
	msgPersistEnabled = false;
	for(int i=0; i<msgPersistCount; i++)
	{
		if(msgPersist[i].buffer)
		{
			free(msgPersist[i].buffer);
			msgPersist[i].buffer = NULL;
		}
		msgPersist[i].msg_length = 0;
	}
	msgPersistPosition = 0;
	msgPersistCount = 0;
	xSemaphoreGive(persist_message_mutex);

	ESP_LOGI(EXAMPLE_TAG, "Persistent messages cleared");
}


/* --------------------------- Tasks and Functions -------------------------- */

static void twai_receive_task(void *arg)
{
    while (1)
    {
        twai_message_t rx_msg;
        twai_receive(&rx_msg, portMAX_DELAY); // If no message available, should block and yield.
        if(rx_msg.identifier == RECEIVE_IDENTIFIER) {
            ESP_LOGD(EXAMPLE_TAG, "Received Message with identifier %08X and length %08X", rx_msg.identifier, rx_msg.data_length_code);
            for (int i = 0; i < rx_msg.data_length_code; i++)
                ESP_LOGD(EXAMPLE_TAG, "RX Data: %02X", rx_msg.data[i]);
            xSemaphoreTake(isotp_mutex, (TickType_t)100);
            isotp_on_can_message(&isotp_link, rx_msg.data, rx_msg.data_length_code);
            xSemaphoreGive(isotp_mutex);
            xSemaphoreGive(isotp_wait_for_data);
        }
    }
    vTaskDelete(NULL);
}

static void twai_transmit_task(void *arg)
{
    while (1)
    {
        twai_message_t tx_msg;
        xQueueReceive(tx_task_queue, &tx_msg, portMAX_DELAY);
        ESP_LOGD(EXAMPLE_TAG, "Sending Message with ID %08X", tx_msg.identifier);
        for (int i = 0; i < tx_msg.data_length_code; i++)
            ESP_LOGD(EXAMPLE_TAG, "TX Data: %02X", tx_msg.data[i]);
        twai_transmit(&tx_msg, portMAX_DELAY);
    }
    vTaskDelete(NULL);
}

static void isotp_processing_task(void *arg)
{
    xSemaphoreTake(isotp_task_sem, portMAX_DELAY);
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(EXAMPLE_TAG, "CAN/TWAI Driver started");
    isotp_init_link(&isotp_link, SEND_IDENTIFIER,
						isotp_send_buf, sizeof(isotp_send_buf), 
						isotp_recv_buf, sizeof(isotp_recv_buf));
    ESP_LOGI(EXAMPLE_TAG, "ISO-TP Handler started");

    xSemaphoreGive(send_queue_start);

    while (1)
    {
        if(isotp_link.send_status != ISOTP_SEND_STATUS_INPROGRESS && isotp_link.receive_status != ISOTP_RECEIVE_STATUS_INPROGRESS) {
            // Link is idle, wait for new data before pumping loop. 
            xSemaphoreTake(isotp_wait_for_data, portMAX_DELAY);
        }
		xSemaphoreTake(isotp_mutex, (TickType_t)100);
        isotp_poll(&isotp_link);
        xSemaphoreGive(isotp_mutex);
        uint16_t out_size;
        uint8_t payload[ISOTP_MAX_RECEIVE_PAYLOAD];
        xSemaphoreTake(isotp_mutex, (TickType_t)100);
        int ret = isotp_receive(&isotp_link, payload, sizeof(payload), &out_size);
		xSemaphoreGive(isotp_mutex);
        if (ISOTP_RET_OK == ret) {
			ESP_LOGD(EXAMPLE_TAG, "Received ISO-TP message with length: %04X", out_size);
			for(int i = 0; i < out_size; i++)
				ESP_LOGD(EXAMPLE_TAG, "ISO-TP data %c", payload[i]);
			ble_send(payload, out_size);

			//if persist is enabled
			if(message_enabled())
				xSemaphoreGive(persist_message_send);
		}
		vTaskDelay(0); // Allow higher priority tasks to run, for example Rx/Tx
    }

    vTaskDelete(NULL);
}

static void persist_task(void *arg)
{
	while(1) {
		xSemaphoreTake(persist_message_send, pdMS_TO_TICKS(50));
		message_send();
		vTaskDelay(pdMS_TO_TICKS(PERSIST_MESSAGE_DELAY));
	}
	vTaskDelete(NULL);
}

static void send_queue_task(void *arg)
{
	xSemaphoreTake(send_queue_start, portMAX_DELAY);
    while (1)
    {
        twai_status_info_t status_info;
        twai_get_status_info(&status_info);
        if (status_info.state == TWAI_STATE_BUS_OFF) {
            twai_initiate_recovery();
        } else if (status_info.state == TWAI_STATE_STOPPED) {
            twai_start();
        }
        send_message_t msg;
		xQueueReceive(send_message_queue, &msg, portMAX_DELAY);
        xSemaphoreTake(isotp_mutex, (TickType_t)100);
        isotp_send(&isotp_link, msg.buffer, msg.msg_length);
        xSemaphoreGive(isotp_mutex);
		xSemaphoreGive(isotp_wait_for_data);
        free(msg.buffer);
    }
	vTaskDelete(NULL);
}

/* ----------- BLE callbacks ---------------- */

void received_from_ble(const void* src, size_t size)
{
	//store current data pointer
	uint8_t* data = (uint8_t*)src;

	ESP_LOGD(EXAMPLE_TAG, "BLE packet size [%d]", size);

	//If the packet does not contain header abort
	while(size >= sizeof(ble_header_t))
	{
		//Confirm the header is valid
		ble_header_t* header = (ble_header_t*)data;
		if(header->hdID != BLE_HEADER_ID)
		{
			ESP_LOGI(EXAMPLE_TAG, "BLE packet header does not match [%02X, %02X]", header->hdID, BLE_HEADER_ID);
			return;
		}

		ESP_LOGD(EXAMPLE_TAG, "BLE header [%02X, %02X, %04X, %04X, %04X]", header->hdID, header->cmdFlags, header->rxID, header->txID, header->cmdSize);

		data += sizeof(ble_header_t);
		size -= sizeof(ble_header_t);
		if(header->cmdSize > size)
		{
			ESP_LOGI(EXAMPLE_TAG, "BLE command size is larger than packet size [%d, %d]", header->cmdSize, size);
			return;
		}

		//Are we in persistent mode?
		if(message_enabled())
		{
			//Should we clear the persist messages in memory?
			if(header->cmdFlags & BLE_COMMAND_FLAG_PER_CLEAR)
			{
				message_clear();
			}

			//Should we disable persist mode?
			if((header->cmdFlags & BLE_COMMAND_FLAG_PER_ENABLE) == 0)
			{
				message_set(false);
			} else {
				//If we are still in persist mode quit
				return;
			}
		} else
		{ 	//Not in persistent mode
			if(header->cmdFlags & BLE_COMMAND_FLAG_PER_CLEAR)
			{
				message_clear();
			}

			if(header->cmdFlags & BLE_COMMAND_FLAG_PER_ADD)
			{
				message_add(data, header->cmdSize);
			}

			if(header->cmdFlags & BLE_COMMAND_FLAG_PER_ENABLE)
			{
				message_set(true);
				return;
			}
		}

		if(header->cmdSize)
		{
			if(!message_enabled())
			{
				ESP_LOGI(EXAMPLE_TAG, "Received a message from BLE stack with length %04X", header->cmdSize);
				send_message_t msg;
				msg.buffer = malloc(header->cmdSize);
				memcpy(msg.buffer, data, header->cmdSize);
				msg.msg_length = header->cmdSize;
				xQueueSend(send_message_queue, &msg, pdMS_TO_TICKS(50));
			}

			data += header->cmdSize;
			size -= header->cmdSize;
		}
	}
}

void notifications_disabled() {
	ws2812_write_leds(red_led_state);
}

void notifications_enabled() {
	ws2812_write_leds(green_led_state);
}


/* ------------ Primary startup ---------------- */

void app_main(void)
{
	// Configure LED enable pin (switches transistor to push LED)
	gpio_config_t io_conf_led;
	io_conf_led.intr_type = GPIO_INTR_DISABLE;
	io_conf_led.mode = GPIO_MODE_OUTPUT;
	io_conf_led.pin_bit_mask = GPIO_OUTPUT_PIN_SEL(LED_ENABLE_GPIO_NUM);
	io_conf_led.pull_down_en = 0;
	io_conf_led.pull_up_en = 0;
	gpio_config(&io_conf_led);
	gpio_set_level(LED_ENABLE_GPIO_NUM, 0);

    // Configure LED to Red
    ws2812_control_init(LED_GPIO_NUM);
    ws2812_write_leds(red_led_state);

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

    // "TWAI" is knockoff CAN. Install TWAI driver.
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "CAN/TWAI Driver installed");

    //Create semaphores and tasks
	tx_task_queue = xQueueCreate(TASK_QUEUE_SIZE, sizeof(twai_message_t));
	send_message_queue = xQueueCreate(MESSAGE_QUEUE_SIZE, sizeof(send_message_t));
    isotp_task_sem = xSemaphoreCreateBinary();
    done_sem = xSemaphoreCreateBinary();
    send_queue_start = xSemaphoreCreateBinary();
    isotp_wait_for_data = xSemaphoreCreateBinary();
	isotp_mutex = xSemaphoreCreateMutex();
	persist_message_mutex = xSemaphoreCreateMutex();
	persist_message_send = xSemaphoreCreateBinary();

    // Tasks :
    // "TWAI_rx" polls the receive queue (blocking) and once a message exists, forwards it into the ISO-TP library.
    // "TWAI_tx" blocks on a send queue which is populated by the callback from the ISO-TP library
    // "ISOTP_process" pumps the ISOTP library's "poll" method, which will call the send queue callback if a message needs to be sent.
    // ISOTP_process also polls the ISOTP library's non-blocking receive method, which will produce a message if one is ready.
    // "MAIN_process_send_queue" processes queued messages from the BLE stack. These messages are dynamically allocated when they are queued and freed in this task. 

    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(isotp_processing_task, "ISOTP_process", 4096, NULL, ISOTP_TSK_PRIO, NULL, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(send_queue_task, "MAIN_process_send_queue", 4096, NULL, MAIN_TSK_PRIO, NULL, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(persist_task, "PERSIST_process", 4096, NULL, PERSIST_TSK_PRIO, NULL, tskNO_AFFINITY);

	xSemaphoreGive(isotp_task_sem); //Start Control task
    xSemaphoreTake(done_sem, portMAX_DELAY);

    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

	vSemaphoreDelete(persist_message_send);
	vSemaphoreDelete(isotp_task_sem);
	vSemaphoreDelete(send_queue_start);
    vSemaphoreDelete(done_sem);
	vSemaphoreDelete(isotp_mutex);
	vSemaphoreDelete(persist_message_mutex);
    vQueueDelete(tx_task_queue);
    vQueueDelete(send_message_queue);
}
