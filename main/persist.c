#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "mutexes.h"
#include "queues.h"
#include "ble_server.h"
#include "constants.h"
#include "led.h"
#include "persist.h"
#include "isotp.h"
#include "isotp_link_containers.h"
#include "connection_handler.h"

#define PERSIST_TAG							"Persist"

typedef struct {
	send_message_t messages[PERSIST_MAX_MESSAGE];
	uint16_t			count;
	uint16_t			position;
	uint16_t			number;
	uint16_t			rxID;
	uint16_t			txID;
	SemaphoreHandle_t	isRunning;
} persist_t;

static bool16		persist_run_task	= false;
static uint16_t		persist_msg_delay	= PERSIST_DEFAULT_MESSAGE_DELAY;
static uint16_t		persist_msg_qdelay	= PERSIST_DEFAULT_QUEUE_DELAY;
static uint16_t		persist_msg_enabled	= false;
static persist_t	persist_msgs[PERSIST_COUNT];

void persist_task(void *arg);

void persist_init()
{
	persist_deinit();

	for (uint16_t i = 0; i < PERSIST_COUNT; i++) {
		persist_t* pPersist			= &persist_msgs[i];
		pPersist->number			= i;
		pPersist->count				= 0;
		pPersist->position			= 0;
		pPersist->rxID				= isotp_link_containers[i].link.send_arbitration_id;
		pPersist->txID				= isotp_link_containers[i].link.receive_arbitration_id;
		pPersist->isRunning			= xSemaphoreCreateMutex();
		persist_message_mutex[i]	= xSemaphoreCreateMutex();
		persist_message_send[i]		= xSemaphoreCreateBinary();
	}

	ESP_LOGI(PERSIST_TAG, "Init");
}

void persist_deinit()
{
	bool16 didDeInit = false;

	for (uint16_t i = 0; i < PERSIST_COUNT; i++) {
		if (persist_msgs[i].isRunning) {
			vSemaphoreDelete(persist_msgs[i].isRunning);
			persist_msgs[i].isRunning = NULL;
			didDeInit = true;
		}

		if (persist_message_send[i]) {
			vSemaphoreDelete(persist_message_send[i]);
			persist_message_send[i] = NULL;
			didDeInit = true;
		}
	
		if (persist_message_mutex[i]) {
			vSemaphoreDelete(persist_message_mutex[i]);
			persist_message_mutex[i] = NULL;
			didDeInit = true;
		}
	}

	if(didDeInit)
		ESP_LOGI(PERSIST_TAG, "Deinit");
}

void persist_start_task()
{
	persist_stop_task();

	persist_run_task = true;

	ESP_LOGI(PERSIST_TAG, "Tasks starting");
	xSemaphoreTake(sync_task_sem, 0);
	for (uint16_t i = 0; i < PERSIST_COUNT; i++) {
		persist_t* pPersist = &persist_msgs[i];
		xTaskCreate(persist_task, "PERSIST_process", TASK_STACK_SIZE, &pPersist->number, PERSIST_TSK_PRIO, NULL);
		xSemaphoreTake(sync_task_sem, portMAX_DELAY);
	}
	ESP_LOGI(PERSIST_TAG, "Tasks started");
}

void persist_stop_task()
{
	if (persist_run_task) {
		persist_run_task = false;

		for (uint16_t i = 0; i < PERSIST_COUNT; i++) {
			persist_t* pPersist = &persist_msgs[i];
			xSemaphoreTake(pPersist->isRunning, portMAX_DELAY);
			xSemaphoreGive(pPersist->isRunning);
		}

		persist_clear();

		ESP_LOGI(PERSIST_TAG, "Tasks stopped");
	}
}

int16_t persist_send(uint16_t persist)
{
	//make sure the persist value is valid
	if (persist >= PERSIST_COUNT)
	{
		return false;
	}

	//set persist pointer
	persist_t* pPersist = &persist_msgs[persist];

	//If persist mode is disabled abort
	if (!persist_msg_enabled || !pPersist->count)
	{
		return false;
	}

	//If be have been disconnected from BLE and UART clear persist message
	if(!ble_connected() && !ch_uart_connected())
	{
		//clear messages
		persist_msg_enabled = false;
		for(uint16_t i = 0; i < pPersist->count; i++)
		{
			send_message_t* pMsg = &pPersist->messages[i];
			if(pMsg->buffer)
			{
				free(pMsg->buffer);
				pMsg->buffer = NULL;
			}
			pMsg->msg_length = 0;
		}
		pPersist->position = 0;
		pPersist->count = 0;

		return false;
	}

	//don't flood the queues
	if(uxQueueMessagesWaiting(isotp_send_message_queue) || !ble_queue_spaces())
	{
		return false;
	}

	//Cycle through messages
	if(pPersist->position >= pPersist->count)
		pPersist->position = 0;

	//Make sure the message has length
	uint16_t mPos = pPersist->position++;
	send_message_t* pMsg = &pPersist->messages[mPos];
	if(pMsg->msg_length == 0)
	{
		return false;
	}

	//We are good, lets send the message!
	send_message_t msg;
	msg.buffer = malloc(pMsg->msg_length);
	if(msg.buffer == NULL){
		ESP_LOGI(PERSIST_TAG, "malloc error %s %d", __func__, __LINE__);
		return false;
	}

	//copy persist message into a new container
	memcpy(msg.buffer, pMsg->buffer, pMsg->msg_length);
	msg.msg_length = pMsg->msg_length;
	msg.rxID = pMsg->rxID;
	msg.txID = pMsg->txID;

	//if we fail to play message into the queue free the memory!
	if (xQueueSend(isotp_send_message_queue, &msg, pdMS_TO_TICKS(TIMEOUT_SHORT)) != pdTRUE) {
		free(msg.buffer);
	}

	ESP_LOGI(PERSIST_TAG, "Message sent with size: %04X", msg.msg_length);

	return true;
}

uint16_t persist_enabled()
{
	return persist_msg_enabled;
}

void persist_set(uint16_t enable)
{
	persist_msg_enabled = enable;

	ESP_LOGI(PERSIST_TAG, "Enabled: %d", enable);
}

int16_t persist_add(uint16_t rx, uint16_t tx, const void* src, size_t size)
{
	//set persist pointer
	persist_t* pPersist = NULL;
	for (uint16_t i = 0; i < PERSIST_COUNT; i++)
	{
		persist_t* persist = &persist_msgs[i];
		if (persist->rxID == rx && persist->txID == tx) {
			pPersist = persist;
			break;
		}
	}

	if (pPersist == NULL)
	{
		return false;
	}

	//check for valid message
	if (!src || size == 0)
	{
		return false;
	}

	if(pPersist->count >= PERSIST_MAX_MESSAGE)
	{
		return false;
	}

	send_message_t* pMsg	= &pPersist->messages[pPersist->count++];
	pMsg->rxID				= rx;
	pMsg->txID				= tx;
	pMsg->msg_length		= 0;
	pMsg->buffer			= malloc(size);
	if(pMsg->buffer == NULL){
		ESP_LOGI(PERSIST_TAG, "malloc error %s %d", __func__, __LINE__);
		return false;
	}
	memcpy(pMsg->buffer, src, size);
	pMsg->msg_length = size;
	ESP_LOGI(PERSIST_TAG, "Message added with size: %04X", size);

	return true;
}

void persist_clear()
{
	persist_msg_enabled = false;
	for (uint16_t d = 0; d < PERSIST_COUNT; d++) {
		//set persist pointer
		persist_t* pPersist = &persist_msgs[d];

		//clear all messages
		for (uint16_t i = 0; i < pPersist->count; i++)
		{
			send_message_t* pMsg = &pPersist->messages[i];
			if (pMsg->buffer)
			{
				free(pMsg->buffer);
				pMsg->buffer = NULL;
			}
			pMsg->msg_length = 0;
		}
		pPersist->position = 0;
		pPersist->count = 0;
	}

	ESP_LOGI(PERSIST_TAG, "Messages cleared");
}


void persist_task(void *arg)
{
	uint16_t number = *((uint16_t*)arg);
	xSemaphoreTake(persist_msgs[number].isRunning, portMAX_DELAY);
	ESP_LOGI(PERSIST_TAG, "Task started: %d", number);
	xSemaphoreGive(sync_task_sem);
	while(persist_run_task) {
		xSemaphoreTake(persist_message_send[number], pdMS_TO_TICKS(TIMEOUT_LONG));
		xSemaphoreTake(persist_message_mutex[number], pdMS_TO_TICKS(TIMEOUT_LONG));
		persist_send(number);
		xSemaphoreGive(persist_message_mutex[number]);
		vTaskDelay(pdMS_TO_TICKS(persist_msg_delay + (ble_queue_waiting() * persist_msg_qdelay)));
	}
	ESP_LOGI(PERSIST_TAG, "Task stopped: %d", number);
	xSemaphoreGive(persist_msgs[number].isRunning);
	vTaskDelete(NULL);
}

void persist_set_delay(uint16_t delay)
{
	persist_msg_delay = delay;
	ESP_LOGI(PERSIST_TAG, "Set delay: %d", delay);
}

void persist_set_q_delay(uint16_t delay)
{
	persist_msg_qdelay = delay;
	ESP_LOGI(PERSIST_TAG, "Set q delay: %d", delay);
}

uint16_t persist_get_delay()
{
	return persist_msg_delay;
}

uint16_t persist_get_q_delay()
{
	return persist_msg_qdelay;
}

void persist_take_all_mutex()
{
	for(uint16_t i = 0; i < PERSIST_COUNT; i++)
		xSemaphoreTake(persist_message_mutex[i], pdMS_TO_TICKS(TIMEOUT_NORMAL));

	ESP_LOGD(PERSIST_TAG, "Took all mutex");
}

void persist_give_all_mutex()
{
	for (uint16_t i = 0; i < PERSIST_COUNT; i++)
		xSemaphoreGive(persist_message_mutex[i]);

	ESP_LOGD(PERSIST_TAG, "Gave all mutex");
}