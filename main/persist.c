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
	uint16_t count;
	uint16_t position;
	uint16_t number;
	uint16_t rxID;
	uint16_t txID;
} persist_t;

uint16_t msgPersistDelay	= PERSIST_DEFAULT_MESSAGE_DELAY;
uint16_t msgPersistQDelay	= PERSIST_DEFAULT_QUEUE_DELAY;
uint16_t msgPersistEnabled	= false;
persist_t msgPersists[PERSIST_COUNT];

void persist_task(void *arg);

void persist_init()
{
	for (uint16_t i = 0; i < PERSIST_COUNT; i++) {
		persist_t* pPersist = &msgPersists[i];
		pPersist->number = i;
		pPersist->count = 0;
		pPersist->position = 0;
		pPersist->rxID = isotp_link_containers[i].link.send_arbitration_id;
		pPersist->txID = isotp_link_containers[i].link.receive_arbitration_id;

		persist_message_mutex[i] = xSemaphoreCreateMutex();
		persist_message_send[i] = xSemaphoreCreateBinary();
	}
}

void persist_deinit()
{
	persist_clear();

	for (uint16_t i = 0; i < PERSIST_COUNT; i++) {
		vSemaphoreDelete(persist_message_send[i]);
		vSemaphoreDelete(persist_message_mutex[i]);
	}
}

void persist_start_task()
{
	for (uint16_t i = 0; i < PERSIST_COUNT; i++) {
		persist_t* pPersist = &msgPersists[i];
		xTaskCreate(persist_task, "PERSIST_process", TASK_STACK_SIZE, &pPersist->number, PERSIST_TSK_PRIO, NULL);
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
	persist_t* pPersist = &msgPersists[persist];

	//If persist mode is disabled abort
	if (!msgPersistEnabled || !pPersist->count)
	{
		return false;
	}

	//If be have been disconnected from BLE and UART clear persist message
	if(!ble_connected() && !ch_uart_connected())
	{
		//clear messages
		msgPersistEnabled = false;
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

	memcpy(msg.buffer, pMsg->buffer, pMsg->msg_length);
	msg.msg_length = pMsg->msg_length;
	msg.rxID = pMsg->rxID;
	msg.txID = pMsg->txID;

	xQueueSend(isotp_send_message_queue, &msg, pdMS_TO_TICKS(TIMEOUT_SHORT));

	ESP_LOGI(PERSIST_TAG, "Persistent message sent [%04X]", msg.msg_length);

	return true;
}

uint16_t persist_enabled()
{
	return msgPersistEnabled;
}

void persist_set(uint16_t enable)
{
	msgPersistEnabled = enable;

	ESP_LOGI(PERSIST_TAG, "Persistent messages: %d", msgPersistEnabled);
}

int16_t persist_add(uint16_t rx, uint16_t tx, const void* src, size_t size)
{
	//set persist pointer
	persist_t* pPersist = NULL;
	for (uint16_t i = 0; i < PERSIST_COUNT; i++)
	{
		persist_t* persist = &msgPersists[i];
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

	send_message_t* pMsg = &pPersist->messages[pPersist->count++];
	pMsg->rxID			= rx;
	pMsg->txID			= tx;
	pMsg->msg_length	= 0;
	pMsg->buffer		= malloc(size);
	if(pMsg->buffer == NULL){
		ESP_LOGI(PERSIST_TAG, "malloc error %s %d", __func__, __LINE__);
		return false;
	}
	memcpy(pMsg->buffer, src, size);
	pMsg->msg_length = size;
	ESP_LOGI(PERSIST_TAG, "Persistent message added [%04X]", size);

	return true;
}

void persist_clear()
{
	msgPersistEnabled = false;
	for (uint16_t d = 0; d < PERSIST_COUNT; d++) {
		//set persist pointer
		persist_t* pPersist = &msgPersists[d];

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

	ESP_LOGI(PERSIST_TAG, "Persistent messages cleared");
}


void persist_task(void *arg)
{
	uint16_t number = *((uint16_t*)arg);
	while(1) {
		xSemaphoreTake(persist_message_send[number], pdMS_TO_TICKS(TIMEOUT_NORMAL));
		xSemaphoreTake(persist_message_mutex[number], pdMS_TO_TICKS(TIMEOUT_NORMAL));
		persist_send(number);
		xSemaphoreGive(persist_message_mutex[number]);
		vTaskDelay(pdMS_TO_TICKS(msgPersistDelay + (ble_queue_waiting() * msgPersistQDelay)));
	}
	vTaskDelete(NULL);
}

void persist_set_delay(uint16_t delay)
{
	msgPersistDelay = delay;
}

void persist_set_q_delay(uint16_t delay)
{
	msgPersistQDelay = delay;
}

uint16_t persist_get_delay()
{
	return msgPersistDelay;
}

uint16_t persist_get_q_delay()
{
	return msgPersistQDelay;
}

void persist_take_all_mutex()
{
	for(uint16_t i = 0; i < PERSIST_COUNT; i++)
		xSemaphoreTake(persist_message_mutex[i], pdMS_TO_TICKS(TIMEOUT_NORMAL));
}

void persist_give_all_mutex()
{
	for (uint16_t i = 0; i < PERSIST_COUNT; i++)
		xSemaphoreGive(persist_message_mutex[i]);
}