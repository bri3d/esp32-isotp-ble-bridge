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

#define PERSIST_TAG					"Persist"
#define MAX_MESSAGE_PERSIST			64
#define PERSIST_MESSAGE_DELAY		2

send_message_t msgPersist[MAX_MESSAGE_PERSIST];
uint16_t msgPersistCount = 0;
uint16_t msgPersistPosition = 0;
uint16_t msgPersistEnabled = false;

void persist_start()
{
	persist_message_mutex = xSemaphoreCreateMutex();
	persist_message_send = xSemaphoreCreateBinary();
}

void persist_stop()
{
	vSemaphoreDelete(persist_message_send);
	vSemaphoreDelete(persist_message_mutex);
}

int16_t persist_send()
{
	xSemaphoreTake(persist_message_mutex, pdMS_TO_TICKS(TIMEOUT_NORMAL));

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
	if(uxQueueMessagesWaiting(isotp_send_message_queue) || !ble_queue_spaces())
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
	msg.rxID = 0x7E0;
	msg.txID = 0x7E8;

	xQueueSend(isotp_send_message_queue, &msg, pdMS_TO_TICKS(TIMEOUT_SHORT));

	ESP_LOGI(PERSIST_TAG, "Persistent message sent [%04X]", msg.msg_length);
	xSemaphoreGive(persist_message_mutex);

	return true;
}

uint16_t persist_enabled()
{
	xSemaphoreTake(persist_message_mutex, pdMS_TO_TICKS(TIMEOUT_NORMAL));
	uint16_t isEnabled = msgPersistEnabled;
	xSemaphoreGive(persist_message_mutex);

	return isEnabled;
}

void persist_set(uint16_t enable)
{
	xSemaphoreTake(persist_message_mutex, pdMS_TO_TICKS(TIMEOUT_NORMAL));
	msgPersistEnabled = enable;

	xSemaphoreGive(persist_message_mutex);
	ESP_LOGI(PERSIST_TAG, "Persistent messages: %d", enable);
}

int16_t persist_add(const void* src, size_t size)
{
	if(!src || size == 0)
		return false;

	xSemaphoreTake(persist_message_mutex, pdMS_TO_TICKS(TIMEOUT_NORMAL));

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

	ESP_LOGI(PERSIST_TAG, "Persistent message added [%04X]", size);

	return true;
}

void persist_clear()
{
	xSemaphoreTake(persist_message_mutex, pdMS_TO_TICKS(TIMEOUT_NORMAL));
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

	ESP_LOGI(PERSIST_TAG, "Persistent messages cleared");
}


void persist_task(void *arg)
{
	while(1) {
		xSemaphoreTake(persist_message_send, pdMS_TO_TICKS(TIMEOUT_SHORT));
		persist_send();
		vTaskDelay(pdMS_TO_TICKS((SEND_QUEUE_SIZE-ble_queue_spaces())*PERSIST_MESSAGE_DELAY));
	}
	vTaskDelete(NULL);
}
