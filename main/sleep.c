#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "isotp.h"
#include "mutexes.h"
#include "queues.h"
#include "constants.h"
#include "sleep.h"
#include "ble_server.h"

#define SLEEP_TAG 			"SLEEP"

RTC_DATA_ATTR 	bool firstBoot 		= true;
uint32_t uartConnected 				= 120;

void sleep_task(void *arg);

void sleep_init()
{
	can_sem = xSemaphoreCreateBinary();
	uart_mutex = xSemaphoreCreateMutex();
	sleep_sem = xSemaphoreCreateBinary();
}

void sleep_deinit()
{
	vSemaphoreDelete(can_sem);
	vSemaphoreDelete(uart_mutex);
	vSemaphoreDelete(sleep_sem);
}

void sleep_start_task()
{
	xTaskCreatePinnedToCore(sleep_task, "SLEEP_process", 4096, NULL, SLEEP_TSK_PRIO, NULL, tskNO_AFFINITY);
}

void sleep_reset_uart()
{
	xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(TIMEOUT_NORMAL));
	if(!uartConnected) {
		ble_stop_advertising();
	}
	uartConnected = 120;
	xSemaphoreGive(uart_mutex);
}

bool sleep_uart_connected()
{
	return uartConnected;
}

bool sleep_uart_connection()
{
	xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(TIMEOUT_NORMAL));
	if(uartConnected) {
		if(--uartConnected == 0) {
			ble_start_advertising();
		}
	}
	xSemaphoreGive(uart_mutex);

	return uartConnected;
}

void sleep_task(void *arg)
{
	while(1)
	{
		//Did we receive a CAN or uart message?
		if((!ble_connected() && !sleep_uart_connection()) && xSemaphoreTake(can_sem, 0) == pdTRUE)
		{
			xSemaphoreGive(sleep_sem);
		}

		xSemaphoreGive(can_sem);
		if(firstBoot) {
			firstBoot = false;
			vTaskDelay(pdMS_TO_TICKS(TIMEOUT_FIRSTBOOT));
		} else {
			vTaskDelay(pdMS_TO_TICKS(TIMEOUT_CAN));
		}
	}
    vTaskDelete(NULL);
}
