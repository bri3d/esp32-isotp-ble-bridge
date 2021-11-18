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
#include "uart.h"

#define SLEEP_TAG 			"SLEEP"

RTC_DATA_ATTR 	uint32_t firstSleep	= TIMEOUT_FIRSTBOOT;
uint32_t 		uartConnected 		= TIMEOUT_UARTCONNECTION;

void sleep_task(void *arg);

void sleep_init()
{
	can_sem = xSemaphoreCreateBinary();
	uart_mutex = xSemaphoreCreateMutex();
	uart_packet_sem = xSemaphoreCreateBinary();
	sleep_sem = xSemaphoreCreateBinary();
}

void sleep_deinit()
{
	vSemaphoreDelete(can_sem);
	vSemaphoreDelete(uart_mutex);
	vSemaphoreDelete(uart_packet_sem);
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
		//count down our first boot timer
		if(firstSleep) {
			firstSleep--;
		}

		//Did we receive a CAN or uart message?
		if((!ble_connected() && !sleep_uart_connection()) && !firstSleep && xSemaphoreTake(can_sem, 0) == pdTRUE) {
			xSemaphoreGive(sleep_sem);
		}

		//check for packet timeout
		if(sleep_uart_connected() && xSemaphoreTake(uart_packet_sem, 0) == pdTRUE) {
        	uart_buffer_clear();
		}

		xSemaphoreGive(can_sem);
		xSemaphoreGive(uart_packet_sem);

		vTaskDelay(pdMS_TO_TICKS(TIMEOUT_CAN * 1000));
	}
    vTaskDelete(NULL);
}
