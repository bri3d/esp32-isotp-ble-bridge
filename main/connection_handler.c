#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "isotp.h"
#include "mutexes.h"
#include "queues.h"
#include "constants.h"
#include "connection_handler.h"
#include "ble_server.h"
#include "uart.h"

#define CH_TAG 			"Connection_handler"

RTC_DATA_ATTR 	uint32_t first_sleep_timer	= TIMEOUT_FIRSTBOOT;
uint32_t 		uart_connection_timer 		= 0;
uint32_t 		uart_packet_timer			= 0;
uint32_t		can_connection_timer		= 0;

void ch_task(void *arg);

void ch_init()
{
	ch_uart_mutex = xSemaphoreCreateMutex();
	ch_uart_packet_timer_sem = xSemaphoreCreateBinary();
	ch_can_timer_sem = xSemaphoreCreateBinary();
	ch_sleep_sem = xSemaphoreCreateBinary();
}

void ch_deinit()
{
	vSemaphoreDelete(ch_uart_mutex);
	vSemaphoreDelete(ch_uart_packet_timer_sem);
	vSemaphoreDelete(ch_can_timer_sem);
	vSemaphoreDelete(ch_sleep_sem);
}

void ch_start_task()
{
	xTaskCreate(ch_task, "Connection_handling_process", TASK_STACK_SIZE_SMALL, NULL, HANDLER_TSK_PRIO, NULL);
}

void ch_reset_uart_timer()
{
	xSemaphoreTake(ch_uart_mutex, pdMS_TO_TICKS(TIMEOUT_NORMAL));
	if(!uart_connection_timer) {
		ble_stop_advertising();
		ch_on_uart_connect();
	}
	uart_connection_timer = TIMEOUT_UARTCONNECTION;
	xSemaphoreGive(ch_uart_mutex);
}

bool ch_uart_connected()
{
	return uart_connection_timer;
}

bool ch_uart_connection_countdown()
{
	xSemaphoreTake(ch_uart_mutex, pdMS_TO_TICKS(TIMEOUT_NORMAL));
	if(uart_connection_timer) {
		if(--uart_connection_timer == 0) {
			ble_start_advertising();
			ch_on_uart_disconnect();
		}
	}
	xSemaphoreGive(ch_uart_mutex);

	return uart_connection_timer;
}

void ch_task(void *arg)
{
	while(1)
	{
		//count down our first boot timer
		if(first_sleep_timer) {
			first_sleep_timer--;
		}

		//Did we receive a CAN or uart message?
		if((!ble_connected() && !ch_uart_connection_countdown()) && !first_sleep_timer && xSemaphoreTake(ch_can_timer_sem, 0) == pdTRUE) {
			if (can_connection_timer) {
				can_connection_timer--;
			}

			if(!can_connection_timer)
				xSemaphoreGive(ch_sleep_sem);
		} else {
			can_connection_timer = TIMEOUT_CANCONNECTION;
		}

		//check for packet timeout
		xSemaphoreTake(uart_buffer_mutex, pdMS_TO_TICKS(TIMEOUT_NORMAL));
		if(ch_uart_connected() && xSemaphoreTake(ch_uart_packet_timer_sem, 0) == pdTRUE) {
			if (uart_packet_timer) {
				uart_packet_timer--;
			}

			if (!uart_packet_timer)
        		uart_buffer_clear();
		} else {
			uart_packet_timer = TIMEOUT_UARTPACKET;
		}
		xSemaphoreGive(uart_buffer_mutex);

		//give semaphores used for countdown timers
		xSemaphoreGive(ch_can_timer_sem);
		xSemaphoreGive(ch_uart_packet_timer_sem);

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
    vTaskDelete(NULL);
}
