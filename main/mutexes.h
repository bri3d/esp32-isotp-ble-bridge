#ifndef MUTEXES_H
#define MUTEXES_H

#include "freertos/semphr.h"

SemaphoreHandle_t ch_can_timer_sem;
SemaphoreHandle_t ch_uart_mutex;
SemaphoreHandle_t ch_uart_packet_timer_sem;
SemaphoreHandle_t ch_sleep_sem;
SemaphoreHandle_t uart_buffer_mutex;
SemaphoreHandle_t isotp_mutex;
SemaphoreHandle_t persist_message_mutex;
SemaphoreHandle_t persist_message_send;

#endif
