#ifndef MUTEXES_H
#define MUTEXES_H

#include "freertos/semphr.h"

SemaphoreHandle_t sleep_can_sem;
SemaphoreHandle_t sleep_uart_mutex;
SemaphoreHandle_t sleep_uart_packet_sem;
SemaphoreHandle_t sleep_sem;
SemaphoreHandle_t uart_buffer_mutex;
SemaphoreHandle_t isotp_mutex;
SemaphoreHandle_t persist_message_mutex;
SemaphoreHandle_t persist_message_send;

#endif
