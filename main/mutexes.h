#ifndef MUTEXES_H
#define MUTEXES_H

#include "freertos/semphr.h"

SemaphoreHandle_t done_sem;
SemaphoreHandle_t isotp_send_queue_sem;
SemaphoreHandle_t isotp_mutex;
SemaphoreHandle_t persist_message_mutex;
SemaphoreHandle_t persist_message_send;

#endif
