#ifndef QUEUES_H
#define QUEUES_H

#include "freertos/queue.h"

QueueHandle_t tx_task_queue;
QueueHandle_t isotp_send_message_queue;

#endif
