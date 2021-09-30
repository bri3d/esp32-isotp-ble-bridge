#ifndef QUEUES_H
#define QUEUES_H

#include "freertos/queue.h"

//Queue sizes
#define TASK_QUEUE_SIZE     		64
#define MESSAGE_QUEUE_SIZE			64

QueueHandle_t tx_task_queue;
QueueHandle_t isotp_send_message_queue;

#endif
