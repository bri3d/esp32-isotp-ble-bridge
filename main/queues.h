#ifndef QUEUES_H
#define QUEUES_H

#include "freertos/queue.h"

//Queue sizes
#define CAN_QUEUE_SIZE     			64
#define ISOTP_QUEUE_SIZE			64
#define UART_QUEUE_SIZE				96

QueueHandle_t uart_send_queue;
QueueHandle_t uart_receive_queue;
QueueHandle_t can_send_queue;
QueueHandle_t isotp_send_message_queue;

#endif
