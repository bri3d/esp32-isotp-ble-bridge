#include "freertos/queue.h"

QueueHandle_t websocket_send_queue;
QueueHandle_t tx_task_queue;
QueueHandle_t isotp_send_message_queue;
