#include "freertos/queue.h"

QueueHandle_t websocket_send_queue;
QueueHandle_t tx_task_queue;
QueueHandle_t send_message_queue;
