#include "freertos/queue.h"

static QueueHandle_t websocket_send_queue;
static QueueHandle_t tx_task_queue;
static QueueHandle_t send_message_queue;
