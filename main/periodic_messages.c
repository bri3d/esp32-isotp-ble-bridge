#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "periodic_messages.h"
#include "queues.h"

void periodic_messages_task(void *arg) {
  while (1) {
    periodic_message_t *periodic_message = (periodic_message_t*)arg;
    send_message_t *msg = periodic_message->msg;
    // clone because isotp_send_message_queue frees the buffer
    send_message_t *cloned_msg = calloc(1, sizeof(send_message_t));
    cloned_msg->rx_id = msg->rx_id;
    cloned_msg->tx_id = msg->tx_id;
    cloned_msg->msg_length = msg->msg_length;
    cloned_msg->buffer = malloc(msg->msg_length);
    memcpy(cloned_msg->buffer, msg->buffer, msg->msg_length);
    xQueueSend(isotp_send_message_queue, cloned_msg, pdMS_TO_TICKS(50));
    vTaskDelay(pdMS_TO_TICKS(periodic_message->interval_ms));
  }
  vTaskDelete(NULL);
}
