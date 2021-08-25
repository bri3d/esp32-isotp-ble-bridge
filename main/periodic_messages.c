#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "periodic_messages.h"
#include "queues.h"

void periodic_messages_task(void *arg) {
  while (1) {
    periodic_message_t *periodic_message = (periodic_message_t*)arg;
    send_message_t msg = periodic_message->msg;
    xQueueSend(isotp_send_message_queue, &msg, pdMS_TO_TICKS(50));
    vTaskDelay(pdMS_TO_TICKS(periodic_message->interval_ms));
  }
  vTaskDelete(NULL);
}
