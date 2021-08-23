#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "isotp.h"
#include "queues.h"

#define ISOTP_CALLBACKS_TAG "isotp_callbacks"

int isotp_user_send_can(uint32_t arbitration_id, const uint8_t *data, uint8_t size)
{
    ESP_LOGD(ISOTP_CALLBACKS_TAG, "isotp_user_send_can");
    twai_message_t frame = {
        .identifier = arbitration_id,
        .data_length_code = size
    };
    memcpy(frame.data, data, sizeof(frame.data));
    xQueueSend(tx_task_queue, &frame, portMAX_DELAY);
    return ISOTP_RET_OK;
}

uint32_t isotp_user_get_ms(void)
{
    return (esp_timer_get_time() / 1000ULL) & 0xFFFFFFFF;
}

void isotp_user_debug(const char *message, ...)
{
    va_list args;
    va_start(args, message);
    esp_log_writev(ESP_LOG_DEBUG, ISOTP_CALLBACKS_TAG, message, args);
    va_end(args);
}
