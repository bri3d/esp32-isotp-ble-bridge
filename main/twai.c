#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "isotp.h"
#include "mutexes.h"
#include "queues.h"
#include "isotp_link_containers.h"

#define TWAI_TAG "twai"

void twai_receive_task(void *arg)
{
    while (1)
    {
        twai_message_t rx_msg;
        twai_receive(&rx_msg, portMAX_DELAY); // If no message available, should block and yield.
        ESP_LOGI(TWAI_TAG, "Received TWAI message with identifier %08X and length %08X", rx_msg.identifier, rx_msg.data_length_code);
        for (int i = 0; i < rx_msg.data_length_code; i++) {
            ESP_LOGD(TWAI_TAG, "RX Data: %02X", rx_msg.data[i]);
        }
        ESP_LOGD(TWAI_TAG, "Taking isotp_mutex");
        xSemaphoreTake(isotp_mutex, (TickType_t)100);
        ESP_LOGD(TWAI_TAG, "Took isotp_mutex");
        for (int i = 0; i < NUM_ISOTP_LINK_CONTAINERS; ++i) {
            IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[i];
            if (rx_msg.identifier == isotp_link_container->link.receive_arbitration_id) {
                ESP_LOGD(TWAI_TAG, "twai_receive_task: link match");
                isotp_on_can_message(&isotp_link_container->link, rx_msg.data, rx_msg.data_length_code);
                xSemaphoreGive(isotp_mutex);
                xSemaphoreGive(isotp_link_container->wait_for_isotp_data_sem);
            }
        }
    }
    vTaskDelete(NULL);
}

void twai_transmit_task(void *arg)
{
    while (1)
    {
        twai_message_t tx_msg;
        xQueueReceive(tx_task_queue, &tx_msg, portMAX_DELAY);
        ESP_LOGD(TWAI_TAG, "Sending TWAI Message with ID %08X", tx_msg.identifier);
        for (int i = 0; i < tx_msg.data_length_code; i++) {
            ESP_LOGD(TWAI_TAG, "TX Data: %02X", tx_msg.data[i]);
        }
        twai_transmit(&tx_msg, portMAX_DELAY);
        ESP_LOGD(TWAI_TAG, "Sent TWAI Message with ID %08X", tx_msg.identifier);
    }
    vTaskDelete(NULL);
}
