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
        twai_message_t twai_rx_msg;
        twai_receive(&twai_rx_msg, portMAX_DELAY); // If no message available, should block and yield.
        ESP_LOGI(TWAI_TAG, "Received TWAI message with identifier %08X and length %08X", twai_rx_msg.identifier, twai_rx_msg.data_length_code);
        ESP_LOGD(
            TWAI_TAG,
             "%02X%02X%02X%02X%02X%02X%02X%02X",
            twai_rx_msg.data[0],
            twai_rx_msg.data[1],
            twai_rx_msg.data[2],
            twai_rx_msg.data[3],
            twai_rx_msg.data[4],
            twai_rx_msg.data[5],
            twai_rx_msg.data[6],
            twai_rx_msg.data[7]
        );
        for (int i = 0; i < twai_rx_msg.data_length_code; i++) {
            ESP_LOGD(TWAI_TAG, "RX Data: %02X", twai_rx_msg.data[i]);
        }
        // short circuit on low-level traffic
        if (twai_rx_msg.identifier < 0x500) {
            continue;
        }
        int isotp_link_container_index = find_isotp_link_container_index_by_receive_arbitration_id(twai_rx_msg.identifier);
        if (isotp_link_container_index != -1) {
            IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[isotp_link_container_index];
            ESP_LOGD(TWAI_TAG, "Taking isotp_mutex");
            xSemaphoreTake(isotp_mutex, (TickType_t)100);
            ESP_LOGD(TWAI_TAG, "Took isotp_mutex");
            isotp_on_can_message(&isotp_link_container->link, twai_rx_msg.data, twai_rx_msg.data_length_code);
            ESP_LOGD(TWAI_TAG, "twai_receive_task: giving isotp_mutex");
            xSemaphoreGive(isotp_mutex);
            ESP_LOGD(TWAI_TAG, "twai_receive_task: giving wait_for_isotp_data_sem");
            xSemaphoreGive(isotp_link_container->wait_for_isotp_data_sem);
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
