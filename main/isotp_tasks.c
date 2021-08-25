#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "ble_server.h"
#include "twai.h"
#include "isotp_link_containers.h"
#include "task_priorities.h"
#include "isotp.h"
#include "mutexes.h"
#include "messages.h"
#include "queues.h"

#define ISOTP_TASKS_TAG "isotp_tasks"

void isotp_processing_task(void *arg)
{
    IsoTpLinkContainer *isotp_link_container = (IsoTpLinkContainer*)arg;
    IsoTpLink *link_ptr = &isotp_link_container->link;
    uint8_t *payload_buf = isotp_link_container->payload_buf;
    while (1)
    {
        if (link_ptr->send_status != ISOTP_SEND_STATUS_INPROGRESS &&
            link_ptr->receive_status != ISOTP_RECEIVE_STATUS_INPROGRESS)
        {
            // Link is idle, wait for new data before pumping loop.
            xSemaphoreTake(isotp_link_container->wait_for_isotp_data_sem, portMAX_DELAY);
        }
        // poll
        xSemaphoreTake(isotp_mutex, (TickType_t)100);
        isotp_poll(link_ptr);
        xSemaphoreGive(isotp_mutex);
        // receive
        xSemaphoreTake(isotp_mutex, (TickType_t)100);
        uint16_t out_size;
        int ret = isotp_receive(link_ptr, payload_buf, ISOTP_BUFSIZE, &out_size);
        xSemaphoreGive(isotp_mutex);
        // if it is time to send fully received + parsed ISO-TP data over BLE and/or websocket
        if (ISOTP_RET_OK == ret) {
            ESP_LOGI(ISOTP_TASKS_TAG, "Received ISO-TP message with length: %04X", out_size);
            for (int i = 0; i < out_size; i++) {
                ESP_LOGD(ISOTP_TASKS_TAG, "payload_buf[%d] = %02x", i, payload_buf[i]);
            }
            ble_send(link_ptr->receive_arbitration_id, link_ptr->send_arbitration_id, payload_buf, out_size);
            // websocket_send(link_ptr->receive_arbitration_id, link_ptr->send_arbitration_id, payload_buf, out_size);
        }
        vTaskDelay(0); // Allow higher priority tasks to run, for example Rx/Tx
    }
    vTaskDelete(NULL);
}

void isotp_send_queue_task(void *arg)
{
    xSemaphoreTake(isotp_send_queue_sem, portMAX_DELAY);
    while (1)
    {
        twai_status_info_t status_info;
        twai_get_status_info(&status_info);
        if (status_info.state == TWAI_STATE_BUS_OFF)
        {
            twai_initiate_recovery();
        }
        else if (status_info.state == TWAI_STATE_STOPPED)
        {
            twai_start();
        }
        send_message_t msg;
        xQueueReceive(isotp_send_message_queue, &msg, portMAX_DELAY);
        xSemaphoreTake(isotp_mutex, (TickType_t)100);
        ESP_LOGI(ISOTP_TASKS_TAG, "isotp_send_queue_task: sending message with %d size (rx id: %08x / tx id: %08x)", msg.msg_length, msg.rx_id, msg.tx_id);
        // flipped
        int isotp_link_container_index = find_isotp_link_container_index_by_receive_arbitration_id(msg.tx_id);
        assert(isotp_link_container_index != -1);
        IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[isotp_link_container_index];
        ESP_LOGI(ISOTP_TASKS_TAG, "isotp_send_queue_task: link match");
        isotp_send(&isotp_link_container->link, msg.buffer, msg.msg_length);
        // cleanup
        xSemaphoreGive(isotp_mutex);
        xSemaphoreGive(isotp_link_container->wait_for_isotp_data_sem);
        free(msg.buffer);
    }
    vTaskDelete(NULL);
}