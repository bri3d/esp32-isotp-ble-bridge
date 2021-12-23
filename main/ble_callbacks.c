#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "messages.h"
#include "endian_helpers.h"
#include "queues.h"
#include "ws2812_control.h"
#include "isotp.h"
#include "isotp_link_containers.h"
#include "task_priorities.h"

#define BLE_CALLBACKS_TAG "ble_callbacks"
#define HEADER_SIZE 8 // 4 byte request arb ID + 4 byte reply arb ID

static uint8_t command_buf[ISOTP_BUFSIZE + HEADER_SIZE];

void received_from_ble(const void *src, size_t size)
{
    ESP_LOGI(BLE_CALLBACKS_TAG, "Received a message from BLE stack with length %08X", size);
    send_message_t msg;
    msg.rx_id = read_uint32_be(src);
    msg.tx_id = read_uint32_be(src + 4);
    msg.msg_length = size - 8;
    msg.reuse_buffer = false;
    msg.buffer = malloc(msg.msg_length);
    assert(msg.buffer != NULL);
    const uint8_t *pdu = src + 8;
    memcpy(msg.buffer, pdu, msg.msg_length);
    ESP_LOGI(BLE_CALLBACKS_TAG, "Received a message from BLE stack with length %08X rx_id %08x tx_id %08x", size, msg.rx_id, msg.tx_id);
    xQueueSend(isotp_send_message_queue, &msg, pdMS_TO_TICKS(50));
}

void notifications_disabled()
{
    ws2812_write_leds(RED_LED_STATE);
}

void notifications_enabled()
{
    ws2812_write_leds(GREEN_LED_STATE);
}

void ble_command_received(uint8_t *input, size_t length)
{
    uint8_t command_id = input[0];
    switch (command_id) {
        case 0x01: { // Command 1: Change global Tx/Rx addresses (deprecated)
            uint16_t tx_address = read_uint16_be(input + 1);
            uint16_t rx_address = read_uint16_be(input + 3);
            ESP_LOGI(BLE_CALLBACKS_TAG, "command_received[0x01]: tx_address = %04x rx_address = %04x", tx_address, rx_address);
            // TODO: change global tx_id + rx_id even though we are trying to move away form that?
            // TODO: send ble_command_received successful response
            break;
        }
        case 0x02: { // Command 2: upload chunk for ISO-TP payload
            uint16_t offset = read_uint16_be(input + 1);
            uint16_t length = read_uint16_be(input + 3);
            uint8_t *bytes = input + 5;
            ESP_LOGI(BLE_CALLBACKS_TAG, "command_received[0x02]: offset = %04x length = %04x", offset, length);
            memcpy(command_buf + offset, bytes, length);
            // TODO: send ble_command_received successful response
            break;
        }
        case 0x03: { // Command 3: flush ISO-TP payload chunks
            uint16_t length = read_uint16_be(input + 1);
            ESP_LOGI(BLE_CALLBACKS_TAG, "command_received[0x03]: length = %04x", length);
            received_from_ble(command_buf, length);
            // TODO: send ble_command_received successful response
            break;
        }
        case 0x04: { // Command 4: start periodic message
            uint8_t periodic_message_index = input[1];
            uint16_t interval_ms = read_uint16_be(input + 2);
            uint32_t rx_address = read_uint32_be(input + 4);
            uint32_t tx_address = read_uint32_be(input + 8);
            uint16_t num_msgs = read_uint16_be(input + 12);
            ESP_LOGI(BLE_CALLBACKS_TAG, "command_received[0x04]: periodic_message_index = %d interval_ms = %d rx_address = %08x tx_address = %08x num_msgs = %08x", periodic_message_index, interval_ms, rx_address, tx_address, num_msgs);
            // isotp_link_contianer
            int isotp_link_container_index = find_isotp_link_container_index_by_receive_arbitration_id(tx_address); // flipped?
            assert(isotp_link_container_index != -1);
            IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[isotp_link_container_index];
            // periodic message
            periodic_message_t *periodic_message = &isotp_link_container->periodic_messages[periodic_message_index];
            // task handle
            TaskHandle_t *task_handle = &isotp_link_container->periodic_message_task_handles[periodic_message_index];
            // interval
            periodic_message->interval_ms = interval_ms;
            // num_msgs
            periodic_message->num_msgs = num_msgs;
            // pdus
            int pointer = 14;
            for (int i = 0; i < num_msgs; ++i) {
                send_message_t *msg = &periodic_message->msgs[i];
                msg->rx_id = rx_address;
                msg->tx_id = tx_address;
                msg->msg_length = read_uint16_be(input + pointer);
                pointer += 2;
                msg->reuse_buffer = true;
                msg->buffer = malloc(msg->msg_length);
                msg->reuse_buffer = true;
                assert(msg->buffer != NULL);
                memcpy(msg->buffer, input + pointer, msg->msg_length);
                pointer += msg->msg_length;
            }
            // task
            xTaskCreatePinnedToCore(periodic_messages_task, "periodic_messages_task", 4096, periodic_message, ISOTP_TSK_PRIO, task_handle, tskNO_AFFINITY);
            // TODO: send ble_command_received successful response
            break;
        }
        case 0x05: { // Command 5: stop periodic message
            uint8_t periodic_message_index = input[1];
            uint32_t rx_address = read_uint32_be(input + 2);
            uint32_t tx_address = read_uint32_be(input + 6);
            ESP_LOGI(BLE_CALLBACKS_TAG, "command_received[0x05]: periodic_message_index = %d rx_address = %08x tx_address = %08x", periodic_message_index, rx_address, tx_address);
            int isotp_link_container_index = find_isotp_link_container_index_by_receive_arbitration_id(tx_address); // flipped?
            assert(isotp_link_container_index != -1);
            IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[isotp_link_container_index];
            TaskHandle_t task_handle = isotp_link_container->periodic_message_task_handles[periodic_message_index];
            if (task_handle != NULL) {
                vTaskDelete(task_handle);
                isotp_link_container->periodic_message_task_handles[periodic_message_index] = NULL;
            }
            // TODO: send ble_command_received successful response
            break;
        }
        case 0x06: { // Command 6: configure ISO-TP link
            int pointer = 1; // skip command_id
            uint32_t link_index = read_uint32_be(input + pointer);
            pointer += 4;
            uint32_t receive_arbitration_id = read_uint32_be(input + pointer);
            pointer += 4;
            uint32_t reply_arbitration_id = read_uint32_be(input + pointer);
            pointer += 4;
            uint32_t name_len = read_uint32_be(input + pointer);
            pointer += 4;
            char *name = malloc(name_len);
            memcpy(name, input + pointer, name_len);
            // TODO: bounds checking on whether isotp_link_containers[link_index] is already initalized
            // TODO: bounds checking on whether link_index > NUM_ISOTP_LINK_CONTAINERS - 1
            configure_isotp_link(link_index, receive_arbitration_id, reply_arbitration_id, name);
            // TODO: send ble_command_received successful response
            break;
        }
        default: {
            break;
        }
    }
}