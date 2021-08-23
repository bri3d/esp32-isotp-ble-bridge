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
    msg.buffer = malloc(msg.msg_length);
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
        case 0x01: { // Command 1 : Change Tx/Rx addresses
            uint16_t tx_address = read_uint16_be(input + 1);
            uint16_t rx_address = read_uint16_be(input + 3);
            ESP_LOGI(BLE_CALLBACKS_TAG, "command_received[0x01]: tx_address = %04x rx_address = %04x", tx_address, rx_address);
            // TODO: change global tx_id + rx_id even though we are trying to move away form that?
            break;
        }
        case 0x02: { // Command 2 : upload chunk for ISO-TP payload
            uint16_t offset = read_uint16_be(input + 1);
            uint16_t length = read_uint16_be(input + 3);
            uint8_t *bytes = input + 5;
            ESP_LOGI(BLE_CALLBACKS_TAG, "command_received[0x02]: offset = %04x length = %04x", offset, length);
            memcpy(command_buf + offset, bytes, length);
            break;
        }
        case 0x03: { // Command 3 : flush ISO-TP payload chunks
            uint16_t length = read_uint16_be(input + 1);
            ESP_LOGI(BLE_CALLBACKS_TAG, "command_received[0x03]: length = %04x", length);
            received_from_ble(command_buf, length);
            break;
        }
        default: {
            break;
        }
    }
}