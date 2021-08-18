#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/twai.h"
#include "soc/dport_reg.h"
#include "isotp.h"
#include "ble_server.h"
#include "wifi_server.h"
#include "web_server.h"
#include "ws2812_control.h"
#include "messages.h"
#include "queues.h"
#include "arbitration_identifiers.h"

/* --------------------- Definitions and static variables ------------------ */
#define RX_TASK_PRIO 3         // Ensure we drain the RX queue as quickly as we reasonably can to prevent overflow and ensure the message pump has fresh data.
#define TX_TASK_PRIO 3         // Ensure we TX messages as quickly as we reasonably can to meet ISO15765-2 timing constraints
#define ISOTP_TSK_PRIO 2       // Run the message pump at a higher priority than the main queue/dequeue task when messages are available
#define SOCKET_TASK_PRI 1      // Run the socket task at a low priority.
#define MAIN_TSK_PRIO 1        // Run the main task at the same priority as the BLE queue/dequeue tasks to help in delivery ordering.
#define TX_GPIO_NUM 5          // For A0
#define RX_GPIO_NUM 4          // For A0
#define SILENT_GPIO_NUM 21     // For A0
#define LED_ENABLE_GPIO_NUM 13 // For A0
#define LED_GPIO_NUM 2         // For A0
#define GPIO_OUTPUT_PIN_SEL(X) ((1ULL << X))
#define ISOTP_BUFSIZE 4096
#define EXAMPLE_TAG "ISOTPtoBLE"

#define ISOTP_MAX_RECEIVE_PAYLOAD 512

// TWAI/CAN configuration

static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

// Mutable globals for semaphores, queues, ISO-TP link

static SemaphoreHandle_t isotp_task_sem;
static SemaphoreHandle_t send_queue_start;
static SemaphoreHandle_t done_sem;
static SemaphoreHandle_t isotp_mutex;
static SemaphoreHandle_t isotp_wait_for_data;

static IsoTpLink isotp_link;

/* Alloc ISO-TP send and receive buffer statically in RAM, required by library */
static uint8_t isotp_recv_buf[ISOTP_BUFSIZE];
static uint8_t isotp_send_buf[ISOTP_BUFSIZE];

// LED colors

static struct led_state red_led_state = {
    .leds[0] = 0x004000};

static struct led_state green_led_state = {
    .leds[0] = 0x400000};

/* ---------------------------- ISOTP Callbacks ---------------------------- */

int isotp_user_send_can(const uint32_t arbitration_id, const uint8_t *data, const uint8_t size)
{
    twai_message_t frame = {.identifier = arbitration_id, .data_length_code = size};
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
}

/* --------------------------- Tasks and Functions -------------------------- */

static void twai_receive_task(void *arg)
{
    while (1)
    {
        twai_message_t rx_msg;
        twai_receive(&rx_msg, portMAX_DELAY); // If no message available, should block and yield.
        if (rx_msg.identifier == receive_identifier)
        {
            ESP_LOGD(EXAMPLE_TAG, "Received Message with identifier %08X and length %08X", rx_msg.identifier, rx_msg.data_length_code);
            for (int i = 0; i < rx_msg.data_length_code; i++)
                ESP_LOGD(EXAMPLE_TAG, "RX Data: %02X", rx_msg.data[i]);
            xSemaphoreTake(isotp_mutex, (TickType_t)100);
            isotp_on_can_message(&isotp_link, rx_msg.data, rx_msg.data_length_code);
            xSemaphoreGive(isotp_mutex);
            xSemaphoreGive(isotp_wait_for_data);
        }
    }
    vTaskDelete(NULL);
}

static void twai_transmit_task(void *arg)
{
    while (1)
    {
        twai_message_t tx_msg;
        xQueueReceive(tx_task_queue, &tx_msg, portMAX_DELAY);
        ESP_LOGD(EXAMPLE_TAG, "Sending Message with ID %08X", tx_msg.identifier);
        for (int i = 0; i < tx_msg.data_length_code; i++)
            ESP_LOGD(EXAMPLE_TAG, "TX Data: %02X", tx_msg.data[i]);
        twai_transmit(&tx_msg, portMAX_DELAY);
    }
    vTaskDelete(NULL);
}

static void configure_isotp_link()
{
    xSemaphoreTake(isotp_mutex, (TickType_t)100);
    isotp_init_link(&isotp_link, send_identifier,
                    isotp_send_buf, sizeof(isotp_send_buf),
                    isotp_recv_buf, sizeof(isotp_recv_buf));
    xSemaphoreGive(isotp_mutex);
}

static void isotp_processing_task(void *arg)
{
    xSemaphoreTake(isotp_task_sem, portMAX_DELAY);
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(EXAMPLE_TAG, "CAN/TWAI Driver started");
    configure_isotp_link();
    ESP_LOGI(EXAMPLE_TAG, "ISO-TP Handler started");

    xSemaphoreGive(send_queue_start);

    while (1)
    {
        if (isotp_link.send_status != ISOTP_SEND_STATUS_INPROGRESS && isotp_link.receive_status != ISOTP_RECEIVE_STATUS_INPROGRESS)
        {
            // Link is idle, wait for new data before pumping loop.
            xSemaphoreTake(isotp_wait_for_data, portMAX_DELAY);
        }
        xSemaphoreTake(isotp_mutex, (TickType_t)100);
        isotp_poll(&isotp_link);
        xSemaphoreGive(isotp_mutex);
        uint16_t out_size;
        uint8_t payload[ISOTP_MAX_RECEIVE_PAYLOAD];
        xSemaphoreTake(isotp_mutex, (TickType_t)100);
        int ret = isotp_receive(&isotp_link, payload, sizeof(payload), &out_size);
        xSemaphoreGive(isotp_mutex);
        if (ISOTP_RET_OK == ret)
        {
            ESP_LOGD(EXAMPLE_TAG, "Received ISO-TP message with length: %04X", out_size);
            for (int i = 0; i < out_size; i++)
                ESP_LOGD(EXAMPLE_TAG, "ISO-TP data %c", payload[i]);
            ble_send(payload, out_size);
            websocket_send(payload, out_size);
        }
        vTaskDelay(0); // Allow higher priority tasks to run, for example Rx/Tx
    }

    vTaskDelete(NULL);
}

static void send_queue_task(void *arg)
{
    xSemaphoreTake(send_queue_start, portMAX_DELAY);
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
        xQueueReceive(send_message_queue, &msg, portMAX_DELAY);
        xSemaphoreTake(isotp_mutex, (TickType_t)100);
        isotp_send(&isotp_link, msg.buffer, msg.msg_length);
        xSemaphoreGive(isotp_mutex);
        xSemaphoreGive(isotp_wait_for_data);
        free(msg.buffer);
    }
    vTaskDelete(NULL);
}

/* ----------- BLE callbacks ---------------- */

void received_from_ble(const void *src, size_t size)
{
    ESP_LOGI(EXAMPLE_TAG, "Received a message from BLE stack with length %08X", size);
    send_message_t msg;
    msg.buffer = malloc(size);
    memcpy(msg.buffer, src, size);
    msg.msg_length = size;
    xQueueSend(send_message_queue, &msg, pdMS_TO_TICKS(50));
}

void notifications_disabled()
{
    ws2812_write_leds(red_led_state);
}

void notifications_enabled()
{
    ws2812_write_leds(green_led_state);
}

void command_received(uint8_t *cmd_str, size_t cmd_length)
{
    if (cmd_str[0] == 0x1)
    {
        // Command 1 : Change Tx/Rx addresses
        uint16_t tx_address = (uint16_t)cmd_str[2] | ((uint16_t)cmd_str[1] << 8);
        uint16_t rx_address = (uint16_t)cmd_str[4] | ((uint16_t)cmd_str[3] << 8);
        update_send_identifier(tx_address);
        update_receive_identifier(rx_address);
        ESP_LOGI(EXAMPLE_TAG, "Changing Tx ID to %04X, Rx ID to %04X", send_identifier, receive_identifier);
        configure_isotp_link();
    }
}

/* ------------ Primary startup ---------------- */

void app_main(void)
{
    // Configure LED enable pin (switches transistor to push LED)
    gpio_config_t io_conf_led;
    io_conf_led.intr_type = GPIO_INTR_DISABLE;
    io_conf_led.mode = GPIO_MODE_OUTPUT;
    io_conf_led.pin_bit_mask = GPIO_OUTPUT_PIN_SEL(LED_ENABLE_GPIO_NUM);
    io_conf_led.pull_down_en = 0;
    io_conf_led.pull_up_en = 0;
    gpio_config(&io_conf_led);
    gpio_set_level(LED_ENABLE_GPIO_NUM, 0);

    // Configure LED to Red
    ws2812_control_init(LED_GPIO_NUM);
    ws2812_write_leds(red_led_state);

    // default to communicating with ECU
    update_send_identifier(0x7E0);
    update_receive_identifier(0x7E8);

    //Create semaphores and tasks
    websocket_send_queue = xQueueCreate(10, sizeof(send_message_t));
    tx_task_queue = xQueueCreate(10, sizeof(twai_message_t));
    send_message_queue = xQueueCreate(10, sizeof(send_message_t));
    isotp_task_sem = xSemaphoreCreateBinary();
    done_sem = xSemaphoreCreateBinary();
    send_queue_start = xSemaphoreCreateBinary();
    isotp_wait_for_data = xSemaphoreCreateBinary();
    isotp_mutex = xSemaphoreCreateMutex();

    // Need to pull down GPIO 21 to unset the "S" (Silent Mode) pin on CAN Xceiver.
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL(SILENT_GPIO_NUM);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(SILENT_GPIO_NUM, 0);

    // "TWAI" is knockoff CAN. Install TWAI driver.
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "CAN/TWAI Driver installed");

    // Setup BLE server
    ble_server_callbacks callbacks = {
        .data_received = received_from_ble,
        .notifications_subscribed = notifications_enabled,
        .notifications_unsubscribed = notifications_disabled};
    ble_server_setup(callbacks);

    // Setup WiFi server
    wifi_server_setup();

    // Setup web server
    web_server_setup();

    // Tasks :
    // "websocket_sendTask" polls the websocket_send_queue queue. This queue is populated when a ISO-TP PDU is received.
    // "TWAI_rx" polls the receive queue (blocking) and once a message exists, forwards it into the ISO-TP library.
    // "TWAI_tx" blocks on a send queue which is populated by the callback from the ISO-TP library
    // "ISOTP_process" pumps the ISOTP library's "poll" method, which will call the send queue callback if a message needs to be sent.
    // ISOTP_process also polls the ISOTP library's non-blocking receive method, which will produce a message if one is ready.
    // "MAIN_process_send_queue" processes queued messages from the BLE stack. These messages are dynamically allocated when they are queued and freed in this task.

    xTaskCreatePinnedToCore(websocket_send_task, "websocket_sendTask", 4096, NULL, SOCKET_TASK_PRI, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(isotp_processing_task, "ISOTP_process", 4096, NULL, ISOTP_TSK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(send_queue_task, "MAIN_process_send_queue", 4096, NULL, MAIN_TSK_PRIO, NULL, tskNO_AFFINITY);

    xSemaphoreGive(isotp_task_sem); //Start Control task
    xSemaphoreTake(done_sem, portMAX_DELAY);

    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

    vSemaphoreDelete(isotp_task_sem);
    vSemaphoreDelete(send_queue_start);
    vSemaphoreDelete(done_sem);
    vSemaphoreDelete(isotp_mutex);
    vQueueDelete(tx_task_queue);
    vQueueDelete(send_message_queue);
}
