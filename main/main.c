#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "soc/dport_reg.h"
#include "assert.h"
#include "isotp.h"
#include "ble_server.h"
//#include "wifi_server.h"
//#include "web_server.h"
#include "ws2812_control.h"
#include "messages.h"
#include "queues.h"
#include "mutexes.h"
#include "endian_helpers.h"
#include "twai.h"
#include "ble_callbacks.h"
#include "isotp_callbacks.h"
#include "isotp_link_containers.h"
#include "isotp_links.h"
#include "isotp_tasks.h"
#include "task_priorities.h"
#include "periodic_messages.h"

#define MAIN_TAG "main"

/* --------------------- Definitions and static variables ------------------ */
#define SILENT_GPIO_NUM 21     // For A0
#define LED_ENABLE_GPIO_NUM 13 // For A0
#define LED_GPIO_NUM 2         // For A0
#define GPIO_OUTPUT_PIN_SEL(X) ((1ULL << X))

/* ------------ Primary startup ---------------- */

void app_main(void)
{
    // turn off logs for production
    esp_log_level_set("*", ESP_LOG_NONE);
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
    ws2812_write_leds(RED_LED_STATE);
    //Create semaphores and tasks
    tx_task_queue = xQueueCreate(128, sizeof(twai_message_t));
    //websocket_send_queue = xQueueCreate(128, sizeof(send_message_t));
    isotp_send_message_queue = xQueueCreate(128, sizeof(send_message_t));
    done_sem = xSemaphoreCreateBinary();
    isotp_send_queue_sem = xSemaphoreCreateBinary();
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
    ESP_LOGI(MAIN_TAG, "CAN/TWAI Driver installed");
    // Setup BLE server
    ble_server_callbacks callbacks = {
        .data_received = received_from_ble,
        .command_received = ble_command_received,
        .notifications_subscribed = notifications_enabled,
        .notifications_unsubscribed = notifications_disabled
    };
    ble_server_setup(callbacks);
    // Setup WiFi
    //wifi_ap_server_setup(); we host an SSID and clients connect to us
    //wifi_station_server_setup(); // we connect to an SSID and clients connect to a bound port
    // Setup web server
    //web_server_setup();
    // CAN/TWAI driver
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(MAIN_TAG, "CAN/TWAI Driver started");
    // mark all isotp_link_containers uninitialized by default
    for (int i = 0; i < NUM_ISOTP_LINK_CONTAINERS; ++i) {
        IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[i];
        IsoTpLink *link_ptr = &isotp_link_container->link;
        link_ptr->initialized = false;
    }
    // Tasks :
    // "websocket_sendTask" polls the websocket_send_queue queue. This queue is populated when a ISO-TP PDU is received.
    // "TWAI_rx" polls the receive queue (blocking) and once a message exists, forwards it into the ISO-TP library.
    // "TWAI_tx" blocks on a send queue which is populated by the callback from the ISO-TP library
    // "ISOTP_process" pumps the ISOTP library's "poll" method, which will call the send queue callback if a message needs to be sent.
    // ISOTP_process also polls the ISOTP library's non-blocking receive method, which will produce a message if one is ready.
    // "MAIN_process_send_queue" processes queued messages from the BLE stack. These messages are dynamically allocated when they are queued and freed in this task.
    //xTaskCreatePinnedToCore(websocket_send_task, "websocket_sendTask", 4096, NULL, SOCKET_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(isotp_send_queue_task, "ISOTP_process_send_queue", 4096, NULL, MAIN_TSK_PRIO, NULL, tskNO_AFFINITY);
    ESP_LOGI(MAIN_TAG, "Tasks started");
    // unlock isotp_send_queue
    xSemaphoreGive(isotp_send_queue_sem);
    // lock done_sem
    xSemaphoreTake(done_sem, portMAX_DELAY);
    // uninstall driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(MAIN_TAG, "Driver uninstalled");
    // cleanup
    vSemaphoreDelete(isotp_send_queue_sem);
    vSemaphoreDelete(done_sem);
    vSemaphoreDelete(isotp_mutex);
    vQueueDelete(tx_task_queue);
    vQueueDelete(isotp_send_message_queue);
}
