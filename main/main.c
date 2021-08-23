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

/* --------------------- Definitions and static variables ------------------ */
#define RX_TASK_PRIO 3         // Ensure we drain the RX queue as quickly as we reasonably can to prevent overflow and ensure the message pump has fresh data.
#define TX_TASK_PRIO 3         // Ensure we TX messages as quickly as we reasonably can to meet ISO15765-2 timing constraints
#define ISOTP_TSK_PRIO 2       // Run the message pump at a higher priority than the main queue/dequeue task when messages are available
#define SOCKET_TASK_PRIO 1      // Run the socket task at a low priority.
#define MAIN_TSK_PRIO 1        // Run the main task at the same priority as the BLE queue/dequeue tasks to help in delivery ordering.
#define SILENT_GPIO_NUM 21     // For A0
#define LED_ENABLE_GPIO_NUM 13 // For A0
#define LED_GPIO_NUM 2         // For A0

#define GPIO_OUTPUT_PIN_SEL(X) ((1ULL << X))
#define MAIN_TAG "main"

/* --------------------- Link containers ------------------ */


/* --------------------------- Tasks and Functions -------------------------- */
static void configure_isotp_links()
{
    // acquire lock
    xSemaphoreTake(isotp_mutex, (TickType_t)100);
    // RX_ID + TX_ID are flipped because this device acts as a "tester" listening for responses from ECUs. the ECU's TX is our RX
    IsoTpLinkContainer *ecu_isotp_link_container = &isotp_link_containers[0];
    IsoTpLinkContainer *tcu_isotp_link_container = &isotp_link_containers[1];
    IsoTpLinkContainer *ptcu_isotp_link_container = &isotp_link_containers[2];
    IsoTpLinkContainer *gateway_isotp_link_container = &isotp_link_containers[3];
    // ECU
    ecu_isotp_link_container->recv_buf = calloc(1, ISOTP_BUFSIZE);
    ecu_isotp_link_container->send_buf = calloc(1, ISOTP_BUFSIZE);
    ecu_isotp_link_container->payload_buf = calloc(1, ISOTP_BUFSIZE);
    assert(ecu_isotp_link_container->recv_buf != NULL);
    assert(ecu_isotp_link_container->send_buf != NULL);
    assert(ecu_isotp_link_container->payload_buf != NULL);
    isotp_init_link(
        &ecu_isotp_link_container->link,
        0x7E0, 0x7E8,
        ecu_isotp_link_container->send_buf, ISOTP_BUFSIZE,
        ecu_isotp_link_container->recv_buf, ISOTP_BUFSIZE
    );
    // TCU
    tcu_isotp_link_container->recv_buf = calloc(1, ISOTP_BUFSIZE);
    tcu_isotp_link_container->send_buf = calloc(1, ISOTP_BUFSIZE);
    tcu_isotp_link_container->payload_buf = calloc(1, ISOTP_BUFSIZE);
    assert(tcu_isotp_link_container->recv_buf != NULL);
    assert(tcu_isotp_link_container->send_buf != NULL);
    assert(tcu_isotp_link_container->payload_buf != NULL);
    isotp_init_link(
        &tcu_isotp_link_container->link,
        0x7E1, 0x7E9,
        tcu_isotp_link_container->send_buf, ISOTP_BUFSIZE,
        tcu_isotp_link_container->recv_buf, ISOTP_BUFSIZE
    );
    // PTCU
    ptcu_isotp_link_container->recv_buf = calloc(1, ISOTP_BUFSIZE);
    ptcu_isotp_link_container->send_buf = calloc(1, ISOTP_BUFSIZE);
    ptcu_isotp_link_container->payload_buf = calloc(1, ISOTP_BUFSIZE);
    assert(ptcu_isotp_link_container->recv_buf != NULL);
    assert(ptcu_isotp_link_container->send_buf != NULL);
    assert(ptcu_isotp_link_container->payload_buf != NULL);
    isotp_init_link(
        &ptcu_isotp_link_container->link,
        0x7E5, 0x7ED,
        ptcu_isotp_link_container->send_buf, ISOTP_BUFSIZE,
        ptcu_isotp_link_container->recv_buf, ISOTP_BUFSIZE
    );
    // Gateway
    gateway_isotp_link_container->recv_buf = calloc(1, ISOTP_BUFSIZE);
    gateway_isotp_link_container->send_buf = calloc(1, ISOTP_BUFSIZE);
    gateway_isotp_link_container->payload_buf = calloc(1, ISOTP_BUFSIZE);
    assert(gateway_isotp_link_container->recv_buf != NULL);
    assert(gateway_isotp_link_container->send_buf != NULL);
    assert(gateway_isotp_link_container->payload_buf != NULL);
    isotp_init_link(
        &gateway_isotp_link_container->link,
        0x607, 0x587,
        gateway_isotp_link_container->send_buf, ISOTP_BUFSIZE,
        gateway_isotp_link_container->recv_buf, ISOTP_BUFSIZE
    );
    // free lock
    xSemaphoreGive(isotp_mutex);
}

static void isotp_processing_task(void *arg)
{
    IsoTpLinkContainer *isotp_link_container = (IsoTpLinkContainer*)arg;
    IsoTpLink link = isotp_link_container->link;
    IsoTpLink *link_ptr = &link;
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
        int ret = isotp_receive(link_ptr, payload_buf, sizeof(payload_buf), &out_size);
        xSemaphoreGive(isotp_mutex);
        // if it is time to send fully received + parsed ISO-TP data over BLE and/or websocket
        if (ISOTP_RET_OK == ret) {
            ESP_LOGI(MAIN_TAG, "Received ISO-TP message with length: %04X", out_size);
            for (int i = 0; i < out_size; i++) {
                ESP_LOGD(MAIN_TAG, "ISO-TP data %02x", payload_buf[i]);
            }
            ble_send(link_ptr->receive_arbitration_id, link_ptr->send_arbitration_id, payload_buf, out_size);
            // websocket_send(link_ptr->receive_arbitration_id, link_ptr->send_arbitration_id, payload_buf, out_size);
        }
        vTaskDelay(0); // Allow higher priority tasks to run, for example Rx/Tx
    }
    vTaskDelete(NULL);
}

static void isotp_send_queue_task(void *arg)
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
        ESP_LOGI(MAIN_TAG, "isotp_send_queue_task: sending message with %d size (rx id: %08x / tx id: %08x)", msg.msg_length, msg.rx_id, msg.tx_id);
        for (int i = 0; i < NUM_ISOTP_LINK_CONTAINERS; ++i) {
            IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[i];
            if (msg.rx_id == isotp_link_container->link.send_arbitration_id) {
                ESP_LOGI(MAIN_TAG, "isotp_send_queue_task: link match");
                isotp_send(&isotp_link_container->link, msg.buffer, msg.msg_length);
                xSemaphoreGive(isotp_mutex);
                xSemaphoreGive(isotp_link_container->wait_for_isotp_data_sem);
            }
        }
        free(msg.buffer);
    }
    vTaskDelete(NULL);
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
    ws2812_write_leds(RED_LED_STATE);

    //Create semaphores and tasks
    websocket_send_queue = xQueueCreate(10, sizeof(send_message_t));
    tx_task_queue = xQueueCreate(10, sizeof(twai_message_t));
    isotp_send_message_queue = xQueueCreate(10, sizeof(send_message_t));
    done_sem = xSemaphoreCreateBinary();
    isotp_send_queue_sem = xSemaphoreCreateBinary();
    isotp_mutex = xSemaphoreCreateMutex();
    for (int i = 0; i < NUM_ISOTP_LINK_CONTAINERS; ++i) {
        IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[i];
        isotp_link_container->wait_for_isotp_data_sem = xSemaphoreCreateBinary();
    }

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

    // ISO-TP handler
    configure_isotp_links();
    ESP_LOGI(MAIN_TAG, "ISO-TP links configured");
    xSemaphoreGive(isotp_send_queue_sem);

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
    xTaskCreatePinnedToCore(isotp_processing_task, "ecu_ISOTP_process", 4096, &isotp_link_containers[0], ISOTP_TSK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(isotp_processing_task, "tcu_ISOTP_process", 4096, &isotp_link_containers[1], ISOTP_TSK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(isotp_processing_task, "ptcu_ISOTP_process", 4096, &isotp_link_containers[2], ISOTP_TSK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(isotp_processing_task, "gateway_ISOTP_process", 4096, &isotp_link_containers[3], ISOTP_TSK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(isotp_send_queue_task, "ISOTP_process_send_queue", 4096, NULL, MAIN_TSK_PRIO, NULL, tskNO_AFFINITY);
    ESP_LOGI(MAIN_TAG, "Tasks started");
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
