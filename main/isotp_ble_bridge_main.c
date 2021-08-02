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

/* --------------------- Definitions and static variables ------------------ */
#define RX_TASK_PRIO 3
#define TX_TASK_PRIO 2
#define ISOTP_TSK_PRIO tskIDLE_PRIORITY // Pump messages at idle priority
#define MAIN_TSK_PRIO 1
#define TX_GPIO_NUM 5 // For A0
#define RX_GPIO_NUM 4 // For A0
#define SILENT_GPIO_NUM 21 // For A0
#define GPIO_OUTPUT_PIN_SEL(X)  ((1ULL<<X))
#define ISOTP_BUFSIZE 4096
#define EXAMPLE_TAG "ISOTPtoBLE"

#define SEND_IDENTIFIER 0x7E0
#define RECEIVE_IDENTIFIER 0x7E8


static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

static QueueHandle_t tx_task_queue;
static SemaphoreHandle_t isotp_task_sem;
static SemaphoreHandle_t periodic_task_sem;
static SemaphoreHandle_t done_sem;
static SemaphoreHandle_t isotp_mutex;

static IsoTpLink isotp_link;

/* Alloc send and receive buffer statically in RAM */
static uint8_t isotp_recv_buf[ISOTP_BUFSIZE];
static uint8_t isotp_send_buf[ISOTP_BUFSIZE];

/* ---------------------------- ISOTP Callbacks ---------------------------- */

int isotp_user_send_can(const uint32_t arbitration_id, const uint8_t* data, const uint8_t size) {
    twai_message_t frame = {.identifier = arbitration_id, .data_length_code = size};
    memcpy(frame.data, data, sizeof(frame.data));
    xQueueSend(tx_task_queue, &frame, portMAX_DELAY);
    return ISOTP_RET_OK;                           
}

uint32_t isotp_user_get_ms(void) {
    return (esp_timer_get_time() / 1000ULL) & 0xFFFFFFFF;
}

void isotp_user_debug(const char* message, ...) {
}

/* --------------------------- Tasks and Functions -------------------------- */

static void twai_receive_task(void *arg)
{
    while (1)
    {
        twai_message_t rx_msg;
        twai_receive(&rx_msg, portMAX_DELAY); // If no message available, should block and yield.
        if(rx_msg.identifier == RECEIVE_IDENTIFIER) {
            ESP_LOGD(EXAMPLE_TAG, "Received Message with identifier %08X and length %08X", rx_msg.identifier, rx_msg.data_length_code);
            for (int i = 0; i < rx_msg.data_length_code; i++)
                ESP_LOGD(EXAMPLE_TAG, "RX Data: %02X", rx_msg.data[i]);
            xSemaphoreTake(isotp_mutex, (TickType_t)100);
            isotp_on_can_message(&isotp_link, rx_msg.data, rx_msg.data_length_code);
            xSemaphoreGive(isotp_mutex);
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

static void isotp_processing_task(void *arg)
{
    xSemaphoreTake(isotp_task_sem, portMAX_DELAY);
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(EXAMPLE_TAG, "CAN/TWAI Driver started");
    isotp_init_link(&isotp_link, SEND_IDENTIFIER,
						isotp_send_buf, sizeof(isotp_send_buf), 
						isotp_recv_buf, sizeof(isotp_recv_buf));
    ESP_LOGI(EXAMPLE_TAG, "ISO-TP Handler started");

    xSemaphoreGive(periodic_task_sem);

    while (1)
    {
        xSemaphoreTake(isotp_mutex, (TickType_t)100);
        isotp_poll(&isotp_link);
        xSemaphoreGive(isotp_mutex);
        uint16_t out_size;
        uint8_t payload[32];
        xSemaphoreTake(isotp_mutex, (TickType_t)100);
        int ret = isotp_receive(&isotp_link, payload, 32, &out_size);
        xSemaphoreGive(isotp_mutex);
        if (ISOTP_RET_OK == ret) {
            ESP_LOGI(EXAMPLE_TAG, "Received ISO-TP message with length: %04X", out_size);
            for(int i = 0; i < out_size; i++) 
                ESP_LOGI(EXAMPLE_TAG, "ISO-TP data %c", payload[i]);
        }
        vTaskDelay(0);
    }

    vTaskDelete(NULL);
}

static void send_periodic_task(void *arg)
{
    xSemaphoreTake(periodic_task_sem, portMAX_DELAY);
    while (1)
    {
        twai_status_info_t status_info;
        twai_get_status_info(&status_info);
        if (status_info.state == TWAI_STATE_BUS_OFF) {
            twai_initiate_recovery();
        } else if (status_info.state == TWAI_STATE_STOPPED) {
            twai_start();
        } 
        uint8_t data[8] = {0x22, 0xF1, 0x90};
        xSemaphoreTake(isotp_mutex, (TickType_t)100);
        isotp_send(&isotp_link, data, 3);
        xSemaphoreGive(isotp_mutex);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
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

    //Create semaphores and tasks
    tx_task_queue = xQueueCreate(10, sizeof(twai_message_t));
    isotp_task_sem = xSemaphoreCreateBinary();
    done_sem = xSemaphoreCreateBinary();
    periodic_task_sem = xSemaphoreCreateBinary();
    isotp_mutex = xSemaphoreCreateMutex();

    // Tasks :
    // "TWAI_rx" polls the receive queue (blocking) and once a message exists, forwards it into the ISO-TP library.
    // "TWAI_tx" blocks on a send queue which is populated by the callback from the ISO-TP library
    // "ISOTP_process" pumps the ISOTP library's "poll" method, which will call the send queue callback if a message needs to be sent.
    // ISOTP_process also polls the ISOTP library's non-blocking receive method, which will produce a message if one is ready.
    // "MAIN_periodic_message" currently just sends a UDS request every 500ms, and makes sure the bus is on. 

    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(isotp_processing_task, "ISOTP_process", 4096, NULL, ISOTP_TSK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(send_periodic_task, "MAIN_periodic_message", 4096, NULL, MAIN_TSK_PRIO, NULL, tskNO_AFFINITY);

    xSemaphoreGive(isotp_task_sem); //Start Control task
    xSemaphoreTake(done_sem, portMAX_DELAY);

    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

    vSemaphoreDelete(isotp_task_sem);
    vSemaphoreDelete(periodic_task_sem);
    vSemaphoreDelete(done_sem);
    vSemaphoreDelete(isotp_mutex);
    vQueueDelete(tx_task_queue);
}
