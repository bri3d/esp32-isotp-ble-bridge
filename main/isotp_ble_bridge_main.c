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
#include "iso15765_2.h"

/* --------------------- Definitions and static variables ------------------ */
//Example Configuration
#define RX_TASK_PRIO 9   //Receiving task priority
#define TX_TASK_PRIO 10  //Sending task priority
#define CTRL_TSK_PRIO 7 //Control task priority
#define MAIN_TSK_PRIO 8
#define TX_GPIO_NUM 5
#define RX_GPIO_NUM 4
#define SILENT_GPIO_NUM 21
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<SILENT_GPIO_NUM))
#define EXAMPLE_TAG "ISOTPtoBLE"

// TX_GPIO_NUM and RX_GPIO_NUM are provided by the ESP KConfig system

static uint8_t isotp_send_frame_callback(canbus_md mode, uint32_t id, uint8_t dlc, uint8_t *dt);
static void on_error(n_rslt err_type);
static uint32_t getms();
static void isotp_data_received(n_indn_t *info);

static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

static QueueHandle_t tx_task_queue;
static SemaphoreHandle_t ctrl_task_sem;
static SemaphoreHandle_t periodic_task_sem;
static SemaphoreHandle_t done_sem;

static iso15765_t isotp_handler =
    {
        .addr_md = N_ADM_NORMAL,
        .can_md = CANBUS_STANDARD,
        .clbs.send_frame = isotp_send_frame_callback,
        .clbs.on_error = on_error,
        .clbs.get_ms = getms,
        .clbs.indn = isotp_data_received,
        .config.stmin = 0x3,
        .config.bs = 0x0f,
        .config.n_bs = 100,
        .config.n_cr = 3};

n_req_t frame_to_send =
    {
        .n_ai.n_pr = 0x07,
        .n_ai.n_sa = 0x00,
        .n_ai.n_ta = 0x04,
        .n_ai.n_ae = 0x00,
        .n_ai.n_tt = N_TA_T_PHY,
        .msg = {0x22, 0xF1, 0x90},
        .msg_sz = 3,
};

static uint32_t getms()
{
    return (esp_timer_get_time() / 1000ULL) & 0xFFFFFFFF;
}

static uint8_t isotp_send_frame_callback(canbus_md mode, uint32_t id, uint8_t dlc, uint8_t *dt)
{
    twai_message_t frame = {.identifier = id, .data_length_code = dlc};
    memcpy(frame.data, dt, sizeof(frame.data));
    xQueueSend(tx_task_queue, &frame, portMAX_DELAY);
    return 0;
}

static void on_error(n_rslt err_type)
{
    ESP_LOGE(EXAMPLE_TAG, "ERROR OCCURED!:%04x", err_type);
}

static void isotp_data_received(n_indn_t *info)
{
    ESP_LOGI(EXAMPLE_TAG, "\n-- Received ISO-TP data! ");
    for (int i = 0; i < info->msg_sz; i++)
        ESP_LOGI(EXAMPLE_TAG, "%02X", info->msg[i]);
}

/* --------------------------- Tasks and Functions -------------------------- */

static void twai_receive_task(void *arg)
{
    while (1)
    {
        twai_message_t rx_msg;
        twai_receive(&rx_msg, portMAX_DELAY); // If no message available, should block and yield.
        ESP_LOGI(EXAMPLE_TAG, "Received Message with identifier %08X and length %08X", rx_msg.identifier, rx_msg.data_length_code);
        for (int i = 0; i < rx_msg.data_length_code; i++)
            ESP_LOGI(EXAMPLE_TAG, "RX Data: %02X", rx_msg.data[i]);
        canbus_frame_t cb_frame = {
            .id = rx_msg.identifier,
            .dlc = rx_msg.data_length_code};
        memcpy(cb_frame.dt, rx_msg.data, sizeof(cb_frame.dt));
        if(rx_msg.identifier == 0x7E8) {
            // iso15765_enqueue copies the message data into queue (pass by copy) so this is OK
            iso15765_enqueue(&isotp_handler, &cb_frame);
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
        ESP_LOGI(EXAMPLE_TAG, "Sending Message with ID %08X", tx_msg.identifier);
        for (int i = 0; i < tx_msg.data_length_code; i++)
            ESP_LOGI(EXAMPLE_TAG, "TX Data: %02X", tx_msg.data[i]);
        twai_status_info_t status_info;
        twai_get_status_info(&status_info);
        ESP_LOGI(EXAMPLE_TAG, "Current bus state : %08X", status_info.state);
        twai_transmit(&tx_msg, portMAX_DELAY);
    }
    vTaskDelete(NULL);
}

static void isotp_processing_task(void *arg)
{
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(EXAMPLE_TAG, "CAN/TWAI Driver started");
    iso15765_init(&isotp_handler);
    ESP_LOGI(EXAMPLE_TAG, "ISO-TP Handler started");

    xSemaphoreGive(periodic_task_sem);

    while (1)
    {
        iso15765_process(&isotp_handler);
        vTaskDelay(pdMS_TO_TICKS(10));
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
        iso15765_send(&isotp_handler, &frame_to_send);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(SILENT_GPIO_NUM, 0);

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "CAN/TWAI Driver installed");

    //Create semaphores and tasks
    tx_task_queue = xQueueCreate(10, sizeof(twai_message_t));
    ctrl_task_sem = xSemaphoreCreateBinary();
    done_sem = xSemaphoreCreateBinary();
    periodic_task_sem = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(isotp_processing_task, "ISOTP_process", 4096, NULL, CTRL_TSK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(send_periodic_task, "MAIN_periodic_message", 4096, NULL, MAIN_TSK_PRIO, NULL, tskNO_AFFINITY);

    xSemaphoreGive(ctrl_task_sem); //Start Control task
    xSemaphoreTake(done_sem, portMAX_DELAY);

    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

    vSemaphoreDelete(ctrl_task_sem);
    vSemaphoreDelete(periodic_task_sem);
    vSemaphoreDelete(done_sem);
    vQueueDelete(tx_task_queue);
}
