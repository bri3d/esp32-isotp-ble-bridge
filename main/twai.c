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
#include "driver/twai.h"
#include "constants.h"

#define TWAI_TAG 		"TWAI"

static const twai_general_config_t g_config = {
	.mode = TWAI_MODE_NORMAL,
	.tx_io = TX_GPIO_NUM,
	.rx_io = RX_GPIO_NUM,
	.clkout_io = TWAI_IO_UNUSED,
	.bus_off_io = TWAI_IO_UNUSED,
	.tx_queue_len = 1024,
	.rx_queue_len = 1024,
	.alerts_enabled = TWAI_ALERT_NONE,
	.clkout_divider = 0,
	.intr_flags = ESP_INTR_FLAG_LEVEL1
};
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

void twai_receive_task(void *arg);
void twai_transmit_task(void *arg);

void twai_init()
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
	ESP_LOGI(TWAI_TAG, "CAN/TWAI Driver installed");
	ESP_ERROR_CHECK(twai_start());
	ESP_LOGI(TWAI_TAG, "CAN/TWAI Driver started");

	can_send_queue = xQueueCreate(TASK_QUEUE_SIZE, sizeof(twai_message_t));
}

void twai_deinit()
{
	ESP_ERROR_CHECK(twai_driver_uninstall());
	ESP_LOGI(TWAI_TAG, "Driver uninstalled");
}

void twai_start_task()
{
	xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
}

void twai_send_isotp_message(IsoTpLinkContainer* link, twai_message_t* msg)
{
	ESP_LOGD(TWAI_TAG, "twai_receive_task: link match");
	ESP_LOGD(TWAI_TAG, "Taking isotp_mutex");
	xSemaphoreTake(isotp_mutex, pdMS_TO_TICKS(TIMEOUT_NORMAL));
	ESP_LOGD(TWAI_TAG, "Took isotp_mutex");
	isotp_on_can_message(&link->link, msg->data, msg->data_length_code);
	ESP_LOGD(TWAI_TAG, "twai_receive_task: giving isotp_mutex");
	xSemaphoreGive(isotp_mutex);
	ESP_LOGD(TWAI_TAG, "twai_receive_task: giving wait_for_isotp_data_sem");
	xSemaphoreGive(link->wait_for_isotp_data_sem);
}

void twai_receive_task(void *arg)
{
    while (1)
	{
		twai_message_t twai_rx_msg;
		twai_receive(&twai_rx_msg, portMAX_DELAY); // If no message available, should block and yield.
		xSemaphoreTake(sleep_can_sem, 0);
		ESP_LOGI(TWAI_TAG, "Received TWAI %08X and length %08X", twai_rx_msg.identifier, twai_rx_msg.data_length_code);
        for (int i = 0; i < twai_rx_msg.data_length_code; i++) {
			ESP_LOGD(TWAI_TAG, "RX Data: %02X", twai_rx_msg.data[i]);
		}
		IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[isotp_link_container_id];
		if(twai_rx_msg.identifier == isotp_link_container->link.receive_arbitration_id) {
			twai_send_isotp_message(isotp_link_container, &twai_rx_msg);
		} else {
			for(uint16_t i = 0; i < NUM_ISOTP_LINK_CONTAINERS; i++) {
				isotp_link_container = &isotp_link_containers[i];
				if(twai_rx_msg.identifier == isotp_link_container->link.receive_arbitration_id) {
					twai_send_isotp_message(isotp_link_container, &twai_rx_msg);
					break;
				}
			}
		}
		taskYIELD();
    }
    vTaskDelete(NULL);
}

void twai_transmit_task(void *arg)
{
    while (1)
    {
        twai_message_t tx_msg;
		xQueueReceive(can_send_queue, &tx_msg, portMAX_DELAY);
        ESP_LOGD(TWAI_TAG, "Sending TWAI Message with ID %08X", tx_msg.identifier);
        for (int i = 0; i < tx_msg.data_length_code; i++) {
            ESP_LOGD(TWAI_TAG, "TX Data: %02X", tx_msg.data[i]);
        }
        twai_transmit(&tx_msg, portMAX_DELAY);
		ESP_LOGD(TWAI_TAG, "Sent TWAI Message with ID %08X", tx_msg.identifier);
		taskYIELD();
    }
    vTaskDelete(NULL);
}
