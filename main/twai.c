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

#define TWAI_TAG 		"twai"

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

// TWAI/CAN configuration
/*static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); */

void twai_install()
{
	// "TWAI" is knockoff CAN. Install TWAI driver.
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
	ESP_LOGI(TWAI_TAG, "CAN/TWAI Driver installed");
	ESP_ERROR_CHECK(twai_start());
	ESP_LOGI(TWAI_TAG, "CAN/TWAI Driver started");
}

void twai_uninstall()
{
	ESP_ERROR_CHECK(twai_driver_uninstall());
	ESP_LOGI(TWAI_TAG, "Driver uninstalled");
}

void twai_receive_task(void *arg)
{
    while (1)
	{
		twai_message_t twai_rx_msg;
		twai_receive(&twai_rx_msg, portMAX_DELAY); // If no message available, should block and yield.
        ESP_LOGI(TWAI_TAG, "Received TWAI %08X and length %08X", twai_rx_msg.identifier, twai_rx_msg.data_length_code);
        for (int i = 0; i < twai_rx_msg.data_length_code; i++) {
            ESP_LOGD(TWAI_TAG, "RX Data: %02X", twai_rx_msg.data[i]);
        }
        for (int i = 0; i < NUM_ISOTP_LINK_CONTAINERS; ++i) {
            IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[i];
            // flipped?
            if (twai_rx_msg.identifier == isotp_link_container->link.receive_arbitration_id) {
				ESP_LOGD(TWAI_TAG, "twai_receive_task: link match");
				ESP_LOGD(TWAI_TAG, "Taking isotp_mutex");
				xSemaphoreTake(isotp_mutex, pdMS_TO_TICKS(TIMEOUT_NORMAL));
				ESP_LOGD(TWAI_TAG, "Took isotp_mutex");
				isotp_on_can_message(&isotp_link_container->link, twai_rx_msg.data, twai_rx_msg.data_length_code);
				ESP_LOGD(TWAI_TAG, "twai_receive_task: giving isotp_mutex");
				xSemaphoreGive(isotp_mutex);
                ESP_LOGD(TWAI_TAG, "twai_receive_task: giving wait_for_isotp_data_sem");
                xSemaphoreGive(isotp_link_container->wait_for_isotp_data_sem);
            }
		}
		vTaskDelay(0);
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
		vTaskDelay(0);
    }
    vTaskDelete(NULL);
}
