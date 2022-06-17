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
#include "twai.h"

#define TWAI_TAG 		"TWAI"

static const twai_general_config_t g_config = {
	.mode = CAN_MODE,
	.tx_io = CAN_TX_PORT,
	.rx_io = CAN_RX_PORT,
	.clkout_io = CAN_CLK_IO,
	.bus_off_io = CAN_BUS_IO,
	.tx_queue_len = CAN_INTERNAL_BUFFER_SIZE,
	.rx_queue_len = CAN_INTERNAL_BUFFER_SIZE,
	.alerts_enabled = CAN_ALERTS,
	.clkout_divider = CAN_CLK_DIVIDER,
	.intr_flags = CAN_FLAGS
};
static const twai_timing_config_t	t_config				= CAN_TIMING;
static const twai_filter_config_t	f_config				= CAN_FILTER;
static SemaphoreHandle_t			twai_receive_task_mutex = NULL;
static SemaphoreHandle_t			twai_send_task_mutex	= NULL;
static bool16						twai_run_task			= false;

void twai_receive_task(void *arg);
void twai_transmit_task(void *arg);

void twai_init()
{
	twai_deinit();

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

	twai_receive_task_mutex = xSemaphoreCreateMutex();
	twai_send_task_mutex	= xSemaphoreCreateMutex();
	can_send_queue			= xQueueCreate(CAN_QUEUE_SIZE, sizeof(twai_message_t));

	ESP_LOGI(TWAI_TAG, "Init");
}

void twai_deinit()
{
	bool16 didDeInit = false;

	twai_driver_uninstall();

	if (can_send_queue) {
		vQueueDelete(can_send_queue);
		can_send_queue = NULL;
		didDeInit = true;
	}

	if (twai_receive_task_mutex) {
		vSemaphoreDelete(twai_receive_task_mutex);
		twai_receive_task_mutex = NULL;
		didDeInit = true;
	}

	if (twai_send_task_mutex) {
		vSemaphoreDelete(twai_send_task_mutex);
		twai_send_task_mutex = NULL;
		didDeInit = true;
	}

	if (didDeInit)
		ESP_LOGI(TWAI_TAG, "Deinit");
}

void twai_start_task()
{
	twai_stop_task();

	twai_run_task = true;

	ESP_LOGI(TWAI_TAG, "Tasks starting");
	xSemaphoreTake(sync_task_sem, 0);
	xTaskCreate(twai_receive_task, "TWAI_rx", TASK_STACK_SIZE, NULL, RX_TASK_PRIO, NULL);
	xSemaphoreTake(sync_task_sem, portMAX_DELAY);
	xTaskCreate(twai_transmit_task, "TWAI_tx", TASK_STACK_SIZE, NULL, TX_TASK_PRIO, NULL);
	xSemaphoreTake(sync_task_sem, portMAX_DELAY);
	ESP_LOGI(TWAI_TAG, "Tasks started");
}

void twai_stop_task()
{
	if (twai_run_task) {
		twai_run_task = false;

		xSemaphoreTake(twai_receive_task_mutex, portMAX_DELAY);
		xSemaphoreGive(twai_receive_task_mutex);
		
		twai_message_t twai_tx_msg;
		xQueueSend(can_send_queue, &twai_tx_msg, portMAX_DELAY);
		xSemaphoreTake(twai_send_task_mutex, portMAX_DELAY);
		xSemaphoreGive(twai_send_task_mutex);

		ESP_LOGI(TWAI_TAG, "Tasks stopped");
	}
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
	xSemaphoreTake(twai_receive_task_mutex, portMAX_DELAY);
	ESP_LOGI(TWAI_TAG, "Receive task started");
	xSemaphoreGive(sync_task_sem);
	twai_message_t twai_rx_msg;
    while (twai_run_task)
	{
		if (twai_receive(&twai_rx_msg, pdMS_TO_TICKS(TIMEOUT_NORMAL)) == ESP_OK) {
			xSemaphoreTake(ch_can_timer_sem, 0);
			ESP_LOGI(TWAI_TAG, "Received TWAI %08X and length %08X", twai_rx_msg.identifier, twai_rx_msg.data_length_code);
			for (int i = 0; i < twai_rx_msg.data_length_code; i++) {
				ESP_LOGD(TWAI_TAG, "RX Data: %02X", twai_rx_msg.data[i]);
			}
			IsoTpLinkContainer* isotp_link_container = &isotp_link_containers[isotp_link_container_id];
			if (twai_rx_msg.identifier == isotp_link_container->link.receive_arbitration_id) {
				twai_send_isotp_message(isotp_link_container, &twai_rx_msg);
			}
			else {
				for (uint16_t i = 0; i < NUM_ISOTP_LINK_CONTAINERS; i++) {
					isotp_link_container = &isotp_link_containers[i];
					if (twai_rx_msg.identifier == isotp_link_container->link.receive_arbitration_id) {
						twai_send_isotp_message(isotp_link_container, &twai_rx_msg);
						break;
					}
				}
			}
		}
		taskYIELD();
    }
	ESP_LOGI(TWAI_TAG, "Receive task stopped");
	xSemaphoreGive(twai_receive_task_mutex);
    vTaskDelete(NULL);
}

void twai_transmit_task(void *arg)
{
	xSemaphoreTake(twai_send_task_mutex, portMAX_DELAY);
	ESP_LOGI(TWAI_TAG, "Send task started");
	xSemaphoreGive(sync_task_sem);
	twai_message_t twai_tx_msg;
    while (twai_run_task)
    {
		if (xQueueReceive(can_send_queue, &twai_tx_msg, portMAX_DELAY) == pdTRUE) {
			if (twai_run_task) {
				ESP_LOGD(TWAI_TAG, "Sending TWAI Message with ID %08X", twai_tx_msg.identifier);
				for (int i = 0; i < twai_tx_msg.data_length_code; i++) {
					ESP_LOGD(TWAI_TAG, "TX Data: %02X", twai_tx_msg.data[i]);
				}
				twai_transmit(&twai_tx_msg, portMAX_DELAY);
				ESP_LOGD(TWAI_TAG, "Sent TWAI Message with ID %08X", twai_tx_msg.identifier);
			}
		}
		taskYIELD();
    }
	ESP_LOGI(TWAI_TAG, "Send task stopped");
	xSemaphoreGive(twai_send_task_mutex);
    vTaskDelete(NULL);
}
