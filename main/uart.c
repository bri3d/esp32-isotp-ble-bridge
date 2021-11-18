#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "mutexes.h"
#include "queues.h"
#include "constants.h"
#include "ble_server.h"
#include "uart.h"

#define UART_TAG 		"UART"

bool 		uart_packet_started		= false;
uint16_t 	uart_buffer_length		= 0;
uint16_t 	uart_buffer_pos 		= 0;
uint8_t  	uart_buffer[UART_BUFFER_SIZE];

bool 		uart_buffer_check_header();
bool 		uart_buffer_add(uint8_t* tmp_buffer, uint16_t size);
bool 		uart_buffer_get(uint8_t* tmp_buffer, uint16_t size);
bool 		uart_buffer_parse();
uint8_t 	uart_buffer_check_byte(uint16_t pos);
uint16_t 	uart_buffer_check_word(uint16_t pos);

void 		uart_send_task(void *arg);
void 		uart_receive_task(void *arg);

void uart_init()
{
	/* Configure parameters of an UART driver,
	 * communication pins and install the driver */
	uart_config_t uart_config = {
		.baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
	};

	uart_send_queue = xQueueCreate(TASK_QUEUE_SIZE, sizeof(send_message_t));
	ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUFFER_SIZE * 2, UART_BUFFER_SIZE * 2, UART_QUEUE_SIZE, &uart_receive_queue, 0));
	ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TXD, UART_RXD, UART_RTS, UART_CTS));
	uart_buffer_mutex = xSemaphoreCreateMutex();
}

void uart_deinit()
{
	ESP_ERROR_CHECK(uart_driver_delete(UART_PORT_NUM));
	vSemaphoreDelete(uart_buffer_mutex);
}

void uart_start_task()
{
	xTaskCreatePinnedToCore(uart_receive_task, "UART_receive_process", 4096, NULL, UART_TSK_PRIO, NULL, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(uart_send_task, "UART_send_process", 4096, NULL, UART_TSK_PRIO, NULL, tskNO_AFFINITY);
}

void uart_send(uint32_t txID, uint32_t rxID, uint8_t flags, const void* src, size_t size)
{
	send_message_t msg;
	msg.buffer = malloc(size+sizeof(ble_header_t));
	msg.msg_length = size+sizeof(ble_header_t);

	//Build header
	ble_header_t* header = (ble_header_t*)msg.buffer;
	header->hdID = BLE_HEADER_ID;
	header->cmdFlags = flags;
	header->rxID = rxID;
	header->txID = txID;
    header->cmdSize = size;
	memcpy(msg.buffer+sizeof(ble_header_t), src, size);

	xQueueSend(uart_send_queue, &msg, 50 / portTICK_PERIOD_MS);
}

void uart_send_task(void *arg)
{
	send_message_t event;
	while(1)
	{
		if(xQueueReceive(uart_send_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
			if(event.msg_length) {
				uart_write_bytes(UART_PORT_NUM, (const char*) event.buffer, event.msg_length);
				free(event.buffer);
			}
		}
	}
}

void uart_buffer_clear()
{
	uart_packet_started	= false;
	uart_buffer_length	= 0;
	uart_buffer_pos 	= 0;
}

bool uart_buffer_check_header()
{
	if(uart_buffer_length) {
		if(uart_buffer_check_byte(0) == BLE_HEADER_ID) {
			if(uart_buffer_length >= sizeof(ble_header_t)) {

				//check command length, fail if oversized
				uint16_t packet_len = uart_buffer_check_word(6) + sizeof(ble_header_t);
				if(packet_len > UART_BUFFER_SIZE) {
					uart_buffer_clear();
					return false;
				}

				return true;
			}
		} else {
			uart_buffer_clear();
			return false;
		}
	}

	return false;
}

bool uart_buffer_add(uint8_t* tmp_buffer, uint16_t size)
{
	bool 		over_flow 	= false;
	uint16_t	tmp_pos 	= 0;

	//check for over flow of the buffer
	if(size + uart_buffer_length > UART_BUFFER_SIZE) {
		size = UART_BUFFER_SIZE - uart_buffer_length;
		over_flow = true;
	}

	//check current write position (buffer pos + length) and if its beyond buffer length, wrap
	uint16_t write_pos = uart_buffer_pos + uart_buffer_length;
	if(write_pos >= UART_BUFFER_SIZE)
		write_pos -= UART_BUFFER_SIZE;

	//if the size of the data extends beyond end of buffer, write partial
	if(write_pos + size > UART_BUFFER_SIZE) {
		uint16_t partial_size = UART_BUFFER_SIZE - write_pos;
		memcpy(&uart_buffer[write_pos], tmp_buffer, partial_size);
		size -= partial_size;
		uart_buffer_length += partial_size;
		tmp_pos = partial_size;
		write_pos = 0;
	}

	//write the remaining data to buffer
	if(size) {
		memcpy(&uart_buffer[write_pos], &tmp_buffer[tmp_pos], size);
		uart_buffer_length += size;
	}

	return over_flow;
}

bool uart_buffer_get(uint8_t* tmp_buffer, uint16_t size)
{
	bool 		over_flow 	= false;
	uint16_t	tmp_pos 	= 0;

	if(size > uart_buffer_length) {
		size = uart_buffer_length;
		over_flow = true;
	}

	if(uart_buffer_pos + size > UART_BUFFER_SIZE) {
		uint16_t partial_size = UART_BUFFER_SIZE - uart_buffer_pos;
		memcpy(tmp_buffer, &uart_buffer[uart_buffer_pos], partial_size);
		size -= partial_size;
		uart_buffer_length -= partial_size;
		uart_buffer_pos = 0;
		tmp_pos = partial_size;
	}

	if(size) {
		memcpy(&tmp_buffer[tmp_pos], &uart_buffer[uart_buffer_pos], size);
		uart_buffer_length -= size;
		uart_buffer_pos += size;
	}

	return over_flow;
}

uint8_t uart_buffer_check_byte(uint16_t pos)
{
	uint16_t tmp_pos = uart_buffer_pos + pos;
	if(tmp_pos >= UART_BUFFER_SIZE) tmp_pos -= UART_BUFFER_SIZE;
	uint8_t tmp_data = uart_buffer[tmp_pos];

	return tmp_data;
}

uint16_t uart_buffer_check_word(uint16_t pos)
{
	uint16_t tmp_pos = uart_buffer_pos + pos;
	uint16_t tmp_pos1 = tmp_pos + 1;
	if(tmp_pos >= UART_BUFFER_SIZE) tmp_pos -= UART_BUFFER_SIZE;
	if(tmp_pos1 >= UART_BUFFER_SIZE) tmp_pos1 -= UART_BUFFER_SIZE;
	uint16_t tmp_data = (uart_buffer[tmp_pos+1] << 8) + uart_buffer[tmp_pos];

	return tmp_data;
}

bool uart_buffer_parse()
{
	if(uart_buffer_check_header()) {
		//if we are just starting a packet set our timeout
		if(!uart_packet_started) {
			uart_packet_started = true;
			xSemaphoreTake(sleep_uart_packet_sem, 0);
		}

		//get packet length
		uint16_t packet_len = uart_buffer_check_word(6) + sizeof(ble_header_t);
		if(packet_len > UART_BUFFER_SIZE)
			packet_len = UART_BUFFER_SIZE;

		//if data is >= to packet length, send it
		if(uart_buffer_length >= packet_len) {
			uint8_t* packet_data = malloc(packet_len);
			uart_buffer_get(packet_data, packet_len);
			uart_data_received((void*)packet_data, packet_len);
			free(packet_data);

			uart_packet_started = false;
			return true;
		}
	}

	return false;
}

void uart_receive_task(void *arg)
{
	uart_event_t 	event;
	while(1)
	{
		if(xQueueReceive(uart_receive_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
			ESP_LOGI(UART_TAG, "uart[%d] event:", UART_PORT_NUM);
			xSemaphoreTake(uart_buffer_mutex, pdMS_TO_TICKS(TIMEOUT_NORMAL));
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
				case UART_DATA:
					ESP_LOGI(UART_TAG, "[UART DATA]: %d", event.size);
					uint8_t* tmp_buffer = malloc(event.size);
					uart_read_bytes(UART_PORT_NUM, tmp_buffer, event.size, portMAX_DELAY);
					uart_buffer_add(tmp_buffer, event.size);
					free(tmp_buffer);
					while(uart_buffer_parse());
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
					ESP_LOGI(UART_TAG, "hw fifo overflow");
					uart_buffer_clear();
					uart_flush_input(UART_PORT_NUM);
					xQueueReset(uart_receive_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
					ESP_LOGI(UART_TAG, "ring buffer full");
					uart_buffer_clear();
					uart_flush_input(UART_PORT_NUM);
					xQueueReset(uart_receive_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
					ESP_LOGI(UART_TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
					ESP_LOGI(UART_TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
					ESP_LOGI(UART_TAG, "uart frame error");
                    break;
                //Others
                default:
					ESP_LOGI(UART_TAG, "uart event type: %d", event.type);
                    break;
			}
			xSemaphoreGive(uart_buffer_mutex);
		}
		taskYIELD();
    }
	vTaskDelete(NULL);
}
