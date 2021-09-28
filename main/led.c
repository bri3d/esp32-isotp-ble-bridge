#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "isotp.h"
#include "mutexes.h"
#include "queues.h"
#include "constants.h"
#include "ws2812_control.h"
#include "led.h"

#define LED_TAG 			"LED"
#define LED_MIN_DELAY		10
#define LED_MAX_DELAY		10000
#define LED_MAX_POSITION	1000
#define LED_TSK_PRIO      	0

SemaphoreHandle_t led_mutex = NULL;
int32_t led_delay 			= LED_MIN_DELAY;
int32_t led_position 		= 0;
int32_t led_position_end 	= LED_MAX_POSITION;
int32_t led_direction  		= 1;
int32_t led_color_from_r	= 0;
int32_t led_color_from_g	= 0;
int32_t led_color_from_b	= 0;
int32_t led_color_add_r		= 0;
int32_t led_color_add_g		= 0;
int32_t led_color_add_b		= 0;
uint16_t led_kill			= false;

// LED colors
static struct led_state led_state = {
	.leds[0] = 0x008000
};

void set_color(int16_t position)
{
	int8_t r = led_color_from_r - (led_color_add_r * position / led_position_end);
	int8_t g = led_color_from_g - (led_color_add_g * position / led_position_end);
	int8_t b = led_color_from_b - (led_color_add_b * position / led_position_end);

	uint32_t color = ((g & 0xFF) << 16) + ((r & 0xFF) << 8) + (b & 0xFF);

	led_state.leds[0] = color;
	ws2812_write_leds(led_state);
}

void led_task(void *arg)
{
	while(led_kill == false) {
		xSemaphoreTake(led_mutex, pdMS_TO_TICKS(TIMEOUT_SHORT));
		if(++led_position >= led_position_end) {
			led_position = 0;
			led_direction++;
		}
		set_color(led_direction%2?led_position:led_position_end-led_position);
		xSemaphoreGive(led_mutex);
		vTaskDelay(pdMS_TO_TICKS(led_delay));
	}
	led_setcolor(LED_OFF, LED_OFF, 1000, 1);
	vSemaphoreDelete(led_mutex);
    led_mutex = NULL;
	vTaskDelete(NULL);
}

void led_start()
{
	if(led_mutex)
		return;

	// Configure LED to Red
	ws2812_control_init(LED_GPIO_NUM);
	led_setcolor(LED_RED, LED_RED, 1000, 1);

	//Start led task
	led_kill = false;
	led_mutex = xSemaphoreCreateMutex();
	xTaskCreatePinnedToCore(led_task, "LED_process", 4096, NULL, LED_TSK_PRIO, NULL, tskNO_AFFINITY);
}

void led_stop()
{
	led_kill = true;
}

void led_setcolor(int32_t from, int32_t to, int16_t delay, int16_t positions)
{
	xSemaphoreTake(led_mutex, pdMS_TO_TICKS(TIMEOUT_SHORT));
	led_delay = delay;
	if(led_delay > LED_MAX_DELAY)
		led_delay = LED_MAX_DELAY;
	if(led_delay < LED_MIN_DELAY)
		led_delay = LED_MIN_DELAY;

	led_position_end = positions;
	if(led_position_end > LED_MAX_POSITION)
		led_position_end = LED_MAX_POSITION;
	if(led_position_end < 1)
		led_position_end = 1;

	led_direction = 1;
	led_position = 0;
	led_color_from_r = (from & 0xFF00) >> 8;
	led_color_from_g = (from & 0xFF0000) >> 16;
	led_color_from_b = from & 0xFF;
	led_color_add_r = ((from & 0xFF00) >> 8) - ((to & 0xFF00) >> 8);
	led_color_add_g = ((from & 0xFF0000) >> 16) - ((to & 0xFF0000) >> 16);
	led_color_add_b = (from & 0xFF) - (to & 0xFF);

	set_color(0);
	xSemaphoreGive(led_mutex);
}
