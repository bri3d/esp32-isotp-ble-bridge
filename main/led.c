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
#define LED_DELAY			50
#define LED_TSK_PRIO      	0
#define LED_FADE_COUNT		1
#define LED_MAX_FADE        10

#define LED_TYPE_COLOR		0
#define LED_TYPE_FADE		1

#define LED_FLAG_REPEAT		1

typedef struct {
	uint16_t position;
	uint16_t direction;
	uint16_t count;
	uint16_t flags;
	uint32_t colors[LED_MAX_FADE];
} led_fade_t;

SemaphoreHandle_t led_mutex = NULL;
uint32_t led_fade 			= 0;
uint16_t led_type			= LED_TYPE_COLOR;

led_fade_t led_fades[LED_FADE_COUNT] = {
//Data fade
{0,1,10,0, {0x000080, 0x000070, 0x000060, 0x000050, 0x000040, 0x000030, 0x200020, 0x400010, 0x600000, 0x800000}}
};

void led_task(void *arg)
{
	while(1) {
		xSemaphoreTake(led_mutex, pdMS_TO_TICKS(TIMEOUT_SHORT));
		if(led_type == LED_TYPE_FADE) {
			led_fade_t* current_fade = &led_fades[led_fade];
			if(++current_fade->position >= current_fade->count) {
				if(current_fade->flags & LED_FLAG_REPEAT) {
					current_fade->position = 0;
					current_fade->direction++;
				} else {
					current_fade->position = current_fade->count - 1;
				}
			}
			ws2812_write_leds(current_fade->colors[current_fade->direction%2?current_fade->position:current_fade->count - current_fade->position - 1]);
		}
		xSemaphoreGive(led_mutex);
		vTaskDelay(pdMS_TO_TICKS(LED_DELAY));
	}
	vTaskDelete(NULL);
}

void led_start()
{
	if(led_mutex)
		return;

	// Configure LED enable pin (switches transistor to push LED)
	gpio_config_t io_conf_led;
	io_conf_led.intr_type = GPIO_INTR_DISABLE;
	io_conf_led.mode = GPIO_MODE_OUTPUT;
	io_conf_led.pin_bit_mask = GPIO_OUTPUT_PIN_SEL(LED_ENABLE_GPIO_NUM);
	io_conf_led.pull_down_en = 0;
	io_conf_led.pull_up_en = 0;
	gpio_config(&io_conf_led);
	gpio_set_level(LED_ENABLE_GPIO_NUM, 0);

	//Start led task
	led_mutex = xSemaphoreCreateMutex();
	ws2812_control_init(LED_GPIO_NUM);
	led_setcolor(LED_RED_HALF);
	xTaskCreatePinnedToCore(led_task, "LED_process", 4096, NULL, LED_TSK_PRIO, NULL, tskNO_AFFINITY);
}

void led_stop()
{
	led_setcolor(LED_OFF);
}

void led_setcolor(uint32_t color)
{
	xSemaphoreTake(led_mutex, pdMS_TO_TICKS(TIMEOUT_SHORT));
	led_type = LED_TYPE_COLOR;
	ws2812_write_leds(color & 0xFFFFFF);
	xSemaphoreGive(led_mutex);
}

void led_setfade(uint16_t fade)
{
	xSemaphoreTake(led_mutex, pdMS_TO_TICKS(TIMEOUT_SHORT));
	led_type = LED_TYPE_FADE;
	if(led_fade >= LED_FADE_COUNT)
		led_fade = LED_FADE_COUNT - 1;

	led_fade_t* current_fade = &led_fades[led_fade];
	current_fade->position = 0;
	current_fade->direction = 1;

	ws2812_write_leds(current_fade->colors[0]);
	xSemaphoreGive(led_mutex);
}

void led_resetfade()
{
	led_setfade(led_fade);
}
