#ifndef LED_H
#define LED_H

#define LED_RED			0x00FF00
#define LED_GREEN		0xFF0000
#define LED_BLUE		0x0000FF
#define LED_RED_HALF	0x008000
#define LED_GREEN_HALF	0x800000
#define LED_BLUE_HALF	0x000080
#define LED_OFF			0x000000

void led_start();
void led_stop();
void led_setcolor(uint32_t color);


#endif