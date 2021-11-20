#ifndef LED_H
#define LED_H

#define LED_RED			0x00FF00
#define LED_GREEN		0xFF0000
#define LED_BLUE		0x0000FF
#define LED_RED_HALF	0x008000
#define LED_GREEN_HALF	0x800000
#define LED_BLUE_HALF	0x000080
#define LED_RED_QRT		0x004000
#define LED_GREEN_QRT	0x400000
#define LED_BLUE_QRT	0x000040
#define LED_RED_EHT		0x002000
#define LED_GREEN_EHT	0x200000
#define LED_BLUE_EHT	0x000020
#define LED_OFF			0x000000

void 		led_init();
void 		led_deinit();
void 		led_setcolor(uint32_t color);
uint32_t 	led_getcolor();


#endif