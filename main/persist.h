#ifndef PERSIST_H
#define PERSIST_H

void 		persist_init();
void 		persist_deinit();
void		persist_start_task();

int16_t 	persist_send();
uint16_t 	persist_enabled();
void 		persist_set(uint16_t enable);
int16_t 	persist_add(const void* src, size_t size);
void 		persist_clear();
void 		persist_task(void *arg);
void 		persist_set_delay(uint16_t delay);
void 		persist_set_q_delay(uint16_t delay);
uint16_t 	persist_get_delay();
uint16_t 	persist_get_q_delay();

#endif
