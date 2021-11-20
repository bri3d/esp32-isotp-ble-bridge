#ifndef TWAI_H
#define TWAI_H

void twai_init();
void twai_deinit();
void twai_start_task();
void twai_transmit_task(void *arg);
void twai_receive_task(void *arg);

#endif