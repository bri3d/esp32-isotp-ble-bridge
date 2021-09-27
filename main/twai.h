#ifndef TWAI_H
#define TWAI_H

void twai_install();
void twai_uninstall();
void twai_transmit_task(void *arg);
void twai_receive_task(void *arg);

#endif