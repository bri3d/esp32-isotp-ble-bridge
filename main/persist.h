#ifndef PERSIST_H
#define PERSIST_H

void persist_start();
void persist_stop();

int16_t persist_send();
uint16_t persist_enabled();
void persist_set(uint16_t enable);
int16_t persist_add(const void* src, size_t size);
void persist_clear();
void persist_task(void *arg);

#endif
