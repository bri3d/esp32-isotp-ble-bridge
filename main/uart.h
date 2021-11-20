#ifndef UART_H
#define UART_H

void uart_init();
void uart_deinit();
void uart_start_task();
void uart_send(uint32_t txID, uint32_t rxID, uint8_t flags, const void* src, size_t size);
void uart_data_received(const void* src, size_t size);
void uart_buffer_clear();

#endif