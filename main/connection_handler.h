#ifndef CONNECTION_HANDLER_H
#define CONNECTION_HANDLER_H

void 		ch_init();
void 		ch_deinit();
void 		ch_start_task();
void		ch_reset_uart_timer();
bool 		ch_uart_connected();

void 		ch_on_uart_connect();
void 		ch_on_uart_disconnect();

#endif