#ifndef SLEEP_H
#define SLEEP_H

void 		sleep_init();
void 		sleep_deinit();
void 		sleep_start_task();
void		sleep_reset_uart();
bool 		sleep_uart_connected();

void 		sleep_on_connection();
void 		sleep_on_disconnection();

#endif