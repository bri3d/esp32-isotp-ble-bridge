#include <stdint.h>

uint32_t send_identifier;
uint32_t receive_identifier;

void update_send_identifier(uint32_t new_send_identifier);
void update_receive_identifier(uint32_t new_receive_identifier);
uint32_t read_uint32_be(uint8_t *data);