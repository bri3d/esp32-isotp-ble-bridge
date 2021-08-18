#include <stdint.h>
#include "arbitration_identifiers.h"

uint32_t read_uint32_be(uint8_t *data) {
  return data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
}

void update_send_identifier(uint32_t new_send_identifier) {
  send_identifier = new_send_identifier;
}

void update_receive_identifier(uint32_t new_receive_identifier) {
  receive_identifier = new_receive_identifier;
}
