#include <stdint.h>
#include "arbitration_identifiers.h"

uint32_t read_uint32_be(uint8_t *data) {
  return data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
}

uint32_t read_uint32_le(uint8_t *data) {
  return data[3] | (data[2] << 8) | (data[1] << 16) | (data[0] << 24);
}
