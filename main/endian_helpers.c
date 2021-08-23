#include <stdint.h>
#include "endian_helpers.h"

uint16_t read_uint16_le(const uint8_t *data) {
  return data[0] | (data[1] << 8);
}

uint16_t read_uint16_be(const uint8_t *data) {
  return data[1] | (data[0] << 8);
}

uint32_t read_uint32_le(const uint8_t *data) {
  return data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
}

uint32_t read_uint32_be(const uint8_t *data) {
  return data[3] | (data[2] << 8) | (data[1] << 16) | (data[0] << 24);
}
