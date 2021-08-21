#include <stdint.h>
#include "esp_log.h"
#include "arbitration_identifiers.h"

const char *ARBITRATION_IDENTIFIERS_TAG = "arbitration_identifiers";

uint32_t read_uint32_be(uint8_t *data) {
  return data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
}

uint32_t read_uint32_le(uint8_t *data) {
  return data[3] | (data[2] << 8) | (data[1] << 16) | (data[0] << 24);
}

void update_send_identifier(uint32_t new_send_identifier) {
  ESP_LOGI(ARBITRATION_IDENTIFIERS_TAG, "update_send_identifier = %08x", new_send_identifier);
  send_identifier = new_send_identifier;
}

void update_receive_identifier(uint32_t new_receive_identifier) {
  ESP_LOGI(ARBITRATION_IDENTIFIERS_TAG, "update_receive_identifier = %08x", new_receive_identifier);
  receive_identifier = new_receive_identifier;
}
