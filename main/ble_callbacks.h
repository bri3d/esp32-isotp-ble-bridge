void notifications_enabled();
void notifications_disabled();
void received_from_ble(const void *src, size_t size);
void ble_command_received(uint8_t *input, size_t length);