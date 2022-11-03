#ifndef __CAN_
#define __CAN__

int can_init();
int can_send(uint16_t arbitration_id, const uint8_t *buf, size_t size);
int can_recv(uint16_t *arbitration_id, uint8_t *buf, size_t *size);
void can_reset();

#endif