#ifndef __PERIODIC_MESSAGES_H__
#define __PERIODIC_MESSAGES_H__

#include <stdint.h>
#include "messages.h"

typedef struct periodic_message_t {
  send_message_t msgs[16];
  size_t num_msgs;
  uint32_t interval_ms;
} periodic_message_t;

void periodic_messages_task(void *arg);

#endif