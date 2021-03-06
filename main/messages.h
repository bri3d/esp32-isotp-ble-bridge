#ifndef __MESSAGES_H__
#define __MESSAGES_H__

// Simple struct for a dynamically sized send message

typedef struct send_message
{
    int32_t msg_length;
    uint8_t *buffer;
    uint32_t rx_id;
    uint32_t tx_id;
    bool reuse_buffer;
} send_message_t;

#endif
