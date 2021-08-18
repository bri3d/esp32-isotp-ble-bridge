// Simple struct for a dynamically sized send message

typedef struct send_message
{
    int32_t msg_length;
    uint8_t *buffer;
} send_message_t;