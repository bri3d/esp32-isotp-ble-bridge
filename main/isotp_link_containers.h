#ifndef __ISOTP_LINK_CONTAINERS_H__
#define __ISOTP_LINK_CONTAINERS_H__

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "isotp_links.h"
#include "periodic_messages.h"

#define NUM_ISOTP_LINK_CONTAINERS 5
#define NUM_PERIODIC_MESSAGES_PER_LINK 16

typedef struct IsoTpLinkContainer {
    IsoTpLink link;
    SemaphoreHandle_t wait_for_isotp_data_sem;
    uint8_t *recv_buf;
    uint8_t *send_buf;
    uint8_t *payload_buf;
    TaskHandle_t periodic_message_task_handles[NUM_PERIODIC_MESSAGES_PER_LINK];
    periodic_message_t periodic_messages[NUM_PERIODIC_MESSAGES_PER_LINK];
} IsoTpLinkContainer;

IsoTpLinkContainer isotp_link_containers[NUM_ISOTP_LINK_CONTAINERS];

int find_isotp_link_container_index_by_send_arbitration_id(uint32_t arbitration_id);
int find_isotp_link_container_index_by_receive_arbitration_id(uint32_t arbitration_id);

#endif
