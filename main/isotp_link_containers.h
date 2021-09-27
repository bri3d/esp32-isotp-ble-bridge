#ifndef __ISOTP_LINK_CONTAINERS_H__
#define __ISOTP_LINK_CONTAINERS_H__

#define NUM_ISOTP_LINK_CONTAINERS 5

typedef struct IsoTpLinkContainer {
    IsoTpLink link;
    SemaphoreHandle_t wait_for_isotp_data_sem;
    uint8_t *recv_buf;
    uint8_t *send_buf;
    uint8_t *payload_buf;
} IsoTpLinkContainer;

IsoTpLinkContainer isotp_link_containers[NUM_ISOTP_LINK_CONTAINERS];

void configure_isotp_links();
void disable_isotp_links();

#endif
