#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "assert.h"
#include "task_priorities.h"
#include "isotp_links.h"
#include "isotp_link_containers.h"
#include "isotp.h"
#include "mutexes.h"
#include "isotp_tasks.h"

void configure_isotp_link(int index, uint32_t receive_arbitration_id, uint32_t reply_arbitration_id, const char *name) {
    IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[index];
    memset(isotp_link_container, 0, sizeof(IsoTpLinkContainer));
    isotp_link_container->wait_for_isotp_data_sem = xSemaphoreCreateBinary();
    isotp_link_container->recv_buf = calloc(1, ISOTP_BUFSIZE);
    isotp_link_container->send_buf = calloc(1, ISOTP_BUFSIZE);
    isotp_link_container->payload_buf = calloc(1, ISOTP_BUFSIZE);
    assert(isotp_link_container->recv_buf != NULL);
    assert(isotp_link_container->send_buf != NULL);
    assert(isotp_link_container->payload_buf != NULL);
    isotp_init_link(
        &isotp_link_container->link,
        receive_arbitration_id, reply_arbitration_id,
        isotp_link_container->send_buf, ISOTP_BUFSIZE,
        isotp_link_container->recv_buf, ISOTP_BUFSIZE
    );
    xTaskCreatePinnedToCore(isotp_processing_task, name, 4096, isotp_link_container, ISOTP_TSK_PRIO, NULL, tskNO_AFFINITY);
}

void configure_isotp_links()
{
    // acquire lock
    xSemaphoreTake(isotp_mutex, (TickType_t)100);
    // RX_ID + TX_ID are flipped because this device acts as a "tester" listening for responses from ECUs. the ECU's TX is our RX
    // TODO: make these configurable via j2534 filters
    configure_isotp_link(0, 0x7E0, 0x7E8, "ecu");
    configure_isotp_link(1, 0x7E1, 0x7E9, "tcu");
    configure_isotp_link(2, 0x7E5, 0x7ED, "ptcu");
    configure_isotp_link(3, 0x607, 0x587, "gateway");
    configure_isotp_link(4, 0x749, 0x729, "awd");
    configure_isotp_link(5, 0x744, 0x724, "suspension1");
    configure_isotp_link(6, 0x692, 0x492, "suspension2");
    // free lock
    xSemaphoreGive(isotp_mutex);
}
