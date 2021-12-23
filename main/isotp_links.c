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
    // acquire lock
    xSemaphoreTake(isotp_mutex, (TickType_t)100);
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
    // free lock
    xSemaphoreGive(isotp_mutex);
}
