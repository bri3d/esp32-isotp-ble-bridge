#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "isotp.h"
#include "mutexes.h"
#include "queues.h"
#include "isotp_link_containers.h"
#include "constants.h"

void configure_isotp_links()
{
    // acquire lock
    xSemaphoreTake(isotp_mutex, pdMS_TO_TICKS(TIMEOUT_LONG));
	// RX_ID + TX_ID are flipped because this device acts as a "tester" listening for responses from ECUs. the ECU's TX is our RX
    // TODO: make these configurable via j2534 filters
    IsoTpLinkContainer *ecu_isotp_link_container = &isotp_link_containers[0];
    IsoTpLinkContainer *tcu_isotp_link_container = &isotp_link_containers[1];
    IsoTpLinkContainer *ptcu_isotp_link_container = &isotp_link_containers[2];
	IsoTpLinkContainer *gateway_isotp_link_container = &isotp_link_containers[3];
	IsoTpLinkContainer *dtc_isotp_link_container = &isotp_link_containers[4];

	// ECU
	ecu_isotp_link_container->recv_buf = calloc(1, ISOTP_BUFSIZE);
	ecu_isotp_link_container->send_buf = calloc(1, ISOTP_BUFSIZE);
	ecu_isotp_link_container->payload_buf = calloc(1, ISOTP_BUFSIZE);
	assert(ecu_isotp_link_container->recv_buf != NULL);
	assert(ecu_isotp_link_container->send_buf != NULL);
	assert(ecu_isotp_link_container->payload_buf != NULL);
	isotp_init_link(
		&ecu_isotp_link_container->link,
		0x7E0, 0x7E8,
		ecu_isotp_link_container->send_buf, ISOTP_BUFSIZE,
		ecu_isotp_link_container->recv_buf, ISOTP_BUFSIZE
	);
    // TCU
    tcu_isotp_link_container->recv_buf = calloc(1, ISOTP_BUFSIZE);
    tcu_isotp_link_container->send_buf = calloc(1, ISOTP_BUFSIZE);
    tcu_isotp_link_container->payload_buf = calloc(1, ISOTP_BUFSIZE);
    assert(tcu_isotp_link_container->recv_buf != NULL);
    assert(tcu_isotp_link_container->send_buf != NULL);
    assert(tcu_isotp_link_container->payload_buf != NULL);
	isotp_init_link(
        &tcu_isotp_link_container->link,
        0x7E1, 0x7E9,
        tcu_isotp_link_container->send_buf, ISOTP_BUFSIZE,
        tcu_isotp_link_container->recv_buf, ISOTP_BUFSIZE
    );
    // PTCU
    ptcu_isotp_link_container->recv_buf = calloc(1, ISOTP_BUFSIZE);
    ptcu_isotp_link_container->send_buf = calloc(1, ISOTP_BUFSIZE);
    ptcu_isotp_link_container->payload_buf = calloc(1, ISOTP_BUFSIZE);
    assert(ptcu_isotp_link_container->recv_buf != NULL);
    assert(ptcu_isotp_link_container->send_buf != NULL);
    assert(ptcu_isotp_link_container->payload_buf != NULL);
    isotp_init_link(
        &ptcu_isotp_link_container->link,
        0x7E5, 0x7ED,
        ptcu_isotp_link_container->send_buf, ISOTP_BUFSIZE,
        ptcu_isotp_link_container->recv_buf, ISOTP_BUFSIZE
    );
    // Gateway
    gateway_isotp_link_container->recv_buf = calloc(1, ISOTP_BUFSIZE);
    gateway_isotp_link_container->send_buf = calloc(1, ISOTP_BUFSIZE);
    gateway_isotp_link_container->payload_buf = calloc(1, ISOTP_BUFSIZE);
    assert(gateway_isotp_link_container->recv_buf != NULL);
    assert(gateway_isotp_link_container->send_buf != NULL);
    assert(gateway_isotp_link_container->payload_buf != NULL);
    isotp_init_link(
        &gateway_isotp_link_container->link,
        0x607, 0x587,
        gateway_isotp_link_container->send_buf, ISOTP_BUFSIZE,
        gateway_isotp_link_container->recv_buf, ISOTP_BUFSIZE
	);
	// DTC
	dtc_isotp_link_container->recv_buf = calloc(1, ISOTP_BUFSIZE);
	dtc_isotp_link_container->send_buf = calloc(1, ISOTP_BUFSIZE);
	dtc_isotp_link_container->payload_buf = calloc(1, ISOTP_BUFSIZE);
	assert(dtc_isotp_link_container->recv_buf != NULL);
	assert(dtc_isotp_link_container->send_buf != NULL);
	assert(dtc_isotp_link_container->payload_buf != NULL);
	isotp_init_link(
		&dtc_isotp_link_container->link,
		0x700, 0x7E8,
		dtc_isotp_link_container->send_buf, ISOTP_BUFSIZE,
		dtc_isotp_link_container->recv_buf, ISOTP_BUFSIZE
	);

	//create semaphores for each link
	for(int i = 0; i < NUM_ISOTP_LINK_CONTAINERS; ++i)
	{
		IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[i];
		isotp_link_container->wait_for_isotp_data_sem = xSemaphoreCreateBinary();
	}

    // free lock
    xSemaphoreGive(isotp_mutex);
}

void disable_isotp_links()
{
	//delete semaphores for each link
	for(int i = 0; i < NUM_ISOTP_LINK_CONTAINERS; ++i)
	{
		IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[i];
		vSemaphoreDelete(isotp_link_container->wait_for_isotp_data_sem);
	}
}
