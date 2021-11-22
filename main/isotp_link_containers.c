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
    IsoTpLinkContainer *ecu_isotp_link_container	= &isotp_link_containers[0];
    IsoTpLinkContainer *tcu_isotp_link_container	= &isotp_link_containers[1];
    IsoTpLinkContainer *haldex_isotp_link_container = &isotp_link_containers[2];
	IsoTpLinkContainer *dtc_isotp_link_container	= &isotp_link_containers[3];

	// ECU
	strcpy(ecu_isotp_link_container->name, "isotp_container_ecu");
	ecu_isotp_link_container->buffer_size = ISOTP_BUFFER_SIZE;
	ecu_isotp_link_container->recv_buf = calloc(1, ecu_isotp_link_container->buffer_size);
	ecu_isotp_link_container->send_buf = calloc(1, ecu_isotp_link_container->buffer_size);
	ecu_isotp_link_container->payload_buf = calloc(1, ecu_isotp_link_container->buffer_size);
	assert(ecu_isotp_link_container->recv_buf != NULL);
	assert(ecu_isotp_link_container->send_buf != NULL);
	assert(ecu_isotp_link_container->payload_buf != NULL);
	isotp_init_link(
		&ecu_isotp_link_container->link,
		0x7E0, 0x7E8,
		ecu_isotp_link_container->send_buf, ecu_isotp_link_container->buffer_size,
		ecu_isotp_link_container->recv_buf, ecu_isotp_link_container->buffer_size
	);

    // TCU
	strcpy(tcu_isotp_link_container->name, "isotp_container_tcu");
	tcu_isotp_link_container->buffer_size = ISOTP_BUFFER_SIZE;
    tcu_isotp_link_container->recv_buf = calloc(1, tcu_isotp_link_container->buffer_size);
    tcu_isotp_link_container->send_buf = calloc(1, tcu_isotp_link_container->buffer_size);
    tcu_isotp_link_container->payload_buf = calloc(1, tcu_isotp_link_container->buffer_size);
    assert(tcu_isotp_link_container->recv_buf != NULL);
    assert(tcu_isotp_link_container->send_buf != NULL);
    assert(tcu_isotp_link_container->payload_buf != NULL);
	isotp_init_link(
        &tcu_isotp_link_container->link,
        0x7E1, 0x7E9,
        tcu_isotp_link_container->send_buf, tcu_isotp_link_container->buffer_size,
        tcu_isotp_link_container->recv_buf, tcu_isotp_link_container->buffer_size
    );

    // HALDEX
	strcpy(haldex_isotp_link_container->name, "isotp_container_haldex");
	haldex_isotp_link_container->buffer_size = ISOTP_BUFFER_SIZE;
	haldex_isotp_link_container->recv_buf = calloc(1, haldex_isotp_link_container->buffer_size);
	haldex_isotp_link_container->send_buf = calloc(1, haldex_isotp_link_container->buffer_size);
	haldex_isotp_link_container->payload_buf = calloc(1, haldex_isotp_link_container->buffer_size);
    assert(haldex_isotp_link_container->recv_buf != NULL);
    assert(haldex_isotp_link_container->send_buf != NULL);
    assert(haldex_isotp_link_container->payload_buf != NULL);
    isotp_init_link(
        &haldex_isotp_link_container->link,
        0x7E5, 0x7ED,
		haldex_isotp_link_container->send_buf, haldex_isotp_link_container->buffer_size,
		haldex_isotp_link_container->recv_buf, haldex_isotp_link_container->buffer_size
    );

	// DTC
	strcpy(dtc_isotp_link_container->name, "isotp_container_dtc");
	dtc_isotp_link_container->buffer_size = ISOTP_BUFFER_SIZE_SMALL;
	dtc_isotp_link_container->recv_buf = calloc(1, dtc_isotp_link_container->buffer_size);
	dtc_isotp_link_container->send_buf = calloc(1, dtc_isotp_link_container->buffer_size);
	dtc_isotp_link_container->payload_buf = calloc(1, dtc_isotp_link_container->buffer_size);
	assert(dtc_isotp_link_container->recv_buf != NULL);
	assert(dtc_isotp_link_container->send_buf != NULL);
	assert(dtc_isotp_link_container->payload_buf != NULL);
	isotp_init_link(
		&dtc_isotp_link_container->link,
		0x700, 0x7E8,
		dtc_isotp_link_container->send_buf, dtc_isotp_link_container->buffer_size,
		dtc_isotp_link_container->recv_buf, dtc_isotp_link_container->buffer_size
	);

	//create semaphores for each link
	for(uint16_t i = 0; i < NUM_ISOTP_LINK_CONTAINERS; i++)
	{
		IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[i];
		isotp_link_container->wait_for_isotp_data_sem = xSemaphoreCreateBinary();
	}

	//reset default link container id
	isotp_link_container_id = 0;

    // free lock
	xSemaphoreGive(isotp_mutex);
}

void disable_isotp_links()
{
	//delete semaphores for each link
	for(uint16_t i = 0; i < NUM_ISOTP_LINK_CONTAINERS; i++)
	{
		IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[i];
		vSemaphoreDelete(isotp_link_container->wait_for_isotp_data_sem);
	}
}
