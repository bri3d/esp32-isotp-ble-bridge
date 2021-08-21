#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * DEFINES
 ****************************************************************************************
 */

#define spp_sprintf(s,...)         sprintf((char*)(s), ##__VA_ARGS__)
#define SPP_DATA_MAX_LEN           (512)
#define SPP_CMD_MAX_LEN            (20)
#define SPP_STATUS_MAX_LEN         (20)
#define SPP_DATA_BUFF_MAX_LEN      (2*1024)
///Attributes State Machine
enum{
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_CHAR,
    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NOTIFY_CHAR,
    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG,

    SPP_IDX_SPP_COMMAND_CHAR,
    SPP_IDX_SPP_COMMAND_VAL,

    SPP_IDX_SPP_STATUS_CHAR,
    SPP_IDX_SPP_STATUS_VAL,
    SPP_IDX_SPP_STATUS_CFG,
    SPP_IDX_NB,
};

typedef struct 
{
	void (*data_received)			(const void* src, size_t size);		/* Data received callback - a full frame has been constructed from the client. Buf is not guaranteed to live and should be copied. */
    void (*command_received)        (uint8_t* cmd_str, size_t length);  /* Command received callback - a command was written to the command GATT */
    void (*notifications_subscribed)();
    void (*notifications_unsubscribed)();
} ble_server_callbacks;

void ble_server_setup(ble_server_callbacks callbacks);
void ble_send(uint32_t tx_id, uint32_t rx_id, const void* src, size_t size);
void ble_set_status(const void* src, size_t size);